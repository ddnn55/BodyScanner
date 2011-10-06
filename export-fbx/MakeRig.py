import sys,yaml
from skeleton import Skeleton
    
def CreateScene(pSdkManager, pScene, files):
        
    lSceneInfo = KFbxDocumentInfo.Create(pSdkManager, "SceneInfo")
    lSceneInfo.mTitle = "Body scan"
    lSceneInfo.mSubject = "A body scan"
    lSceneInfo.mAuthor = "Team KinectOpenNiRosBoostGraphLibDaiPclEigen"
    lSceneInfo.mRevision = "rev. 0.1"
    lSceneInfo.mKeywords = "body mesh rigged"
    lSceneInfo.mComment = "no comments"
    pScene.SetSceneInfo(lSceneInfo)

    import os,glob
    
    skelFiles = glob.glob(os.path.dirname(files['skeleton'])+'/*.yaml')
    skelFiles.sort()
    print 'starting export'
    baseSkeletonData = SkeletonFromYaml(skelFiles[0])
    print 'imported skeleton'
    lMeshNode,T,V = CreateMesh(pSdkManager, files['obj'])
    pScene.GetRootNode().AddChild(lMeshNode)
    print 'converted mesh'
    CreateSkeletonNodes(pSdkManager, "Skeleton", baseSkeletonData)
    pScene.GetRootNode().AddChild(baseSkeletonData.torsoNode)
    return
    print 'created skeleton'
    LinkMeshToSkeleton(lSdkManager, lMeshNode, baseSkeletonData, files['weights'])
    print 'added skin'
    print 'now adding animation (slow)'
    skels = [(int(fname[-10:-5])/15.,SkeletonFromYaml(fname)) for fname in skelFiles[1:]]
    print 'imported all skeletons'
    AnimateSkeleton(pSdkManager, pScene, baseSkeletonData, skels)
    print 'animated skeleton'
    
    

    return True

def SkeletonFromYaml(fname):
    # apparently the yaml parser in python doesn't understand this syntax,
    # so we transform it!
    y = open(fname, 'r').read().replace('    - confidence --', '    confidence:')
    return Skeleton(yaml.load(y))

def CreateMesh(pSdkManager, inputName):
    f = open(inputName, 'r')
    T = []
    V = []
    scale = 50
    for line in f:
        if line.startswith('f'):
            indices = [int(i)-1 for i in line.split()[1:]] # one-based .obj indices, zero-based .fbx
            T.append(indices)
        elif line.startswith('v'):
            parts = line.split()
            vertex = [float(v) for v in line.split()[1:]]
            # scale up xyz
            vertex[0] *= scale
            vertex[1] = 50-vertex[1]*scale
            vertex[2] *= scale
            V.append(vertex)
    
    lMesh = KFbxMesh.Create(pSdkManager,inputName)
    lMesh.InitControlPoints(len(V))
    
    layer = lMesh.GetLayer(0)
    if not layer:
        lMesh.CreateLayer()
        layer = lMesh.GetLayer(0)
    
    vtxcLayer = KFbxLayerElementVertexColor.Create(lMesh, "")
    vtxcLayer.SetMappingMode(KFbxLayerElement.eBY_CONTROL_POINT)
    vtxcLayer.SetReferenceMode(KFbxLayerElement.eDIRECT)

    vtxColors = vtxcLayer.GetDirectArray()
    for i,(x,y,z,r,g,b) in enumerate(V):
        lMesh.SetControlPointAt(KFbxVector4(x,y,z), i)
        vtxColors.Add(KFbxColor(r,g,b,1))
    
    layer.SetVertexColors(vtxcLayer)
    
    for ijk in T:
        lMesh.BeginPolygon()
        for i in ijk:
            lMesh.AddPolygon(i)
        lMesh.EndPolygon()
    
    # wrap the mesh in a node, ok, sure, why not FBX, if you insist
    lNode = KFbxNode.Create(pSdkManager,inputName)
    lNode.SetNodeAttribute(lMesh)
    return lNode,T,V


# Set the influence of the skeleton segments over the cylinder.
def LinkMeshToSkeleton(pSdkManager, pMesh, skeletonData, weightFile):
    
    #import pdb; pdb.set_trace()
    bindings = []
    node = None
    f = open(weightFile, 'r')
    line = f.readline()
    while line:
        if line.startswith('b'):
            jointName = line[2:].strip()
            node = getattr(skeletonData, jointName+'Node') # get the node associated with the joint
            cluster = KFbxCluster.Create(pSdkManager, jointName)
            cluster.SetLink(node)
            cluster.SetLinkMode(KFbxCluster.eNORMALIZE)
            bindings.append((cluster,node))
        else:
            parts = line.split()
            index,weight = int(parts[0]),float(parts[1])
            cluster.AddControlPointIndex(index,weight)
        line = f.readline()

        
    # Now we have the mesh and the skeleton correctly positioned,
    # set the Transform and TransformLink matrix accordingly.
    
    scene = pMesh.GetScene()
    if scene:
        matrix = scene.GetEvaluator().GetNodeGlobalTransform(pMesh)
    else:
        matrix = KFbxXMatrix()
    for cluster,node in bindings:
        cluster.SetTransformMatrix(matrix)    
    
    for cluster,node in bindings:
        if scene:
            matrix = scene.GetEvaluator().GetNodeGlobalTransform(node)
        else:
            matrix = KFbxXMatrix()
        cluster.SetTransformLinkMatrix(matrix)
    
    # Add the clusters to the mesh by creating a skin and adding those clusters to that skin.
    # After add that skin.
    lSkin = KFbxSkin.Create(pSdkManager, "")
    for cluster,node in bindings:
        lSkin.AddCluster(cluster)
    pMesh.GetNodeAttribute().AddDeformer(lSkin)
    
# Add the specified node to the node array. Also, add recursively
# all the parent node of the specified node to the array.
def AddNodeRecursively(pNodeArray, pNode):
    if pNode:
        AddNodeRecursively(pNodeArray, pNode.GetParent())
        found = False 
        for node in pNodeArray:
            if node.GetName() == pNode.GetName():
                found = True
        if not found:
            # Node not in the list, add it
            pNodeArray += [pNode]
    
# Store the Bind Pose
def StoreBindPose(pSdkManager, pScene, pMesh, pSkeletonRoot):
    # In the bind pose, we must store all the link's global matrix at the time of the bind.
    # Plus, we must store all the parent(s) global matrix of a link, even if they are not
    # themselves deforming any model.

    # In this example, since there is only one model deformed, we don't need walk through the scene

    # Now list the all the link involve in the mesh deformation
    lClusteredFbxNodes = []
    if pMesh and pMesh.GetNodeAttribute():
        lSkinCount = 0
        lClusterCount = 0
        lNodeAttributeType = pMesh.GetNodeAttribute().GetAttributeType()
        if lNodeAttributeType in (KFbxNodeAttribute.eMESH, KFbxNodeAttribute.eNURB, KFbxNodeAttribute.ePATCH):
            lSkinCount = pMesh.GetNodeAttribute().GetDeformerCount(KFbxDeformer.eSKIN)
            for i in range(lSkinCount):
                lSkin = pMesh.GetNodeAttribute().GetDeformer(i, KFbxDeformer.eSKIN)
                lClusterCount += lSkin.GetClusterCount()
                
        # If we found some clusters we must add the node
        if lClusterCount:
            # Again, go through all the skins get each cluster link and add them
            for i in range(lSkinCount):
                lSkin = pMesh.GetNodeAttribute().GetDeformer(i, KFbxDeformer.eSKIN)
                lClusterCount = lSkin.GetClusterCount()
                for j in range(lClusterCount):
                    lClusterNode = lSkin.GetCluster(j).GetLink()
                    AddNodeRecursively(lClusteredFbxNodes, lClusterNode)
                    
            # Add the mesh to the pose
            lClusteredFbxNodes += [pMesh]
            
    # Now create a bind pose with the link list
    if len(lClusteredFbxNodes):
        # A pose must be named. Arbitrarily use the name of the mesh node.
        lPose = KFbxPose.Create(pSdkManager, pMesh.GetName())
        lPose.SetIsBindPose(True)

        for lKFbxNode in lClusteredFbxNodes:
            lBindMatrix = KFbxXMatrix()
            lScene = lKFbxNode.GetScene()
            if lScene:
                lBindMatrix = lScene.GetEvaluator().GetNodeGlobalTransform(lKFbxNode)
            lPose.Add(lKFbxNode, KFbxMatrix(lBindMatrix))

        # Add the pose to the scene
        pScene.AddPose(lPose)


def CreateSkeletonNodes(pSdkManager, pName, skeletonData):

    # Create skeleton root
    lRootName = pName + "Torso"
    lTorsoAttribute = KFbxSkeleton.Create(lSdkManager, lRootName)
    lTorsoAttribute.SetSkeletonType(KFbxSkeleton.eROOT)
    torso = KFbxNode.Create(lSdkManager, lRootName)
    torso.SetNodeAttribute(lTorsoAttribute)    
    torso.LclTranslation.Set(fbxDouble3(*skeletonData.torso))

    def MakeLimb(name, skeletonData):
        pos = getattr(skeletonData, name)
        lLimbNodeName = pName + name
        lSkeletonLimbNodeAttribute = KFbxSkeleton.Create(lSdkManager, lLimbNodeName)
        lSkeletonLimbNodeAttribute.SetSkeletonType(KFbxSkeleton.eLIMB_NODE)
        lSkeletonLimbNodeAttribute.Size.Set(100.0)
        lSkeletonLimbNode = KFbxNode.Create(lSdkManager, lLimbNodeName)
        lSkeletonLimbNode.SetNodeAttribute(lSkeletonLimbNodeAttribute)    
        lSkeletonLimbNode.LclTranslation.Set(fbxDouble3(*pos)) # oh, Lcl = Local, maybe I could have avoided relativizing
        setattr(skeletonData, name+'Node', lSkeletonLimbNode)
        return lSkeletonLimbNode

    # Build skeleton node connections.
    skeletonData.torsoNode = torso

    neck = MakeLimb("neck", skeletonData)
    head = MakeLimb("head", skeletonData) 

    leftArm = [
        MakeLimb("left_shoulder", skeletonData),
        MakeLimb("left_elbow", skeletonData),
        MakeLimb("left_hand", skeletonData)
    ]
    rightArm = [
        MakeLimb("right_shoulder", skeletonData),
        MakeLimb("right_elbow", skeletonData),
        MakeLimb("right_hand", skeletonData)
    ]
    leftLeg = [
        MakeLimb("left_hip", skeletonData),
        MakeLimb("left_knee", skeletonData),
        MakeLimb("left_foot", skeletonData)
    ]
    rightLeg = [
        MakeLimb("right_hip", skeletonData),
        MakeLimb("right_knee", skeletonData),
        MakeLimb("right_foot", skeletonData)
    ]

    AddChain(torso, neck, head)
    AddChain(neck,*leftArm)
    AddChain(neck,*rightArm)    
    AddChain(torso,*leftLeg)
    AddChain(torso,*rightLeg)



def AddChain(*nodes):
    for i in range(len(nodes)-1):
        nodes[i].AddChild(nodes[i+1])

def constrain(val, lo, hi):
    if val < lo: return lo
    if val > hi: return hi
    return val

jointNames = "torso head neck left_shoulder right_shoulder left_elbow right_elbow left_hand right_hand left_hip right_hip left_knee right_knee left_foot right_foot".split()

# Create two animation stacks.
def AnimateSkeleton(pSdkManager, pScene, body, skels):
    lKeyIndex = 0
    lTime = KTime()

    lRoot = body.torsoNode

    # First animation stack.
    lAnimStackName = "Bend on 2 sides"
    lAnimStack = KFbxAnimStack.Create(pScene, lAnimStackName)

    # The animation nodes can only exist on AnimLayers therefore it is mandatory to
    # add at least one AnimLayer to the AnimStack. And for the purpose of this example,
    # one layer is all we need.
    lAnimLayer = KFbxAnimLayer.Create(pScene, "Base Layer")
    lAnimStack.AddMember(lAnimLayer)

    baseTime = skels[0][0] # first time

    # for each joint
    for jointName in jointNames:
        node = getattr(body, jointName+'Node') # get the node associated with the joint
        for coordId,coord in enumerate(['X','Y','Z']):
            # create and define the values of this position variable over time
            lCurve = node.LclTranslation.GetCurve(lAnimLayer, coord, True)
            lCurve.KeyModifyBegin()
            for t,skel in skels:
                pos = getattr(skel, jointName) # get the position (at this keyframe) of this joint
                if pos is not None:
                    lTime.SetSecondDouble(t-baseTime) # set time
                    lKeyIndex = lCurve.KeyAdd(lTime)[0]
                    lCurve.KeySetValue(lKeyIndex, pos[coordId])
                    # use linear because we have a dense sampling,
                    # using CUBIC cause a lot of overshoot
                    lCurve.KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef.eINTERPOLATION_LINEAR)
            lCurve.KeyModifyEnd()







if __name__ == "__main__":
    try:
        import FbxCommon
        from fbx import *
    except ImportError:
        import platform
        msg = 'You need to copy the content in compatible subfolder under /lib/python<version> into your python install folder such as '
        if platform.system() == 'Windows' or platform.system() == 'Microsoft':
            msg += '"Python26/Lib/site-packages"'
        elif platform.system() == 'Linux':
            msg += '"/usr/local/lib/python2.6/site-packages"'
        elif platform.system() == 'Darwin':
            msg += '"/Library/Frameworks/Python.framework/Versions/2.6/lib/python2.6/site-packages"'        
        msg += ' folder.'
        print(msg) 
        sys.exit(1)

    # Prepare the FBX SDK.
    (lSdkManager, lScene) = FbxCommon.InitializeSdkObjects()
    # embed?
    #lSdkManager.GetIOSettings().SetBoolProp(EXP_FBX_EMBEDDED, True)
    
    # The example can take an output file name as an argument.
    if len(sys.argv) < 5:
        print 'usage: MakeRig.py <skeleton.yaml> <mesh.obj> <weights.w> <output.fbx>'
        sys.exit(1)
    else:
        files = {
            'skeleton': sys.argv[1],
            'obj': sys.argv[2],
            'weights': sys.argv[3],
            'output': sys.argv[4]
        }
        assert files['skeleton'].endswith('.yaml')
        assert files['obj'].endswith('.obj')
        assert files['weights'].endswith('.w')
        assert files['output'].endswith('.fbx')
        

    # Create the scene.
    lResult = CreateScene(lSdkManager, lScene, files)

    if lResult == False:
        print("\n\nAn error occurred while creating the scene...\n")
        lSdkManager.Destroy()
        sys.exit(1)

    # Save the scene.
    # 1 = text version 2012
    # 3 = (working?) binary
    lResult = FbxCommon.SaveScene(lSdkManager, lScene, files['output'], 3)

    if lResult == False:
        print("\n\nAn error occurred while saving the scene...\n")
        lSdkManager.Destroy()
        sys.exit(1)

    # Destroy all objects created by the FBX SDK.
    lSdkManager.Destroy()
   
    sys.exit(0)
