import sys

OUTPUT_NAME = "dragon16k.fbx"

gExportVertexCacheMCFormat = True
gCacheType = 0
    
def CreateScene(pSdkManager, pScene, pSampleFileName):
    lMeshNode,T,V = CreateMesh(pSdkManager, 'dragon16k.obj')
    
    SetMeshDefaultPosition(lMeshNode)
        
    lSceneInfo = KFbxDocumentInfo.Create(pSdkManager, "SceneInfo")
    lSceneInfo.mTitle = "Dragon wag"
    lSceneInfo.mSubject = "A test, to see if we can make a dragon move"
    lSceneInfo.mAuthor = "Team KinectOpenNiRosBoostGraphLibDaiPclEigen"
    lSceneInfo.mRevision = "rev. 0.1"
    lSceneInfo.mKeywords = "deformed dragon mesh rigged"
    lSceneInfo.mComment = "no particular comments required."
    pScene.SetSceneInfo(lSceneInfo)

    lSkeletonRoot = CreateSkeleton(pSdkManager, "Skeleton")

    pScene.GetRootNode().AddChild(lMeshNode)
    pScene.GetRootNode().AddChild(lSkeletonRoot)

    LinkMeshToSkeleton(lSdkManager, lMeshNode, T, V, lSkeletonRoot)
    StoreBindPose(lSdkManager, lScene, lMeshNode, lSkeletonRoot)
    StoreRestPose(lSdkManager, lScene, lSkeletonRoot)

    AnimateSkeleton(pSdkManager, pScene, lSkeletonRoot)
        
    return True

def CreateMesh(pSdkManager, inputName):
    f = open(inputName, 'r')
    T = []
    V = []
    for line in f:
        if line.startswith('f'):
            indices = [int(i)-1 for i in line.split()[1:]] # one-based .obj indices, zero-based .fbx
            T.append(indices)
        elif line.startswith('v'):
            vertex = [float(v)*1000 for v in line.split()[1:]]
            V.append(vertex)
    
    lMesh = KFbxMesh.Create(pSdkManager,inputName)
    lMesh.InitControlPoints(len(V))
    for i,(x,y,z) in enumerate(V):
        lMesh.SetControlPointAt(KFbxVector4(x,y,z), i)
    
    for ijk in T:
        lMesh.BeginPolygon()
        for i in ijk:
            lMesh.AddPolygon(i)
        lMesh.EndPolygon()
    
    # wrap the mesh in a node, ok, sure, why not FBX, if you insist
    lNode = KFbxNode.Create(pSdkManager,inputName)
    lNode.SetNodeAttribute(lMesh)
    return lNode,T,V

    


# Cube is translated to the left.
def SetMeshDefaultPosition(pMesh):
    pMesh.LclTranslation.Set(fbxDouble3(0, 0, 0.0))
    pMesh.LclRotation.Set(fbxDouble3(0.0, 0.0, 0.0))
    pMesh.LclScaling.Set(fbxDouble3(1.0, 1.0, 1.0))






# --------------------------------------------------------------------------------
# --------------------------------------------------------------------------------
# --------------------------------------------------------------------------------
# --------------------------------------------------------------------------------
# --------------------------------------------------------------------------------
# --------------------------------------------------------------------------------
# --------------------------------------------------------------------------------
# --------------------------------------------------------------------------------
# --------------------------------------------------------------------------------
# --------------------------------------------------------------------------------



    
# Create a skeleton with 2 segments.
def CreateSkeleton(pSdkManager, pName):
    # Create skeleton root
    lRootName = pName + "Root"
    lSkeletonRootAttribute = KFbxSkeleton.Create(lSdkManager, lRootName)
    lSkeletonRootAttribute.SetSkeletonType(KFbxSkeleton.eROOT)
    lSkeletonRoot = KFbxNode.Create(lSdkManager, lRootName)
    lSkeletonRoot.SetNodeAttribute(lSkeletonRootAttribute)    
    lSkeletonRoot.LclTranslation.Set(fbxDouble3(-50.0, 40.0, 0.0))
    
    # Create skeleton first limb node.
    lLimbNodeName1 = pName + "LimbNode1"
    lSkeletonLimbNodeAttribute1 = KFbxSkeleton.Create(lSdkManager, lLimbNodeName1)
    lSkeletonLimbNodeAttribute1.SetSkeletonType(KFbxSkeleton.eLIMB_NODE)
    lSkeletonLimbNodeAttribute1.Size.Set(100.0)
    lSkeletonLimbNode1 = KFbxNode.Create(lSdkManager, lLimbNodeName1)
    lSkeletonLimbNode1.SetNodeAttribute(lSkeletonLimbNodeAttribute1)    
    lSkeletonLimbNode1.LclTranslation.Set(fbxDouble3(20.0, 40.0, 0.0))
    
    # Create skeleton second limb node.
    lLimbNodeName2 = pName + "LimbNode2"
    lSkeletonLimbNodeAttribute2 = KFbxSkeleton.Create(lSdkManager, lLimbNodeName2)
    lSkeletonLimbNodeAttribute2.SetSkeletonType(KFbxSkeleton.eLIMB_NODE)
    lSkeletonLimbNodeAttribute2.Size.Set(100.0)
    lSkeletonLimbNode2 = KFbxNode.Create(lSdkManager, lLimbNodeName2)
    lSkeletonLimbNode2.SetNodeAttribute(lSkeletonLimbNodeAttribute2)    
    lSkeletonLimbNode2.LclTranslation.Set(fbxDouble3(60.0, 40.0, 0.0))
    
    # Build skeleton node hierarchy. 
    lSkeletonRoot.AddChild(lSkeletonLimbNode1)
    lSkeletonLimbNode1.AddChild(lSkeletonLimbNode2)
    return lSkeletonRoot

def constrain(val, lo, hi):
    if val < lo: return lo
    if val > hi: return hi
    return val

# Set the influence of the skeleton segments over the cylinder.
def LinkMeshToSkeleton(pSdkManager, pMesh, T, V, pSkeletonRoot):
    lLimbNode1 = pSkeletonRoot.GetChild(0)
    lLimbNode2 = lLimbNode1.GetChild(0)
    
    # Bottom section of cylinder is clustered to skeleton root.
    lClusterToRoot = KFbxCluster.Create(pSdkManager, "")
    lClusterToRoot.SetLink(pSkeletonRoot)
    lClusterToRoot.SetLinkMode(KFbxCluster.eNORMALIZE)
    for i,(x,y,z) in enumerate(V):
        lClusterToRoot.AddControlPointIndex(i, constrain(1-(x+30)/60, 0, 1))
        
    # Center section of cylinder is clustered to skeleton limb node.
    lClusterToLimbNode1 = KFbxCluster.Create(pSdkManager, "")
    lClusterToLimbNode1.SetLink(lLimbNode1)
    lClusterToLimbNode1.SetLinkMode(KFbxCluster.eNORMALIZE)
    for i,(x,y,z) in enumerate(V):
        lClusterToLimbNode1.AddControlPointIndex(i, constrain(1-(x-20)/60, 0, 1))
        
    # Top section of cylinder is clustered to skeleton limb.
    lClusterToLimbNode2 = KFbxCluster.Create(pSdkManager, "")
    lClusterToLimbNode2.SetLink(lLimbNode2)
    lClusterToLimbNode2.SetLinkMode(KFbxCluster.eNORMALIZE)
    for i,(x,y,z) in enumerate(V):
        lClusterToLimbNode2.AddControlPointIndex(i, constrain(1-(x-60)/60, 0, 1))
        
    # Now we have the Patch and the skeleton correctly positioned,
    # set the Transform and TransformLink matrix accordingly.
    lXMatrix = KFbxXMatrix()
    lScene = pMesh.GetScene()
    if lScene:
        lXMatrix = lScene.GetEvaluator().GetNodeGlobalTransform(pMesh)
    lClusterToRoot.SetTransformMatrix(lXMatrix)
    lClusterToLimbNode1.SetTransformMatrix(lXMatrix)
    lClusterToLimbNode2.SetTransformMatrix(lXMatrix)
    lScene = pSkeletonRoot.GetScene()
    if lScene:
        lXMatrix = lScene.GetEvaluator().GetNodeGlobalTransform(pSkeletonRoot)
    lClusterToRoot.SetTransformLinkMatrix(lXMatrix)
    lScene = lLimbNode1.GetScene()
    if lScene:
        lXMatrix = lScene.GetEvaluator().GetNodeGlobalTransform(lLimbNode1)
    lClusterToLimbNode1.SetTransformLinkMatrix(lXMatrix)
    lScene = lLimbNode2.GetScene()
    if lScene:
        lXMatrix = lScene.GetEvaluator().GetNodeGlobalTransform(lLimbNode2)
    lClusterToLimbNode2.SetTransformLinkMatrix(lXMatrix)
    
    # Add the clusters to the patch by creating a skin and adding those clusters to that skin.
    # After add that skin.
    lSkin = KFbxSkin.Create(pSdkManager, "")
    lSkin.AddCluster(lClusterToRoot)
    lSkin.AddCluster(lClusterToLimbNode1)
    lSkin.AddCluster(lClusterToLimbNode2)
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

    # Now list the all the link involve in the patch deformation
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
                    
            # Add the patch to the pose
            lClusteredFbxNodes += [pMesh]
            
    # Now create a bind pose with the link list
    if len(lClusteredFbxNodes):
        # A pose must be named. Arbitrarily use the name of the patch node.
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

# Store a Rest Pose
def StoreRestPose(pSdkManager, pScene, pSkeletonRoot):
    # This example show an arbitrary rest pose assignment.
    # This rest pose will set the bone rotation to the same value 
    # as time 1 second in the first animation stack, but the 
    # position of the bone will be set elsewhere in the scene.

    # Create the rest pose
    lPose = KFbxPose.Create(pSdkManager,"A Rest Pose")

    # Set the skeleton root node to the global position (10, 10, 10)
    # and global rotation of 45deg along the Z axis.
    lT = KFbxVector4(10.0, 10.0, 10.0)
    lR = KFbxVector4(0.0,  0.0, 45.0)
    lS = KFbxVector4(1.0, 1.0, 1.0)

    lTransformMatrix = KFbxMatrix()
    lTransformMatrix.SetTRS(lT, lR, lS)

    # Add the skeleton root node to the pose
    lKFbxNode = pSkeletonRoot
    lPose.Add(lKFbxNode, lTransformMatrix, False) # It's a global matrix

    # Set the lLimbNode1 node to the local position of (0, 40, 0)
    # and local rotation of -90deg along the Z axis. This show that
    # you can mix local and global coordinates in a rest pose.
    lT.Set(0.0, 40.0,   0.0)
    lR.Set(0.0,  0.0, -90.0)
    lTransformMatrix.SetTRS(lT, lR, lS)

    # Add the skeleton second node to the pose
    lKFbxNode = lKFbxNode.GetChild(0)
    lPose.Add(lKFbxNode, lTransformMatrix, True) # It's a local matrix

    # Set the lLimbNode2 node to the local position of (0, 40, 0)
    # and local rotation of 45deg along the Z axis.
    lT.Set(0.0, 40.0, 0.0)
    lR.Set(0.0,  0.0, 45.0)
    lTransformMatrix.SetTRS(lT, lR, lS)

    # Add the skeleton second node to the pose
    lKFbxNode = lKFbxNode.GetChild(0)
    lPose.Add(lKFbxNode, lTransformMatrix, True) # It's a local matrix

    # Now add the pose to the scene
    pScene.AddPose(lPose)
    
# Create two animation stacks.
def AnimateSkeleton(pSdkManager, pScene, pSkeletonRoot):
    lKeyIndex = 0
    lTime = KTime()

    lRoot = pSkeletonRoot
    lLimbNode1 = pSkeletonRoot.GetChild(0)

    # First animation stack.
    lAnimStackName = "Bend on 2 sides"
    lAnimStack = KFbxAnimStack.Create(pScene, lAnimStackName)

    # The animation nodes can only exist on AnimLayers therefore it is mandatory to
    # add at least one AnimLayer to the AnimStack. And for the purpose of this example,
    # one layer is all we need.
    lAnimLayer = KFbxAnimLayer.Create(pScene, "Base Layer")
    lAnimStack.AddMember(lAnimLayer)

    # Create the AnimCurve on the Rotation.Z channel
    lCurve = lRoot.LclRotation.GetCurve(lAnimLayer, "Z", True)
    if lCurve:
        lCurve.KeyModifyBegin()
        lTime.SetSecondDouble(0.0)
        lKeyIndex = lCurve.KeyAdd(lTime)[0]
        lCurve.KeySetValue(lKeyIndex, 0.0)
        lCurve.KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef.eINTERPOLATION_CUBIC)

        lTime.SetSecondDouble(1.0)
        lKeyIndex = lCurve.KeyAdd(lTime)[0]
        lCurve.KeySetValue(lKeyIndex, 45.0)
        lCurve.KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef.eINTERPOLATION_CUBIC)

        lTime.SetSecondDouble(2.0)
        lKeyIndex = lCurve.KeyAdd(lTime)[0]
        lCurve.KeySetValue(lKeyIndex, -45.0)
        lCurve.KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef.eINTERPOLATION_CUBIC)

        lTime.SetSecondDouble(3.0)
        lKeyIndex = lCurve.KeyAdd(lTime)[0]
        lCurve.KeySetValue(lKeyIndex, 0.0)
        lCurve.KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef.eINTERPOLATION_CUBIC)
        lCurve.KeyModifyEnd()
    
    # Same thing for the next object
    lCurve = lLimbNode1.LclRotation.GetCurve(lAnimLayer, "Z", True)
    if lCurve:
        lCurve.KeyModifyBegin()
        lTime.SetSecondDouble(0.0)
        lKeyIndex = lCurve.KeyAdd(lTime)[0]
        lCurve.KeySetValue(lKeyIndex, 0.0)
        lCurve.KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef.eINTERPOLATION_CUBIC)

        lTime.SetSecondDouble(1.0)
        lKeyIndex = lCurve.KeyAdd(lTime)[0]
        lCurve.KeySetValue(lKeyIndex, -90.0)
        lCurve.KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef.eINTERPOLATION_CUBIC)

        lTime.SetSecondDouble(2.0)
        lKeyIndex = lCurve.KeyAdd(lTime)[0]
        lCurve.KeySetValue(lKeyIndex, 90.0)
        lCurve.KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef.eINTERPOLATION_CUBIC)

        lTime.SetSecondDouble(3.0)
        lKeyIndex = lCurve.KeyAdd(lTime)[0]
        lCurve.KeySetValue(lKeyIndex, 0.0)
        lCurve.KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef.eINTERPOLATION_CUBIC)
        lCurve.KeyModifyEnd()

    # Second animation stack.
    lAnimStackName = "Bend and turn around"
    lAnimStack = KFbxAnimStack.Create(pScene, lAnimStackName)

    # The animation nodes can only exist on AnimLayers therefore it is mandatory to
    # add at least one AnimLayer to the AnimStack. And for the purpose of this example,
    # one layer is all we need.
    lAnimLayer = KFbxAnimLayer.Create(pScene, "Base Layer")
    lAnimStack.AddMember(lAnimLayer)

    # Create the AnimCurve on the Rotation.Y channel
    lCurve = lRoot.LclRotation.GetCurve(lAnimLayer, "Y", True)
    if lCurve:
        lCurve.KeyModifyBegin()
        lTime.SetSecondDouble(0.0)
        lKeyIndex = lCurve.KeyAdd(lTime)[0]
        lCurve.KeySetValue(lKeyIndex, 0.0)
        lCurve.KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef.eINTERPOLATION_CUBIC)

        lTime.SetSecondDouble(2.0)
        lKeyIndex = lCurve.KeyAdd(lTime)[0]
        lCurve.KeySetValue(lKeyIndex, 720.0)
        lCurve.KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef.eINTERPOLATION_CUBIC)
        lCurve.KeyModifyEnd()

    lCurve = lLimbNode1.LclRotation.GetCurve(lAnimLayer, "Z", True)
    if lCurve:
        lCurve.KeyModifyBegin()
        lTime.SetSecondDouble(0.0)
        lKeyIndex = lCurve.KeyAdd(lTime)[0]
        lCurve.KeySetValue(lKeyIndex, 0.0)
        lCurve.KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef.eINTERPOLATION_CUBIC)

        lTime.SetSecondDouble(1.0)
        lKeyIndex = lCurve.KeyAdd(lTime)[0]
        lCurve.KeySetValue(lKeyIndex, 90.0)
        lCurve.KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef.eINTERPOLATION_CUBIC)

        lTime.SetSecondDouble(2.0)
        lKeyIndex = lCurve.KeyAdd(lTime)[0]
        lCurve.KeySetValue(lKeyIndex, 0.0)
        lCurve.KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef.eINTERPOLATION_CUBIC)
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
    lSampleFileName = ""
    if len(sys.argv) > 1:
        lSampleFileName = sys.argv[1]
    # A default output file name is given otherwise.
    else:
        lSampleFileName = OUTPUT_NAME

    # Create the scene.
    lResult = CreateScene(lSdkManager, lScene, lSampleFileName)

    if lResult == False:
        print("\n\nAn error occurred while creating the scene...\n")
        lSdkManager.Destroy()
        sys.exit(1)

    # Save the scene.
    # 1 = text version 2012
    # 3 = (working?) binary
    lResult = FbxCommon.SaveScene(lSdkManager, lScene, lSampleFileName, 3)

    if lResult == False:
        print("\n\nAn error occurred while saving the scene...\n")
        lSdkManager.Destroy()
        sys.exit(1)

    # Destroy all objects created by the FBX SDK.
    lSdkManager.Destroy()
   
    sys.exit(0)
