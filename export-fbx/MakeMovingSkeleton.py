import sys,yaml

OUTPUT_NAME = "david-skeleton.fbx"

gExportVertexCacheMCFormat = True
gCacheType = 0

def CreateScene(pSdkManager, pScene, pSampleFileName):
    
    lSceneInfo = KFbxDocumentInfo.Create(pSdkManager, "SceneInfo")
    lSceneInfo.mTitle = "David skeleton"
    lSceneInfo.mSubject = "Can we import skeleton data?"
    lSceneInfo.mAuthor = "Team KinectOpenNiRosBoostGraphLibDaiPclEigen"
    lSceneInfo.mRevision = "rev. 0.1"
    lSceneInfo.mKeywords = "skeleton openni range scan nite"
    lSceneInfo.mComment = "no particular comments required."
    pScene.SetSceneInfo(lSceneInfo)

    # make all skeletons
    skelFile = yaml.load(open('../recordings/david2/david2_skeleton.yaml','r'))
    skels = [(float(ts),Skeleton(skData)) for ts,skData in sorted(skelFile.iteritems())]
    
    baseSkeleton = skels[0][1]
    CreateSkeletonNodes(pSdkManager, "Skeleton", baseSkeleton)
    
    AnimateSkeleton(pSdkManager, pScene, baseSkeleton, skels[1:])

    pScene.GetRootNode().AddChild(baseSkeleton.torsoNode)
        
    return True

    
# Create a skeleton with 2 segments.
def CreateSkeletonNodes(pSdkManager, pName, skeleton):
    
    # Create skeleton root
    lRootName = pName + "Torso"
    lTorsoAttribute = KFbxSkeleton.Create(lSdkManager, lRootName)
    lTorsoAttribute.SetSkeletonType(KFbxSkeleton.eROOT)
    torso = KFbxNode.Create(lSdkManager, lRootName)
    torso.SetNodeAttribute(lTorsoAttribute)    
    torso.LclTranslation.Set(fbxDouble3(*skeleton.torso))
     
    def MakeLimb(name, skeleton):
        pos = getattr(skeleton, name)
        lLimbNodeName = pName + name
        lSkeletonLimbNodeAttribute = KFbxSkeleton.Create(lSdkManager, lLimbNodeName)
        lSkeletonLimbNodeAttribute.SetSkeletonType(KFbxSkeleton.eLIMB_NODE)
        lSkeletonLimbNodeAttribute.Size.Set(100.0)
        lSkeletonLimbNode = KFbxNode.Create(lSdkManager, lLimbNodeName)
        lSkeletonLimbNode.SetNodeAttribute(lSkeletonLimbNodeAttribute)    
        lSkeletonLimbNode.LclTranslation.Set(fbxDouble3(*pos)) # oh, Lcl = Local, maybe I could have avoided relativizing
        setattr(skeleton, name+'Node', lSkeletonLimbNode)
        return lSkeletonLimbNode
    
    # Build skeleton node connections.
    skeleton.torsoNode = torso
    
    neck = MakeLimb("neck", skeleton)
    head = MakeLimb("head", skeleton) 
    
    leftArm = [
        MakeLimb("leftShoulder", skeleton),
        MakeLimb("leftElbow", skeleton),
        MakeLimb("leftHand", skeleton)
    ]
    rightArm = [
        MakeLimb("rightShoulder", skeleton),
        MakeLimb("rightElbow", skeleton),
        MakeLimb("rightHand", skeleton)
    ]
    leftLeg = [
        MakeLimb("leftHip", skeleton),
        MakeLimb("leftKnee", skeleton),
        MakeLimb("leftFoot", skeleton)
    ]
    rightLeg = [
        MakeLimb("rightHip", skeleton),
        MakeLimb("rightKnee", skeleton),
        MakeLimb("rightFoot", skeleton)
    ]
    
    AddChain(torso, neck, head)
    AddChain(neck,*leftArm)
    AddChain(neck,*rightArm)    
    AddChain(torso,*leftLeg)
    AddChain(torso,*rightLeg)


class Skeleton(object):
    # parse and convert to relative coordinates
    def __init__(self, sk):
        self.sk = sk
        self.pos = {} # cache
        
        self.torso = self.GetPos('torso')
        self.head = self.GetPosRel('head', 'neck')
        self.neck = self.GetPosRel('neck', 'torso')

        self.leftShoulder = self.GetPosRel('left_shoulder', 'neck')
        self.rightShoulder = self.GetPosRel('right_shoulder', 'neck')

        self.leftElbow = self.GetPosRel('left_elbow', 'left_shoulder')
        self.rightElbow = self.GetPosRel('right_elbow', 'right_shoulder')

        self.leftHand = self.GetPosRel('left_hand', 'left_elbow')
        self.rightHand = self.GetPosRel('right_hand', 'right_elbow')

        self.leftHip = self.GetPosRel('left_hip', 'torso')
        self.rightHip = self.GetPosRel('right_hip', 'torso')

        self.leftKnee = self.GetPosRel('left_knee', 'left_hip')
        self.rightKnee = self.GetPosRel('right_knee', 'right_hip')
        self.leftFoot = self.GetPosRel('left_foot', 'left_knee')
        self.rightFoot = self.GetPosRel('right_foot', 'right_knee')

    def GetPos(self, jointName):
        pos = self.pos.get(jointName, None)
        if pos is not None: # cache the parse/transform
            return pos
        else:
            # maybe the _1 indicates that it tracks several people?
            joint = self.sk.get('/'+jointName+'_1',None)
            if joint is None:
                return None
            else:
                tr = joint['translation']
                return ( # shuffle so that the skelton is upright, remember to do this transform on the mesh too
                    float(tr[1].replace('y -- ', ''))*50,
                    float(tr[2].replace('z -- ', ''))*50,
                    float(tr[0].replace('x -- ', ''))*50
                )

    def GetPosRel(self, jointName, parentName):
        j = self.GetPos(jointName)
        p = self.GetPos(parentName)
        if j is None or p is None:
            return None
        else:
            return (j[0]-p[0], j[1]-p[1], j[2]-p[2])




def AddChain(*nodes):
    for i in range(len(nodes)-1):
        nodes[i].AddChild(nodes[i+1])

def constrain(val, lo, hi):
    if val < lo: return lo
    if val > hi: return hi
    return val

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
    jointNames = "torso head neck leftShoulder rightShoulder leftElbow rightElbow leftHand rightHand leftHip rightHip leftKnee rightKnee leftFoot rightFoot".split()
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
