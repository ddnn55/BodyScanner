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

    lSkeletonRoot = CreateSkeleton(pSdkManager, "Skeleton")

    pScene.GetRootNode().AddChild(lSkeletonRoot)
        
    return True

    
# Create a skeleton with 2 segments.
def CreateSkeleton(pSdkManager, pName):
    # Create skeleton root
    lRootName = pName + "Torso"
    lSkeletonRootAttribute = KFbxSkeleton.Create(lSdkManager, lRootName)
    lSkeletonRootAttribute.SetSkeletonType(KFbxSkeleton.eROOT)
    lSkeletonRoot = KFbxNode.Create(lSdkManager, lRootName)
    lSkeletonRoot.SetNodeAttribute(lSkeletonRootAttribute)    
    lSkeletonRoot.LclTranslation.Set(fbxDouble3(0.0, 00.0, 0.0))

    def MakeLimb(name, pos):
        lLimbNodeName = pName + name
        lSkeletonLimbNodeAttribute = KFbxSkeleton.Create(lSdkManager, lLimbNodeName)
        lSkeletonLimbNodeAttribute.SetSkeletonType(KFbxSkeleton.eLIMB_NODE)
        lSkeletonLimbNodeAttribute.Size.Set(100.0)
        lSkeletonLimbNode = KFbxNode.Create(lSdkManager, lLimbNodeName)
        lSkeletonLimbNode.SetNodeAttribute(lSkeletonLimbNodeAttribute)    
        lSkeletonLimbNode.LclTranslation.Set(fbxDouble3(*pos))
        return lSkeletonLimbNode
    
    sk = {'/left_hand_1': {'translation': ['x -- 2.61650214587', 'y -- 0.42341399938', 'z -- 0.741739473523']}, '/right_hand_1': {'translation': ['x -- 2.57094874609', 'y -- -0.230743653914', 'z -- 0.704013261464']}, '/left_knee_1': {'translation': ['x -- 2.61901994098', 'y -- 0.247822165975', 'z -- -0.438225348963']}, '/left_shoulder_1': {'translation': ['x -- 2.62625012634', 'y -- 0.262870272978', 'z -- 0.363720638888']}, '/left_hip_1': {'translation': ['x -- 2.63965052228', 'y -- 0.192339774104', 'z -- -0.0376897763944']}, '/right_elbow_1': {'translation': ['x -- 2.67929422096', 'y -- -0.275082718219', 'z -- 0.464603531212']}, '/head_1': {'translation': ['x -- 2.62009243892', 'y -- 0.133765620842', 'z -- 0.574407307017']}, '/right_foot_1': {'translation': ['x -- 2.62011127176', 'y -- -0.0365560763252', 'z -- -0.783027446477']}, '/left_foot_1': {'translation': ['x -- 2.61006204065', 'y -- 0.279194157608', 'z -- -0.776382878318']}, '/torso_1': {'translation': ['x -- 2.6342453413', 'y -- 0.109085436012', 'z -- 0.170089673852']}, '/left_elbow_1': {'translation': ['x -- 2.6694900322', 'y -- 0.47918342009', 'z -- 0.490597951981']}, '/right_shoulder_1': {'translation': ['x -- 2.62971094745', 'y -- -0.0210120804922', 'z -- 0.380479201619']}, '/neck_1': {'translation': ['x -- 2.62798041482', 'y -- 0.120929086706', 'z -- 0.372099904995']}, '/right_knee_1': {'translation': ['x -- 2.61471274281', 'y -- 0.00723112289278', 'z -- -0.43181264393']}, '/right_hip_1': {'translation': ['x -- 2.64326965169', 'y -- 0.00574492112083', 'z -- -0.0279414280981']}}
    
    def GetPos(jointName): # TODO: cache this to avoid reparsing
        # maybe the _1 indicates that it tracks several people?
        tr = sk['/'+jointName+'_1']['translation']
        return ( # shuffle so that the skelton is upright, remember to do this transform on the mesh too
            float(tr[1].replace('y -- ', ''))*50,
            float(tr[2].replace('z -- ', ''))*50,
            float(tr[0].replace('x -- ', ''))*50
        )
    
    def GetPosRel(jointName, parentName):
        x,y,z = GetPos(jointName)
        px,py,pz = GetPos(parentName)
        return (x-px, y-py, z-pz)
     
    # these are in relative coordinates
    head = MakeLimb("Head", GetPosRel('head', 'neck'))
    neck = MakeLimb("Neck", GetPosRel('neck', 'torso'))
    
    leftShoulder = MakeLimb("LeftShoulder", GetPosRel('left_shoulder', 'neck'))
    rightShoulder = MakeLimb("RightShoulder", GetPosRel('right_shoulder', 'neck'))

    leftElbow = MakeLimb("LeftElbow", GetPosRel('left_elbow', 'left_shoulder'))
    rightElbow = MakeLimb("RightElbow", GetPosRel('right_elbow', 'right_shoulder'))

    leftHand = MakeLimb("LeftHand", GetPosRel('left_hand', 'left_elbow'))
    rightHand = MakeLimb("RightHand", GetPosRel('right_hand', 'right_elbow'))

    leftHip = MakeLimb("LeftHip", GetPosRel('left_hip', 'torso'))
    rightHip = MakeLimb("RightHip", GetPosRel('right_hip', 'torso'))

    leftKnee = MakeLimb("LeftKnee", GetPosRel('left_knee', 'left_hip'))
    rightKnee = MakeLimb("RightKnee", GetPosRel('right_knee', 'right_hip'))

    leftFoot = MakeLimb("LeftFoot", GetPosRel('left_foot', 'left_knee'))
    rightFoot = MakeLimb("RightFoot", GetPosRel('right_foot', 'right_knee'))
        
    # Build skeleton node connections.
    AddChain(lSkeletonRoot, neck, head)
    AddChain(neck, leftShoulder, leftElbow, leftHand)
    AddChain(neck, rightShoulder, rightElbow, rightHand)
    
    AddChain(lSkeletonRoot, leftHip, leftKnee, leftFoot)
    AddChain(lSkeletonRoot, rightHip, rightKnee, rightFoot)
    
    return lSkeletonRoot

def AddChain(*nodes):
    for i in range(len(nodes)-1):
        nodes[i].AddChild(nodes[i+1])

def constrain(val, lo, hi):
    if val < lo: return lo
    if val > hi: return hi
    return val

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
