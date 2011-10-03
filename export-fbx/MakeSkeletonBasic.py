import sys,yaml

OUTPUT_NAME = "basic-skeleton.fbx"

gExportVertexCacheMCFormat = True
gCacheType = 0

def CreateScene(pSdkManager, pScene, pSampleFileName):
        
    lSceneInfo = KFbxDocumentInfo.Create(pSdkManager, "SceneInfo")
    lSceneInfo.mTitle = "Basic skeleton"
    lSceneInfo.mSubject = "Can we build a skeleton?"
    lSceneInfo.mAuthor = "Team KinectOpenNiRosBoostGraphLibDaiPclEigen"
    lSceneInfo.mRevision = "rev. 0.1"
    lSceneInfo.mKeywords = "skeleton"
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

    def MakeLimb(name, x, y, z):
        lLimbNodeName = pName + name
        lSkeletonLimbNodeAttribute = KFbxSkeleton.Create(lSdkManager, lLimbNodeName)
        lSkeletonLimbNodeAttribute.SetSkeletonType(KFbxSkeleton.eLIMB_NODE)
        lSkeletonLimbNodeAttribute.Size.Set(100.0)
        lSkeletonLimbNode = KFbxNode.Create(lSdkManager, lLimbNodeName)
        lSkeletonLimbNode.SetNodeAttribute(lSkeletonLimbNodeAttribute)    
        lSkeletonLimbNode.LclTranslation.Set(fbxDouble3(x,y,z))
        return lSkeletonLimbNode
    
    # these are in relative coordinates
    head = MakeLimb("Head", 0, 5, 0)
    neck = MakeLimb("Neck", 0, 10, 0)
    
    leftShoulder = MakeLimb("LeftShoulder", -5, 0, 0)
    rightShoulder = MakeLimb("RightShoulder", 5, 0, 0)

    leftElbow = MakeLimb("LeftElbow", -5, -3, 0)
    rightElbow = MakeLimb("RightElbow", 5, -3, 0)

    leftHand = MakeLimb("LeftHand", -5, -2, 0)
    rightHand = MakeLimb("RightHand", 5, -2, 0)

    leftHip = MakeLimb("LeftHip", -5, -10, 0)
    rightHip = MakeLimb("RightHip", 5, -10, 0)

    leftKnee = MakeLimb("LeftKnee", 0, -7, 0)
    rightKnee = MakeLimb("RightKnee", 0, -7, 0)

    leftFoot = MakeLimb("LeftFoot", 0, -7, 0)
    rightFoot = MakeLimb("RightFoot",0, -7, 0)
        
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
