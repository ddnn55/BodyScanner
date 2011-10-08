/*




Having trouble getting this to build correctly.
Instead of converting the export to C++,
have the segmentation output a text file containing weights.







*/




#include <fbxsdk.h>

#include "../Common/Common.h"

#define SAMPLE_FILENAME "ExportScene01.fbx"


// Function prototypes.
bool CreateScene(KFbxSdkManager* pSdkManager, KFbxScene* pScene);

KFbxNode* CreateMesh(KFbxScene* pScene, char* pName);

/*
KFbxNode* CreateSkeleton(KFbxScene* pScene, char* pName);

void LinkPatchToSkeleton(KFbxScene* pScene, KFbxNode* pPatch, KFbxNode* pSkeletonRoot);
void StoreBindPose(KFbxScene* pScene, KFbxNode* pPatch, KFbxNode* pSkeletonRoot);
void StoreRestPose(KFbxScene* pScene, KFbxNode* pSkeletonRoot);
void AnimateSkeleton(KFbxScene* pScene, KFbxNode* pSkeletonRoot);
void AddNodeRecursively(KArrayTemplate<KFbxNode*>& pNodeArray, KFbxNode* pNode);

void SetXMatrix(KFbxXMatrix& pXMatrix, const KFbxMatrix& pMatrix);
*/

int main(int argc, char** argv)
{
    KFbxSdkManager* lSdkManager = NULL;
    KFbxScene* lScene = NULL;
    bool lResult;

    // Prepare the FBX SDK.
    InitializeSdkObjects(lSdkManager, lScene);

    // Create the scene.
    lResult = CreateScene(lSdkManager, lScene);

    if(lResult == false)
    {
        printf("\n\nAn error occurred while creating the scene...\n");
        DestroySdkObjects(lSdkManager);
        return 0;
    }

    // Save the scene.

    // The example can take an output file name as an argument.
    if(argc > 1)
    {
        lResult = SaveScene(lSdkManager, lScene, argv[1]);
    }
    // A default output file name is given otherwise.
    else
    {
        lResult = SaveScene(lSdkManager, lScene, SAMPLE_FILENAME);
    }

    if(lResult == false)
    {
        printf("\n\nAn error occurred while saving the scene...\n");
        DestroySdkObjects(lSdkManager);
        return 0;
    }

    // Destroy all objects created by the FBX SDK.
    DestroySdkObjects(lSdkManager);

    return 0;
}

bool CreateScene(KFbxSdkManager *pSdkManager, KFbxScene* pScene)
{
    // create scene info
    KFbxDocumentInfo* sceneInfo = KFbxDocumentInfo::Create(pSdkManager,"SceneInfo");
    sceneInfo->mTitle = "Mesh";
    sceneInfo->mSubject = "Mesh thing";
    sceneInfo->mAuthor = "Team opennirospcleigenubuntuvmware";
    sceneInfo->mRevision = "0.1";
    sceneInfo->mKeywords = "mesh";
    sceneInfo->mComment = "none";

    // we need to add the sceneInfo before calling AddThumbNailToScene because
    // that function is asking the scene for the sceneInfo.
    pScene->SetSceneInfo(sceneInfo);

    KFbxNode* lPatch = CreateMesh(pScene, "Patch");
    KFbxNode* lSkeletonRoot = CreateSkeleton(pScene, "Skeleton");


    // Build the node tree.
    KFbxNode* lRootNode = pScene->GetRootNode();
    lRootNode->AddChild(lPatch);
    lRootNode->AddChild(lSkeletonRoot);

/*
	// Store poses
    LinkPatchToSkeleton(pScene, lPatch, lSkeletonRoot);
    StoreBindPose(pScene, lPatch, lSkeletonRoot);
    StoreRestPose(pScene, lSkeletonRoot);

	// Animation
    AnimateSkeleton(pScene, lSkeletonRoot);
*/
    return true;
}

struct Tri {int i,j,k;};
struct Ver {float x,y,z;};

// Create a cylinder centered on the Z axis. 
KFbxNode* CreateMesh(KFbxScene* pScene, char* pName)
{
    float scale = 1000;
    FILE *f = fopen("../dragon16k.obj", "r");
    std::vector<Tri> T;
    std::vector<Ver> V;
    
    char line[255];
    char *rest;
    while(fgets(line, 255, f)) {
        if(line[0] == 'f') {
            Tri t;
            rest = &line[1];
            while(rest[0] == ' ') rest++; // eat space
            t.i = atoi(rest);
            while(rest[0] != ' ') rest++; // go past
            while(rest[0] == ' ') rest++; // eat space
            t.j = atoi(rest);
            while(rest[0] != ' ') rest++; // go past
            while(rest[0] == ' ') rest++; // eat space
            t.k = atoi(rest);
            T.push_back(t);
        } else if(line[0] == 'v') {
            Ver v;
            rest = &line[1];
            while(rest[0] == ' ') rest++; // eat space
            v.x = atof(rest)*scale;
            while(rest[0] != ' ') rest++; // go past
            while(rest[0] == ' ') rest++; // eat space
            v.y = atof(rest)*scale;
            while(rest[0] != ' ') rest++; // go past
            while(rest[0] == ' ') rest++; // eat space
            v.z = atof(rest)*scale;
            V.push_back(v);
        }
    }
    
    KFbxMesh* lMesh = KFbxMesh::Create(pScene,pName);
    lMesh->InitControlPoints(V.size());
    KFbxVector4* points = lMesh->GetControlPoints();
    for(int i = 0; i < V.size(); i++) {
        Ver &v = V[i];
        points[i].Set(v.x,v.y,v.z);
    }
    
    for ijk in T:
    for(int t = 0; t < T.size(); t++) {
        lMesh->BeginPolygon();
        lMesh->AddPolygon(T[t].i);
        lMesh->AddPolygon(T[t].j);
        lMesh->AddPolygon(T[t].k);
        lMesh->EndPolygon();
    }
    
    # wrap the mesh in a node, ok, sure, why not FBX, if you insist
    KFbxNode *lNode = KFbxNode::Create(pSdkManager,pName);
    lNode->SetNodeAttribute(lMesh);
    return lNode,T,V;
}

/*
// Create a skeleton with 2 segments.
KFbxNode* CreateSkeleton(KFbxScene* pScene, char* pName)
{
    // Create skeleton root. 
    KString lRootName(pName);
    lRootName += "Root";
    KFbxSkeleton* lSkeletonRootAttribute = KFbxSkeleton::Create(pScene, pName);
    lSkeletonRootAttribute->SetSkeletonType(KFbxSkeleton::eROOT);
    KFbxNode* lSkeletonRoot = KFbxNode::Create(pScene,lRootName.Buffer());
    lSkeletonRoot->SetNodeAttribute(lSkeletonRootAttribute);    
    lSkeletonRoot->LclTranslation.Set(KFbxVector4(0.0, -40.0, 0.0));

    // Create skeleton first limb node. 
    KString lLimbNodeName1(pName);
    lLimbNodeName1 += "LimbNode1";
    KFbxSkeleton* lSkeletonLimbNodeAttribute1 = KFbxSkeleton::Create(pScene,lLimbNodeName1);
    lSkeletonLimbNodeAttribute1->SetSkeletonType(KFbxSkeleton::eLIMB_NODE);
    lSkeletonLimbNodeAttribute1->Size.Set(1.0);
    KFbxNode* lSkeletonLimbNode1 = KFbxNode::Create(pScene,lLimbNodeName1.Buffer());
    lSkeletonLimbNode1->SetNodeAttribute(lSkeletonLimbNodeAttribute1);    
    lSkeletonLimbNode1->LclTranslation.Set(KFbxVector4(0.0, 40.0, 0.0));

    // Create skeleton second limb node. 
    KString lLimbNodeName2(pName);
    lLimbNodeName2 += "LimbNode2";
    KFbxSkeleton* lSkeletonLimbNodeAttribute2 = KFbxSkeleton::Create(pScene,lLimbNodeName2);
    lSkeletonLimbNodeAttribute2->SetSkeletonType(KFbxSkeleton::eLIMB_NODE);
    lSkeletonLimbNodeAttribute2->Size.Set(1.0);
    KFbxNode* lSkeletonLimbNode2 = KFbxNode::Create(pScene,lLimbNodeName2.Buffer());
    lSkeletonLimbNode2->SetNodeAttribute(lSkeletonLimbNodeAttribute2);    
    lSkeletonLimbNode2->LclTranslation.Set(KFbxVector4(0.0, 40.0, 0.0));

    // Build skeleton node hierarchy. 
    lSkeletonRoot->AddChild(lSkeletonLimbNode1);
    lSkeletonLimbNode1->AddChild(lSkeletonLimbNode2);

    return lSkeletonRoot;
}

// Set the influence of the skeleton segments over the cylinder.
// The link mode is KFbxLink::eTOTAL1 which means the total
// of the weights assigned to a given control point must equal 1.
void LinkPatchToSkeleton(KFbxScene* pScene, KFbxNode* pPatch, KFbxNode* pSkeletonRoot)
{
    int i, j;
    KFbxXMatrix lXMatrix;

    KFbxNode* lRoot = pSkeletonRoot;
    KFbxNode* lLimbNode1 = pSkeletonRoot->GetChild(0);
    KFbxNode* lLimbNode2 = lLimbNode1->GetChild(0);

    // Bottom section of cylinder is clustered to skeleton root.
    KFbxCluster *lClusterToRoot = KFbxCluster::Create(pScene,"");
    lClusterToRoot->SetLink(lRoot);
    lClusterToRoot->SetLinkMode(KFbxCluster::eTOTAL1);
    for(i=0; i<4; ++i)
        for(j=0; j<4; ++j)
            lClusterToRoot->AddControlPointIndex(4*i + j, 1.0 - 0.25*i);

    // Center section of cylinder is clustered to skeleton limb node.
    KFbxCluster* lClusterToLimbNode1 = KFbxCluster::Create(pScene, "");
    lClusterToLimbNode1->SetLink(lLimbNode1);
    lClusterToLimbNode1->SetLinkMode(KFbxCluster::eTOTAL1);

    for (i =1; i<6; ++i)
        for (j=0; j<4; ++j)
            lClusterToLimbNode1->AddControlPointIndex(4*i + j, (i == 1 || i == 5 ? 0.25 : 0.50));


    // Top section of cylinder is clustered to skeleton limb.

    KFbxCluster * lClusterToLimbNode2 = KFbxCluster::Create(pScene,"");
    lClusterToLimbNode2->SetLink(lLimbNode2);
    lClusterToLimbNode2->SetLinkMode(KFbxCluster::eTOTAL1);

    for (i=3; i<7; ++i)
        for (j=0; j<4; ++j)
            lClusterToLimbNode2->AddControlPointIndex(4*i + j, 0.25*(i - 2));

    // Now we have the Patch and the skeleton correctly positioned,
    // set the Transform and TransformLink matrix accordingly.
	KFbxScene* lScene = pPatch->GetScene();
    if( lScene ) lXMatrix = pPatch->EvaluateGlobalTransform();

    lClusterToRoot->SetTransformMatrix(lXMatrix);
    lClusterToLimbNode1->SetTransformMatrix(lXMatrix);
    lClusterToLimbNode2->SetTransformMatrix(lXMatrix);



    if( lScene ) lXMatrix = lRoot->EvaluateGlobalTransform();
    lClusterToRoot->SetTransformLinkMatrix(lXMatrix);


    if( lScene ) lXMatrix = lLimbNode1->EvaluateGlobalTransform();
    lClusterToLimbNode1->SetTransformLinkMatrix(lXMatrix);


    if( lScene ) lXMatrix = lLimbNode2->EvaluateGlobalTransform();
    lClusterToLimbNode2->SetTransformLinkMatrix(lXMatrix);


    // Add the clusters to the patch by creating a skin and adding those clusters to that skin.
    // After add that skin.

    KFbxGeometry* lPatchAttribute = (KFbxGeometry*) pPatch->GetNodeAttribute();
    KFbxSkin* lSkin = KFbxSkin::Create(pScene, "");
    lSkin->AddCluster(lClusterToRoot);
    lSkin->AddCluster(lClusterToLimbNode1);
    lSkin->AddCluster(lClusterToLimbNode2);
    lPatchAttribute->AddDeformer(lSkin);

}

// Create two animation stacks.
void AnimateSkeleton(KFbxScene* pScene, KFbxNode* pSkeletonRoot)
{
    KString lAnimStackName;
    KTime lTime;
    int lKeyIndex = 0;

    KFbxNode* lRoot = pSkeletonRoot;
    KFbxNode* lLimbNode1 = pSkeletonRoot->GetChild(0);

    // First animation stack.
    lAnimStackName = "Bend on 2 sides";
    KFbxAnimStack* lAnimStack = KFbxAnimStack::Create(pScene, lAnimStackName);

        // The animation nodes can only exist on AnimLayers therefore it is mandatory to
        // add at least one AnimLayer to the AnimStack. And for the purpose of this example,
        // one layer is all we need.
        KFbxAnimLayer* lAnimLayer = KFbxAnimLayer::Create(pScene, "Base Layer");
        lAnimStack->AddMember(lAnimLayer);

    // Create the AnimCurve on the Rotation.Z channel
    KFbxAnimCurve* lCurve = lRoot->LclRotation.GetCurve<KFbxAnimCurve>(lAnimLayer, KFCURVENODE_R_Z, true);
    if (lCurve)
    {
        lCurve->KeyModifyBegin();
        lTime.SetSecondDouble(0.0);
        lKeyIndex = lCurve->KeyAdd(lTime);
        lCurve->KeySetValue(lKeyIndex, 0.0);
        lCurve->KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef::eINTERPOLATION_CUBIC);

        lTime.SetSecondDouble(1.0);
        lKeyIndex = lCurve->KeyAdd(lTime);
        lCurve->KeySetValue(lKeyIndex, 45.0);
        lCurve->KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef::eINTERPOLATION_CUBIC);

        lTime.SetSecondDouble(2.0);
        lKeyIndex = lCurve->KeyAdd(lTime);
        lCurve->KeySetValue(lKeyIndex, -45.0);
        lCurve->KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef::eINTERPOLATION_CUBIC);

        lTime.SetSecondDouble(3.0);
        lKeyIndex = lCurve->KeyAdd(lTime);
        lCurve->KeySetValue(lKeyIndex, 0.0);
        lCurve->KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef::eINTERPOLATION_CUBIC);
        lCurve->KeyModifyEnd();
    }

    // Same thing for the next object
    lCurve = lLimbNode1->LclRotation.GetCurve<KFbxAnimCurve>(lAnimLayer, KFCURVENODE_R_Z, true);
    if (lCurve)
    {
        lCurve->KeyModifyBegin();
        lTime.SetSecondDouble(0.0);
        lKeyIndex = lCurve->KeyAdd(lTime);
        lCurve->KeySetValue(lKeyIndex, 0.0);
        lCurve->KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef::eINTERPOLATION_CUBIC);

        lTime.SetSecondDouble(1.0);
        lKeyIndex = lCurve->KeyAdd(lTime);
        lCurve->KeySetValue(lKeyIndex, -90.0);
        lCurve->KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef::eINTERPOLATION_CUBIC);

        lTime.SetSecondDouble(2.0);
        lKeyIndex = lCurve->KeyAdd(lTime);
        lCurve->KeySetValue(lKeyIndex, 90.0);
        lCurve->KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef::eINTERPOLATION_CUBIC);

        lTime.SetSecondDouble(3.0);
        lKeyIndex = lCurve->KeyAdd(lTime);
        lCurve->KeySetValue(lKeyIndex, 0.0);
        lCurve->KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef::eINTERPOLATION_CUBIC);
        lCurve->KeyModifyEnd();
    }

    // Second animation stack.
    lAnimStackName = "Bend and turn around";
    lAnimStack = KFbxAnimStack::Create(pScene, lAnimStackName);

        // The animation nodes can only exist on AnimLayers therefore it is mandatory to
        // add at least one AnimLayer to the AnimStack. And for the purpose of this example,
        // one layer is all we need.
        lAnimLayer = KFbxAnimLayer::Create(pScene, "Base Layer");
        lAnimStack->AddMember(lAnimLayer);

    // Create the AnimCurve on the Rotation.Y channel
    lCurve = lRoot->LclRotation.GetCurve<KFbxAnimCurve>(lAnimLayer, KFCURVENODE_R_Y, true);
    if (lCurve)
    {
        lCurve->KeyModifyBegin();
        lTime.SetSecondDouble(0.0);
        lKeyIndex = lCurve->KeyAdd(lTime);
        lCurve->KeySetValue(lKeyIndex, 0.0);
        lCurve->KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef::eINTERPOLATION_CUBIC);

        lTime.SetSecondDouble(2.0);
        lKeyIndex = lCurve->KeyAdd(lTime);
        lCurve->KeySetValue(lKeyIndex, 720.0);
        lCurve->KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef::eINTERPOLATION_CUBIC);
        lCurve->KeyModifyEnd();
    }

    lCurve = lLimbNode1->LclRotation.GetCurve<KFbxAnimCurve>(lAnimLayer, KFCURVENODE_R_Z, true);
    if (lCurve)
    {
        lCurve->KeyModifyBegin();
        lTime.SetSecondDouble(0.0);
        lKeyIndex = lCurve->KeyAdd(lTime);
        lCurve->KeySetValue(lKeyIndex, 0.0);
        lCurve->KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef::eINTERPOLATION_CUBIC);

        lTime.SetSecondDouble(1.0);
        lKeyIndex = lCurve->KeyAdd(lTime);
        lCurve->KeySetValue(lKeyIndex, 90.0);
        lCurve->KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef::eINTERPOLATION_CUBIC);

        lTime.SetSecondDouble(2.0);
        lKeyIndex = lCurve->KeyAdd(lTime);
        lCurve->KeySetValue(lKeyIndex, 0.0);
        lCurve->KeySetInterpolation(lKeyIndex, KFbxAnimCurveDef::eINTERPOLATION_CUBIC);
        lCurve->KeyModifyEnd();
    }
}

// Store the Bind Pose
void StoreBindPose(KFbxScene* pScene, KFbxNode* pPatch, KFbxNode* pSkeletonRoot)
{
    // In the bind pose, we must store all the link's global matrix at the time of the bind.
    // Plus, we must store all the parent(s) global matrix of a link, even if they are not
    // themselves deforming any model.

    // In this example, since there is only one model deformed, we don't need walk through 
    // the scene
    //

    // Now list the all the link involve in the patch deformation
    KArrayTemplate<KFbxNode*> lClusteredFbxNodes;
    int                       i, j;

    if (pPatch && pPatch->GetNodeAttribute())
    {
        int lSkinCount=0;
        int lClusterCount=0;
        switch (pPatch->GetNodeAttribute()->GetAttributeType())
        {
        case KFbxNodeAttribute::eMESH:
        case KFbxNodeAttribute::eNURB:
        case KFbxNodeAttribute::ePATCH:

            lSkinCount = ((KFbxGeometry*)pPatch->GetNodeAttribute())->GetDeformerCount(KFbxDeformer::eSKIN);
            //Go through all the skins and count them
            //then go through each skin and get their cluster count
            for(i=0; i<lSkinCount; ++i)
            {
                KFbxSkin *lSkin=(KFbxSkin*)((KFbxGeometry*)pPatch->GetNodeAttribute())->GetDeformer(i, KFbxDeformer::eSKIN);
                lClusterCount+=lSkin->GetClusterCount();
            }
            break;
        }
        //if we found some clusters we must add the node
        if (lClusterCount)
        {
            //Again, go through all the skins get each cluster link and add them
            for (i=0; i<lSkinCount; ++i)
            {
                KFbxSkin *lSkin=(KFbxSkin*)((KFbxGeometry*)pPatch->GetNodeAttribute())->GetDeformer(i, KFbxDeformer::eSKIN);
                lClusterCount=lSkin->GetClusterCount();
                for (j=0; j<lClusterCount; ++j)
                {
                    KFbxNode* lClusterNode = lSkin->GetCluster(j)->GetLink();
                    AddNodeRecursively(lClusteredFbxNodes, lClusterNode);
                }

            }

            // Add the patch to the pose
            lClusteredFbxNodes.Add(pPatch);
        }
    }

    // Now create a bind pose with the link list
    if (lClusteredFbxNodes.GetCount())
    {
        // A pose must be named. Arbitrarily use the name of the patch node.
        KFbxPose* lPose = KFbxPose::Create(pScene,pPatch->GetName());

        // default pose type is rest pose, so we need to set the type as bind pose
        lPose->SetIsBindPose(true);

        for (i=0; i<lClusteredFbxNodes.GetCount(); i++)
        {
            KFbxNode*  lKFbxNode   = lClusteredFbxNodes.GetAt(i);
            KFbxMatrix lBindMatrix = lKFbxNode->EvaluateGlobalTransform();

            lPose->Add(lKFbxNode, lBindMatrix);
        }

        // Add the pose to the scene
        pScene->AddPose(lPose);
    }
}

// Store a Rest Pose
void StoreRestPose(KFbxScene* pScene, KFbxNode* pSkeletonRoot)
{
    // This example show an arbitrary rest pose assignment.
    // This rest pose will set the bone rotation to the same value 
    // as time 1 second in the first stack of animation, but the 
    // position of the bone will be set elsewhere in the scene.
    KString     lNodeName;
    KFbxNode*   lKFbxNode;
    KFbxMatrix  lTransformMatrix;
    KFbxVector4 lT,lR,lS(1.0, 1.0, 1.0);

    // Create the rest pose
    KFbxPose* lPose = KFbxPose::Create(pScene,"A Bind Pose");

    // Set the skeleton root node to the global position (10, 10, 10)
    // and global rotation of 45deg along the Z axis.
    lT.Set(10.0, 10.0, 10.0);
    lR.Set( 0.0,  0.0, 45.0);

    lTransformMatrix.SetTRS(lT, lR, lS);

    // Add the skeleton root node to the pose
    lKFbxNode = pSkeletonRoot;
    lPose->Add(lKFbxNode, lTransformMatrix, false); // it's a global matrix

    // Set the lLimbNode1 node to the local position of (0, 40, 0)
    // and local rotation of -90deg along the Z axis. This show that
    // you can mix local and global coordinates in a rest pose.
    lT.Set(0.0, 40.0,   0.0);
    lR.Set(0.0,  0.0, -90.0);

    lTransformMatrix.SetTRS(lT, lR, lS);

    // Add the skeleton second node to the pose
    lKFbxNode = lKFbxNode->GetChild(0);
    lPose->Add(lKFbxNode, lTransformMatrix, true); // it's a local matrix

    // Set the lLimbNode2 node to the local position of (0, 40, 0)
    // and local rotation of 45deg along the Z axis.
    lT.Set(0.0, 40.0, 0.0);
    lR.Set(0.0,  0.0, 45.0);

    lTransformMatrix.SetTRS(lT, lR, lS);

    // Add the skeleton second node to the pose
    lKFbxNode = lKFbxNode->GetChild(0);
    lNodeName = lKFbxNode->GetName();
    lPose->Add(lKFbxNode, lTransformMatrix, true); //it's a local matrix

    // Now add the pose to the scene
    pScene->AddPose(lPose);
}

// Add the specified node to the node array. Also, add recursively
// all the parent node of the specified node to the array.
void AddNodeRecursively(KArrayTemplate<KFbxNode*>& pNodeArray, KFbxNode* pNode)
{
    if (pNode)
    {
        AddNodeRecursively(pNodeArray, pNode->GetParent());

        if (pNodeArray.Find(pNode) == -1)
        {
            // Node not in the list, add it
            pNodeArray.Add(pNode);
        }
    }
}

void SetXMatrix(KFbxXMatrix& pXMatrix, const KFbxMatrix& pMatrix)
{
    memcpy((double*)pXMatrix, &pMatrix.mData[0][0], sizeof(pMatrix.mData));
}
*/

