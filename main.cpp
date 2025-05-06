#include <iostream>
#include "PxPhysicsAPI.h"

using namespace std;
using namespace physx;

PxDefaultAllocator      gAllocator;
PxDefaultErrorCallback  gErrorCallback;
PxFoundation* gFoundation = NULL;
PxPhysics* gPhysics = NULL;
PxDefaultCpuDispatcher* gDispatcher = NULL;
PxScene* gScene = NULL;
PxPvd* gPvd = NULL;
PxMaterial* gMaterial;
PxControllerManager* gControllerManager;

const PxReal ElapsedTime = 1.0f / 30.0f; // 60Hz

// PhysXの初期化
void initPhysics()
{
    gFoundation
        = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

    // PVDの設定
    gPvd = PxCreatePvd(*gFoundation);
    PxPvdTransport* transport
        = PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
    gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

    gPhysics = PxCreatePhysics(
        PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);
    PxInitExtensions(*gPhysics, gPvd);

    // Sceneの作成
    PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
    sceneDesc.gravity = PxVec3(0.0f, -9.8f, 0.0f);
    gDispatcher = PxDefaultCpuDispatcherCreate(0);
    sceneDesc.cpuDispatcher = gDispatcher;
    sceneDesc.filterShader = PxDefaultSimulationFilterShader;
    gScene = gPhysics->createScene(sceneDesc);

    // CCTマネージャーの作成
    gControllerManager =  PxCreateControllerManager(*gScene);

    // PVDの設定
    PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
    if (pvdClient)
    {
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
    }
}

/// <summary>
/// カプセル型キャラコンの作成
/// </summary>
PxController* CreateCharacterController(
    const PxVec3& position,
    float radius = 0.5f,
    float height = 1.8f
) {
    // カプセルコントローラの記述
    PxCapsuleControllerDesc desc;
    desc.radius = radius;
    desc.height = height;
    desc.position = PxExtendedVec3(position.x, position.y, position.z);
    desc.material = gMaterial;
    desc.upDirection = PxVec3(0, 1, 0);
    desc.slopeLimit = cosf(PxPi / 4);  // 45度
    desc.contactOffset = 0.1f;
    desc.stepOffset = 0.5f;
    desc.density = 10.0f;
    desc.scaleCoeff = 0.8f;
    desc.reportCallback = nullptr;  // 衝突コールバックなどを設定可能

    // 正しく初期化されているか確認
    if (!desc.isValid()) {
        printf("Character controller descriptor is invalid.\n");
        return nullptr;
    }
    // コントローラの作成
    PxController* controller = gControllerManager->createController(desc);
    if (!controller) {
        printf("Failed to create character controller.\n");
        return nullptr;
    }

    PxShape* shape;
    controller->getActor()->getShapes(&shape, 1);
    shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);
    shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, true);

    return controller;
}

void createFloatingPlane()
{
    // 平面のサイズ（10×10）、中心を(0,0,0)に配置
    PxVec3 planeCenter(0.0f, 0.0f, 0.0f);
    PxVec3 halfExtents(5.0f, 1.0f, 5.0f); // Yの厚みは0.2程度にしておく

    // 平面用のBoxジオメトリ
    PxBoxGeometry boxGeometry(halfExtents);

    // Transform（回転なし、座標(0,0,0)）
    PxTransform pose(planeCenter);

    // 静的なアクター（動かないオブジェクト）
    PxRigidStatic* staticActor = gPhysics->createRigidStatic(pose);

    // シェイプ（形状）を追加
    PxShape* shape = gPhysics->createShape(boxGeometry, *gMaterial);
    shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);
    shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, true);  // レイキャストなどで必要
    staticActor->attachShape(*shape);

    // シーンに追加
    gScene->addActor(*staticActor);

}

/// <summary>
/// Dynamic Rigidbodyの作成
/// </summary>
PxRigidDynamic* createDynamic(const PxTransform& t,
    const PxGeometry& geometry, PxReal density = 10.0f)
{
    PxRigidDynamic* rigid_dynamic
        = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, density);
    gScene->addActor(*rigid_dynamic);
    return rigid_dynamic;
}

// シミュレーションステップを進める
void stepPhysics()
{
    gScene->simulate(ElapsedTime);
    gScene->fetchResults(true);
}

int main(void)
{
    initPhysics();
    cout << "PhysXHelloWorld" << endl;
    cout << "Start simulation" << endl;

    const PxU32 kMaxSimulationStep = 100;

    // 静摩擦係数、動摩擦係数、反発係数の順
    gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

    // 半径1mの球を高さ10mから落とす
    PxRigidDynamic* sphere = createDynamic(
        PxTransform((PxVec3(0.0f, 10.0f, 0.0f))),
        PxSphereGeometry(1.0f));

    // 平面を作成
    createFloatingPlane();

    // コントローラーも作ってみる
    PxController* cct = CreateCharacterController( PxVec3(0.0f, 10.0f, 0.0f));

    for (PxU32 i = 0; i != kMaxSimulationStep; i++) {
        PxVec3 p = sphere->getGlobalPose().p;
        
        auto currentPosition = cct->getPosition();
        PxControllerFilters filters;
        cct->move(PxVec3(0.0f, -0.5f, 0.0f), 0.01f, ElapsedTime, filters);

        cout << "Step: " << i <<
            ", Position: (" << currentPosition.x << ", " << currentPosition.y << ", " << currentPosition.z << ")" << endl;

        stepPhysics();
    }

    cout << "End simulation" << endl;
    int tmp;
    cin >> tmp;
    return 0;
}