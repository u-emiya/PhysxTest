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

// PhysX�̏�����
void initPhysics()
{
    gFoundation
        = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

    // PVD�̐ݒ�
    gPvd = PxCreatePvd(*gFoundation);
    PxPvdTransport* transport
        = PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
    gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

    gPhysics = PxCreatePhysics(
        PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);
    PxInitExtensions(*gPhysics, gPvd);

    // Scene�̍쐬
    PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
    sceneDesc.gravity = PxVec3(0.0f, -9.8f, 0.0f);
    gDispatcher = PxDefaultCpuDispatcherCreate(0);
    sceneDesc.cpuDispatcher = gDispatcher;
    sceneDesc.filterShader = PxDefaultSimulationFilterShader;
    gScene = gPhysics->createScene(sceneDesc);

    // CCT�}�l�[�W���[�̍쐬
    gControllerManager =  PxCreateControllerManager(*gScene);

    // PVD�̐ݒ�
    PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
    if (pvdClient)
    {
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
    }
}

/// <summary>
/// �J�v�Z���^�L�����R���̍쐬
/// </summary>
PxController* CreateCharacterController(
    const PxVec3& position,
    float radius = 0.5f,
    float height = 1.8f
) {
    // �J�v�Z���R���g���[���̋L�q
    PxCapsuleControllerDesc desc;
    desc.radius = radius;
    desc.height = height;
    desc.position = PxExtendedVec3(position.x, position.y, position.z);
    desc.material = gMaterial;
    desc.upDirection = PxVec3(0, 1, 0);
    desc.slopeLimit = cosf(PxPi / 4);  // 45�x
    desc.contactOffset = 0.1f;
    desc.stepOffset = 0.5f;
    desc.density = 10.0f;
    desc.scaleCoeff = 0.8f;
    desc.reportCallback = nullptr;  // �Փ˃R�[���o�b�N�Ȃǂ�ݒ�\

    // ����������������Ă��邩�m�F
    if (!desc.isValid()) {
        printf("Character controller descriptor is invalid.\n");
        return nullptr;
    }
    // �R���g���[���̍쐬
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
    // ���ʂ̃T�C�Y�i10�~10�j�A���S��(0,0,0)�ɔz�u
    PxVec3 planeCenter(0.0f, 0.0f, 0.0f);
    PxVec3 halfExtents(5.0f, 1.0f, 5.0f); // Y�̌��݂�0.2���x�ɂ��Ă���

    // ���ʗp��Box�W�I���g��
    PxBoxGeometry boxGeometry(halfExtents);

    // Transform�i��]�Ȃ��A���W(0,0,0)�j
    PxTransform pose(planeCenter);

    // �ÓI�ȃA�N�^�[�i�����Ȃ��I�u�W�F�N�g�j
    PxRigidStatic* staticActor = gPhysics->createRigidStatic(pose);

    // �V�F�C�v�i�`��j��ǉ�
    PxShape* shape = gPhysics->createShape(boxGeometry, *gMaterial);
    shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);
    shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, true);  // ���C�L���X�g�ȂǂŕK�v
    staticActor->attachShape(*shape);

    // �V�[���ɒǉ�
    gScene->addActor(*staticActor);

}

/// <summary>
/// Dynamic Rigidbody�̍쐬
/// </summary>
PxRigidDynamic* createDynamic(const PxTransform& t,
    const PxGeometry& geometry, PxReal density = 10.0f)
{
    PxRigidDynamic* rigid_dynamic
        = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, density);
    gScene->addActor(*rigid_dynamic);
    return rigid_dynamic;
}

// �V�~�����[�V�����X�e�b�v��i�߂�
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

    // �Ö��C�W���A�����C�W���A�����W���̏�
    gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

    // ���a1m�̋�������10m���痎�Ƃ�
    PxRigidDynamic* sphere = createDynamic(
        PxTransform((PxVec3(0.0f, 10.0f, 0.0f))),
        PxSphereGeometry(1.0f));

    // ���ʂ��쐬
    createFloatingPlane();

    // �R���g���[���[������Ă݂�
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