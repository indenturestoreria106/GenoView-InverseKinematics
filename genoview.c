#include "raylib.h"
#include "raymath.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include "rlgl.h"

#include <assert.h>

//----------------------------------------------------------------------------------
// Math
//----------------------------------------------------------------------------------

static inline float Max(float x, float y)
{
    return x > y ? x : y;
}

static inline float Min(float x, float y)
{
    return x < y ? x : y;
}

// This is a safe version of QuaternionBetween which returns a 180 deg rotation
// at the singularity where vectors are facing exactly in opposite directions
static inline Quaternion QuaternionBetween(Vector3 p, Vector3 q)
{
    Vector3 c = Vector3CrossProduct(p, q);

    Quaternion o = {
        c.x,
        c.y,
        c.z,
        sqrtf(Vector3DotProduct(p, p) * Vector3DotProduct(q, q)) + 
            Vector3DotProduct(p, q),
    };
    
    return QuaternionLength(o) < 1e-8f ?
        QuaternionFromAxisAngle((Vector3){ 1.0f, 0.0f, 0.0f }, PI) :
        QuaternionNormalize(o);
}

// Puts the quaternion in the hemisphere closest to the identity
static inline Quaternion QuaternionAbsolute(Quaternion q)
{
    if (q.w < 0.0f)
    {
        q.x = -q.x;
        q.y = -q.y;
        q.z = -q.z;
        q.w = -q.w;
    }

    return q;
}

// Quaternion exponent, log, and angle axis functions (see: https://theorangeduck.com/page/exponential-map-angle-axis-angular-velocity)

static inline Quaternion QuaternionExp(Vector3 v)
{
    float halfangle = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);

    if (halfangle < 1e-4f)
    {
        return QuaternionNormalize((Quaternion){ v.x, v.y, v.z, 1.0f });
    }
    else
    {
        float c = cosf(halfangle);
        float s = sinf(halfangle) / halfangle;
        return (Quaternion){ s * v.x, s * v.y, s * v.z, c };
    }
}

static inline Vector3 QuaternionLog(Quaternion q)
{
    float length = sqrtf(q.x*q.x + q.y*q.y + q.z*q.z);

    if (length < 1e-4f)
    {
        return (Vector3){ q.x, q.y, q.z };
    }
    else
    {
        float halfangle = atan2f(length, q.w);
        return Vector3Scale((Vector3){ q.x, q.y, q.z }, halfangle / length);
    }
}

static inline Vector3 QuaternionToScaledAngleAxis(Quaternion q)
{
    return Vector3Scale(QuaternionLog(q), 2.0f);
}

static inline Quaternion QuaternionFromScaledAngleAxis(Vector3 v)
{
    return QuaternionExp(Vector3Scale(v, 0.5f));
}

//----------------------------------------------------------------------------------
// Camera
//----------------------------------------------------------------------------------

// Basic Orbit Camera with simple controls
typedef struct {

    Camera3D cam3d;
    float azimuth;
    float altitude;
    float distance;
    Vector3 offset;

} OrbitCamera;

static inline void OrbitCameraInit(OrbitCamera* camera)
{
    memset(&camera->cam3d, 0, sizeof(Camera3D));
    camera->cam3d.position = (Vector3){ 2.0f, 3.0f, 5.0f };
    camera->cam3d.target = (Vector3){ -0.5f, 1.0f, 0.0f };
    camera->cam3d.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera->cam3d.fovy = 45.0f;
    camera->cam3d.projection = CAMERA_PERSPECTIVE;

    camera->azimuth = 0.0f;
    camera->altitude = 0.4f;
    camera->distance = 4.0f;
    camera->offset = Vector3Zero();
}

static inline void OrbitCameraUpdate(
    OrbitCamera* camera,
    Vector3 target,
    float azimuthDelta,
    float altitudeDelta,
    float offsetDeltaX,
    float offsetDeltaY,
    float mouseWheel,
    float dt)
{
    camera->azimuth = camera->azimuth + 1.0f * dt * -azimuthDelta;
    camera->altitude = Clamp(camera->altitude + 1.0f * dt * altitudeDelta, 0.0, 0.4f * PI);
    camera->distance = Clamp(camera->distance +  20.0f * dt * -mouseWheel, 0.1f, 100.0f);
    
    Quaternion rotationAzimuth = QuaternionFromAxisAngle((Vector3){0, 1, 0}, camera->azimuth);
    Vector3 position = Vector3RotateByQuaternion((Vector3){0, 0, camera->distance}, rotationAzimuth);
    Vector3 axis = Vector3Normalize(Vector3CrossProduct(position, (Vector3){0, 1, 0}));

    Quaternion rotationAltitude = QuaternionFromAxisAngle(axis, camera->altitude);

    Vector3 localOffset = (Vector3){ dt * offsetDeltaX, dt * -offsetDeltaY, 0.0f };
    localOffset = Vector3RotateByQuaternion(localOffset, rotationAzimuth);

    camera->offset = Vector3Add(camera->offset, Vector3RotateByQuaternion(localOffset, rotationAltitude));

    Vector3 cameraTarget = Vector3Add(camera->offset, target);
    Vector3 eye = Vector3Add(cameraTarget, Vector3RotateByQuaternion(position, rotationAltitude));

    camera->cam3d.target = cameraTarget;
    camera->cam3d.position = eye;
}

//----------------------------------------------------------------------------------
// Shadow Maps
//----------------------------------------------------------------------------------

typedef struct
{
    Vector3 target;
    Vector3 position;
    Vector3 up;
    double width;
    double height;
    double near;
    double far;
    
} ShadowLight;

RenderTexture2D LoadShadowMap(int width, int height)
{
    RenderTexture2D target = { 0 };
    target.id = rlLoadFramebuffer();
    target.texture.width = width;
    target.texture.height = height;
    assert(target.id);
    
    rlEnableFramebuffer(target.id);

    target.depth.id = rlLoadTextureDepth(width, height, false);
    target.depth.width = width;
    target.depth.height = height;
    target.depth.format = 19;       //DEPTH_COMPONENT_24BIT?
    target.depth.mipmaps = 1;
    rlFramebufferAttach(target.id, target.depth.id, RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_TEXTURE2D, 0);
    assert(rlFramebufferComplete(target.id));

    rlDisableFramebuffer();

    return target;
}

void UnloadShadowMap(RenderTexture2D target)
{
    if (target.id > 0)
    {
        rlUnloadFramebuffer(target.id);
    }
}

void BeginShadowMap(RenderTexture2D target, ShadowLight shadowLight)
{
    BeginTextureMode(target);
    ClearBackground(WHITE);
    
    rlDrawRenderBatchActive();      // Update and draw internal render batch

    rlMatrixMode(RL_PROJECTION);    // Switch to projection matrix
    rlPushMatrix();                 // Save previous matrix, which contains the settings for the 2d ortho projection
    rlLoadIdentity();               // Reset current matrix (projection)

    rlOrtho(
        -shadowLight.width/2, shadowLight.width/2, 
        -shadowLight.height/2, shadowLight.height/2, 
        shadowLight.near, shadowLight.far);

    rlMatrixMode(RL_MODELVIEW);     // Switch back to modelview matrix
    rlLoadIdentity();               // Reset current matrix (modelview)

    // Setup Camera view
    Matrix matView = MatrixLookAt(shadowLight.position, shadowLight.target, shadowLight.up);
    rlMultMatrixf(MatrixToFloat(matView));      // Multiply modelview matrix by view matrix (camera)

    rlEnableDepthTest();            // Enable DEPTH_TEST for 3D    
}

void EndShadowMap()
{
    rlDrawRenderBatchActive();      // Update and draw internal render batch

    rlMatrixMode(RL_PROJECTION);    // Switch to projection matrix
    rlPopMatrix();                  // Restore previous matrix (projection) from matrix stack

    rlMatrixMode(RL_MODELVIEW);     // Switch back to modelview matrix
    rlLoadIdentity();               // Reset current matrix (modelview)

    rlDisableDepthTest();           // Disable DEPTH_TEST for 2D

    EndTextureMode();
}

void SetShaderValueShadowMap(Shader shader, int locIndex, RenderTexture2D target)
{
    if (locIndex > -1)
    {
        rlEnableShader(shader.id);
        int slot = 10; // Can be anything 0 to 15, but 0 will probably be taken up
        rlActiveTextureSlot(slot);
        rlEnableTexture(target.depth.id);
        rlSetUniform(locIndex, &slot, SHADER_UNIFORM_INT, 1);
    }
}

//----------------------------------------------------------------------------------
// GBuffer
//----------------------------------------------------------------------------------

typedef struct
{
    unsigned int id;        // OpenGL framebuffer object id
    Texture color;          // Color buffer attachment texture
    Texture normal;         // Normal buffer attachment texture
    Texture depth;          // Depth buffer attachment texture
    
} GBuffer;

GBuffer LoadGBuffer(int width, int height)
{
    GBuffer target = { 0 };
    target.id = rlLoadFramebuffer();
    assert(target.id);
    
    rlEnableFramebuffer(target.id);

    target.color.id = rlLoadTexture(NULL, width, height, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8, 1);
    target.color.width = width;
    target.color.height = height;
    target.color.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;
    target.color.mipmaps = 1;
    rlFramebufferAttach(target.id, target.color.id, RL_ATTACHMENT_COLOR_CHANNEL0, RL_ATTACHMENT_TEXTURE2D, 0);
    
    target.normal.id = rlLoadTexture(NULL, width, height, PIXELFORMAT_UNCOMPRESSED_R16G16B16A16, 1);
    target.normal.width = width;
    target.normal.height = height;
    target.normal.format = PIXELFORMAT_UNCOMPRESSED_R16G16B16A16;
    target.normal.mipmaps = 1;
    rlFramebufferAttach(target.id, target.normal.id, RL_ATTACHMENT_COLOR_CHANNEL1, RL_ATTACHMENT_TEXTURE2D, 0);
    
    target.depth.id = rlLoadTextureDepth(width, height, false);
    target.depth.width = width;
    target.depth.height = height;
    target.depth.format = 19;       //DEPTH_COMPONENT_24BIT?
    target.depth.mipmaps = 1;
    rlFramebufferAttach(target.id, target.depth.id, RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_TEXTURE2D, 0);

    assert(rlFramebufferComplete(target.id));

    rlDisableFramebuffer();

    return target;
}

void UnloadGBuffer(GBuffer target)
{
    if (target.id > 0)
    {
        rlUnloadFramebuffer(target.id);
    }
}

void BeginGBuffer(GBuffer target, Camera3D camera)
{
    rlDrawRenderBatchActive();      // Update and draw internal render batch

    rlEnableFramebuffer(target.id); // Enable render target
    rlActiveDrawBuffers(2);

    // Set viewport and RLGL internal framebuffer size
    rlViewport(0, 0, target.color.width, target.color.height);
    rlSetFramebufferWidth(target.color.width);
    rlSetFramebufferHeight(target.color.height);

    ClearBackground(BLACK);

    rlMatrixMode(RL_PROJECTION);    // Switch to projection matrix
    rlPushMatrix();                 // Save previous matrix, which contains the settings for the 2d ortho projection
    rlLoadIdentity();               // Reset current matrix (projection)

    float aspect = (float)target.color.width/(float)target.color.height;

    // NOTE: zNear and zFar values are important when computing depth buffer values
    if (camera.projection == CAMERA_PERSPECTIVE)
    {
        // Setup perspective projection
        double top = rlGetCullDistanceNear()*tan(camera.fovy*0.5*DEG2RAD);
        double right = top*aspect;

        rlFrustum(-right, right, -top, top, rlGetCullDistanceNear(), rlGetCullDistanceFar());
    }
    else if (camera.projection == CAMERA_ORTHOGRAPHIC)
    {
        // Setup orthographic projection
        double top = camera.fovy/2.0;
        double right = top*aspect;

        rlOrtho(-right, right, -top,top, rlGetCullDistanceNear(), rlGetCullDistanceFar());
    }

    rlMatrixMode(RL_MODELVIEW);     // Switch back to modelview matrix
    rlLoadIdentity();               // Reset current matrix (modelview)

    // Setup Camera view
    Matrix matView = MatrixLookAt(camera.position, camera.target, camera.up);
    rlMultMatrixf(MatrixToFloat(matView));      // Multiply modelview matrix by view matrix (camera)

    rlEnableDepthTest();            // Enable DEPTH_TEST for 3D
}

void EndGBuffer(int windowWidth, int windowHeight)
{
    rlDrawRenderBatchActive();      // Update and draw internal render batch
    
    rlDisableDepthTest();           // Disable DEPTH_TEST for 2D
    rlActiveDrawBuffers(1);
    rlDisableFramebuffer();         // Disable render target (fbo)

    rlMatrixMode(RL_PROJECTION);        // Switch to projection matrix
    rlPopMatrix();                  // Restore previous matrix (projection) from matrix stack
    rlLoadIdentity();                   // Reset current matrix (projection)
    rlOrtho(0, windowWidth, windowHeight, 0, 0.0f, 1.0f);

    rlMatrixMode(RL_MODELVIEW);         // Switch back to modelview matrix
    rlLoadIdentity();                   // Reset current matrix (modelview)
}

//----------------------------------------------------------------------------------
// Geno Character and Animation
//----------------------------------------------------------------------------------

Model LoadGenoModel(const char* fileName)
{
    Model model = { 0 };
    model.transform = MatrixIdentity();
  
    FILE* f = fopen(fileName, "rb");
    if (f == NULL)
    {
        TRACELOG(LOG_ERROR, "MODEL Unable to read skinned model file %s", fileName);
        return model;
    }
    
    model.materialCount = 1;
    model.materials = RL_CALLOC(model.materialCount, sizeof(Mesh));
    model.materials[0] = LoadMaterialDefault();

    model.meshCount = 1;
    model.meshes = RL_CALLOC(model.meshCount, sizeof(Mesh));
    model.meshMaterial = RL_CALLOC(model.meshCount, sizeof(Mesh));
    model.meshMaterial[0] = 0;

    fread(&model.meshes[0].vertexCount, sizeof(int), 1, f);
    fread(&model.meshes[0].triangleCount, sizeof(int), 1, f);
    fread(&model.boneCount, sizeof(int), 1, f);

    model.meshes[0].boneCount = model.boneCount;
    model.meshes[0].vertices = RL_CALLOC(model.meshes[0].vertexCount * 3, sizeof(float));
    model.meshes[0].texcoords = RL_CALLOC(model.meshes[0].vertexCount * 2, sizeof(float));
    model.meshes[0].normals = RL_CALLOC(model.meshes[0].vertexCount * 3, sizeof(float));
    model.meshes[0].boneIds = RL_CALLOC(model.meshes[0].vertexCount * 4, sizeof(unsigned char));
    model.meshes[0].boneWeights = RL_CALLOC(model.meshes[0].vertexCount * 4, sizeof(float));
    model.meshes[0].indices = RL_CALLOC(model.meshes[0].triangleCount * 3, sizeof(unsigned short));
    model.meshes[0].animVertices = RL_CALLOC(model.meshes[0].vertexCount * 3, sizeof(float));
    model.meshes[0].animNormals = RL_CALLOC(model.meshes[0].vertexCount * 3, sizeof(float));
    model.bones =  RL_CALLOC(model.boneCount, sizeof(BoneInfo));
    model.bindPose =  RL_CALLOC(model.boneCount, sizeof(Transform));
    
    fread(model.meshes[0].vertices, sizeof(float), model.meshes[0].vertexCount * 3, f);
    fread(model.meshes[0].texcoords, sizeof(float), model.meshes[0].vertexCount * 2, f);
    fread(model.meshes[0].normals, sizeof(float), model.meshes[0].vertexCount * 3, f);
    fread(model.meshes[0].boneIds, sizeof(unsigned char), model.meshes[0].vertexCount * 4, f);
    fread(model.meshes[0].boneWeights, sizeof(float), model.meshes[0].vertexCount * 4, f);
    fread(model.meshes[0].indices, sizeof(unsigned short), model.meshes[0].triangleCount * 3, f);
    memcpy(model.meshes[0].animVertices, model.meshes[0].vertices, sizeof(float) * model.meshes[0].vertexCount * 3);
    memcpy(model.meshes[0].animNormals, model.meshes[0].normals, sizeof(float) * model.meshes[0].vertexCount * 3);
    fread(model.bones, sizeof(BoneInfo), model.boneCount, f);
    fread(model.bindPose, sizeof(Transform), model.boneCount, f);
    fclose(f);
    
    model.meshes[0].boneMatrices = RL_CALLOC(model.boneCount, sizeof(Matrix));
    for (int i = 0; i < model.boneCount; i++)
    {
        model.meshes[0].boneMatrices[i] = MatrixIdentity();
    }
    
    UploadMesh(&model.meshes[0], true);
    
    return model;
}

static inline int FindModelBoneIndex(Model model, const char* boneName)
{
    for (int i = 0; i < model.boneCount; i++)
    {
        if (strcmp(model.bones[i].name, boneName) == 0)
        {
            return i;
        }
    }
    
    return -1;
}

static inline void BackwardKinematics(Transform* localTransforms, Transform* globalTransforms, Model model)
{
    for (int i = 0; i < model.boneCount; i++)
    {
        int p = model.bones[i].parent;

        if (p == -1)
        {
            localTransforms[i] = globalTransforms[i];
        }
        else
        {
            localTransforms[i].translation = 
                Vector3RotateByQuaternion(
                    Vector3Subtract(globalTransforms[i].translation, globalTransforms[p].translation), QuaternionInvert(globalTransforms[p].rotation));                
            localTransforms[i].rotation = QuaternionMultiply(QuaternionInvert(globalTransforms[p].rotation), globalTransforms[i].rotation);
            globalTransforms[i].scale = (Vector3){ 1.0f, 1.0f, 1.0f };
        }
    }
}

static inline void ForwardKinematics(Transform* globalTransforms, Transform* localTransforms, Model model)
{
    for (int i = 0; i < model.boneCount; i++)
    {
        int p = model.bones[i].parent;

        if (p == -1)
        {
            globalTransforms[i] = localTransforms[i];
        }
        else
        {
            assert(p < i);
            
            globalTransforms[i].translation = Vector3Add(
                Vector3RotateByQuaternion(localTransforms[i].translation, globalTransforms[p].rotation), 
                globalTransforms[p].translation);                
            globalTransforms[i].rotation = QuaternionMultiply(globalTransforms[p].rotation, localTransforms[i].rotation);
            globalTransforms[i].scale = (Vector3){ 1.0f, 1.0f, 1.0f };
        }
    }
}

static inline void UpdateModelPoseFromTransforms(Model model, Transform* globalTransforms)
{
    Matrix bindPoseMatrix = { 0 };
    Matrix currentPoseMatrix = { 0 };

    // Update all bones and bone matrices of model
    for (int boneIndex = 0; boneIndex < model.boneCount; boneIndex++)
    {
        // Compute runtime bone matrix from model current pose
        //-----------------------------------------------------------------------------------
        Transform *bindPoseTransform = &model.bindPose[boneIndex];
        bindPoseMatrix = MatrixMultiply(
            MatrixMultiply(MatrixScale(bindPoseTransform->scale.x, bindPoseTransform->scale.y, bindPoseTransform->scale.z),
                QuaternionToMatrix(bindPoseTransform->rotation)),
            MatrixTranslate(bindPoseTransform->translation.x, bindPoseTransform->translation.y, bindPoseTransform->translation.z));

        Transform *currentPoseTransform = &globalTransforms[boneIndex];
        currentPoseMatrix = MatrixMultiply(
            MatrixMultiply(MatrixScale(currentPoseTransform->scale.x, currentPoseTransform->scale.y, currentPoseTransform->scale.z),
                QuaternionToMatrix(currentPoseTransform->rotation)),
            MatrixTranslate(currentPoseTransform->translation.x, currentPoseTransform->translation.y, currentPoseTransform->translation.z));

        model.meshes[0].boneMatrices[boneIndex] = MatrixMultiply(MatrixInvert(bindPoseMatrix), currentPoseMatrix);
        //-----------------------------------------------------------------------------------
    }
}

ModelAnimation LoadGenoModelAnimation(const char* fileName)
{
    ModelAnimation animation = { 0 };
    
    FILE* f = fopen(fileName, "rb");
    if (f == NULL)
    {
        TRACELOG(LOG_ERROR, "MODEL ANIMATION Unable to read animation file %s", fileName);
        return animation;
    }
    
    fread(&animation.frameCount, sizeof(int), 1, f);
    fread(&animation.boneCount, sizeof(int), 1, f);
    
    animation.bones = RL_CALLOC(animation.boneCount, sizeof(BoneInfo));
    fread(animation.bones, sizeof(BoneInfo), animation.boneCount, f);        
    
    animation.framePoses = RL_CALLOC(animation.frameCount, sizeof(Transform*));
    for (int i = 0; i < animation.frameCount; i++)
    {
        animation.framePoses[i] = RL_CALLOC(animation.boneCount, sizeof(Transform));
        fread(animation.framePoses[i], sizeof(Transform), animation.boneCount, f);        
    }

    fclose(f);
    
    return animation;
}

ModelAnimation LoadEmptyModelAnimation(Model model)
{
    ModelAnimation animation = { 0 };
    animation.frameCount = 1;
    animation.boneCount = model.boneCount;
    
    animation.bones = RL_CALLOC(animation.boneCount, sizeof(BoneInfo));
    memcpy(animation.bones, model.bones, animation.boneCount * sizeof(BoneInfo));
    
    animation.framePoses = RL_CALLOC(animation.frameCount, sizeof(Transform*));
    for (int i = 0; i < animation.frameCount; i++)
    {
        animation.framePoses[i] = RL_CALLOC(animation.boneCount, sizeof(Transform));
        memcpy(animation.framePoses[i], model.bindPose, animation.boneCount * sizeof(Transform));
    }

    return animation;
}

typedef struct ModelAnimationContacts
{
    int frameCount;
    float* leftContacts;
    float* rightContacts;
    
} ModelAnimationContacts;

ModelAnimationContacts LoadModelAnimationContacts(const char* fileName)
{
    ModelAnimationContacts contacts = { 0 };
    
    FILE* f = fopen(fileName, "rb");
    if (f == NULL)
    {
        TRACELOG(LOG_ERROR, "MODEL ANIMATION CONTACTS Unable to read contacts file %s", fileName);
        return contacts;
    }
    
    fread(&contacts.frameCount, sizeof(int), 1, f);

    contacts.leftContacts = RL_CALLOC(contacts.frameCount, sizeof(float));
    contacts.rightContacts = RL_CALLOC(contacts.frameCount, sizeof(float));
    fread(contacts.leftContacts, sizeof(float), contacts.frameCount, f);        
    fread(contacts.rightContacts, sizeof(float), contacts.frameCount, f);        
    
    fclose(f);
    
    return contacts;
}

void UnloadModelAnimationContacts(ModelAnimationContacts contacts)
{
    RL_FREE(contacts.leftContacts);
    RL_FREE(contacts.rightContacts);
}

//----------------------------------------------------------------------------------
// Debug Draw
//----------------------------------------------------------------------------------

static inline void DrawTransform(Transform t, float scale)
{
    Matrix rotMatrix = QuaternionToMatrix(t.rotation);
  
    DrawLine3D(
        t.translation,
        Vector3Add(t.translation, (Vector3){ scale * rotMatrix.m0, scale * rotMatrix.m1, scale * rotMatrix.m2 }),
        RED);
        
    DrawLine3D(
        t.translation,
        Vector3Add(t.translation, (Vector3){ scale * rotMatrix.m4, scale * rotMatrix.m5, scale * rotMatrix.m6 }),
        GREEN);
        
    DrawLine3D(
        t.translation,
        Vector3Add(t.translation, (Vector3){ scale * rotMatrix.m8, scale * rotMatrix.m9, scale * rotMatrix.m10 }),
        BLUE);
}

static inline void DrawTransformThick(Transform t, float scale, float thickness)
{
    Matrix rotMatrix = QuaternionToMatrix(t.rotation);
  
    DrawCapsule(
        t.translation,
        Vector3Add(t.translation, (Vector3){ scale * rotMatrix.m0, scale * rotMatrix.m1, scale * rotMatrix.m2 }),
        thickness,
        7, 7,
        RED);
        
    DrawCapsule(
        t.translation,
        Vector3Add(t.translation, (Vector3){ scale * rotMatrix.m4, scale * rotMatrix.m5, scale * rotMatrix.m6 }),
        thickness,
        7, 7,
        GREEN);
        
    DrawCapsule(
        t.translation,
        Vector3Add(t.translation, (Vector3){ scale * rotMatrix.m8, scale * rotMatrix.m9, scale * rotMatrix.m10 }),
        thickness,
        7, 7,
        BLUE);
}

static inline void DrawModelTransforms(Transform* globalTransforms, Model model, Color color)
{
    for (int i = 0; i < model.boneCount; i++)
    {
        DrawSphereWires(
            globalTransforms[i].translation,
            0.01f,
            4,
            6,
            color);
            
        DrawTransform(globalTransforms[i], 0.1f);

        if (model.bones[i].parent != -1)
        {
            DrawLine3D(
                globalTransforms[i].translation,
                globalTransforms[model.bones[i].parent].translation,
                color);
        }
    }
}

static inline void DrawLegTransforms(
    Transform* globalTransforms, 
    Color color,
    int hipBoneIndex,
    int kneeBoneIndex,
    int heelBoneIndex,
    int toeBoneIndex,
    int toeEndBoneIndex)
{
    DrawTransformThick(globalTransforms[hipBoneIndex], 0.1f, 0.005f);
    DrawTransformThick(globalTransforms[kneeBoneIndex], 0.1f, 0.005f);
    DrawTransformThick(globalTransforms[heelBoneIndex], 0.1f, 0.005f);
    DrawTransformThick(globalTransforms[toeBoneIndex], 0.1f, 0.005f);
    DrawTransformThick(globalTransforms[toeEndBoneIndex], 0.1f, 0.005f);
    
    DrawCapsule(
        globalTransforms[hipBoneIndex].translation,
        globalTransforms[kneeBoneIndex].translation,
        0.0025f,
        7, 7, color);
        
    DrawCapsule(
        globalTransforms[kneeBoneIndex].translation,
        globalTransforms[heelBoneIndex].translation,
        0.0025f,
        7, 7, color);
        
    DrawCapsule(
        globalTransforms[heelBoneIndex].translation,
        globalTransforms[toeBoneIndex].translation,
        0.0025f,
        7, 7, color);
        
    DrawCapsule(
        globalTransforms[toeBoneIndex].translation,
        globalTransforms[toeEndBoneIndex].translation,
        0.0025f,
        7, 7, color);
}

//----------------------------------------------------------------------------------
// Inverse Kinematics
//----------------------------------------------------------------------------------

static inline void TwoBoneInverseKinematics(
    Quaternion *localHip,
    Quaternion *localKnee,
    Transform globalPelvis, 
    Transform globalHip, 
    Transform globalKnee, 
    Transform globalHeel, 
    Vector3 targetHeel, 
    Vector3 sideVector,
    float maxExtension,
    float softening)
{
    // Softly clamp the target based on the distance given by maxExtension 
    
    Vector3 targetClamp = targetHeel;
    float targetLength = Vector3Distance(targetHeel, globalHip.translation);

    if (targetLength > maxExtension - softening)
    {
        // Smoothly clamp when it gets within softening distance of the maxExtension
        
        float saturation = 1.0f - expf(
            -Max(targetLength - maxExtension + softening, 0.0f) / softening);
        
        targetClamp = Vector3Add(
            globalHip.translation, 
            Vector3Scale(Vector3Subtract(targetHeel, globalHip.translation), 
                (maxExtension - softening + softening * saturation) / targetLength)); 
    }
    
    // Compute the rotation axis based on vector perpendicular to the plane 
    // rotation which is closed to the provided knee side vector
    
    Vector3 axisDwn = Vector3Normalize(
        Vector3Subtract(globalHeel.translation, globalHip.translation));
        
    Vector3 axisFwd = Vector3Normalize(Vector3CrossProduct(axisDwn, sideVector));
    Vector3 axisRot = Vector3Normalize(Vector3CrossProduct(axisDwn, axisFwd));

    Vector3 a = globalHip.translation;
    Vector3 b = globalKnee.translation;
    Vector3 c = globalHeel.translation;
    Vector3 t = targetClamp;
    
    // Compute the change in rotation angle required using the cosine rule
    
    float lab = Vector3Distance(b, a);
    float lcb = Vector3Distance(b, c);
    float lat = Vector3Distance(t, a);
    float lca = Vector3Distance(a, c);

    float acab0 = acosf(Clamp(Vector3DotProduct(
        Vector3Scale(Vector3Subtract(c, a), 1.0f / lca), 
        Vector3Scale(Vector3Subtract(b, a), 1.0f / lab)), -1.0f, +1.0f));
        
    float babc0 = acosf(Clamp(Vector3DotProduct(
        Vector3Scale(Vector3Subtract(a, b), 1.0f / lab), 
        Vector3Scale(Vector3Subtract(c, b), 1.0f / lcb)), -1.0f, +1.0f));

    float acab1 = acosf(Clamp(
        (lab * lab + lat * lat - lcb * lcb) / (2.0 * lab * lat), -1.0f, +1.0f));
        
    float babc1 = acosf(Clamp(
        (lab * lab + lcb * lcb - lat * lat) / (2.0 * lab * lcb), -1.0f, +1.0f));
        
    // Compute the three world-space rotations needed to solve the two-bone ik problem
        
    Quaternion r0 = QuaternionFromScaledAngleAxis(Vector3Scale(axisRot, acab1 - acab0));
    Quaternion r1 = QuaternionFromScaledAngleAxis(Vector3Scale(axisRot, babc1 - babc0));
    Quaternion r2 = QuaternionNormalize(QuaternionBetween(
        Vector3Subtract(globalHeel.translation, globalHip.translation), 
        Vector3Subtract(targetClamp, globalHip.translation)));
    
    // Update the local space rotations for the hip and the knee
    
    *localHip = QuaternionMultiply(QuaternionMultiply(QuaternionMultiply(
        QuaternionInvert(globalPelvis.rotation), r2), r0), globalHip.rotation);
        
    *localKnee = QuaternionMultiply(QuaternionMultiply(
        QuaternionInvert(globalHip.rotation), r1), globalKnee.rotation);
}

static inline Quaternion BoneOrientTowards(
    Transform boneParentTransform,
    Transform boneTransform,
    Transform boneChildTransform,
    Vector3 target)
{
    Quaternion desiredRotation = QuaternionMultiply(QuaternionNormalize(
        QuaternionBetween(
            Vector3Subtract(boneChildTransform.translation, boneTransform.translation),
            Vector3Subtract(target, boneTransform.translation))), 
            boneTransform.rotation);

    return QuaternionMultiply(
        QuaternionInvert(boneParentTransform.rotation), desiredRotation);
}

static inline void SolveLegChain(
    Model model,
    Transform* localTransforms,
    Transform* globalTransforms,
    Vector3 target,
    Vector3 *targetHeel,
    Vector3 *targetToe,
    Vector3 *targetToeEnd,
    int pelvisBoneIndex,
    int hipBoneIndex,
    int kneeBoneIndex,
    int heelBoneIndex,
    int toeBoneIndex,
    int toeEndBoneIndex,
    bool enableHeightClamp,
    bool enableHeelLookAt,
    bool enableToeLookAt,
    float heelMinHeight,
    float toeMinHeight,
    float toeEndMinHeight,
    float softening,
    Vector3 kneeSideVector)
{
    *targetToe = target;
    
    if (enableHeightClamp)
    {
        targetToe->y = Max(targetToe->y, toeMinHeight);
    }
    
    // Find the Heel Target Location

    *targetHeel = Vector3Add(*targetToe, Vector3Subtract(
        globalTransforms[heelBoneIndex].translation, 
        globalTransforms[toeBoneIndex].translation));

    if (enableHeightClamp)
    {
        targetHeel->y = Max(targetHeel->y, heelMinHeight);
    }
    
    // Solve Two-Bone Inverse Kinematics to place heel
    
    Vector3 sideVector = Vector3RotateByQuaternion(
        kneeSideVector, 
        globalTransforms[kneeBoneIndex].rotation);
    
    float maxExtension = Vector3Distance(
        globalTransforms[hipBoneIndex].translation, 
        globalTransforms[heelBoneIndex].translation);
    
    Quaternion modifiedHip, modifiedKnee;

    TwoBoneInverseKinematics(
        &modifiedHip, 
        &modifiedKnee, 
        globalTransforms[pelvisBoneIndex], 
        globalTransforms[hipBoneIndex], 
        globalTransforms[kneeBoneIndex], 
        globalTransforms[heelBoneIndex], 
        *targetHeel, 
        sideVector,
        maxExtension,
        softening);
        
    localTransforms[hipBoneIndex].rotation = modifiedHip;
    localTransforms[kneeBoneIndex].rotation = modifiedKnee;
    
    // Orient Toe towards Target
    
    if (enableHeelLookAt)
    {
        ForwardKinematics(globalTransforms, localTransforms, model);
        
        localTransforms[heelBoneIndex].rotation = BoneOrientTowards(
            globalTransforms[kneeBoneIndex],
            globalTransforms[heelBoneIndex],
            globalTransforms[toeBoneIndex],
            *targetToe);
    }
    
    // Orient Toe-End
    
    if (enableToeLookAt)
    {          
        ForwardKinematics(globalTransforms, localTransforms, model);
        
        *targetToeEnd = globalTransforms[toeEndBoneIndex].translation;
        
        if (enableHeightClamp)
        {
            targetToeEnd->y = Max(targetToeEnd->y, toeEndMinHeight);
        }

        localTransforms[toeBoneIndex].rotation = BoneOrientTowards(
            globalTransforms[heelBoneIndex],
            globalTransforms[toeBoneIndex],
            globalTransforms[toeEndBoneIndex],
            *targetToeEnd);
    }
    
    // Recompute final global transforms
    
    ForwardKinematics(globalTransforms, localTransforms, model);
}

//----------------------------------------------------------------------------------
// Foot Locking
//----------------------------------------------------------------------------------

void InertializeCubicUpdate(
    Vector3* position, 
    Vector3* velocity, 
    float* time,
    Vector3 inputPosition,
    Vector3 inputVelocity,
    Vector3 offsetPosition,
    Vector3 offsetVelocity,
    float deltaTime,
    float blendTime)
{
    float t = Clamp((*time + deltaTime) / Max(blendTime, 1e-8f), 0.0f, 1.0f);
    float w0 = 2.0f * t * t * t - 3.0f * t * t + 1.0f;
    float w1 = (t * t * t - 2.0f * t * t + t) * blendTime;
    float w2 = (6.0f * t * t - 6.0f * t) / Max(blendTime, 1e-8f);
    float w3 = 3.0f * t * t - 4.0f * t + 1.0f;
    
    *position = Vector3Add(inputPosition, Vector3Add(
        Vector3Scale(offsetPosition, w0), Vector3Scale(offsetVelocity, w1)));
    *velocity = Vector3Add(inputVelocity, Vector3Add(
        Vector3Scale(offsetPosition, w2), Vector3Scale(offsetVelocity, w3)));
    *time = *time + deltaTime;
}

void InertializeCubicTransition(
    Vector3* offsetPosition, 
    Vector3* offsetVelocity, 
    float* time,
    Vector3 sourcePosition,
    Vector3 sourceVelocity,
    Vector3 destinationPosition,
    Vector3 destinationVelocity,
    float blendTime)
{
    float t = Clamp(*time / Max(blendTime, 1e-8f), 0.0f, 1.0f);
    float w0 = 2.0f * t * t * t - 3.0f * t * t + 1.0f;
    float w1 = (t * t * t - 2.0f * t * t + t) * blendTime;
    float w2 = (6.0f * t * t - 6.0f * t) / Max(blendTime, 1e-8f);
    float w3 = 3.0f * t * t - 4.0f * t + 1.0f;
  
    *offsetPosition = Vector3Subtract(Vector3Add(sourcePosition, 
        Vector3Add(Vector3Scale(*offsetPosition, w0), 
                   Vector3Scale(*offsetVelocity, w1))), destinationPosition);
    *offsetVelocity = Vector3Subtract(Vector3Add(sourceVelocity,
        Vector3Add(Vector3Scale(*offsetPosition, w2), 
                   Vector3Scale(*offsetVelocity, w3))), destinationVelocity);
    *time = 0.0f;
}

typedef struct FootLockingState
{
    Vector3 position;           // Current Position
    Vector3 velocity;           // Current Velocity
    Vector3 inputPosition;      // Input Source Position
    Vector3 inputVelocity;      // Input Source Velocity
    Vector3 offsetPosition;     // Inertialization Offset
    Vector3 offsetVelocity;     // Inertialization Offset Velocity
    float time;                 // Time since Inertialization transition
    Vector3 contact;            // Contact Location
    bool locked;                // If the contact is currently locked
    
} FootLockingState;

void ResetFootLockingState(FootLockingState* state, Vector3 inputPosition)
{
    state->position = inputPosition;
    state->velocity = Vector3Zero();
    state->inputPosition = inputPosition;
    state->inputVelocity = Vector3Zero();
    state->offsetPosition = Vector3Zero();
    state->offsetVelocity = Vector3Zero();
    state->time = 0.0f;
    state->contact = Vector3Zero();
    state->locked = false;
}

void UpdateFootLockingState(
    FootLockingState* state, 
    Vector3 inputPosition, 
    bool inputContact, 
    float contactHeight, 
    float deltaTime,
    float unlockDistance,
    float lockDistance,
    float blendTime)
{    
    // Update Input State Position and Velocity
    
    state->inputVelocity = Vector3Scale(
        Vector3Subtract(inputPosition, state->inputPosition), 
        1.0f / Max(deltaTime, 1e-8f));
    
    state->inputPosition = inputPosition;
    
    // Update Cubic Inertialization
    
    InertializeCubicUpdate(
        &state->position, 
        &state->velocity, 
        &state->time,
        state->locked ? state->contact : state->inputPosition,
        state->locked ? Vector3Zero() : state->inputVelocity,
        state->offsetPosition, 
        state->offsetVelocity,
        deltaTime,
        blendTime);
    
    // Check the distance of the input location from the locked state
    
    float inputDistance = Vector3Distance(state->position, state->inputPosition);
    
    if (!state->locked && inputContact && inputDistance < lockDistance)
    {
        // Lock if the input wants it the current state is within locking distanced

        state->locked = true;
        state->contact = state->inputPosition;
        state->contact.y = contactHeight;
        
        InertializeCubicTransition(
            &state->offsetPosition, 
            &state->offsetVelocity, 
            &state->time,
            state->inputPosition,
            state->inputVelocity,
            state->contact,
            Vector3Zero(),
            blendTime);
    }
    else if (state->locked && (!inputContact || inputDistance > unlockDistance))
    {
        // Unlock if the input wants to unlock or the distance is too far

        state->locked = false;
        
        InertializeCubicTransition(
            &state->offsetPosition, 
            &state->offsetVelocity, 
            &state->time,
            state->contact,
            Vector3Zero(),
            state->inputPosition,
            state->inputVelocity,
            blendTime);
    }
}

//----------------------------------------------------------------------------------
// Offline Foot Sliding Removal
//----------------------------------------------------------------------------------

static inline void RemoveFootSliding(
    Model model,
    ModelAnimation animation,
    ModelAnimationContacts contacts,
    int pelvisBoneIndex,
    int leftHipBoneIndex,
    int leftKneeBoneIndex,
    int leftHeelBoneIndex,
    int leftToeBoneIndex,
    int leftToeEndBoneIndex,
    int rightHipBoneIndex,
    int rightKneeBoneIndex,
    int rightHeelBoneIndex,
    int rightToeBoneIndex,
    int rightToeEndBoneIndex,
    bool enableHeightClamp,
    bool enableHeelLookAt,
    bool enableToeLookAt,
    float heelMinHeight,
    float toeMinHeight,
    float toeEndMinHeight,
    float softening,
    float contactThreshold,
    Vector3 leftKneeSideVector,
    Vector3 rightKneeSideVector)
{   
    Vector3* pelvisLocations = RL_CALLOC(animation.frameCount, sizeof(Vector3));
    Vector3* leftToeLocations = RL_CALLOC(animation.frameCount, sizeof(Vector3));
    Vector3* rightToeLocations = RL_CALLOC(animation.frameCount, sizeof(Vector3));
    
    for (int i = 0; i < animation.frameCount; i++)
    {
        pelvisLocations[i] = animation.framePoses[i][pelvisBoneIndex].translation;
        leftToeLocations[i] = animation.framePoses[i][leftToeBoneIndex].translation;
        rightToeLocations[i] = animation.framePoses[i][rightToeBoneIndex].translation;
    }
    
    float softFactor = 0.05f;
    float hardFactor = 0.9f;
    
    int iterations = 25000;

    for (int iteration = 0; iteration < iterations; iteration++)
    {
        for (int i = 0; i < animation.frameCount; i++)
        {
            // Left Leg Inter-Frame Constraints
            
            {
                if (i > 0)
                {
                    // Get the toe and hip positions in the input animation
                    
                    Vector3 restPrevToe = animation.framePoses[i - 1][leftToeBoneIndex].translation;
                    Vector3 restCurrToe = animation.framePoses[i - 0][leftToeBoneIndex].translation;
                    Vector3 restPrevHip = animation.framePoses[i - 1][pelvisBoneIndex].translation;
                    Vector3 restCurrHip = animation.framePoses[i - 0][pelvisBoneIndex].translation;

                    // Get the modified toe and hip positions

                    Vector3 consPrevToe = leftToeLocations[i - 1];
                    Vector3 consCurrToe = leftToeLocations[i - 0];
                    Vector3 consPrevHip = pelvisLocations[i - 1];
                    Vector3 consCurrHip = pelvisLocations[i - 0];

                    // If the contact is active on both previous and current frames

                    if (contacts.leftContacts[i - 1] > contactThreshold && 
                        contacts.leftContacts[i - 0] > contactThreshold)
                    {
                        // Compute the mid-point between the current contacts and set the 
                        // height to the ground height
                        
                        Vector3 toeTarget = Vector3Lerp(consPrevToe, consCurrToe, 0.5f);
                        toeTarget.y = toeMinHeight;

                        // Move the locations toward the mid-point
                    
                        leftToeLocations[i - 1] = Vector3Lerp(consPrevToe, toeTarget, hardFactor);
                        leftToeLocations[i - 0] = Vector3Lerp(consCurrToe, toeTarget, hardFactor);
                    }
                    else
                    {
                        // Find the target toe locations relative to each other and remove
                        // any ground penetration
                        
                        Vector3 prevToeTarget = Vector3Add(consCurrToe, 
                            Vector3Subtract(restPrevToe, restCurrToe));
                        Vector3 currToeTarget = Vector3Add(consPrevToe, 
                            Vector3Subtract(restCurrToe, restPrevToe));
                        prevToeTarget.y = Max(prevToeTarget.y, toeMinHeight);
                        currToeTarget.y = Max(currToeTarget.y, toeMinHeight);

                        // Softly move toward their targets to bias it toward the source animation

                        leftToeLocations[i - 1] = Vector3Lerp(consPrevToe, prevToeTarget, softFactor);
                        leftToeLocations[i - 0] = Vector3Lerp(consCurrToe, currToeTarget, softFactor);
                    }
                    
                    // Softly move the pelvis toward the correct location from the source animation

                    pelvisLocations[i - 1] = Vector3Lerp(consPrevHip, Vector3Add(consCurrHip, Vector3Subtract(restPrevHip, restCurrHip)), softFactor);
                    pelvisLocations[i - 0] = Vector3Lerp(consCurrHip, Vector3Add(consPrevHip, Vector3Subtract(restCurrHip, restPrevHip)), softFactor);
                }

                // Left Leg In-Frame Constraints
                
                // Find the distance from the hip to the toe in the source animation
                
                Vector3 restHip = animation.framePoses[i][pelvisBoneIndex].translation;
                Vector3 restToe = animation.framePoses[i][leftToeBoneIndex].translation;
                float restLength = Vector3Distance(restHip, restToe);

                // Find the current direction from the hip to the toe

                Vector3 currHip = pelvisLocations[i];
                Vector3 currToe = leftToeLocations[i];
                Vector3 currDirection = Vector3Normalize(Vector3Subtract(currHip, currToe));

                // Softly enforce the hip-to-toe length from the source animation

                pelvisLocations[i] = Vector3Lerp(currHip, Vector3Add(
                    currToe, Vector3Scale(currDirection, +restLength)), softFactor);
                leftToeLocations[i] = Vector3Lerp(currToe, Vector3Add(
                    currHip, Vector3Scale(currDirection, -restLength)), softFactor);
            }
            
            // Right Leg Inter-Frame Constraints
            
            {
                if (i > 0)
                {
                    Vector3 restPrevToe = animation.framePoses[i - 1][rightToeBoneIndex].translation;
                    Vector3 restCurrToe = animation.framePoses[i - 0][rightToeBoneIndex].translation;
                    Vector3 restPrevHip = animation.framePoses[i - 1][pelvisBoneIndex].translation;
                    Vector3 restCurrHip = animation.framePoses[i - 0][pelvisBoneIndex].translation;

                    Vector3 consPrevToe = rightToeLocations[i - 1];
                    Vector3 consCurrToe = rightToeLocations[i - 0];
                    Vector3 consPrevHip = pelvisLocations[i - 1];
                    Vector3 consCurrHip = pelvisLocations[i - 0];

                    if (contacts.rightContacts[i - 1] > contactThreshold && 
                        contacts.rightContacts[i - 0] > contactThreshold)
                    {
                        Vector3 toeTarget = Vector3Lerp(consPrevToe, consCurrToe, 0.5f);
                        toeTarget.y = toeMinHeight;

                        rightToeLocations[i - 1] = Vector3Lerp(consPrevToe, toeTarget, hardFactor);
                        rightToeLocations[i - 0] = Vector3Lerp(consCurrToe, toeTarget, hardFactor);
                    }
                    else
                    {
                        Vector3 prevToeTarget = Vector3Add(consCurrToe, 
                            Vector3Subtract(restPrevToe, restCurrToe));
                        Vector3 currToeTarget = Vector3Add(consPrevToe, 
                            Vector3Subtract(restCurrToe, restPrevToe));
                        prevToeTarget.y = Max(prevToeTarget.y, toeMinHeight);
                        currToeTarget.y = Max(currToeTarget.y, toeMinHeight);

                        rightToeLocations[i - 1] = Vector3Lerp(consPrevToe, prevToeTarget, softFactor);
                        rightToeLocations[i - 0] = Vector3Lerp(consCurrToe, currToeTarget, softFactor);
                    }

                    Vector3 prevHipTarget = Vector3Add(consCurrHip, Vector3Subtract(restPrevHip, restCurrHip));
                    Vector3 currHipTarget = Vector3Add(consPrevHip, Vector3Subtract(restCurrHip, restPrevHip));

                    pelvisLocations[i - 1] = Vector3Lerp(consPrevHip, prevHipTarget, softFactor);
                    pelvisLocations[i - 0] = Vector3Lerp(consCurrHip, currHipTarget, softFactor);
                }

                // Right Leg In-Frame Constraints

                Vector3 restHip = animation.framePoses[i][pelvisBoneIndex].translation;
                Vector3 restToe = animation.framePoses[i][rightToeBoneIndex].translation;
                float restLength = Vector3Distance(restHip, restToe);

                Vector3 currHip = pelvisLocations[i];
                Vector3 currToe = rightToeLocations[i];
                Vector3 currDirection = Vector3Normalize(Vector3Subtract(currHip, currToe));

                pelvisLocations[i] = Vector3Lerp(currHip, Vector3Add(
                    currToe, Vector3Scale(currDirection, restLength)), softFactor);
                rightToeLocations[i] = Vector3Lerp(currToe, Vector3Add(
                    currHip, Vector3Scale(currDirection, -restLength)), softFactor);
            }
        }
    }
    
    Transform* localTransforms = RL_CALLOC(animation.boneCount, sizeof(Transform));
    Transform* globalTransforms = RL_CALLOC(animation.boneCount, sizeof(Transform));
    
    for (int i = 0; i < animation.frameCount; i++)
    {
        BackwardKinematics(localTransforms, animation.framePoses[i], model);
        
        assert(pelvisBoneIndex == 0);
        localTransforms[pelvisBoneIndex].translation = pelvisLocations[i];
        
        ForwardKinematics(globalTransforms, localTransforms, model);
        
        Vector3 leftTargetHeel, leftTargetToe, leftTargetToeEnd;
        
        SolveLegChain(
            model,
            localTransforms,
            globalTransforms,
            leftToeLocations[i],
            &leftTargetHeel,
            &leftTargetToe,
            &leftTargetToeEnd,
            pelvisBoneIndex,
            leftHipBoneIndex,
            leftKneeBoneIndex,
            leftHeelBoneIndex,
            leftToeBoneIndex,
            leftToeEndBoneIndex,
            enableHeightClamp,
            enableHeelLookAt,
            enableToeLookAt,
            heelMinHeight,
            toeMinHeight,
            toeEndMinHeight,
            softening,
            leftKneeSideVector);
        
        Vector3 rightTargetHeel, rightTargetToe, rightTargetToeEnd;

        SolveLegChain(
            model,
            localTransforms,
            globalTransforms,
            rightToeLocations[i],
            &rightTargetHeel,
            &rightTargetToe,
            &rightTargetToeEnd,
            pelvisBoneIndex,
            rightHipBoneIndex,
            rightKneeBoneIndex,
            rightHeelBoneIndex,
            rightToeBoneIndex,
            rightToeEndBoneIndex,
            enableHeightClamp,
            enableHeelLookAt,
            enableToeLookAt,
            heelMinHeight,
            toeMinHeight,
            toeEndMinHeight,
            softening,
            rightKneeSideVector);
        
        ForwardKinematics(animation.framePoses[i], localTransforms, model);
    }
    
    RL_FREE(localTransforms);
    RL_FREE(globalTransforms);
    
    RL_FREE(pelvisLocations);
    RL_FREE(leftToeLocations);
    RL_FREE(rightToeLocations);
}


//----------------------------------------------------------------------------------
// App
//----------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    // Init Window
    
    // const int screenWidth = 640;
    // const int screenHeight = 380;
    const int screenWidth = 1280;
    const int screenHeight = 720;
    
    SetConfigFlags(FLAG_VSYNC_HINT);
    InitWindow(screenWidth, screenHeight, "GenoView");
    SetTargetFPS(60);

    // Shaders
    
    Shader shadowShader = LoadShader("./resources/shadow.vs", "./resources/shadow.fs");
    int shadowShaderLightClipNear = GetShaderLocation(shadowShader, "lightClipNear");
    int shadowShaderLightClipFar = GetShaderLocation(shadowShader, "lightClipFar");
    
    Shader skinnedShadowShader = LoadShader("./resources/skinnedShadow.vs", "./resources/shadow.fs");
    int skinnedShadowShaderLightClipNear = GetShaderLocation(skinnedShadowShader, "lightClipNear");
    int skinnedShadowShaderLightClipFar = GetShaderLocation(skinnedShadowShader, "lightClipFar");
    
    Shader skinnedBasicShader = LoadShader("./resources/skinnedBasic.vs", "./resources/basic.fs");
    int skinnedBasicShaderSpecularity = GetShaderLocation(skinnedBasicShader, "specularity");
    int skinnedBasicShaderGlossiness = GetShaderLocation(skinnedBasicShader, "glossiness");
    int skinnedBasicShaderCamClipNear = GetShaderLocation(skinnedBasicShader, "camClipNear");
    int skinnedBasicShaderCamClipFar = GetShaderLocation(skinnedBasicShader, "camClipFar");

    Shader basicShader = LoadShader("./resources/basic.vs", "./resources/basic.fs");
    int basicShaderSpecularity = GetShaderLocation(basicShader, "specularity");
    int basicShaderGlossiness = GetShaderLocation(basicShader, "glossiness");
    int basicShaderCamClipNear = GetShaderLocation(basicShader, "camClipNear");
    int basicShaderCamClipFar = GetShaderLocation(basicShader, "camClipFar");
    
    Shader lightingShader = LoadShader("./resources/post.vs", "./resources/lighting.fs");
    int lightingShaderGBufferColor = GetShaderLocation(lightingShader, "gbufferColor");
    int lightingShaderGBufferNormal = GetShaderLocation(lightingShader, "gbufferNormal");
    int lightingShaderGBufferDepth = GetShaderLocation(lightingShader, "gbufferDepth");
    int lightingShaderSSAO = GetShaderLocation(lightingShader, "ssao");
    int lightingShaderCamPos = GetShaderLocation(lightingShader, "camPos");
    int lightingShaderCamInvViewProj = GetShaderLocation(lightingShader, "camInvViewProj");
    int lightingShaderLightDir = GetShaderLocation(lightingShader, "lightDir");
    int lightingShaderSunColor = GetShaderLocation(lightingShader, "sunColor");
    int lightingShaderSunStrength = GetShaderLocation(lightingShader, "sunStrength");
    int lightingShaderSkyColor = GetShaderLocation(lightingShader, "skyColor");
    int lightingShaderSkyStrength = GetShaderLocation(lightingShader, "skyStrength");
    int lightingShaderGroundStrength = GetShaderLocation(lightingShader, "groundStrength");
    int lightingShaderAmbientStrength = GetShaderLocation(lightingShader, "ambientStrength");
    int lightingShaderExposure = GetShaderLocation(lightingShader, "exposure");
    int lightingShaderCamClipNear = GetShaderLocation(lightingShader, "camClipNear");
    int lightingShaderCamClipFar = GetShaderLocation(lightingShader, "camClipFar");
    
    Shader ssaoShader = LoadShader("./resources/post.vs", "./resources/ssao.fs");
    int ssaoShaderGBufferNormal = GetShaderLocation(ssaoShader, "gbufferNormal");
    int ssaoShaderGBufferDepth = GetShaderLocation(ssaoShader, "gbufferDepth");
    int ssaoShaderCamView = GetShaderLocation(ssaoShader, "camView");
    int ssaoShaderCamProj = GetShaderLocation(ssaoShader, "camProj");
    int ssaoShaderCamInvProj = GetShaderLocation(ssaoShader, "camInvProj");
    int ssaoShaderCamInvViewProj = GetShaderLocation(ssaoShader, "camInvViewProj");
    int ssaoShaderLightViewProj = GetShaderLocation(ssaoShader, "lightViewProj");
    int ssaoShaderShadowMap = GetShaderLocation(ssaoShader, "shadowMap");
    int ssaoShaderShadowInvResolution = GetShaderLocation(ssaoShader, "shadowInvResolution");
    int ssaoShaderCamClipNear = GetShaderLocation(ssaoShader, "camClipNear");
    int ssaoShaderCamClipFar = GetShaderLocation(ssaoShader, "camClipFar");
    int ssaoShaderLightClipNear = GetShaderLocation(ssaoShader, "lightClipNear");
    int ssaoShaderLightClipFar = GetShaderLocation(ssaoShader, "lightClipFar");
    int ssaoShaderLightDir = GetShaderLocation(ssaoShader, "lightDir");
    
    Shader blurShader = LoadShader("./resources/post.vs", "./resources/blur.fs");
    int blurShaderGBufferNormal = GetShaderLocation(blurShader, "gbufferNormal");
    int blurShaderGBufferDepth = GetShaderLocation(blurShader, "gbufferDepth");
    int blurShaderInputTexture = GetShaderLocation(blurShader, "inputTexture");
    int blurShaderCamInvProj = GetShaderLocation(blurShader, "camInvProj");
    int blurShaderCamClipNear = GetShaderLocation(blurShader, "camClipNear");
    int blurShaderCamClipFar = GetShaderLocation(blurShader, "camClipFar");
    int blurShaderInvTextureResolution = GetShaderLocation(blurShader, "invTextureResolution");
    int blurShaderBlurDirection = GetShaderLocation(blurShader, "blurDirection");

    Shader fxaaShader = LoadShader("./resources/post.vs", "./resources/fxaa.fs");
    int fxaaShaderInputTexture = GetShaderLocation(fxaaShader, "inputTexture");
    int fxaaShaderInvTextureResolution = GetShaderLocation(fxaaShader, "invTextureResolution");
    
    // Objects
    
    Mesh groundMesh = GenMeshPlane(20.0f, 20.0f, 10, 10);
    Model groundModel = LoadModelFromMesh(groundMesh);
    Vector3 groundPosition = (Vector3){ 0.0f, -0.01f, 0.0f };
    
    Model genoModel = LoadGenoModel("./resources/Geno.bin");
    Vector3 genoPosition = (Vector3){ 0.0f, 0.0f, 0.0f };

    // Animation
    
    ModelAnimation testAnimation = LoadGenoModelAnimation("./resources/run2_subject4.bin");    
    ModelAnimationContacts testContacts = LoadModelAnimationContacts("./resources/run2_subject4_contacts.bin");
    // ModelAnimation testAnimation = LoadGenoModelAnimation("./resources/walk2_subject4.bin");    
    // ModelAnimationContacts testContacts = LoadModelAnimationContacts("./resources/walk2_subject4_contacts.bin");

    assert(testAnimation.boneCount == genoModel.boneCount);
    assert(testContacts.frameCount == testAnimation.frameCount);

    // int animationFrame = 0;
    int animationFrame = 200;    
    
    // Camera
    
    OrbitCamera camera;
    OrbitCameraInit(&camera);
    
    rlSetClipPlanes(0.01f, 50.0f);
    
    // Shadows
    
    Vector3 lightDir = Vector3Normalize((Vector3){ 0.35f, -1.0f, -0.35f });
    
    ShadowLight shadowLight = (ShadowLight){ 0 };
    shadowLight.target = Vector3Zero();
    shadowLight.position = Vector3Scale(lightDir, -5.0f);
    shadowLight.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    shadowLight.width = 5.0f;
    shadowLight.height = 5.0f;
    shadowLight.near = 0.01f;
    shadowLight.far = 10.0f;
    
    int shadowWidth = 1024;
    int shadowHeight = 1024;
    Vector2 shadowInvResolution = (Vector2){ 1.0f / shadowWidth, 1.0f / shadowHeight };
    RenderTexture2D shadowMap = LoadShadowMap(shadowWidth, shadowHeight);    
    
    // GBuffer and Render Textures
    
    GBuffer gbuffer = LoadGBuffer(screenWidth, screenHeight);
    RenderTexture2D lighted = LoadRenderTexture(screenWidth, screenHeight);
    RenderTexture2D ssaoFront = LoadRenderTexture(screenWidth, screenHeight);
    RenderTexture2D ssaoBack = LoadRenderTexture(screenWidth, screenHeight);
    
    // Animation
    
    int pelvisBoneIndex = FindModelBoneIndex(genoModel, "Hips");
    
    int leftHipBoneIndex = FindModelBoneIndex(genoModel, "LeftUpLeg");
    int leftKneeBoneIndex = FindModelBoneIndex(genoModel, "LeftLeg");
    int leftHeelBoneIndex = FindModelBoneIndex(genoModel, "LeftFoot");
    int leftToeBoneIndex = FindModelBoneIndex(genoModel, "LeftToeBase");
    int leftToeEndBoneIndex = FindModelBoneIndex(genoModel, "LeftToeBaseEnd");
    
    int rightHipBoneIndex = FindModelBoneIndex(genoModel, "RightUpLeg");
    int rightKneeBoneIndex = FindModelBoneIndex(genoModel, "RightLeg");
    int rightHeelBoneIndex = FindModelBoneIndex(genoModel, "RightFoot");
    int rightToeBoneIndex = FindModelBoneIndex(genoModel, "RightToeBase");
    int rightToeEndBoneIndex = FindModelBoneIndex(genoModel, "RightToeBaseEnd");
    
    Transform *localTransforms = RL_CALLOC(genoModel.boneCount, sizeof(Transform));
    Transform *globalTransforms = RL_CALLOC(genoModel.boneCount, sizeof(Transform));
    Transform *modifyTransforms = RL_CALLOC(genoModel.boneCount, sizeof(Transform));
    
    BackwardKinematics(localTransforms, genoModel.bindPose, genoModel);
    memcpy(modifyTransforms, localTransforms, genoModel.boneCount * sizeof(Transform));
    ForwardKinematics(globalTransforms, modifyTransforms, genoModel);

    float heelMinHeight = globalTransforms[leftHeelBoneIndex].translation.y;
    float toeMinHeight = globalTransforms[leftToeBoneIndex].translation.y;
    float toeEndMinHeight = globalTransforms[leftToeEndBoneIndex].translation.y;
    
    BackwardKinematics(localTransforms, testAnimation.framePoses[animationFrame], genoModel);
    memcpy(modifyTransforms, localTransforms, genoModel.boneCount * sizeof(Transform));
    ForwardKinematics(globalTransforms, modifyTransforms, genoModel);

    Vector3 leftTarget = globalTransforms[leftToeBoneIndex].translation;
    Vector3 leftTargetHeel = globalTransforms[leftHeelBoneIndex].translation;
    Vector3 leftTargetToe = globalTransforms[leftToeBoneIndex].translation;
    Vector3 leftTargetToeEnd = globalTransforms[leftToeEndBoneIndex].translation;
    
    Vector3 rightTarget = globalTransforms[rightToeBoneIndex].translation;
    Vector3 rightTargetHeel = globalTransforms[rightHeelBoneIndex].translation;
    Vector3 rightTargetToe = globalTransforms[rightToeBoneIndex].translation;
    Vector3 rightTargetToeEnd = globalTransforms[rightToeEndBoneIndex].translation;
    
    // UI
    
    bool drawBoneTransforms = false;
    bool drawLeg = true;
    bool drawDebug = true;
    // bool drawDebug = false;
    bool drawTarget = true;
    // bool drawTarget = false;
    bool drawContactStates = true;
    bool drawUI = false;
    float cameraHeight = 0.25f;
    // float cameraHeight = 0.75f;
    
    bool enableInverseKinematics = true;
    // bool enableInverseKinematics = false;
    float pelvisOffset = 0.0f;
    // float pelvisOffset = 0.1;
    // float pelvisOffset = 0.05;
    float softening = 0.005f; 
    bool enableHeelLookAt = true;
    // bool enableHeelLookAt = false;
    bool enableToeLookAt = true;
    // bool enableToeLookAt = false;
    bool enableHeightClamp = true;
    // bool enableHeightClamp = false;
    
    float contactThreshold = 0.75f;
    float unlockDistance = 0.25f;
    float lockDistance = 0.01f;
    float lockBlendTime = 0.5f;
    
    // Modes
    
    bool manualInteraction = true;
    // bool manualInteraction = false;
    // bool offlineCleanup = true;
    bool offlineCleanup = false;
    
    // Inertialization State
    
    FootLockingState leftLockState, rightLockState;
    ResetFootLockingState(&leftLockState, globalTransforms[leftToeBoneIndex].translation);
    ResetFootLockingState(&rightLockState, globalTransforms[rightToeBoneIndex].translation);
    
    // Offline Foot Sliding Removal
    
    if (offlineCleanup)
    {
        RemoveFootSliding(
            genoModel,
            testAnimation,
            testContacts,
            pelvisBoneIndex,
            leftHipBoneIndex,
            leftKneeBoneIndex,
            leftHeelBoneIndex,
            leftToeBoneIndex,
            leftToeEndBoneIndex,
            rightHipBoneIndex,
            rightKneeBoneIndex,
            rightHeelBoneIndex,
            rightToeBoneIndex,
            rightToeEndBoneIndex,
            enableHeightClamp,
            enableHeelLookAt,
            enableToeLookAt,
            heelMinHeight,
            toeMinHeight,
            toeEndMinHeight,
            softening,
            contactThreshold,
            (Vector3){ 1.0f, 0.0f, 0.0f },
            (Vector3){ 1.0f, 0.0f, 0.0f });    
    }
    
    // Go
    
    while (!WindowShouldClose())
    {
        float deltaTime = 1.0f / 60.0f;
        
        if (!manualInteraction || IsKeyPressed(KEY_RIGHT))
        {
            animationFrame = (animationFrame + 1) % testAnimation.frameCount;
        }
        
        // Copy in Animation Pose
        
        BackwardKinematics(localTransforms, testAnimation.framePoses[animationFrame], genoModel);
        
        // Inverse Kinematics
        
        memcpy(modifyTransforms, localTransforms, genoModel.boneCount * sizeof(Transform));
        
        // Apply Pelvis Offset
        
        modifyTransforms[pelvisBoneIndex].translation.y -= pelvisOffset;
        
        ForwardKinematics(globalTransforms, modifyTransforms, genoModel);

        // Move and Clamp Toe Target

        if (manualInteraction && IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
        {
            Ray clickRay = GetScreenToWorldRay(GetMousePosition(), camera.cam3d);
            
            leftTarget = Vector3Add(clickRay.position, 
                Vector3Scale(clickRay.direction, 
                Vector3Distance(leftTarget, camera.cam3d.position)));
        }
        
        if (manualInteraction && enableInverseKinematics)
        {
            SolveLegChain(
                genoModel,
                modifyTransforms,
                globalTransforms,
                leftTarget,
                &leftTargetHeel,
                &leftTargetToe,
                &leftTargetToeEnd,
                pelvisBoneIndex,
                leftHipBoneIndex,
                leftKneeBoneIndex,
                leftHeelBoneIndex,
                leftToeBoneIndex,
                leftToeEndBoneIndex,
                enableHeightClamp,
                enableHeelLookAt,
                enableToeLookAt,
                heelMinHeight,
                toeMinHeight,
                toeEndMinHeight,
                softening,
                (Vector3){ 1.0f, 0.0f, 0.0f });
        }
        
        if (!manualInteraction)
        {
            UpdateFootLockingState(
                &leftLockState, 
                globalTransforms[leftToeBoneIndex].translation, 
                testContacts.leftContacts[animationFrame] > contactThreshold,
                toeMinHeight,
                deltaTime,
                unlockDistance,
                lockDistance,
                lockBlendTime);
            
            leftTarget = leftLockState.position;
            
            if (enableInverseKinematics)
            {
                SolveLegChain(
                    genoModel,
                    modifyTransforms,
                    globalTransforms,
                    leftTarget,
                    &leftTargetHeel,
                    &leftTargetToe,
                    &leftTargetToeEnd,
                    pelvisBoneIndex,
                    leftHipBoneIndex,
                    leftKneeBoneIndex,
                    leftHeelBoneIndex,
                    leftToeBoneIndex,
                    leftToeEndBoneIndex,
                    enableHeightClamp,
                    enableHeelLookAt,
                    enableToeLookAt,
                    heelMinHeight,
                    toeMinHeight,
                    toeEndMinHeight,
                    softening,
                    (Vector3){ 1.0f, 0.0f, 0.0f });  
            }
                
            UpdateFootLockingState(
                &rightLockState, 
                globalTransforms[rightToeBoneIndex].translation, 
                testContacts.rightContacts[animationFrame] > contactThreshold,
                toeMinHeight,
                deltaTime,
                unlockDistance,
                lockDistance,
                lockBlendTime);
                
            rightTarget = rightLockState.position;
            
            if (enableInverseKinematics)
            {
                SolveLegChain(
                    genoModel,
                    modifyTransforms,
                    globalTransforms,
                    rightTarget,
                    &rightTargetHeel,
                    &rightTargetToe,
                    &rightTargetToeEnd,
                    pelvisBoneIndex,
                    rightHipBoneIndex,
                    rightKneeBoneIndex,
                    rightHeelBoneIndex,
                    rightToeBoneIndex,
                    rightToeEndBoneIndex,
                    enableHeightClamp,
                    enableHeelLookAt,
                    enableToeLookAt,
                    heelMinHeight,
                    toeMinHeight,
                    toeEndMinHeight,
                    softening,
                    (Vector3){ 1.0f, 0.0f, 0.0f });
            }
        }
        
        // Update bone transforms
        
        UpdateModelPoseFromTransforms(genoModel, globalTransforms);
        
        // Shadow Light Tracks Character
        
        Vector3 hipPosition = globalTransforms[pelvisBoneIndex].translation;
        
        shadowLight.target = (Vector3){ hipPosition.x, 0.0f, hipPosition.z };
        shadowLight.position = Vector3Add(shadowLight.target, Vector3Scale(lightDir, -5.0f));

        // Update Camera
        
        OrbitCameraUpdate(
            &camera,
            (Vector3){ hipPosition.x, cameraHeight, hipPosition.z },
            (IsKeyDown(KEY_LEFT_CONTROL) && IsMouseButtonDown(0)) ? GetMouseDelta().x : 0.0f,
            (IsKeyDown(KEY_LEFT_CONTROL) && IsMouseButtonDown(0)) ? GetMouseDelta().y : 0.0f,
            (IsKeyDown(KEY_LEFT_CONTROL) && IsMouseButtonDown(1)) ? GetMouseDelta().x : 0.0f,
            (IsKeyDown(KEY_LEFT_CONTROL) && IsMouseButtonDown(1)) ? GetMouseDelta().y : 0.0f,
            GetMouseWheelMove(),
            deltaTime);
        
        // Render
        
        rlDisableColorBlend();
        
        BeginDrawing();
        
        // Render Shadow Maps
        
        BeginShadowMap(shadowMap, shadowLight);  
        
        Matrix lightViewProj = MatrixMultiply(rlGetMatrixModelview(), rlGetMatrixProjection());
        float lightClipNear = rlGetCullDistanceNear();
        float lightClipFar = rlGetCullDistanceFar();
        
        SetShaderValue(shadowShader, shadowShaderLightClipNear, &lightClipNear, SHADER_UNIFORM_FLOAT);
        SetShaderValue(shadowShader, shadowShaderLightClipFar, &lightClipFar, SHADER_UNIFORM_FLOAT);
        SetShaderValue(skinnedShadowShader, skinnedShadowShaderLightClipNear, &lightClipNear, SHADER_UNIFORM_FLOAT);
        SetShaderValue(skinnedShadowShader, skinnedShadowShaderLightClipFar, &lightClipFar, SHADER_UNIFORM_FLOAT);
        
        groundModel.materials[0].shader = shadowShader;
        DrawModel(groundModel, groundPosition, 1.0f, WHITE);
        
        genoModel.materials[0].shader = skinnedShadowShader;
        DrawModel(genoModel, genoPosition, 1.0f, WHITE);
        
        EndShadowMap();
        
        // Render GBuffer
        
        BeginGBuffer(gbuffer, camera.cam3d);
        
        Matrix camView = rlGetMatrixModelview();
        Matrix camProj = rlGetMatrixProjection();
        Matrix camInvProj = MatrixInvert(camProj);
        Matrix camInvViewProj = MatrixInvert(MatrixMultiply(camView, camProj));
        float camClipNear = rlGetCullDistanceNear();
        float camClipFar = rlGetCullDistanceFar();

        float specularity = 0.5f;
        float glossiness = 10.0f;        
        
        SetShaderValue(basicShader, basicShaderSpecularity, &specularity, SHADER_UNIFORM_FLOAT);
        SetShaderValue(basicShader, basicShaderGlossiness, &glossiness, SHADER_UNIFORM_FLOAT);
        SetShaderValue(basicShader, basicShaderCamClipNear, &camClipNear, SHADER_UNIFORM_FLOAT);
        SetShaderValue(basicShader, basicShaderCamClipFar, &camClipFar, SHADER_UNIFORM_FLOAT);
        
        SetShaderValue(skinnedBasicShader, skinnedBasicShaderSpecularity, &specularity, SHADER_UNIFORM_FLOAT);
        SetShaderValue(skinnedBasicShader, skinnedBasicShaderGlossiness, &glossiness, SHADER_UNIFORM_FLOAT);
        SetShaderValue(skinnedBasicShader, skinnedBasicShaderCamClipNear, &camClipNear, SHADER_UNIFORM_FLOAT);
        SetShaderValue(skinnedBasicShader, skinnedBasicShaderCamClipFar, &camClipFar, SHADER_UNIFORM_FLOAT);        
        
        groundModel.materials[0].shader = basicShader;
        DrawModel(groundModel, groundPosition, 1.0f, (Color){ 190, 190, 190, 255 });
        
        genoModel.materials[0].shader = skinnedBasicShader;
        DrawModel(genoModel, genoPosition, 1.0f, ORANGE);       
        
        EndGBuffer(screenWidth, screenHeight);
        
        // Render SSAO and Shadows
        
        BeginTextureMode(ssaoFront);
        
        BeginShaderMode(ssaoShader);
        
        SetShaderValueTexture(ssaoShader, ssaoShaderGBufferNormal, gbuffer.normal);
        SetShaderValueTexture(ssaoShader, ssaoShaderGBufferDepth, gbuffer.depth);
        SetShaderValueMatrix(ssaoShader, ssaoShaderCamView, camView);
        SetShaderValueMatrix(ssaoShader, ssaoShaderCamProj, camProj);
        SetShaderValueMatrix(ssaoShader, ssaoShaderCamInvProj, camInvProj);
        SetShaderValueMatrix(ssaoShader, ssaoShaderCamInvViewProj, camInvViewProj);
        SetShaderValueMatrix(ssaoShader, ssaoShaderLightViewProj, lightViewProj);
        SetShaderValueShadowMap(ssaoShader, ssaoShaderShadowMap, shadowMap);
        SetShaderValue(ssaoShader, ssaoShaderShadowInvResolution, &shadowInvResolution, SHADER_UNIFORM_VEC2);
        SetShaderValue(ssaoShader, ssaoShaderCamClipNear, &camClipNear, SHADER_UNIFORM_FLOAT);
        SetShaderValue(ssaoShader, ssaoShaderCamClipFar, &camClipFar, SHADER_UNIFORM_FLOAT);
        SetShaderValue(ssaoShader, ssaoShaderLightClipNear, &lightClipNear, SHADER_UNIFORM_FLOAT);
        SetShaderValue(ssaoShader, ssaoShaderLightClipFar, &lightClipFar, SHADER_UNIFORM_FLOAT);
        SetShaderValue(ssaoShader, ssaoShaderLightDir, &lightDir, SHADER_UNIFORM_VEC3);
        
        ClearBackground(WHITE);
        
        DrawTextureRec(
            ssaoFront.texture,
            (Rectangle){ 0, 0, ssaoFront.texture.width, -ssaoFront.texture.height },
            (Vector2){ 0, 0 },
            WHITE);

        EndShaderMode();

        EndTextureMode();
        
        // Blur Horizontal
        
        BeginTextureMode(ssaoBack);
        
        BeginShaderMode(blurShader);
        
        Vector2 blurDirection = (Vector2){ 1.0f, 0.0f };
        Vector2 blurInvTextureResolution = (Vector2){ 1.0f / ssaoFront.texture.width, 1.0f / ssaoFront.texture.height };
        
        SetShaderValueTexture(blurShader, blurShaderGBufferNormal, gbuffer.normal);
        SetShaderValueTexture(blurShader, blurShaderGBufferDepth, gbuffer.depth);
        SetShaderValueTexture(blurShader, blurShaderInputTexture, ssaoFront.texture);
        SetShaderValueMatrix(blurShader, blurShaderCamInvProj, camInvProj);
        SetShaderValue(blurShader, blurShaderCamClipNear, &camClipNear, SHADER_UNIFORM_FLOAT);
        SetShaderValue(blurShader, blurShaderCamClipFar, &camClipFar, SHADER_UNIFORM_FLOAT);
        SetShaderValue(blurShader, blurShaderInvTextureResolution, &blurInvTextureResolution, SHADER_UNIFORM_VEC2);
        SetShaderValue(blurShader, blurShaderBlurDirection, &blurDirection, SHADER_UNIFORM_VEC2);

        DrawTextureRec(
            ssaoBack.texture,
            (Rectangle){ 0, 0, ssaoBack.texture.width, -ssaoBack.texture.height },
            (Vector2){ 0, 0 },
            WHITE);

        EndShaderMode();

        EndTextureMode();
      
        // Blur Vertical
        
        BeginTextureMode(ssaoFront);
        
        BeginShaderMode(blurShader);
        
        blurDirection = (Vector2){ 0.0f, 1.0f };
        
        SetShaderValueTexture(blurShader, blurShaderInputTexture, ssaoBack.texture);
        SetShaderValue(blurShader, blurShaderBlurDirection, &blurDirection, SHADER_UNIFORM_VEC2);

        DrawTextureRec(
            ssaoFront.texture,
            (Rectangle){ 0, 0, ssaoFront.texture.width, -ssaoFront.texture.height },
            (Vector2){ 0, 0 },
            WHITE);

        EndShaderMode();

        EndTextureMode();
      
        // Light GBuffer
        
        BeginTextureMode(lighted);
        
        BeginShaderMode(lightingShader);
        
        Vector3 sunColor = (Vector3){ 253.0f / 255.0f, 255.0f / 255.0f, 232.0f / 255.0f };
        float sunStrength = 0.25f;
        Vector3 skyColor = (Vector3){ 174.0f / 255.0f, 183.0f / 255.0f, 190.0f / 255.0f };
        float skyStrength = 0.15f;
        float groundStrength = 0.1f;
        float ambientStrength = 1.0f;
        float exposure = 0.9f;
        
        SetShaderValueTexture(lightingShader, lightingShaderGBufferColor, gbuffer.color);
        SetShaderValueTexture(lightingShader, lightingShaderGBufferNormal, gbuffer.normal);
        SetShaderValueTexture(lightingShader, lightingShaderGBufferDepth, gbuffer.depth);
        SetShaderValueTexture(lightingShader, lightingShaderSSAO, ssaoFront.texture);
        SetShaderValue(lightingShader, lightingShaderCamPos, &camera.cam3d.position, SHADER_UNIFORM_VEC3);
        SetShaderValueMatrix(lightingShader, lightingShaderCamInvViewProj, camInvViewProj);
        SetShaderValue(lightingShader, lightingShaderLightDir, &lightDir, SHADER_UNIFORM_VEC3);
        SetShaderValue(lightingShader, lightingShaderSunColor, &sunColor, SHADER_UNIFORM_VEC3);
        SetShaderValue(lightingShader, lightingShaderSunStrength, &sunStrength, SHADER_UNIFORM_FLOAT);
        SetShaderValue(lightingShader, lightingShaderSkyColor, &skyColor, SHADER_UNIFORM_VEC3);
        SetShaderValue(lightingShader, lightingShaderSkyStrength, &skyStrength, SHADER_UNIFORM_FLOAT);
        SetShaderValue(lightingShader, lightingShaderGroundStrength, &groundStrength, SHADER_UNIFORM_FLOAT);
        SetShaderValue(lightingShader, lightingShaderAmbientStrength, &ambientStrength, SHADER_UNIFORM_FLOAT);
        SetShaderValue(lightingShader, lightingShaderExposure, &exposure, SHADER_UNIFORM_FLOAT);
        SetShaderValue(lightingShader, lightingShaderCamClipNear, &camClipNear, SHADER_UNIFORM_FLOAT);
        SetShaderValue(lightingShader, lightingShaderCamClipFar, &camClipFar, SHADER_UNIFORM_FLOAT);
        
        ClearBackground(RAYWHITE);
        
        DrawTextureRec(
            gbuffer.color,
            (Rectangle){ 0, 0, gbuffer.color.width, -gbuffer.color.height },
            (Vector2){ 0, 0 },
            WHITE);
        
        EndShaderMode();        
        
        // Debug Draw
        
        BeginMode3D(camera.cam3d);
        
        if (drawBoneTransforms)
        {
            DrawModelTransforms(globalTransforms, genoModel, GRAY);
        }
        
        if (drawLeg)
        {
            DrawLegTransforms(
                globalTransforms, 
                GRAY,
                leftHipBoneIndex,
                leftKneeBoneIndex,
                leftHeelBoneIndex,
                leftToeBoneIndex,
                leftToeEndBoneIndex);
                
            if (!manualInteraction)
            {
                DrawLegTransforms(
                    globalTransforms, 
                    GRAY,
                    rightHipBoneIndex,
                    rightKneeBoneIndex,
                    rightHeelBoneIndex,
                    rightToeBoneIndex,
                    rightToeEndBoneIndex);
            }
        }
        
        if (drawTarget)
        {
            DrawSphere(leftTarget, 0.0125f, GOLD);

            if (!manualInteraction)
            {
                DrawSphere(rightTarget, 0.0125f, GOLD);
            }
        }
        
        if (drawDebug)
        {
            DrawSphere(leftTargetHeel, 0.02f, PINK);
            DrawSphere(leftTargetToe, 0.02f, PINK);
            DrawCapsule(leftTargetToe, leftTargetHeel, 0.01f, 7, 7, PINK);
            
            DrawSphere(globalTransforms[leftHeelBoneIndex].translation, 0.015f, PURPLE);
            DrawSphere(globalTransforms[leftToeBoneIndex].translation, 0.015f, PURPLE);
            DrawSphere(globalTransforms[leftToeEndBoneIndex].translation, 0.015f, PURPLE);
            DrawCapsule(
                globalTransforms[leftHeelBoneIndex].translation, 
                globalTransforms[leftToeBoneIndex].translation, 0.0075f, 7, 7, PURPLE);
            DrawCapsule(
                globalTransforms[leftToeBoneIndex].translation, 
                globalTransforms[leftToeEndBoneIndex].translation, 0.0075f, 7, 7, PURPLE);
                
            if (!manualInteraction)
            {
                DrawSphere(rightTargetHeel, 0.02f, PINK);
                DrawSphere(rightTargetToe, 0.02f, PINK);
                DrawCapsule(rightTargetToe, rightTargetHeel, 0.01f, 7, 7, PINK);
                
                DrawSphere(globalTransforms[rightHeelBoneIndex].translation, 0.015f, PURPLE);
                DrawSphere(globalTransforms[rightToeBoneIndex].translation, 0.015f, PURPLE);
                DrawSphere(globalTransforms[rightToeEndBoneIndex].translation, 0.015f, PURPLE);
                DrawCapsule(
                    globalTransforms[rightHeelBoneIndex].translation, 
                    globalTransforms[rightToeBoneIndex].translation, 0.0075f, 7, 7, PURPLE);
                DrawCapsule(
                    globalTransforms[rightToeBoneIndex].translation, 
                    globalTransforms[rightToeEndBoneIndex].translation, 0.0075f, 7, 7, PURPLE);
                    
            }
        }
        
        if (!manualInteraction && drawContactStates)
        {
            DrawSphere(leftLockState.inputPosition, 0.015f, PURPLE);
            DrawSphere(rightLockState.inputPosition, 0.015f, PURPLE);
            
            if (drawTarget)
            {
                DrawSphere(leftTarget, leftLockState.locked ? 2.0f * 0.0125f : 0.0125f, GOLD);
                DrawSphere(rightTarget, rightLockState.locked ? 2.0f * 0.0125f : 0.0125f, GOLD);
            }
        }
            
        EndMode3D();

        EndTextureMode();
        
        // Render Final with FXAA
        
        BeginShaderMode(fxaaShader);

        Vector2 fxaaInvTextureResolution = (Vector2){ 1.0f / lighted.texture.width, 1.0f / lighted.texture.height };
        
        SetShaderValueTexture(fxaaShader, fxaaShaderInputTexture, lighted.texture);
        SetShaderValue(fxaaShader, fxaaShaderInvTextureResolution, &fxaaInvTextureResolution, SHADER_UNIFORM_VEC2);
        
        DrawTextureRec(
            lighted.texture,
            (Rectangle){ 0, 0, lighted.texture.width, -lighted.texture.height },
            (Vector2){ 0, 0 },
            WHITE);
        
        EndShaderMode();
  
        // UI
  
        rlEnableColorBlend();
        
        if (drawUI)
        {
            GuiGroupBox((Rectangle){ screenWidth - 260, 10, 240, 160 }, "Rendering");

            GuiCheckBox((Rectangle){ screenWidth - 250, 20, 20, 20 }, "Draw Transforms", &drawBoneTransforms);
            GuiCheckBox((Rectangle){ screenWidth - 250, 50, 20, 20 }, "Draw Leg", &drawLeg);
            GuiCheckBox((Rectangle){ screenWidth - 250, 80, 20, 20 }, "Draw Target", &drawTarget);
            GuiCheckBox((Rectangle){ screenWidth - 250, 110, 20, 20 }, "Draw Debug", &drawDebug);
            GuiCheckBox((Rectangle){ screenWidth - 250, 140, 20, 20 }, "Draw Contacts", &drawContactStates);

            GuiGroupBox((Rectangle){ screenWidth - 260, 180, 240, 310 }, "Inverse Kinematics");

            GuiCheckBox((Rectangle){ screenWidth - 250, 190, 20, 20 }, "Enable", &enableInverseKinematics);
            GuiSlider((Rectangle){ screenWidth - 250 + 90, 220, 100, 20 }, "Pelvis Offset", TextFormat("%4.4f", pelvisOffset), &pelvisOffset, 0.0f, 0.1f);
            GuiSlider((Rectangle){ screenWidth - 250 + 90, 250, 100, 20 }, "Softening", TextFormat("%4.4f", softening), &softening, 0.0f, 0.025f);
            GuiCheckBox((Rectangle){ screenWidth - 250, 280, 20, 20 }, "Heel Look-At", &enableHeelLookAt);
            GuiCheckBox((Rectangle){ screenWidth - 250, 310, 20, 20 }, "Toe Look-At", &enableToeLookAt);
            GuiCheckBox((Rectangle){ screenWidth - 250, 340, 20, 20 }, "Height Clamp", &enableHeightClamp);
            GuiSlider((Rectangle){ screenWidth - 250 + 90, 370, 100, 20 }, "Lock Threshold", TextFormat("%4.4f", contactThreshold), &contactThreshold, 0.0f, 1.0f);
            GuiSlider((Rectangle){ screenWidth - 250 + 90, 400, 100, 20 }, "Lock Blend Time", TextFormat("%4.4f", lockBlendTime), &lockBlendTime, 0.0f, 1.0f);
            GuiSlider((Rectangle){ screenWidth - 250 + 90, 430, 100, 20 }, "Unlock Distance", TextFormat("%4.4f", unlockDistance), &unlockDistance, 0.0f, 0.5f);
        }

        EndDrawing();
    }

    RL_FREE(localTransforms);
    RL_FREE(globalTransforms);
    RL_FREE(modifyTransforms);

    UnloadRenderTexture(lighted);
    UnloadRenderTexture(ssaoBack);
    UnloadRenderTexture(ssaoFront);
    UnloadRenderTexture(lighted);
    UnloadGBuffer(gbuffer);

    UnloadShadowMap(shadowMap);
    
    UnloadModelAnimation(testAnimation);
    UnloadModelAnimationContacts(testContacts);

    UnloadModel(genoModel);
    UnloadModel(groundModel);
    
    UnloadShader(fxaaShader);    
    UnloadShader(blurShader);    
    UnloadShader(ssaoShader);    
    UnloadShader(lightingShader);    
    UnloadShader(basicShader);    
    UnloadShader(skinnedBasicShader);
    UnloadShader(skinnedShadowShader);
    UnloadShader(shadowShader);
        
    CloseWindow();

    return 0;
}

