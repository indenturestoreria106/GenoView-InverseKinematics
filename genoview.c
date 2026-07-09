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
        sqrtf(Vector3DotProduct(p, p) * Vector3DotProduct(q, q)) + Vector3DotProduct(p, q),
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
    float maxLengthBuffer)
{
    Vector3 targetClamp = targetHeel;
    float targetLength = Vector3Distance(targetHeel, globalHip.translation);

    float maxExtension = 
        Vector3Distance(globalHip.translation, globalKnee.translation) + 
        Vector3Distance(globalKnee.translation, globalHeel.translation) - 
        maxLengthBuffer;

    if (targetLength > maxExtension)
    {
        float saturation = (1.0f - expf(-(targetLength - maxExtension) / maxLengthBuffer));
        
        targetClamp = Vector3Add(
            globalHip.translation, 
            Vector3Scale(Vector3Subtract(targetHeel, globalHip.translation), 
                (maxExtension + maxLengthBuffer * saturation) / targetLength));        
    }
    
    Vector3 axisDwn = Vector3Normalize(Vector3Subtract(globalHeel.translation, globalHip.translation));
    Vector3 axisFwd = Vector3Normalize(Vector3CrossProduct(axisDwn, sideVector));
    Vector3 axisRot = Vector3Normalize(Vector3CrossProduct(axisDwn, axisFwd));

    Vector3 a = globalHip.translation;
    Vector3 b = globalKnee.translation;
    Vector3 c = globalHeel.translation;
    Vector3 t = targetClamp;
    
    float lab = Vector3Distance(b, a);
    float lcb = Vector3Distance(b, c);
    float lat = Vector3Distance(t, a);
    float lca = Vector3Distance(a, c);

    float acab0 = acosf(Clamp(Vector3DotProduct(Vector3Scale(Vector3Subtract(c, a), 1.0f / lca), Vector3Scale(Vector3Subtract(b, a), 1.0f / lab)), -1.0f, +1.0f));
    float babc0 = acosf(Clamp(Vector3DotProduct(Vector3Scale(Vector3Subtract(a, b), 1.0f / lab), Vector3Scale(Vector3Subtract(c, b), 1.0f / lcb)), -1.0f, +1.0f));

    float acab1 = acosf(Clamp((lab * lab + lat * lat - lcb * lcb) / (2.0 * lab * lat), -1.0f, +1.0f));
    float babc1 = acosf(Clamp((lab * lab + lcb * lcb - lat * lat) / (2.0 * lab * lcb), -1.0f, +1.0f));

    Quaternion r0 = QuaternionFromScaledAngleAxis(Vector3Scale(axisRot, acab1 - acab0));
    Quaternion r1 = QuaternionFromScaledAngleAxis(Vector3Scale(axisRot, babc1 - babc0));
    Quaternion r2 = QuaternionNormalize(QuaternionBetween(
        Vector3Subtract(globalHeel.translation, globalHip.translation), 
        Vector3Subtract(targetClamp, globalHip.translation)));
    
    *localHip = QuaternionMultiply(QuaternionMultiply(QuaternionMultiply(QuaternionInvert(globalPelvis.rotation), r2), r0), globalHip.rotation);
    *localKnee = QuaternionMultiply(QuaternionMultiply(QuaternionInvert(globalHip.rotation), r1), globalKnee.rotation);
}

//----------------------------------------------------------------------------------
// App
//----------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    // Init Window
    
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
    
    Transform *localTransforms = RL_CALLOC(genoModel.boneCount, sizeof(Transform));
    Transform *globalTransforms = RL_CALLOC(genoModel.boneCount, sizeof(Transform));
    Transform *modifyTransforms = RL_CALLOC(genoModel.boneCount, sizeof(Transform));
    
    BackwardKinematics(localTransforms, genoModel.bindPose, genoModel);
    memcpy(modifyTransforms, localTransforms, genoModel.boneCount * sizeof(Transform));
    ForwardKinematics(globalTransforms, modifyTransforms, genoModel);

    int pelvisBoneIndex = FindModelBoneIndex(genoModel, "Hips");
    int leftHipBoneIndex = FindModelBoneIndex(genoModel, "LeftUpLeg");
    int leftKneeBoneIndex = FindModelBoneIndex(genoModel, "LeftLeg");
    int leftHeelBoneIndex = FindModelBoneIndex(genoModel, "LeftFoot");
    int leftToeBoneIndex = FindModelBoneIndex(genoModel, "LeftToeBase");
    int leftToeEndBoneIndex = FindModelBoneIndex(genoModel, "LeftToeBaseEnd");

    Vector3 target = globalTransforms[leftToeBoneIndex].translation;
    Vector3 targetHeel = globalTransforms[leftHeelBoneIndex].translation;
    Vector3 targetToe = globalTransforms[leftToeBoneIndex].translation;
    Vector3 targetToeEnd = globalTransforms[leftToeEndBoneIndex].translation;
    
    float heelMinHeight = globalTransforms[leftHeelBoneIndex].translation.y;
    float toeMinHeight = globalTransforms[leftToeBoneIndex].translation.y;
    float toeEndMinHeight = globalTransforms[leftToeEndBoneIndex].translation.y;
    
    // UI
    
    bool drawBoneTransforms = false;
    bool drawLeg = true;
    bool drawDebug = true;
    
    bool enableInverseKinematics = true;
    float pelvisOffset = 0.0f;
    float maxLengthBuffer = 0.005f; 
    bool enableHeelLookAt = true;
    bool enableToeLookAt = true;
    bool enableHeightClamp = true;
    
    // Go
    
    while (!WindowShouldClose())
    {
        // Inverse Kinematics
        
        memcpy(modifyTransforms, localTransforms, genoModel.boneCount * sizeof(Transform));
        
        // Apply Pelvis Offset
        
        modifyTransforms[pelvisBoneIndex].translation.y -= pelvisOffset;
        
        ForwardKinematics(globalTransforms, modifyTransforms, genoModel);

        // Move and Clamp Toe Target

        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
        {
            Ray clickRay = GetScreenToWorldRay(GetMousePosition(), camera.cam3d);
            
            target = Vector3Add(clickRay.position, 
                Vector3Scale(clickRay.direction, 
                Vector3Distance(target, camera.cam3d.position)));
        }
        
        if (enableInverseKinematics)
        {
            targetToe = target;
            
            if (enableHeightClamp)
            {
                targetToe.y = Max(targetToe.y, toeMinHeight);   
            }
            
            // Find the Heel Target Location

            targetHeel = Vector3Add(targetToe, Vector3Subtract(
                globalTransforms[leftHeelBoneIndex].translation, 
                globalTransforms[leftToeBoneIndex].translation));

            if (enableHeightClamp)
            {
                targetHeel.y = Max(targetHeel.y, heelMinHeight);
            }
            
            // Solve Two-Bone Inverse Kinematics to place heel
            
            Vector3 sideVector = Vector3RotateByQuaternion(
                (Vector3){ 1.0f, 0.0f, 0.0f }, 
                globalTransforms[leftKneeBoneIndex].rotation);
            
            Quaternion modifiedHip, modifiedKnee;
            
            TwoBoneInverseKinematics(
                &modifiedHip, 
                &modifiedKnee, 
                globalTransforms[pelvisBoneIndex], 
                globalTransforms[leftHipBoneIndex], 
                globalTransforms[leftKneeBoneIndex], 
                globalTransforms[leftHeelBoneIndex], 
                targetHeel, 
                sideVector,
                maxLengthBuffer);
                
            modifyTransforms[leftHipBoneIndex].rotation = modifiedHip;
            modifyTransforms[leftKneeBoneIndex].rotation = modifiedKnee;
            
            // Orient Toe towards Target
            
            if (enableHeelLookAt)
            {
                ForwardKinematics(globalTransforms, modifyTransforms, genoModel);
                
                Quaternion leftHeelRotation = QuaternionMultiply(QuaternionNormalize(QuaternionBetween(
                    Vector3Subtract(globalTransforms[leftToeBoneIndex].translation, globalTransforms[leftHeelBoneIndex].translation),
                    Vector3Subtract(targetToe, globalTransforms[leftHeelBoneIndex].translation))), 
                    globalTransforms[leftHeelBoneIndex].rotation);

                modifyTransforms[leftHeelBoneIndex].rotation = QuaternionMultiply(QuaternionInvert(globalTransforms[leftKneeBoneIndex].rotation), leftHeelRotation);
            }
            
            // Orient Toe-End
            
            if (enableToeLookAt)
            {          
                ForwardKinematics(globalTransforms, modifyTransforms, genoModel);
                
                targetToeEnd = globalTransforms[leftToeEndBoneIndex].translation;
                
                if (enableHeightClamp)
                {
                    targetToeEnd.y = Max(targetToeEnd.y, toeEndMinHeight);
                }
                
                Quaternion leftToeRotation = QuaternionMultiply(QuaternionNormalize(QuaternionBetween(
                    Vector3Subtract(globalTransforms[leftToeEndBoneIndex].translation, globalTransforms[leftToeBoneIndex].translation),
                    Vector3Subtract(targetToeEnd, globalTransforms[leftToeBoneIndex].translation))), 
                    globalTransforms[leftToeBoneIndex].rotation);

                modifyTransforms[leftToeBoneIndex].rotation = QuaternionMultiply(QuaternionInvert(globalTransforms[leftHeelBoneIndex].rotation), leftToeRotation);
            }
            
            // Recompute final global transforms
            
            ForwardKinematics(globalTransforms, modifyTransforms, genoModel);
        }
        
        // Update bone transforms
        
        UpdateModelPoseFromTransforms(genoModel, globalTransforms);
        
        // Shadow Light Tracks Character
        
        Vector3 hipPosition = (Vector3){ 0.0f, 0.5f, 0.0f };
        
        shadowLight.target = (Vector3){ hipPosition.x, 0.0f, hipPosition.z };
        shadowLight.position = Vector3Add(shadowLight.target, Vector3Scale(lightDir, -5.0f));

        // Update Camera
        
        OrbitCameraUpdate(
            &camera,
            (Vector3){ hipPosition.x, 0.75f, hipPosition.z },
            (IsKeyDown(KEY_LEFT_CONTROL) && IsMouseButtonDown(0)) ? GetMouseDelta().x : 0.0f,
            (IsKeyDown(KEY_LEFT_CONTROL) && IsMouseButtonDown(0)) ? GetMouseDelta().y : 0.0f,
            (IsKeyDown(KEY_LEFT_CONTROL) && IsMouseButtonDown(1)) ? GetMouseDelta().x : 0.0f,
            (IsKeyDown(KEY_LEFT_CONTROL) && IsMouseButtonDown(1)) ? GetMouseDelta().y : 0.0f,
            GetMouseWheelMove(),
            GetFrameTime());
        
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
        }
        
        if (drawDebug)
        {
            DrawSphere(target, 0.0125f, GOLD);
            
            DrawSphere(targetHeel, 0.02f, PINK);
            DrawSphere(targetToe, 0.02f, PINK);
            DrawCapsule(targetToe, targetHeel, 0.01f, 7, 7, PINK);
            
            DrawSphere(globalTransforms[leftHeelBoneIndex].translation, 0.015f, PURPLE);
            DrawSphere(globalTransforms[leftToeBoneIndex].translation, 0.015f, PURPLE);
            DrawSphere(globalTransforms[leftToeEndBoneIndex].translation, 0.015f, PURPLE);
            DrawCapsule(
                globalTransforms[leftHeelBoneIndex].translation, 
                globalTransforms[leftToeBoneIndex].translation, 0.0075f, 7, 7, PURPLE);
            DrawCapsule(
                globalTransforms[leftToeBoneIndex].translation, 
                globalTransforms[leftToeEndBoneIndex].translation, 0.0075f, 7, 7, PURPLE);
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
  
        GuiGroupBox((Rectangle){ 20, 10, 190, 180 }, "Camera");

        GuiLabel((Rectangle){ 30, 20, 150, 20 }, "Ctrl + Left Click - Rotate");
        GuiLabel((Rectangle){ 30, 40, 150, 20 }, "Ctrl + Right Click - Pan");
        GuiLabel((Rectangle){ 30, 60, 150, 20 }, "Mouse Scroll - Zoom");
        GuiLabel((Rectangle){ 30, 80, 150, 20 }, TextFormat("Target: [% 5.3f % 5.3f % 5.3f]", camera.cam3d.target.x, camera.cam3d.target.y, camera.cam3d.target.z));
        GuiLabel((Rectangle){ 30, 100, 150, 20 }, TextFormat("Offset: [% 5.3f % 5.3f % 5.3f]", camera.offset.x, camera.offset.y, camera.offset.z));
        GuiLabel((Rectangle){ 30, 120, 150, 20 }, TextFormat("Azimuth: %5.3f", camera.azimuth));
        GuiLabel((Rectangle){ 30, 140, 150, 20 }, TextFormat("Altitude: %5.3f", camera.altitude));
        GuiLabel((Rectangle){ 30, 160, 150, 20 }, TextFormat("Distance: %5.3f", camera.distance));
  
        GuiGroupBox((Rectangle){ screenWidth - 260, 10, 240, 100 }, "Rendering");

        GuiCheckBox((Rectangle){ screenWidth - 250, 20, 20, 20 }, "Draw Transforms", &drawBoneTransforms);
        GuiCheckBox((Rectangle){ screenWidth - 250, 50, 20, 20 }, "Draw Leg", &drawLeg);
        GuiCheckBox((Rectangle){ screenWidth - 250, 80, 20, 20 }, "Draw Debug", &drawDebug);

        GuiGroupBox((Rectangle){ screenWidth - 260, 120, 240, 200 }, "Inverse Kinematics");

        GuiCheckBox((Rectangle){ screenWidth - 250, 130, 20, 20 }, "Enable", &enableInverseKinematics);
        GuiSlider((Rectangle){ screenWidth - 250 + 90, 160, 100, 20 }, "Pelvis Offset", TextFormat("%4.4f", pelvisOffset), &pelvisOffset, 0.0f, 0.1f);
        GuiSlider((Rectangle){ screenWidth - 250 + 90, 190, 100, 20 }, "Extension Buffer", TextFormat("%4.4f", maxLengthBuffer), &maxLengthBuffer, 0.0f, 0.025f);
        GuiCheckBox((Rectangle){ screenWidth - 250, 220, 20, 20 }, "Heel Look-At", &enableHeelLookAt);
        GuiCheckBox((Rectangle){ screenWidth - 250, 250, 20, 20 }, "Toe Look-At", &enableToeLookAt);
        GuiCheckBox((Rectangle){ screenWidth - 250, 280, 20, 20 }, "Height Clamp", &enableHeightClamp);
  
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

