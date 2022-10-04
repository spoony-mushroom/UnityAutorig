#ifndef AUTORIG_PLUGIN_INCLUDED
#define AUTORIG_PLUGIN_INCLUDED

#ifdef _WIN32
#ifdef UnityAutoRig_EXPORTS
#define AUTORIG_API __declspec(dllexport)
#else
#define AUTORIG_API
#endif
#else
#define AUTORIG_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

struct Vec3 {
  float x;
  float y;
  float z;

  bool operator==(const Vec3& v2) const {
    return x == v2.x && y == v2.y && z == v2.z;
  }
};

int AUTORIG_API GetSkeletonSize();

int AUTORIG_API AutoRig(const Vec3* verts,
                        int numVerts,
                        const int* triangles,
                        int numIndices,
                        Vec3* skeletonOut,
                        float* weightsOut);

const char* AUTORIG_API GetDebugMessages();

#ifdef __cplusplus
}
#endif

#endif