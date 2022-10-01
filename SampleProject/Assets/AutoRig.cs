using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

/*
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
*/

namespace Spoony.AutoRig
{
    readonly struct Vec3
    {
        public readonly float x;
        public readonly float y;
        public readonly float z;
        public Vec3(float x_, float y_, float z_)
        {
            x = x_;
            y = y_;
            z = z_;
        }
        public static implicit operator Vector3(in Vec3 v) => new Vector3(v.x, v.y, v.z);
        public static implicit operator Vec3(Vector3 v) => new Vec3(v.x, v.y, v.z);
    }

    public static class AutoRigExtensions
    {
        [DllImport("UnityAutoRig")]
        internal static extern int GetSkeletonSize();

        [DllImport("UnityAutoRig")]
        internal static extern int AutoRig([In] Vec3[] verts,
            int numVerts, [In] int[] triangles, int numTriangles,
            [Out] Vec3[] skelOut, [Out] float[] weightsOut);

        [DllImport("UnityAutoRig")]
        [return: MarshalAs(UnmanagedType.LPStr)]
        internal static extern string GetDebugMessages();

#if UNITY_EDITOR
        [MenuItem("GameObject/Auto-rig selected gameobject")]
        public static void AutoRigSelected()
        {
            if (Selection.activeGameObject.TryGetComponent<MeshFilter>(out var meshFilter))
            {
                var skinnedMesh = meshFilter.sharedMesh.AutoRig();
                return;
            }

            Debug.Log("No mesh selected");
        }

        [MenuItem("Test/AutoRig")]
        public static void Test()
        {
            int skelSize = GetSkeletonSize();
            Debug.Log($"SkeletonSize: {skelSize}");
        }
#endif

        static Mesh AutoRig(this Mesh mesh)
        {
            int skelSize = GetSkeletonSize();
            Debug.Log($"SkeletonSize: {skelSize}");
            var verts = mesh.vertices.Select(v => (Vec3)v).ToArray();
            var tris = mesh.triangles;
            var skelOut = new Vec3[skelSize];
            var weightsOut = new float[skelSize * verts.Length];

            var success = AutoRig(verts, verts.Length, tris, tris.Length, skelOut, weightsOut);
            if (success == 0)
            {
                Debug.LogError(GetDebugMessages());
            }
            else
            {
                Debug.Log("Success!");
            }
            return null;
        }
    }
}
