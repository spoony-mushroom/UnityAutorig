#include "plugin.h"
#include <mesh.h>
#include <span>
#include <tuple>
#include <unordered_map>
#include <vector>

const int PINOCCHIO_HUMAN_SKEL_SIZE = 18;

namespace {
// Boost hash_combine function
template <typename T>
inline void hash_combine(std::size_t& seed, T&& v) {
  std::hash<std::remove_cvref_t<T>> hasher;
  seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

template <typename T, typename... Args>
inline void hash_combine(std::size_t& seed, T&& v, Args... args) {
  hash_combine(seed, std::forward<T>(v));
  hash_combine(seed, std::forward<Args>(args)...);
}
}  // namespace

namespace std {
template <>
struct hash<Vec3> {
  size_t operator()(const Vec3& vec) const {
    size_t seed = 0;
    hash_combine(seed, vec.x, vec.y, vec.z);
    return seed;
  }
};
}  // namespace std

namespace {

std::tuple<std::vector<Vec3>, std::vector<int>, std::unordered_map<int, int>>
Preprocess(std::span<const Vec3> verts, std::span<const int> triangles) {
  // Merge duplicate vertices (caused by double-normals in Unity import)
  std::unordered_map<Vec3, std::vector<int>> uniqueVerts;
  for (int i = 0; i < verts.size(); ++i) {
    auto& vert = verts[i];
    if (auto it = uniqueVerts.find(vert); it != uniqueVerts.end()) {
      it->second.push_back(i);
      continue;
    }
    uniqueVerts[vert].push_back(i);
  }

  std::vector<Vec3> mergedVerts(uniqueVerts.size());
  std::unordered_map<int, int> mappings;
  auto uniqueVertsIter = uniqueVerts.begin();
  for (int i = 0; i < mergedVerts.size(); ++i, ++uniqueVertsIter) {
    const auto& dupeList = uniqueVertsIter->second;
    mergedVerts[i] = verts[dupeList[0]];
    for (int origIndex : dupeList) {
      mappings[origIndex] = i;
    }
    ++uniqueVertsIter;
  }

  std::vector<int> mergedTris(triangles.size());
  for (int i = 0; i < triangles.size(); ++i)
  {
    mergedTris[i] = mappings[i];
  }

  return std::make_tuple(std::move(mergedVerts), std::move(mergedTris),
                         std::move(mappings));
}

Mesh MakeMesh(std::span<const Vec3> verts, std::span<const int> triangles) {
  Mesh m;
  m.vertices.reserve(verts.size());
  for (const auto& vec : verts) {
    m.vertices.emplace_back().pos = Vector3(vec.x, vec.y, vec.z);
  }

  m.edges.reserve(triangles.size() * 3);
  for (int index : triangles) {
    m.edges.emplace_back().vertex = index;
  }

  m.computeVertexNormals();
  m.computeTopology();
  return m;
}
}  // anonymous namespace

int AUTORIG_API GetSkeletonSize() {
  return PINOCCHIO_HUMAN_SKEL_SIZE;
}

int AUTORIG_API AutoRig(const Vec3* verts,
                        int numVerts,
                        const int* triangles,
                        int numIndices,
                        Vec3* skeletonOut,
                        float* weightsOut) {
  auto [processedVerts, processedTris, mappings] =
      Preprocess({verts, (size_t)numVerts}, {triangles, (size_t)numIndices});
  Mesh m = MakeMesh(processedVerts, processedTris);
  return 1;
}