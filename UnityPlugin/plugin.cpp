#include "plugin.h"
#include <fstream>
#include <span>
#include <sstream>
#include <tuple>
#include <unordered_map>
#include <vector>
#include "debugging.h"
#include "mesh.h"
#include "pinocchioApi.h"
#include "skeleton.h"

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

Vec3 FromUnitySpace(const Vec3& vec) {
  return {-vec.x, vec.y, vec.z};
}
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
    mergedVerts[i] = FromUnitySpace(verts[dupeList[0]]);
    for (int origIndex : dupeList) {
      mappings[origIndex] = i;
    }
  }

  std::vector<int> mergedTris(triangles.size());
  for (int i = 0; i < triangles.size(); i += 3) {
    // Because we inverted x, we also need to change the windings
    mergedTris[i] = mappings[triangles[i + 1]];
    mergedTris[i + 1] = mappings[triangles[i]];
    mergedTris[i + 2] = mappings[triangles[i + 2]];
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

  m.edges.reserve(triangles.size());
  for (int index : triangles) {
    m.edges.emplace_back().vertex = index;
  }

  m.fixDupFaces();
  m.computeTopology();
  if (!m.integrityCheck()) {
    throw "Failed integrity check";
  }
  m.computeVertexNormals();
  return m;
}

static std::string s_latestMessages;
static std::ostringstream s_debugStream;
static HumanSkeleton s_humanSkeleton;

}  // anonymous namespace

int AUTORIG_API GetSkeletonSize() {
  return s_humanSkeleton.fPrev().size();
}

int AUTORIG_API AutoRig(const Vec3* verts,
                        int numVerts,
                        const int* triangles,
                        int numIndices,
                        Vec3* skeletonOut,
                        float* weightsOut) {
  Debugging::setOutStream(s_debugStream);
  try {
    auto [processedVerts, processedTris, mappings] =
        Preprocess({verts, (size_t)numVerts}, {triangles, (size_t)numIndices});
    Mesh m = MakeMesh(processedVerts, processedTris);

    HumanSkeleton skel;
    skel.scale(0.7);
    auto pinoOut = autorig(skel, m);
    if (pinoOut.embedding.size() == 0) {
      Debugging::out() << "Error embedding";
      return 0;
    }

    for (int i = 0; i < (int)pinoOut.embedding.size(); ++i) {
      pinoOut.embedding[i] = (pinoOut.embedding[i] - m.toAdd) / m.scale;
    }

    ofstream os("skeleton.out");
    for (int i = 0; i < (int)pinoOut.embedding.size(); ++i) {
      const auto& bone = pinoOut.embedding[i];
      skeletonOut[i] = Vec3{-(float)bone[0], (float)bone[1], (float)bone[2]};
      os << i << " " << pinoOut.embedding[i][0] << " "
         << pinoOut.embedding[i][1] << " " << pinoOut.embedding[i][2] << " "
         << skel.fPrev()[i] << endl;
    }

    // output attachment
    auto weightsPtr = weightsOut;
    std::ofstream astrm("attachment.out");
    for (int i = 0; i < (int)m.vertices.size(); ++i) {
      Vector<double, -1> v = pinoOut.attachment->getWeights(i);
      for (int j = 0; j < v.size(); ++j) {
        astrm << v[j] << " ";
        *weightsPtr++ = v[j];
      }
      astrm << endl;
    }

    return 1;
  } catch (...) {
    return 0;
  }
}

const char* AUTORIG_API GetDebugMessages() {
  s_latestMessages = s_debugStream.str();
  s_debugStream.clear();
  return s_latestMessages.c_str();
}