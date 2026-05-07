#pragma once

#include "graphics/shape.h"
#include "Eigen/StdList"
#include "Eigen/StdVector"
#include <Eigen/SparseCholesky>
#include <Eigen/Sparse>
#include <map>

class Shader;

// Per-vertex curve classification used by ARAP-L:
//   Interior — not on any user-drawn curve (no inequality constraints generated)
//   DCurve   — on a Dp silhouette (inequality query points)
//   BCurve   — on a Bp merging boundary
enum class CurveType : int {
    Interior = 0,
    DCurve = 1,
    BCurve = 2
};

// Optional metadata produced by `monster` and consumed by ARAP to enforce
// the three constraint sets described in the ARAP-L paper:
//   1) equality pairs   (g_i == g_j on all 3 coords)
//   2) z >= z pairs     (built dynamically per frame from depth ordering)
//   3) z <= z pairs     (built dynamically per frame from depth ordering)
struct ArapLMetadata {
    std::vector<int> partId;                    // size n
    std::vector<int> partDepth;                 // size num_parts (smaller = closer to camera)
    std::vector<CurveType> curveType;           // size n
    std::vector<std::pair<int,int>> equalityPairs; // static pairs (e.g. armpit endpoints)
};

class ARAP
{
private:
    Shape m_shape;
    Eigen::SparseMatrix<double> m_L; // Laplacian for current shape
    void initializeLMat();
    std::unordered_set<int> m_lastAnchors;
    std::vector<Eigen::Vector3f> m_p; // Rest pose (inflated shape)
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> m_solver;
    std::vector<std::vector<std::pair<int,float>>> m_neighbors;
    void initializeSolve();

    // ARAP-L bookkeeping (only used when init is called with metadata).
    bool m_hasArapL = false;
    ArapLMetadata m_meta;
    std::vector<int> m_layerPerPart;                 // partId -> layer index after sort
    std::vector<std::pair<int,int>> m_eqStatic;      // sanitized copy of metadata pairs
    std::vector<Eigen::Vector3f> m_gCurrent;         // last solved positions (warm start)

    // Build C_geq (frontPairs) or C_leq (backPairs) from the current pose.
    // Only D-curve vertices generate constraints (per ARAP-L spec).
    std::vector<std::pair<int,int>> buildInequalities(
        const std::vector<Eigen::Vector3f> &g,
        bool buildGeq) const;

    // Mean edge length of the current mesh (used to size the spatial hash grid).
    double meanEdgeLength(const std::vector<Eigen::Vector3f> &g) const;

    // Project the three constraint sets on the post-global-step vertex list.
    void projectConstraints(std::vector<Eigen::Vector3f> &g,
                            const std::vector<std::pair<int,int>> &Cgeq,
                            const std::vector<std::pair<int,int>> &Cleq) const;

public:
    ARAP();
    void init(Eigen::Vector3f &min, Eigen::Vector3f &max,
              std::vector<Eigen::Vector3f> V, std::vector<Eigen::Vector3i> T);
    // ARAP-L initialization. The plain init() above keeps default behavior unchanged.
    void init(Eigen::Vector3f &min, Eigen::Vector3f &max,
              std::vector<Eigen::Vector3f> V, std::vector<Eigen::Vector3i> T,
              const ArapLMetadata &meta);

    // Re-initialize the underlying GL Shape with a texture. Call AFTER one of
    // the init() overloads above so all ARAP solver state is set up first;
    // this only swaps the GPU buffers/UVs so painted textures show up while
    // ARAP deformation continues to drive vertex positions.
    void initShapeWithTexture(const std::vector<Eigen::Vector3f> &V,
                              const std::vector<Eigen::Vector3i> &T,
                              const std::vector<Eigen::Vector2f> &texcoordsPerCorner,
                              GLuint diffuseTexture);

    // Returns the current (deformed) vertex positions. Used to keep external
    // copies of the mesh (e.g. a textured render Shape) in sync with ARAP.
    const std::vector<Eigen::Vector3f>& getDeformedVertices() const {
        return m_shape.getVertices();
    }

    // Reset all ARAP state so nothing remains drawn after a canvas clear.
    void clear();
    void move(int vertex, Eigen::Vector3f pos);
    std::map<int, Eigen::Vector3f> getNeighbors(Eigen::Vector3f &i);
    std::map<int, Eigen::Vector3f> getNeighborsByIndex(int idx);

    int getClosestVertex(Eigen::Vector3f start, Eigen::Vector3f ray, float threshold)
    {
        return m_shape.getClosestVertex(start, ray, threshold);
    }

    void draw(Shader *shader, GLenum mode)
    {
        m_shape.draw(shader, mode);
    }

    void draw(Shader *shader)
    {
        m_shape.draw(shader);
    }

    void destroyGL()
    {
        m_shape.destroyGL();
    }

    void init(const std::vector<Eigen::Vector3f> &vertices,
              const std::vector<Eigen::Vector3i> &triangles,
              const std::vector<Eigen::Vector2f> &texcoordsPerCorner,
              GLuint diffuseTexture)
    {
        m_shape.init(vertices, triangles, texcoordsPerCorner, diffuseTexture);
    }

    SelectMode select(Shader *shader, int vertex)
    {
        return m_shape.select(shader, vertex);
    }

    bool selectWithSpecifiedMode(Shader *shader, int vertex, SelectMode mode)
    {
        return m_shape.selectWithSpecifiedMode(shader, vertex, mode);
    }

    bool getAnchorPos(int lastSelected, Eigen::Vector3f& pos, Eigen::Vector3f ray, Eigen::Vector3f start)
    {
        return m_shape.getAnchorPos(lastSelected, pos, ray, start);
    }
};
