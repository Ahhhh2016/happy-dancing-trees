#pragma once

#include "graphics/shape.h"
#include "Eigen/StdVector"
#include <Eigen/SparseCholesky>
#include <Eigen/Sparse>
#include <unordered_set>
#include <map>
#include <utility>
#include <vector>

class Shader;

struct OrderingConstraint {
    int i, j;
    enum Type { GEQ, LEQ } type;
};

struct EqualityConstraint {
    int i, j;
};

// Per-vertex curve classification used by the ARAP-L (layered ARAP) solver.
// The integer mapping must agree with whatever produces the sidecar metadata
// (currently the `_arapl.txt` file consumed by GLWidget).
enum class CurveType : int {
    Interior = 0,
    DCurve = 1,
};

// Sidecar metadata required to enable ARAP-L. When present we can enforce
// per-part layer ordering and hard equality pairs in addition to vanilla ARAP.
struct ArapLMetadata {
    std::vector<int> partId;                       // per-vertex part index
    std::vector<CurveType> curveType;              // per-vertex curve type
    std::vector<int> partDepth;                    // per-part stroke depth
    std::vector<std::pair<int,int>> equalityPairs; // hard equality pairs
};

class ARAP
{
public:
    ARAP();

    void resetToRestPose();

    void init(Eigen::Vector3f &coeffMin, Eigen::Vector3f &coeffMax,
              std::vector<Eigen::Vector3f> V, std::vector<Eigen::Vector3i> T);

    // ARAP-L variant: enables layered constraints when sidecar metadata is supplied.
    void init(Eigen::Vector3f &coeffMin, Eigen::Vector3f &coeffMax,
              std::vector<Eigen::Vector3f> V, std::vector<Eigen::Vector3i> T,
              const ArapLMetadata &meta);

    // Texture variant - re-initialises the rendered Shape with UV / texture data
    // while preserving any post-settle / warm-started ARAP pose.
    void initTexture(const std::vector<Eigen::Vector3f> &vertices,
                     const std::vector<Eigen::Vector3i> &triangles,
                     const std::vector<Eigen::Vector2f> &texcoordsPerCorner,
                     GLuint diffuseTexture);

    // Tear down GL buffers and reset all solver / constraint state. Used by the
    // GLWidget when the underlying mesh is replaced or cleared.
    void clear();

    void move(int vertex, Eigen::Vector3f targetPosition);

    void destroyGL() {
        m_shape.destroyGL();
        m_shape.clearAnchors();
    }

    std::vector<OrderingConstraint> m_ordering_constraints;
    std::vector<EqualityConstraint> m_equality_constraints;

    Eigen::VectorXd solveZWithConstraints(const Eigen::VectorXd &pz_init, int vertex,
                                          const Eigen::Vector3f &targetPosition);

    void addOrderingConstraint(int i, int j, OrderingConstraint::Type type);
    void addEqualityConstraint(int i, int j);
    void clearLayeringConstraints();


    // shape.cpp delegations
    int getClosestVertex(Eigen::Vector3f start, Eigen::Vector3f ray, float threshold)
    { return m_shape.getClosestVertex(start, ray, threshold); }

    void draw(Shader *shader)
    { m_shape.draw(shader); }

    void draw(Shader *shader, GLenum mode)
    { m_shape.draw(shader, mode); }

    SelectMode select(Shader *shader, int vertex)
    { return m_shape.select(shader, vertex); }

    bool selectWithSpecifiedMode(Shader *shader, int vertex, SelectMode mode)
    { return m_shape.selectWithSpecifiedMode(shader, vertex, mode); }

    bool getAnchorPos(int lastSelected, Eigen::Vector3f &pos,
                      Eigen::Vector3f ray, Eigen::Vector3f start)
    { return m_shape.getAnchorPos(lastSelected, pos, ray, start); }

    std::map<int, Eigen::Vector3f> getNeighborsByIndex(int idx);
    std::map<int, Eigen::Vector3f> getNeighbors(Eigen::Vector3f &i);

    void moveAnchors(const std::map<int, Eigen::Vector3f>& anchorTargets);

    Shape& getShape() { return m_shape; }
    const std::vector<Eigen::Vector3f>& getRestPose() const { return m_p; }
    void setVerticesDirect(const std::vector<Eigen::Vector3f>& verts) {
        m_shape.setVertices(verts);
    }

private:
    void initializeLMat();
    void initializeSolve();

    // ARAP-L helpers
    double meanEdgeLength(const std::vector<Eigen::Vector3f> &g) const;
    std::vector<std::pair<int,int>>
        buildInequalities(const std::vector<Eigen::Vector3f> &g, bool buildGeq) const;
    void projectConstraints(std::vector<Eigen::Vector3f> &g,
                            const std::vector<std::pair<int,int>> &Cgeq,
                            const std::vector<std::pair<int,int>> &Cleq) const;

    Shape  m_shape;

    std::vector<Eigen::Vector3f> m_p;        // rest-pose vertices
    // Last accepted (deformed) pose. Refreshed at the end of every move() so
    // anchors that were previously dragged stay put on subsequent moves.
    std::vector<Eigen::Vector3f> m_gCurrent;

    Eigen::SparseMatrix<double>                          m_L;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>>   m_solver;
    std::unordered_set<int>                              m_lastAnchors;
    std::vector<std::vector<std::pair<int,float>>>       m_neighbors;

    // ARAP-L state. Disabled (m_hasArapL == false) when no sidecar metadata
    // is supplied; in that case the solver runs vanilla ARAP.
    bool m_hasArapL = false;
    ArapLMetadata m_meta;
    std::vector<std::pair<int,int>> m_eqStatic; // sanitized hard equality pairs
    std::vector<int> m_layerPerPart;            // partId -> layer index by depth
};
