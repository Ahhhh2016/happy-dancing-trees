#pragma once

#include "graphics/shape.h"
#include "Eigen/StdList"
#include "Eigen/StdVector"
#include <Eigen/SparseCholesky>
#include <Eigen/Sparse>
#include <map>
#include <vector>

class Shader;

// 不等式约束：vertex i 的 z 必须相对 vertex j 满足 GEQ / LEQ（分层）
struct OrderingConstraint {
    int i, j;
    enum Type { GEQ, LEQ } type;
};

// 等式约束：vertex i 和 j 的 z 必须相同（如背面吸附）
struct EqualityConstraint {
    int i, j;
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

    std::vector<OrderingConstraint> m_ordering_constraints;
    std::vector<EqualityConstraint> m_equality_constraints;

    Eigen::VectorXd solveZWithConstraints(const Eigen::VectorXd &pz_init, int vertex,
                                          const Eigen::Vector3f &targetPosition);

public:
    ARAP();
    void init(Eigen::Vector3f &min, Eigen::Vector3f &max,
              std::vector<Eigen::Vector3f> V, std::vector<Eigen::Vector3i> T);
    void init(Eigen::Vector3f &coeffMin, Eigen::Vector3f &coeffMax, std::vector<Eigen::Vector3f> V, std::vector<Eigen::Vector3i> T, Shape &s);
    void initTexture(const std::vector<Eigen::Vector3f> &vertices,
                     const std::vector<Eigen::Vector3i> &triangles,
                     const std::vector<Eigen::Vector2f> &texcoordsPerCorner,
                     GLuint diffuseTexture);
    void move(int vertex, Eigen::Vector3f pos);

    void addOrderingConstraint(int i, int j, OrderingConstraint::Type type);
    void addEqualityConstraint(int i, int j);
    void clearLayeringConstraints();

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
