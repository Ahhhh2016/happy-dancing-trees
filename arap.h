#pragma once

#include "graphics/shape.h"
#include "Eigen/StdVector"
#include <Eigen/SparseCholesky>
#include <Eigen/Sparse>
#include <unordered_set>
#include <map>

class Shader;


class ARAP
{
public:
    ARAP();

    void resetToRestPose();

    void init(Eigen::Vector3f &coeffMin, Eigen::Vector3f &coeffMax,
              std::vector<Eigen::Vector3f> V, std::vector<Eigen::Vector3i> T);

    // Texture variant - initialises shape with UV / texture data
    void initTexture(const std::vector<Eigen::Vector3f> &vertices,
                     const std::vector<Eigen::Vector3i> &triangles,
                     const std::vector<Eigen::Vector2f> &texcoordsPerCorner,
                     GLuint diffuseTexture);


    void move(int vertex, Eigen::Vector3f targetPosition);

    void destroyGL() {
        m_shape.destroyGL();
        m_shape.clearAnchors();
    }

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

    Shape  m_shape;

    std::vector<Eigen::Vector3f> m_p;   // rest-pose vertices

    Eigen::SparseMatrix<double>                          m_L;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>>   m_solver;
    std::unordered_set<int>                              m_lastAnchors;
    std::vector<std::vector<std::pair<int,float>>>       m_neighbors;
};
