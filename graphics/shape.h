#ifndef SHAPE_H
#define SHAPE_H

#include "Eigen/Geometry"
#include <Eigen/Core>
#include <vector>
#include <unordered_set>
#include <GL/glew.h>

class Shader;

enum class SelectMode {
    None,
    Anchor,
    Unanchor
};

class Shape {
public:
    Shape();
    ~Shape();

    void destroyGL();
    void init(const std::vector<Eigen::Vector3f> &vertices, const std::vector<Eigen::Vector3f> &normals, const std::vector<Eigen::Vector3i> &triangles);
    void init(const std::vector<Eigen::Vector3f> &vertices, const std::vector<Eigen::Vector3i> &triangles);
    /// Expands triangles into corner lists; \p texcoordsPerCorner size must be 3 × triangle count (or empty for no UV).
    void init(const std::vector<Eigen::Vector3f> &vertices, const std::vector<Eigen::Vector3i> &triangles,
              const std::vector<Eigen::Vector2f> &texcoordsPerCorner, GLuint diffuseTexture);
    void init(const std::vector<Eigen::Vector3f> &vertices, const std::vector<Eigen::Vector3i> &triangles, const std::vector<Eigen::Vector4i> &tetIndices);

    void setVertices(const std::vector<Eigen::Vector3f> &vertices);
    void setVertices(const std::vector<Eigen::Vector3f> &vertices, const std::vector<Eigen::Vector3f> &normals);

    void setModelMatrix(const Eigen::Affine3f &model);
    void toggleWireframe();
    void draw(Shader *shader, GLenum mode);

    int getClosestVertex(Eigen::Vector3f start, Eigen::Vector3f ray, float threshold);
    bool getAnchorPos(int lastSelected, Eigen::Vector3f& pos, Eigen::Vector3f ray, Eigen::Vector3f start);
    SelectMode select(Shader *shader, int closest_vertex);
    bool selectWithSpecifiedMode(Shader *shader, int closest_vertex, SelectMode mode);
    void draw(Shader *shader);

    void clearAnchors();
    void clearVertices() {
        m_vertices = std::vector<Eigen::Vector3f>();
    }

    const std::vector<Eigen::Vector3f>& getVertices() const;
    const std::vector<Eigen::Vector3i>& getFaces() const;
    const std::unordered_set<int>& getAnchors() const;

private:
    void destroySurfaceGL();
    void destroyTetGL();

private:
    GLuint m_surfaceVao;
    GLuint m_surfaceVbo;
    GLuint m_surfaceIbo;
    GLuint m_tetVao = static_cast<GLuint>(-1);
    GLuint m_tetVbo;
    GLuint m_tetIbo;

    int m_numSurfaceVertices;
    int m_verticesSize;
    int m_numTetVertices = 0;
    std::vector<Eigen::Vector3i> m_faces;
    std::vector<Eigen::Vector3f> m_vertices;
    std::unordered_set<int> m_anchors;

    float m_red, m_blue, m_green, m_alpha;
    Eigen::Matrix4f m_modelMatrix;
    bool m_wireframe;

    GLuint m_diffuseTex = 0;
    bool m_hasTexture = false;
    int lastSelected;

    void selectHelper();
    void updateMesh(const std::vector<Eigen::Vector3i> &faces,
                    const std::vector<Eigen::Vector3f> &vertices,
                    std::vector<Eigen::Vector3f>& verts,
                    std::vector<Eigen::Vector3f>& normals,
                    std::vector<Eigen::Vector3f>& colors);
    Eigen::Vector3f getNormal(const Eigen::Vector3i& face);
    Eigen::Vector3f getNormal(const Eigen::Vector3i& face, const std::vector<Eigen::Vector3f>& vertices);
};

#endif // SHAPE_H
