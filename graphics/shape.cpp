#include "shape.h"

#include "graphics/shader.h"
#include <iostream>
#include <limits>

using namespace Eigen;
using namespace std;

Shape::Shape()
    : m_surfaceVao(0),
      m_surfaceVbo(0),
      m_surfaceIbo(0),
      m_tetVao(static_cast<GLuint>(-1)),
      m_tetVbo(0),
      m_tetIbo(0),
      m_numSurfaceVertices(0),
      m_verticesSize(0),
      m_numTetVertices(0),
      m_red(0.93f),
      m_blue(1.0f),
      m_green(0.8f),
      m_alpha(1.0f),
      m_modelMatrix(Matrix4f::Identity()),
      m_wireframe(false),
      m_diffuseTex(0),
      m_hasTexture(false),
      lastSelected(-1)
{
}

Shape::~Shape() = default;

void Shape::destroySurfaceGL()
{
    if (m_diffuseTex != 0) {
        glDeleteTextures(1, &m_diffuseTex);
        m_diffuseTex = 0;
    }
    m_hasTexture = false;
    if (m_surfaceVbo != 0) {
        glDeleteBuffers(1, &m_surfaceVbo);
        m_surfaceVbo = 0;
    }
    if (m_surfaceIbo != 0) {
        glDeleteBuffers(1, &m_surfaceIbo);
        m_surfaceIbo = 0;
    }
    if (m_surfaceVao != 0) {
        glDeleteVertexArrays(1, &m_surfaceVao);
        m_surfaceVao = 0;
    }
}

void Shape::destroyTetGL()
{
    if (m_tetVao == static_cast<GLuint>(-1)) {
        return;
    }
    if (m_tetVbo != 0) {
        glDeleteBuffers(1, &m_tetVbo);
        m_tetVbo = 0;
    }
    if (m_tetIbo != 0) {
        glDeleteBuffers(1, &m_tetIbo);
        m_tetIbo = 0;
    }
    glDeleteVertexArrays(1, &m_tetVao);
    m_tetVao = static_cast<GLuint>(-1);
}

void Shape::destroyGL()
{
    destroySurfaceGL();
    destroyTetGL();
}

void Shape::init(const vector<Vector3f> &vertices, const vector<Vector3i> &triangles)
{
    destroySurfaceGL();
    m_vertices = vertices;
    m_faces = triangles;
    m_verticesSize = static_cast<int>(vertices.size());
    m_numSurfaceVertices = static_cast<int>(triangles.size() * 3);
    m_hasTexture = false;

    vector<Vector3f> verts;
    vector<Vector3f> normals;
    vector<Vector3f> colors;
    updateMesh(triangles, vertices, verts, normals, colors);

    vector<Vector3i> flatFaces;
    flatFaces.reserve(triangles.size());
    for (int s = 0; s < static_cast<int>(triangles.size()) * 3; s += 3) {
        flatFaces.emplace_back(s, s + 1, s + 2);
    }

    glGenBuffers(1, &m_surfaceVbo);
    glGenBuffers(1, &m_surfaceIbo);
    glGenVertexArrays(1, &m_surfaceVao);

    glBindBuffer(GL_ARRAY_BUFFER, m_surfaceVbo);
    const GLsizeiptr posBytes = static_cast<GLsizeiptr>(sizeof(float) * verts.size() * 3);
    const GLsizeiptr normBytes = static_cast<GLsizeiptr>(sizeof(float) * normals.size() * 3);
    const GLsizeiptr colorBytes = static_cast<GLsizeiptr>(sizeof(float) * colors.size() * 3);
    glBufferData(GL_ARRAY_BUFFER, posBytes + normBytes + colorBytes, nullptr, GL_DYNAMIC_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, posBytes, verts.data());
    glBufferSubData(GL_ARRAY_BUFFER, posBytes, normBytes, normals.data());
    glBufferSubData(GL_ARRAY_BUFFER, posBytes + normBytes, colorBytes, colors.data());
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_surfaceIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * 3 * flatFaces.size(), flatFaces.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    glBindVertexArray(m_surfaceVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_surfaceVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, static_cast<GLvoid *>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, reinterpret_cast<GLvoid *>(posBytes));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, reinterpret_cast<GLvoid *>(posBytes + normBytes));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_surfaceIbo);
    glBindVertexArray(0);
}

void Shape::init(const vector<Vector3f> &vertices, const vector<Vector3f> &normals, const vector<Vector3i> &triangles)
{
    if (vertices.size() != normals.size()) {
        cerr << "Vertices and normals size mismatch" << endl;
        return;
    }

    destroySurfaceGL();
    m_vertices = vertices;
    m_faces = triangles;
    m_verticesSize = static_cast<int>(vertices.size());
    m_numSurfaceVertices = static_cast<int>(triangles.size() * 3);
    m_hasTexture = false;

    vector<Vector3f> verts;
    vector<Vector3f> norms;
    vector<Vector3f> colors;
    vector<Vector3i> flatFaces;
    flatFaces.reserve(triangles.size());
    verts.reserve(triangles.size() * 3);
    norms.reserve(triangles.size() * 3);
    colors.reserve(triangles.size() * 3);

    for (int s = 0; s < static_cast<int>(triangles.size()) * 3; s += 3) {
        flatFaces.emplace_back(s, s + 1, s + 2);
    }
    for (const auto &tri : triangles) {
        for (int i = 0; i < 3; ++i) {
            const int idx = tri[i];
            verts.push_back(vertices[idx]);
            norms.push_back(normals[idx]);
            colors.emplace_back(1.0f, 0.0f, 0.0f);
        }
    }

    glGenBuffers(1, &m_surfaceVbo);
    glGenBuffers(1, &m_surfaceIbo);
    glGenVertexArrays(1, &m_surfaceVao);

    glBindBuffer(GL_ARRAY_BUFFER, m_surfaceVbo);
    const GLsizeiptr posBytes = static_cast<GLsizeiptr>(sizeof(float) * verts.size() * 3);
    const GLsizeiptr normBytes = static_cast<GLsizeiptr>(sizeof(float) * norms.size() * 3);
    const GLsizeiptr colorBytes = static_cast<GLsizeiptr>(sizeof(float) * colors.size() * 3);
    glBufferData(GL_ARRAY_BUFFER, posBytes + normBytes + colorBytes, nullptr, GL_DYNAMIC_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, posBytes, verts.data());
    glBufferSubData(GL_ARRAY_BUFFER, posBytes, normBytes, norms.data());
    glBufferSubData(GL_ARRAY_BUFFER, posBytes + normBytes, colorBytes, colors.data());
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_surfaceIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * 3 * flatFaces.size(), flatFaces.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    glBindVertexArray(m_surfaceVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_surfaceVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, static_cast<GLvoid *>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, reinterpret_cast<GLvoid *>(posBytes));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, reinterpret_cast<GLvoid *>(posBytes + normBytes));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_surfaceIbo);
    glBindVertexArray(0);
}

void Shape::init(const vector<Vector3f> &vertices,
                 const vector<Vector3i> &triangles,
                 const vector<Vector2f> &texcoordsPerCorner,
                 GLuint diffuseTexture)
{
    destroySurfaceGL();
    m_vertices = vertices;
    m_faces = triangles;
    m_verticesSize = static_cast<int>(vertices.size());
    m_numSurfaceVertices = static_cast<int>(triangles.size() * 3);

    vector<Vector3f> verts;
    vector<Vector3f> normals;
    vector<Vector2f> uvs;
    vector<Vector3i> flatFaces;
    verts.reserve(triangles.size() * 3);
    normals.reserve(triangles.size() * 3);
    uvs.reserve(triangles.size() * 3);
    flatFaces.reserve(triangles.size());

    const bool useTex = (diffuseTexture != 0 && texcoordsPerCorner.size() == triangles.size() * 3);
    for (size_t fi = 0; fi < triangles.size(); ++fi) {
        const Vector3i &f = triangles[fi];
        const Vector3f &v0 = vertices[f[0]];
        const Vector3f &v1 = vertices[f[1]];
        const Vector3f &v2 = vertices[f[2]];
        Vector3f n = (v1 - v0).cross(v2 - v0);
        if (n.squaredNorm() > 1e-10f) {
            n.normalize();
        }

        const int s = static_cast<int>(verts.size());
        flatFaces.emplace_back(s, s + 1, s + 2);

        verts.push_back(v0);
        verts.push_back(v1);
        verts.push_back(v2);
        normals.push_back(n);
        normals.push_back(n);
        normals.push_back(n);

        const size_t base = fi * 3;
        if (useTex) {
            uvs.push_back(texcoordsPerCorner[base]);
            uvs.push_back(texcoordsPerCorner[base + 1]);
            uvs.push_back(texcoordsPerCorner[base + 2]);
        } else {
            uvs.emplace_back(0.f, 0.f);
            uvs.emplace_back(0.f, 0.f);
            uvs.emplace_back(0.f, 0.f);
        }
    }

    glGenBuffers(1, &m_surfaceVbo);
    glGenBuffers(1, &m_surfaceIbo);
    glGenVertexArrays(1, &m_surfaceVao);

    glBindBuffer(GL_ARRAY_BUFFER, m_surfaceVbo);
    const GLsizeiptr posBytes = static_cast<GLsizeiptr>(sizeof(float) * verts.size() * 3);
    const GLsizeiptr normBytes = static_cast<GLsizeiptr>(sizeof(float) * normals.size() * 3);
    const GLsizeiptr uvBytes = static_cast<GLsizeiptr>(sizeof(float) * uvs.size() * 2);
    glBufferData(GL_ARRAY_BUFFER, posBytes + normBytes + uvBytes, nullptr, GL_DYNAMIC_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, posBytes, verts.data());
    glBufferSubData(GL_ARRAY_BUFFER, posBytes, normBytes, normals.data());
    glBufferSubData(GL_ARRAY_BUFFER, posBytes + normBytes, uvBytes, uvs.data());
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_surfaceIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * 3 * flatFaces.size(), flatFaces.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    glBindVertexArray(m_surfaceVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_surfaceVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, static_cast<GLvoid *>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, reinterpret_cast<GLvoid *>(posBytes));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, reinterpret_cast<GLvoid *>(posBytes + normBytes));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_surfaceIbo);
    glBindVertexArray(0);

    m_diffuseTex = diffuseTexture;
    m_hasTexture = useTex;
    if (!m_hasTexture && m_diffuseTex != 0) {
        glDeleteTextures(1, &m_diffuseTex);
        m_diffuseTex = 0;
    }
}

void Shape::init(const vector<Vector3f> &vertices, const vector<Vector3i> &triangles, const vector<Vector4i> &tetIndices)
{
    destroyTetGL();
    init(vertices, triangles);

    vector<Vector2i> lines;
    for (const Vector4i &tet : tetIndices) {
        lines.emplace_back(tet[0], tet[1]);
        lines.emplace_back(tet[0], tet[2]);
        lines.emplace_back(tet[0], tet[3]);
        lines.emplace_back(tet[1], tet[2]);
        lines.emplace_back(tet[1], tet[3]);
        lines.emplace_back(tet[2], tet[3]);
    }

    glGenBuffers(1, &m_tetVbo);
    glGenBuffers(1, &m_tetIbo);
    glGenVertexArrays(1, &m_tetVao);

    glBindBuffer(GL_ARRAY_BUFFER, m_tetVbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices.size() * 3, vertices.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_tetIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * 2 * lines.size(), lines.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    glBindVertexArray(m_tetVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_tetVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, static_cast<GLvoid *>(0));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_tetIbo);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    m_numTetVertices = static_cast<int>(lines.size() * 2);
}

void Shape::setVertices(const vector<Vector3f> &vertices)
{
    if (vertices.size() != static_cast<size_t>(m_verticesSize)) {
        cerr << "Vertex count mismatch" << endl;
        return;
    }
    m_vertices = vertices;

    if (m_hasTexture) {
        vector<float> positions;
        positions.reserve(m_faces.size() * 9);
        for (const Vector3i &face : m_faces) {
            for (int v : {face[0], face[1], face[2]}) {
                positions.push_back(vertices[v].x());
                positions.push_back(vertices[v].y());
                positions.push_back(vertices[v].z());
            }
        }

        glBindBuffer(GL_ARRAY_BUFFER, m_surfaceVbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, positions.size() * sizeof(float), positions.data());
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
    else {

    vector<Vector3f> verts;
    vector<Vector3f> normals;
    vector<Vector3f> colors;
    updateMesh(m_faces, vertices, verts, normals, colors);



        glBindBuffer(GL_ARRAY_BUFFER, m_surfaceVbo);
        const GLsizeiptr posBytes = static_cast<GLsizeiptr>(sizeof(float) * verts.size() * 3);
        const GLsizeiptr normBytes = static_cast<GLsizeiptr>(sizeof(float) * normals.size() * 3);
        const GLsizeiptr colorBytes = static_cast<GLsizeiptr>(sizeof(float) * colors.size() * 3);
        glBufferData(GL_ARRAY_BUFFER, posBytes + normBytes + colorBytes, nullptr, GL_DYNAMIC_DRAW);
        glBufferSubData(GL_ARRAY_BUFFER, 0, posBytes, verts.data());
        glBufferSubData(GL_ARRAY_BUFFER, posBytes, normBytes, normals.data());
        glBufferSubData(GL_ARRAY_BUFFER, posBytes + normBytes, colorBytes, colors.data());
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        if (m_tetVao != static_cast<GLuint>(-1)) {
            glBindBuffer(GL_ARRAY_BUFFER, m_tetVbo);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * vertices.size() * 3, vertices.data());
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }
    }
    buildAnchorVBO();
}

void Shape::setVertices(const vector<Vector3f> &vertices, const vector<Vector3f> &normals)
{
    if (vertices.size() != normals.size()) {
        cerr << "Vertices and normals size mismatch" << endl;
        return;
    }
    if (vertices.size() != static_cast<size_t>(m_verticesSize)) {
        cerr << "Vertex count mismatch" << endl;
        return;
    }

    m_vertices = vertices;
    glBindBuffer(GL_ARRAY_BUFFER, m_surfaceVbo);
    const GLsizeiptr posBytes = static_cast<GLsizeiptr>(sizeof(float) * vertices.size() * 3);
    const GLsizeiptr normBytes = static_cast<GLsizeiptr>(sizeof(float) * normals.size() * 3);
    glBufferSubData(GL_ARRAY_BUFFER, 0, posBytes, vertices.data());
    glBufferSubData(GL_ARRAY_BUFFER, posBytes, normBytes, normals.data());
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void Shape::setModelMatrix(const Affine3f &model)
{
    m_modelMatrix = model.matrix();
}

void Shape::toggleWireframe()
{
    m_wireframe = !m_wireframe;
}

void Shape::draw(Shader *shader, GLenum mode)
{
    Matrix3f m3 = m_modelMatrix.topLeftCorner(3, 3);
    Matrix3f inverseTransposeModel = m3.inverse().transpose();

    switch (mode) {
    case GL_TRIANGLES:
        if (m_wireframe && m_tetVao != static_cast<GLuint>(-1)) {
            shader->setUniform("wire", 1);
            shader->setUniform("useTexture", 0);
            shader->setUniform("model", m_modelMatrix);
            shader->setUniform("inverseTransposeModel", inverseTransposeModel);
            shader->setUniform("red", 1.0f);
            shader->setUniform("green", 1.0f);
            shader->setUniform("blue", 1.0f);
            shader->setUniform("alpha", 1.0f);
            glBindVertexArray(m_tetVao);
            glDrawElements(GL_LINES, m_numTetVertices, GL_UNSIGNED_INT, reinterpret_cast<GLvoid *>(0));
            glBindVertexArray(0);
            return;
        }

        shader->setUniform("wire", 0);
        shader->setUniform("model", m_modelMatrix);
        shader->setUniform("inverseTransposeModel", inverseTransposeModel);
        shader->setUniform("useTexture", m_hasTexture ? 1 : 0);
        if (m_hasTexture && m_diffuseTex != 0) {
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, m_diffuseTex);
            shader->setUniform("diffuseTex", 0);
        }
        shader->setUniform("red", m_red);
        shader->setUniform("green", m_green);
        shader->setUniform("blue", m_blue);
        shader->setUniform("alpha", m_alpha);
        glBindVertexArray(m_surfaceVao);
        glDrawElements(GL_TRIANGLES, m_numSurfaceVertices, GL_UNSIGNED_INT, reinterpret_cast<GLvoid *>(0));
        glBindVertexArray(0);
        if (m_hasTexture && m_diffuseTex != 0) {
            glBindTexture(GL_TEXTURE_2D, 0);
        }
        return;

    case GL_POINTS:
        shader->setUniform("model", m_modelMatrix);
        shader->setUniform("inverseTransposeModel", inverseTransposeModel);
        glBindVertexArray(m_surfaceVao);
        glDrawElements(GL_POINTS, m_numSurfaceVertices, GL_UNSIGNED_INT, reinterpret_cast<GLvoid *>(0));
        glBindVertexArray(0);
        return;
    default:
        return;
    }
}

int Shape::getClosestVertex(Vector3f start, Vector3f ray, float threshold)
{
    int closestVertex = -1;
    int i = 0;
    float dist = numeric_limits<float>::max();
    ParametrizedLine<float, 3> line = ParametrizedLine<float, 3>::Through(start, start + ray);

    for (const Vector3f &v : m_vertices) {
        float d = line.distance(v);
        if (d < dist) {
            dist = d;
            closestVertex = i;
        }
        ++i;
    }

    if (dist >= threshold) {
        closestVertex = -1;
    }
    return closestVertex;
}

bool Shape::getAnchorPos(int lastSelectedVertex, Vector3f &pos, Vector3f ray, Vector3f start)
{
    bool isAnchor = m_anchors.find(lastSelectedVertex) != m_anchors.end();
    if (isAnchor) {
        const Vector3f &oldPos = m_vertices[lastSelectedVertex];
        ParametrizedLine<float, 3> line = ParametrizedLine<float, 3>::Through(start, start + ray);
        pos = line.projection(oldPos);
    }
    return isAnchor;
}

void Shape::selectHelper()
{
    if (m_hasTexture) {
        return;
    }
    vector<Vector3f> verts;
    vector<Vector3f> normals;
    vector<Vector3f> colors;
    updateMesh(m_faces, m_vertices, verts, normals, colors);

    glBindBuffer(GL_ARRAY_BUFFER, m_surfaceVbo);
    const GLsizeiptr posBytes = static_cast<GLsizeiptr>(sizeof(float) * verts.size() * 3);
    const GLsizeiptr normBytes = static_cast<GLsizeiptr>(sizeof(float) * normals.size() * 3);
    const GLsizeiptr colorBytes = static_cast<GLsizeiptr>(sizeof(float) * colors.size() * 3);
    glBufferData(GL_ARRAY_BUFFER, posBytes + normBytes + colorBytes, nullptr, GL_DYNAMIC_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, posBytes, verts.data());
    glBufferSubData(GL_ARRAY_BUFFER, posBytes, normBytes, normals.data());
    glBufferSubData(GL_ARRAY_BUFFER, posBytes + normBytes, colorBytes, colors.data());

    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, reinterpret_cast<GLvoid *>(posBytes + normBytes));

    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

SelectMode Shape::select(Shader *shader, int closest_vertex)
{
    (void)shader;
    if (closest_vertex == -1) {
        return SelectMode::None;
    }

    const bool vertexIsNowSelected = m_anchors.find(closest_vertex) == m_anchors.end();
    if (vertexIsNowSelected) {
        m_anchors.insert(closest_vertex);
    } else {
        m_anchors.erase(closest_vertex);
    }

    selectHelper();
    buildAnchorVBO();
    return vertexIsNowSelected ? SelectMode::Anchor : SelectMode::Unanchor;
}

bool Shape::selectWithSpecifiedMode(Shader *shader, int closest_vertex, SelectMode mode)
{
    (void)shader;
    switch (mode) {
    case SelectMode::None:
        return false;
    case SelectMode::Anchor:
        if (m_anchors.find(closest_vertex) != m_anchors.end()) {
            return false;
        }
        m_anchors.insert(closest_vertex);
        break;
    case SelectMode::Unanchor:
        if (m_anchors.find(closest_vertex) == m_anchors.end()) {
            return false;
        }
        m_anchors.erase(closest_vertex);
        break;
    }

    selectHelper();
    buildAnchorVBO();
    return true;
}

void Shape::updateMesh(const vector<Vector3i> &faces,
                       const vector<Vector3f> &vertices,
                       vector<Vector3f> &verts,
                       vector<Vector3f> &normals,
                       vector<Vector3f> &colors)
{
    verts.clear();
    normals.clear();
    colors.clear();

    verts.reserve(faces.size() * 3);
    normals.reserve(faces.size() * 3);
    colors.reserve(faces.size() * 3);

    for (const Vector3i &face : faces) {
        Vector3f n = getNormal(face, vertices);
        for (int v : {face[0], face[1], face[2]}) {
            verts.push_back(vertices[v]);
            normals.push_back(n);
            if (m_anchors.find(v) == m_anchors.end()) {
                colors.emplace_back(1.0f, 0.0f, 0.0f);
            } else {
                colors.emplace_back(0.0f, 1.0f - m_green, 1.0f - m_blue);
            }
        }
    }
}

Vector3f Shape::getNormal(const Vector3i &face, const vector<Vector3f> &vertices)
{
    const Vector3f &v1 = vertices[face[0]];
    const Vector3f &v2 = vertices[face[1]];
    const Vector3f &v3 = vertices[face[2]];
    Vector3f n = (v2 - v1).cross(v3 - v1);
    if (n.squaredNorm() > 1e-10f) {
        return n.normalized();
    }
    return Vector3f::UnitY();
}

Vector3f Shape::getNormal(const Vector3i &face)
{
    return getNormal(face, m_vertices);
}


void Shape::draw(Shader *shader)
{
    Eigen::Matrix3f m3 = m_modelMatrix.topLeftCorner(3, 3);
    Eigen::Matrix3f inverseTransposeModel = m3.inverse().transpose();

    if (m_wireframe && m_tetVao != static_cast<GLuint>(-1)) {
        shader->setUniform("wire", 1);
        shader->setUniform("useTexture", 0);
        shader->setUniform("model", m_modelMatrix);
        shader->setUniform("inverseTransposeModel", inverseTransposeModel);
        shader->setUniform("red", 1.0f);
        shader->setUniform("green", 1.0f);
        shader->setUniform("blue", 1.0f);
        shader->setUniform("alpha", 1.0f);
        glBindVertexArray(m_tetVao);
        glDrawElements(GL_LINES, m_numTetVertices, GL_UNSIGNED_INT, reinterpret_cast<GLvoid *>(0));
        glBindVertexArray(0);
        return;
    }

    shader->setUniform("wire", 0);
    shader->setUniform("model", m_modelMatrix);
    shader->setUniform("inverseTransposeModel", inverseTransposeModel);
    shader->setUniform("useTexture", m_hasTexture ? 1 : 0);
    if (m_hasTexture && m_diffuseTex != 0) {
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, m_diffuseTex);
        shader->setUniform("diffuseTex", 0);
    }
    shader->setUniform("red", m_red);
    shader->setUniform("green", m_green);
    shader->setUniform("blue", m_blue);
    shader->setUniform("alpha", m_alpha);
    glBindVertexArray(m_surfaceVao);
    glDrawElements(GL_TRIANGLES, m_numSurfaceVertices, GL_UNSIGNED_INT, reinterpret_cast<GLvoid *>(0));
    glBindVertexArray(0);
    if (m_hasTexture && m_diffuseTex != 0) {
        glBindTexture(GL_TEXTURE_2D, 0);
    }
}

void Shape::clearAnchors() {
    m_anchors.clear();
}

void Shape::buildAnchorVBO()
{
    // Destroy old anchor VBO if exists
    if (m_anchorVao != 0) {
        glDeleteVertexArrays(1, &m_anchorVao);
        glDeleteBuffers(1, &m_anchorVbo);
    }
    m_anchorVao = 0;
    m_anchorVbo = 0;
    m_numAnchors = 0;

    if (m_anchors.empty()) return;

    // Build position + color arrays for anchors only
    std::vector<float> data; // interleaved: x,y,z, r,g,b
    data.reserve(m_anchors.size() * 6);

    for (int idx : m_anchors) {
        if (idx < 0 || idx >= static_cast<int>(m_vertices.size())) continue;
        const Vector3f& pos = m_vertices[idx];
        // Position
        data.push_back(pos.x());
        data.push_back(pos.y());
        data.push_back(pos.z());
        // Color: black for anchors
        data.push_back(0.0f);
        data.push_back(0.0f);
        data.push_back(0.0f);
        m_numAnchors++;
    }

    if (m_numAnchors == 0) return;

    glGenVertexArrays(1, &m_anchorVao);
    glGenBuffers(1, &m_anchorVbo);
    glBindVertexArray(m_anchorVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_anchorVbo);
    glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(float), data.data(), GL_DYNAMIC_DRAW);

    // Position: location 0, stride = 6 floats, offset = 0
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), static_cast<GLvoid*>(0));
    // Color: location 2, stride = 6 floats, offset = 12 bytes
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<GLvoid*>(12));

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void Shape::drawAnchors(Shader *shader)
{
    if (m_numAnchors == 0 || m_anchorVao == 0) return;

    shader->setUniform("model", m_modelMatrix);
    Eigen::Matrix3f id = Eigen::Matrix3f::Identity();
    shader->setUniform("inverseTransposeModel", id);
    glBindVertexArray(m_anchorVao);
    glDrawArrays(GL_POINTS, 0, m_numAnchors);
    glBindVertexArray(0);
}

const vector<Vector3f> &Shape::getVertices() const { return m_vertices; }
const vector<Vector3i> &Shape::getFaces() const { return m_faces; }
const unordered_set<int> &Shape::getAnchors() const { return m_anchors; }
