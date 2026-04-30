#include "shape.h"

#include <iostream>

#include "graphics/shader.h"

using namespace Eigen;

#if defined(_OPENMP)
#define OMP_PARALLEL_FOR _Pragma("omp parallel for schedule(static)")
#else
#define OMP_PARALLEL_FOR
#endif

Shape::Shape()
    : m_tetVao(static_cast<GLuint>(-1)),
      m_surfaceVao(0),
      m_surfaceVbo(0),
      m_surfaceIbo(0),
      m_tetVbo(0),
      m_tetIbo(0),
      m_numSurfaceVertices(),
      m_verticesSize(),
      m_modelMatrix(Eigen::Matrix4f::Identity()),
      m_wireframe(false)
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

void Shape::init(const std::vector<Eigen::Vector3d> &vertices, const std::vector<Eigen::Vector3d> &normals, const std::vector<Eigen::Vector3i> &triangles)
{
    if(vertices.size() != normals.size()) {
        std::cerr << "Vertices and normals are not the same size" << std::endl;
        return;
    }
    destroySurfaceGL();

    std::vector<Eigen::Vector2f> uvZero(vertices.size(), Eigen::Vector2f::Zero());
    const size_t n = vertices.size();
    const GLsizeiptr posBytes = static_cast<GLsizeiptr>(sizeof(double) * n * 3);
    const GLsizeiptr normBytes = static_cast<GLsizeiptr>(sizeof(double) * n * 3);
    const GLsizeiptr uvBytes = static_cast<GLsizeiptr>(sizeof(float) * n * 2);

    glGenBuffers(1, &m_surfaceVbo);
    glGenBuffers(1, &m_surfaceIbo);
    glGenVertexArrays(1, &m_surfaceVao);

    glBindBuffer(GL_ARRAY_BUFFER, m_surfaceVbo);
    glBufferData(GL_ARRAY_BUFFER, posBytes + normBytes + uvBytes, nullptr, GL_DYNAMIC_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, posBytes, static_cast<const void *>(vertices.data()));
    glBufferSubData(GL_ARRAY_BUFFER, posBytes, normBytes, static_cast<const void *>(normals.data()));
    glBufferSubData(GL_ARRAY_BUFFER, posBytes + normBytes, uvBytes, static_cast<const void *>(uvZero.data()));
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_surfaceIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * 3 * triangles.size(), static_cast<const void *>(triangles.data()), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    glBindVertexArray(m_surfaceVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_surfaceVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 0, static_cast<GLvoid *>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_DOUBLE, GL_FALSE, 0, reinterpret_cast<GLvoid *>(posBytes));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, reinterpret_cast<GLvoid *>(posBytes + normBytes));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_surfaceIbo);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    m_numSurfaceVertices = static_cast<unsigned int>(triangles.size() * 3);
    m_verticesSize = static_cast<unsigned int>(vertices.size());
    m_faces = triangles;
}

void Shape::init(const std::vector<Eigen::Vector3d> &vertices, const std::vector<Eigen::Vector3i> &triangles)
{
    init(vertices, triangles, {}, 0);
}

void Shape::init(const std::vector<Eigen::Vector3d> &vertices, const std::vector<Eigen::Vector3i> &triangles,
                 const std::vector<Eigen::Vector2f> &texcoordsPerCorner, GLuint diffuseTexture)
{
    destroySurfaceGL();

    std::vector<Eigen::Vector3d> verts;
    std::vector<Eigen::Vector3d> normals;
    std::vector<Eigen::Vector3i> faces;
    std::vector<Eigen::Vector2f> uvs;
    verts.reserve(triangles.size() * 3);
    normals.reserve(triangles.size() * 3);
    faces.reserve(triangles.size());
    uvs.reserve(triangles.size() * 3);

    const bool useTex = (diffuseTexture != 0 &&
                         texcoordsPerCorner.size() == triangles.size() * 3);

    for (size_t fi = 0; fi < triangles.size(); ++fi) {
        const auto &f = triangles[fi];
        const auto &v1 = vertices[f[0]];
        const auto &v2 = vertices[f[1]];
        const auto &v3 = vertices[f[2]];
        const auto e1 = v2 - v1;
        const auto e2 = v3 - v1;
        const auto n = e1.cross(e2);
        const int s = static_cast<int>(verts.size());
        faces.emplace_back(s, s + 1, s + 2);
        normals.push_back(n);
        normals.push_back(n);
        normals.push_back(n);
        verts.push_back(v1);
        verts.push_back(v2);
        verts.push_back(v3);
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

    const size_t n = verts.size();
    const GLsizeiptr posBytes = static_cast<GLsizeiptr>(sizeof(double) * n * 3);
    const GLsizeiptr normBytes = static_cast<GLsizeiptr>(sizeof(double) * n * 3);
    const GLsizeiptr uvBytes = static_cast<GLsizeiptr>(sizeof(float) * n * 2);

    glGenBuffers(1, &m_surfaceVbo);
    glGenBuffers(1, &m_surfaceIbo);
    glGenVertexArrays(1, &m_surfaceVao);

    glBindBuffer(GL_ARRAY_BUFFER, m_surfaceVbo);
    glBufferData(GL_ARRAY_BUFFER, posBytes + normBytes + uvBytes, nullptr, GL_DYNAMIC_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, posBytes, static_cast<const void *>(verts.data()));
    glBufferSubData(GL_ARRAY_BUFFER, posBytes, normBytes, static_cast<const void *>(normals.data()));
    glBufferSubData(GL_ARRAY_BUFFER, posBytes + normBytes, uvBytes, static_cast<const void *>(uvs.data()));
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_surfaceIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * 3 * faces.size(), static_cast<const void *>(faces.data()), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    glBindVertexArray(m_surfaceVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_surfaceVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 0, static_cast<GLvoid *>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_DOUBLE, GL_FALSE, 0, reinterpret_cast<GLvoid *>(posBytes));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, reinterpret_cast<GLvoid *>(posBytes + normBytes));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_surfaceIbo);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    m_diffuseTex = diffuseTexture;
    m_hasTexture = useTex;

    m_numSurfaceVertices = static_cast<unsigned int>(faces.size() * 3);
    m_verticesSize = static_cast<unsigned int>(vertices.size());
    m_faces = triangles;

    if (vertices.size() > 4) { //shape
        m_red = 0.93f;
        m_green = 0.8f;
        m_blue = 1.f;
        m_alpha = 1.f;
    } else { //ground
        m_red = 1;
        m_green = 1;
        m_blue = 1;
        m_alpha = 1.f;
    }
}

void Shape::init(const std::vector<Eigen::Vector3d> &vertices, const std::vector<Eigen::Vector3i> &triangles, const std::vector<Eigen::Vector4i> &tetIndices)
{
    destroyTetGL();
    init(vertices, triangles);

    std::vector<Eigen::Vector2i> lines;
    for(Vector4i tet : tetIndices) {
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
    glBufferData(GL_ARRAY_BUFFER, sizeof(double) * vertices.size() * 3, vertices.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_tetIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * 2 * lines.size(), static_cast<const void *>(lines.data()), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    glBindVertexArray(m_tetVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_tetVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 0, static_cast<GLvoid *>(0));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_tetIbo);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    m_numTetVertices = lines.size() * 2;
}

void Shape::setVertices(const std::vector<Eigen::Vector3d> &vertices)
{
    if(vertices.size() != m_verticesSize) {
        std::cerr << "You can't set vertices to a vector that is a different length that what shape was inited with" << std::endl;
        return;
    }
    std::vector<Eigen::Vector3d> verts(m_faces.size() * 3);
    std::vector<Eigen::Vector3d> normals(m_faces.size() * 3);
    const int faceCount = static_cast<int>(m_faces.size());
    OMP_PARALLEL_FOR
    for(int fi = 0; fi < faceCount; ++fi) {
        const auto &f = m_faces[fi];
        const auto &v1 = vertices[f[0]];
        const auto &v2 = vertices[f[1]];
        const auto &v3 = vertices[f[2]];
        const auto e1 = v2 - v1;
        const auto e2 = v3 - v1;
        const auto n = e1.cross(e2);
        const int base = 3 * fi;
        normals[base] = n;
        normals[base + 1] = n;
        normals[base + 2] = n;
        verts[base] = v1;
        verts[base + 1] = v2;
        verts[base + 2] = v3;
    }
    glBindBuffer(GL_ARRAY_BUFFER, m_surfaceVbo);
    const size_t n = static_cast<size_t>(m_faces.size()) * 3u;
    const GLsizeiptr posBytes = static_cast<GLsizeiptr>(sizeof(double) * n * 3);
    const GLsizeiptr normBytes = static_cast<GLsizeiptr>(sizeof(double) * n * 3);
    glBufferSubData(GL_ARRAY_BUFFER, 0, posBytes, static_cast<const void *>(verts.data()));
    glBufferSubData(GL_ARRAY_BUFFER, posBytes, normBytes, static_cast<const void *>(normals.data()));
    if(m_tetVao != static_cast<GLuint>(-1)) {
        glBindBuffer(GL_ARRAY_BUFFER, m_tetVbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(double) * vertices.size() * 3, static_cast<const void *>(vertices.data()));
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void Shape::setModelMatrix(const Eigen::Affine3f &model)
{
    m_modelMatrix = model.matrix();
}

void Shape::toggleWireframe()
{
    m_wireframe = !m_wireframe;
}

void Shape::setVertices(const std::vector<Eigen::Vector3d> &vertices, const std::vector<Eigen::Vector3d> &normals)
{
    if(vertices.size() != normals.size()) {
        std::cerr << "Vertices and normals are not the same size" << std::endl;
        return;
    }
    if(vertices.size() != m_verticesSize) {
        std::cerr << "You can't set vertices to a vector that is a different length that what shape was inited with" << std::endl;
        return;
    }
    glBindBuffer(GL_ARRAY_BUFFER, m_surfaceVbo);
    const size_t n = vertices.size();
    const GLsizeiptr posBytes = static_cast<GLsizeiptr>(sizeof(double) * n * 3);
    const GLsizeiptr normBytes = static_cast<GLsizeiptr>(sizeof(double) * n * 3);
    glBufferSubData(GL_ARRAY_BUFFER, 0, posBytes, static_cast<const void *>(vertices.data()));
    glBufferSubData(GL_ARRAY_BUFFER, posBytes, normBytes, static_cast<const void *>(normals.data()));
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void Shape::draw(Shader *shader)
{
    Eigen::Matrix3f m3 = m_modelMatrix.topLeftCorner(3, 3);
    Eigen::Matrix3f inverseTransposeModel = m3.inverse().transpose();

    if(m_wireframe && m_tetVao != static_cast<GLuint>(-1)) {
        shader->setUniform("wire", 1);
        shader->setUniform("useTexture", 0);
        shader->setUniform("model", m_modelMatrix);
        shader->setUniform("inverseTransposeModel", inverseTransposeModel);
        shader->setUniform("red",   1);
        shader->setUniform("green", 1);
        shader->setUniform("blue",  1);
        shader->setUniform("alpha", 1);
        glBindVertexArray(m_tetVao);
        glDrawElements(GL_LINES, m_numTetVertices, GL_UNSIGNED_INT, reinterpret_cast<GLvoid *>(0));
        glBindVertexArray(0);
    } else {
        shader->setUniform("wire", 0);
        shader->setUniform("model", m_modelMatrix);
        shader->setUniform("inverseTransposeModel", inverseTransposeModel);
        shader->setUniform("useTexture", m_hasTexture ? 1 : 0);
        if (m_hasTexture && m_diffuseTex != 0) {
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, m_diffuseTex);
            shader->setUniform("diffuseTex", 0);
        }
        shader->setUniform("red",   m_red);
        shader->setUniform("green", m_green);
        shader->setUniform("blue",  m_blue);
        shader->setUniform("alpha", m_alpha);
        glBindVertexArray(m_surfaceVao);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(m_numSurfaceVertices), GL_UNSIGNED_INT, reinterpret_cast<GLvoid *>(0));
        glBindVertexArray(0);
        if (m_hasTexture && m_diffuseTex != 0) {
            glBindTexture(GL_TEXTURE_2D, 0);
        }
    }
}
