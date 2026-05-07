#include "glwidget.h"
#include "util/tiny_obj_loader.h"

#include <QApplication>
#include <QKeyEvent>
#include <QFileInfo>
#include <QImage>
#include <QFile>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include "monster.h"

#define SPEED 1.5
#define ROTATE_SPEED 0.0025

using namespace std;

GLWidget::GLWidget(QWidget *parent) :
    QOpenGLWidget(parent),
    m_deltaTimeProvider(),
    m_intervalTimer(),
    m_camera(),
    m_shader(nullptr),
    m_mesh(),
    m_meshLoaded(false),
    m_glInitialized(false),
    m_wireframe(false),
    m_pendingMeshPath(),
    m_forward(),
    m_sideways(),
    m_vertical(),
    m_lastX(),
    m_lastY(),
    m_rightCapture(false),
    m_leftCapture(false),
    m_rightClickSelectMode(SelectMode::None),
    m_lastSelectedVertex(-1)
{
    setMouseTracking(true);
    QApplication::setOverrideCursor(Qt::ArrowCursor);
    setFocusPolicy(Qt::StrongFocus);
    connect(&m_intervalTimer, SIGNAL(timeout()), this, SLOT(tick()));
}

GLWidget::~GLWidget()
{
    makeCurrent();
    if (m_shader != nullptr) {
        delete m_shader;
        m_shader = nullptr;
    }
    m_mesh.destroyGL();
    doneCurrent();
}

void GLWidget::setMeshPath(const std::string &path)
{
    m_pendingMeshPath = path;

    if (m_glInitialized) {
        makeCurrent();
        loadMeshFromFile(path);
        doneCurrent();
        update();
    }
}

void GLWidget::clearMesh()
{
    m_pendingMeshPath.clear();
    if (m_glInitialized) {
        makeCurrent();
        m_mesh.destroyGL();
        // Also tear down ARAP's internal Shape buffers and solver state so the
        // anchors / surface / wireframe from the previous mesh stop drawing.
        m_arap.clear();
        doneCurrent();
    }
    m_meshLoaded = false;
    m_lastSelectedVertex = -1;
    m_rightClickSelectMode = SelectMode::None;
    m_leftCapture = false;
    m_rightCapture = false;
    update();
}

void GLWidget::centerView()
{
    const Eigen::Vector3f eye(0.f, 0.f, -5.f);
    const Eigen::Vector3f target(0.f, 0.f, 0.f);
    m_camera.lookAt(eye, target);
    m_camera.setOrbitPoint(target);
    update();
}

Eigen::Vector3f GLWidget::MakeArap(Eigen::MatrixXd V, Eigen::MatrixXi T)
{
    std::vector<Eigen::Vector3f> vertices;
    vertices.reserve(static_cast<size_t>(V.rows()));
    for (int r = 0; r < V.rows(); ++r) {
        vertices.emplace_back(static_cast<float>(V(r, 0)),
                              static_cast<float>(V(r, 1)),
                              static_cast<float>(V(r, 2)));
    }

    std::vector<Eigen::Vector3i> triangles;
    triangles.reserve(static_cast<size_t>(T.rows()));
    for (int r = 0; r < T.rows(); ++r) {
        triangles.emplace_back(T(r, 0), T(r, 1), T(r, 2));
    }

    Eigen::Vector3f coeffMin = Eigen::Vector3f::Zero();
    Eigen::Vector3f coeffMax = Eigen::Vector3f::Zero();
    m_arap.init(coeffMin, coeffMax, vertices, triangles);
    return coeffMax - coeffMin;
}

// ================== Basic OpenGL Overrides

void GLWidget::initializeGL()
{
    // Initialize GL extension wrangler
    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (err != GLEW_OK) fprintf(stderr, "Error while initializing GLEW: %s\n", glewGetErrorString(err));
    fprintf(stdout, "Successfully initialized GLEW %s\n", glewGetString(GLEW_VERSION));

    // Set clear color to white
    glClearColor(1, 1, 1, 1);

    // Enable depth-testing and backface culling
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    // Initialize shaders
    m_shader = new Shader(":resources/shaders/shader.vert", ":resources/shaders/shader.frag");
    m_pointShader = new Shader(":resources/shaders/anchorPoint.vert", ":resources/shaders/anchorPoint.geom", ":resources/shaders/anchorPoint.frag");

    // Initialize camera aimed at origin so newly loaded meshes appear centered.
    Eigen::Vector3f eye    = {0, 0, -5};
    Eigen::Vector3f target = {0, 0,  0};
    m_camera.lookAt(eye, target);
    m_camera.setOrbitPoint(target);
    m_camera.setPerspective(120, width() / static_cast<float>(height()), 0.1, 50);

    m_deltaTimeProvider.start();
    m_intervalTimer.start(1000 / 60);

    m_glInitialized = true;
    if (!m_pendingMeshPath.empty() && !m_meshLoaded) {
        loadMeshFromFile(m_pendingMeshPath);
    }
}

void GLWidget::paintGL()
{

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (m_meshLoaded && m_shader != nullptr) {
        // Toggle wireframe via polygon mode so it works for any mesh,
        // not just those with tet indices.
        glPolygonMode(GL_FRONT_AND_BACK, m_wireframe ? GL_LINE : GL_FILL);
        m_shader->bind();
        m_shader->setUniform("proj", m_camera.getProjection());
        m_shader->setUniform("view", m_camera.getView());
        m_arap.draw(m_shader);
        m_shader->unbind();
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    glClear(GL_DEPTH_BUFFER_BIT);

    // Don't draw anchor points after a clear / before any mesh is loaded.
    if (m_meshLoaded && m_showAnchors) {
        m_pointShader->bind();
        m_pointShader->setUniform("proj",   m_camera.getProjection());
        m_pointShader->setUniform("view",   m_camera.getView());
        m_pointShader->setUniform("vSize",  m_vSize);
        m_pointShader->setUniform("width",  width());
        m_pointShader->setUniform("height", height());
        m_arap.draw(m_pointShader, GL_POINTS);
        m_pointShader->unbind();
    }
}

void GLWidget::setAnchorsVisible(bool visible)
{
    if (m_showAnchors == visible) return;
    m_showAnchors = visible;
    update();
}

void GLWidget::toggleAnchorsVisible()
{
    setAnchorsVisible(!m_showAnchors);
}

void GLWidget::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
    m_camera.setAspect(static_cast<float>(w) / std::max(h, 1));
}

void GLWidget::loadMeshFromFile(const std::string &path)
{
    if (!QFileInfo::exists(QString::fromStdString(path))) {
        std::cerr << "OBJ file not found: " << path << std::endl;
        return;
    }

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string err;
    bool ok = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, path.c_str(),
                               /*mtl_basedir=*/nullptr, /*triangulate=*/true);
    if (!err.empty()) std::cerr << "tinyobj: " << err << std::endl;
    if (!ok) {
        std::cerr << "Failed to load OBJ: " << path << std::endl;
        return;
    }

    // Load as float from the start
    std::vector<Eigen::Vector3f> vertices;
    vertices.reserve(attrib.vertices.size() / 3);
    for (size_t i = 0; i + 2 < attrib.vertices.size(); i += 3) {
        vertices.emplace_back(attrib.vertices[i + 0],
                              attrib.vertices[i + 1],
                              attrib.vertices[i + 2]);
    }

    std::vector<Eigen::Vector3i> triangles;
    std::vector<Eigen::Vector2f> cornerUVs;
    const bool hasTexCoords = !attrib.texcoords.empty();

    auto cornerUv = [&](const tinyobj::index_t &idx) -> Eigen::Vector2f {
        if (hasTexCoords && idx.texcoord_index >= 0) {
            const int ti = idx.texcoord_index;
            const size_t o = static_cast<size_t>(ti) * 2;
            if (o + 1 < attrib.texcoords.size()) {
                return {static_cast<float>(attrib.texcoords[o]),
                        static_cast<float>(attrib.texcoords[o + 1])};
            }
        }
        return {0.f, 0.f};
    };

    for (const auto &shape : shapes) {
        size_t index_offset = 0;
        for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); ++f) {
            int fv = shape.mesh.num_face_vertices[f];
            if (fv < 3) {
                index_offset += static_cast<size_t>(fv);
                continue;
            }

            const tinyobj::index_t &i0 = shape.mesh.indices[index_offset + 0];
            for (int k = 1; k + 1 < fv; ++k) {
                const tinyobj::index_t &i1 = shape.mesh.indices[index_offset + k];
                const tinyobj::index_t &i2 = shape.mesh.indices[index_offset + k + 1];
                triangles.emplace_back(i0.vertex_index, i1.vertex_index, i2.vertex_index);
                cornerUVs.push_back(cornerUv(i0));
                cornerUVs.push_back(cornerUv(i1));
                cornerUVs.push_back(cornerUv(i2));
            }
            index_offset += static_cast<size_t>(fv);
        }
    }

    if (vertices.empty() || triangles.empty()) {
        std::cerr << "OBJ has no mesh data: " << path << std::endl;
        return;
    }

    // Center and normalize scale using float
    Eigen::Vector3f mn = vertices.front();
    Eigen::Vector3f mx = vertices.front();
    for (const auto &v : vertices) {
        mn = mn.cwiseMin(v);
        mx = mx.cwiseMax(v);
    }
    const Eigen::Vector3f center = 0.5f * (mn + mx);
    const float diag = (mx - mn).norm();
    const float scale = (diag > 1e-8f) ? (2.0f / diag) : 1.0f;
    for (auto &v : vertices) {
        v = (v - center) * scale;
    }

    const QFileInfo objFi(QString::fromStdString(path));
    const QString texPath =
        objFi.absolutePath() + QLatin1Char('/') + objFi.completeBaseName() + QStringLiteral("_texture.png");

    GLuint texId = 0;
    if (QFile::exists(texPath)) {
        QImage img(texPath);
        if (!img.isNull()) {
            img = img.convertToFormat(QImage::Format_RGBA8888);
            glGenTextures(1, &texId);
            glBindTexture(GL_TEXTURE_2D, texId);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img.width(), img.height(), 0,
                         GL_RGBA, GL_UNSIGNED_BYTE, img.constBits());
            glBindTexture(GL_TEXTURE_2D, 0);
        }
    }

    const bool textured =
        (texId != 0 && hasTexCoords && cornerUVs.size() == triangles.size() * 3);
    if (texId != 0 && !textured) {
        glDeleteTextures(1, &texId);
        texId = 0;
    }

    // Try to load ARAP-L metadata sidecar (<basename>_arapl.txt) written by monster.
    ArapLMetadata meta;
    bool hasMeta = false;
    {
        const QString metaPath = objFi.absolutePath() + QLatin1Char('/') +
                                 objFi.completeBaseName() + QStringLiteral("_arapl.txt");
        if (QFile::exists(metaPath)) {
            std::ifstream in(metaPath.toStdString());
            if (in) {
                std::string tag;
                int n = 0;
                int pCount = 0;
                in >> tag >> n;
                in >> tag >> pCount;
                meta.partDepth.assign(std::max(0, pCount), 0);
                for (int i = 0; i < pCount; ++i) in >> meta.partDepth[i];
                in >> tag; // "v"
                meta.partId.assign(std::max(0, n), -1);
                meta.curveType.assign(std::max(0, n), CurveType::Interior);
                for (int i = 0; i < n; ++i) {
                    int pid = -1;
                    int ctype = 0;
                    in >> pid >> ctype;
                    meta.partId[i] = pid;
                    meta.curveType[i] = static_cast<CurveType>(ctype);
                }
                int eqCount = 0;
                in >> tag >> eqCount;
                meta.equalityPairs.reserve(std::max(0, eqCount));
                for (int k = 0; k < eqCount; ++k) {
                    int a = -1, b = -1;
                    in >> a >> b;
                    meta.equalityPairs.push_back({a, b});
                }
                hasMeta = static_cast<int>(meta.partId.size()) ==
                          static_cast<int>(vertices.size());
                if (!hasMeta) {
                    std::cerr << "ARAP-L sidecar size mismatch (" << meta.partId.size()
                              << " vs " << vertices.size() << "), ignoring." << std::endl;
                }
            }
        }
    }

    // Initialize ARAP with the mesh (and ARAP-L metadata when available).
    Eigen::Vector3f coeffMin, coeffMax;
    if (hasMeta) {
        m_arap.init(coeffMin, coeffMax, vertices, triangles, meta);
    } else {
        m_arap.init(coeffMin, coeffMax, vertices, triangles);
    }

    // Push the painted texture into ARAP's render Shape so paint is visible on
    // the same mesh that ARAP deforms. Any previously-bound texture for this
    // Shape is replaced; UV bytes will be preserved by setVertices() under
    // subsequent ARAP::move() calls.
    if (textured) {
        m_arap.initShapeWithTexture(vertices, triangles, cornerUVs, texId);
    }

    float extentLength = (coeffMax - coeffMin).norm();
    m_vSize = 0.005f * extentLength;
    m_movementScaling = extentLength * 0.5f;
    m_vertexSelectionThreshold = extentLength * 0.025f;

    m_meshLoaded = true;

    std::cout << "Loaded OBJ: " << path
              << "  verts=" << vertices.size()
              << "  tris="  << triangles.size()
              << "  textured=" << (textured ? "yes" : "no")
              << std::endl;
}

// Event Listeners

void GLWidget::mousePressEvent(QMouseEvent *event)
{
    const int currX = event->position().x();
    const int currY = event->position().y();

    const Eigen::Vector3f ray = transformToWorldRay(currX, currY);
    const int closest_vertex = m_arap.getClosestVertex(m_camera.getPosition(), ray, m_vertexSelectionThreshold);

    // Switch on button
    switch (event->button()) {
    case Qt::MouseButton::RightButton: {
        m_rightCapture = true;
        m_rightClickSelectMode = m_arap.select(m_pointShader, closest_vertex);
        break;
    }
    case Qt::MouseButton::LeftButton: {
        m_leftCapture = true;
        m_lastSelectedVertex = closest_vertex;
        break;
    }
    default: break;
    }

    // Set last mouse coordinates
    m_lastX = currX;
    m_lastY = currY;
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
    // Return if neither mouse button is currently held down
    if (!(m_leftCapture || m_rightCapture)) {
        return;
    }

    // Get current mouse coordinates
    const int currX = event->position().x();
    const int currY = event->position().y();

    const Eigen::Vector3f ray = transformToWorldRay(event->position().x(), event->position().y());

    // If right is held down
    if (m_rightCapture) {
        const int closest_vertex = m_arap.getClosestVertex(m_camera.getPosition(), ray, m_vertexSelectionThreshold);

        // Anchor/un-anchor the vertex
        if (m_rightClickSelectMode == SelectMode::None) {
            m_rightClickSelectMode = m_arap.select(m_pointShader, closest_vertex);
        } else {
            m_arap.selectWithSpecifiedMode(m_pointShader, closest_vertex, m_rightClickSelectMode);
        }

        return;
    }

    Eigen::Vector3f pos;
    if (m_lastSelectedVertex != -1 && m_arap.getAnchorPos(m_lastSelectedVertex, pos, ray, m_camera.getPosition())) {
        m_arap.move(m_lastSelectedVertex, pos);
    } else {
        // Rotate the camera
        const int deltaX = currX - m_lastX;
        const int deltaY = currY - m_lastY;
        if (deltaX != 0 || deltaY != 0) {
            m_camera.rotate(deltaY * ROTATE_SPEED, -deltaX * ROTATE_SPEED);
        }
    }

    // Set last mouse coordinates
    m_lastX = currX;
    m_lastY = currY;
}

void GLWidget::mouseReleaseEvent(QMouseEvent *event)
{
    m_leftCapture = false;
    m_lastSelectedVertex = -1;

    m_rightCapture = false;
    m_rightClickSelectMode = SelectMode::None;
}

void GLWidget::wheelEvent(QWheelEvent *event)
{
    float zoom = 1.0f - event->pixelDelta().y() * 0.1f / 120.0f;
    m_camera.zoom(zoom);
}

void GLWidget::keyPressEvent(QKeyEvent *event)
{
    if (event->isAutoRepeat()) return;

    switch (event->key())
    {
    case Qt::Key_W: m_forward  += SPEED; break;
    case Qt::Key_S: m_forward  -= SPEED; break;
    case Qt::Key_A: m_sideways -= SPEED; break;
    case Qt::Key_D: m_sideways += SPEED; break;
    case Qt::Key_F: m_vertical -= SPEED; break;
    case Qt::Key_R: m_vertical += SPEED; break;
    case Qt::Key_C: m_camera.toggleIsOrbiting(); break;
    case Qt::Key_Equal: m_vSize *= 11.0f / 10.0f; break;
    case Qt::Key_Minus: m_vSize *= 10.0f / 11.0f; break;
    case Qt::Key_P: toggleAnchorsVisible(); break;
    case Qt::Key_Escape: QApplication::quit();
    }
}

void GLWidget::keyReleaseEvent(QKeyEvent *event)
{
    if (event->isAutoRepeat()) return;

    switch (event->key())
    {
    case Qt::Key_W: m_forward  -= SPEED; break;
    case Qt::Key_S: m_forward  += SPEED; break;
    case Qt::Key_A: m_sideways += SPEED; break;
    case Qt::Key_D: m_sideways -= SPEED; break;
    case Qt::Key_F: m_vertical += SPEED; break;
    case Qt::Key_R: m_vertical -= SPEED; break;
    }
}

//Physics Tick

void GLWidget::tick()
{
    float deltaSeconds = m_deltaTimeProvider.restart() / 1000.0f;

    auto look = m_camera.getLook();
    look.y() = 0;
    if (look.squaredNorm() > 1e-8f) look.normalize();
    Eigen::Vector3f perp(-look.z(), 0, look.x());
    Eigen::Vector3f moveVec = m_forward * look
                              + m_sideways * perp
                              + m_vertical * Eigen::Vector3f::UnitY();
    moveVec *= deltaSeconds;
    m_camera.move(moveVec);
    update();
}

Eigen::Vector3f GLWidget::transformToWorldRay(int x, int y)
{
    Eigen::Vector4f clipCoords = Eigen::Vector4f(
        (float(x) / width()) * 2.0f - 1.0f,
        1.0f - (float(y) / height()) * 2.0f,
        -1.0f,
        1.0f);

    Eigen::Vector4f transformed_coords = m_camera.getProjection().inverse() * clipCoords;
    transformed_coords = Eigen::Vector4f(transformed_coords.x(), transformed_coords.y(), -1.0f, 0.0f);
    transformed_coords = m_camera.getView().inverse() * transformed_coords;

    return Eigen::Vector3f(transformed_coords.x(), transformed_coords.y(), transformed_coords.z()).normalized();
}

