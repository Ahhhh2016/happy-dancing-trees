#include "glwidget.h"

#include "util/tiny_obj_loader.h"

#include <QApplication>
#include <QKeyEvent>
#include <QFileInfo>
#include <algorithm>
#include <cmath>
#include <iostream>

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
    m_rotateCapture(false)
{
    setMouseTracking(true);
    QApplication::setOverrideCursor(Qt::ArrowCursor);
    setFocusPolicy(Qt::StrongFocus);
    connect(&m_intervalTimer, SIGNAL(timeout()), this, SLOT(tick()));
}

GLWidget::~GLWidget()
{
    if (m_shader != nullptr) delete m_shader;
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

    // Initialize the shader
    m_shader = new Shader(":/resources/shaders/shader.vert", ":/resources/shaders/shader.frag");

    // Initialize camera with a reasonable transform
    Eigen::Vector3f eye    = {0, 2, -5};
    Eigen::Vector3f target = {0, 1,  0};
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
        m_mesh.draw(m_shader);
        m_shader->unbind();
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
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
    // Older tinyobjloader API: (attrib, shapes, materials, err, filename, ...)
    bool ok = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, path.c_str(),
                               /*mtl_basedir=*/nullptr, /*triangulate=*/true);
    if (!err.empty()) std::cerr << "tinyobj: " << err << std::endl;
    if (!ok) {
        std::cerr << "Failed to load OBJ: " << path << std::endl;
        return;
    }

    std::vector<Eigen::Vector3d> vertices;
    vertices.reserve(attrib.vertices.size() / 3);
    for (size_t i = 0; i + 2 < attrib.vertices.size(); i += 3) {
        vertices.emplace_back(attrib.vertices[i + 0],
                              attrib.vertices[i + 1],
                              attrib.vertices[i + 2]);
    }

    std::vector<Eigen::Vector3i> triangles;
    for (const auto &shape : shapes) {
        size_t index_offset = 0;
        for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); ++f) {
            int fv = shape.mesh.num_face_vertices[f];
            if (fv < 3) { index_offset += fv; continue; }

            int i0 = shape.mesh.indices[index_offset + 0].vertex_index;
            for (int k = 1; k + 1 < fv; ++k) {
                int i1 = shape.mesh.indices[index_offset + k].vertex_index;
                int i2 = shape.mesh.indices[index_offset + k + 1].vertex_index;
                triangles.emplace_back(i0, i1, i2);
            }
            index_offset += fv;
        }
    }

    if (vertices.empty() || triangles.empty()) {
        std::cerr << "OBJ has no mesh data: " << path << std::endl;
        return;
    }

    // Center and normalize scale so the mesh fits a unit-ish view.
    Eigen::Vector3d mn = vertices.front();
    Eigen::Vector3d mx = vertices.front();
    for (const auto &v : vertices) {
        mn = mn.cwiseMin(v);
        mx = mx.cwiseMax(v);
    }
    const Eigen::Vector3d center = 0.5 * (mn + mx);
    const double diag = (mx - mn).norm();
    const double scale = (diag > 1e-8) ? (2.0 / diag) : 1.0;
    for (auto &v : vertices) {
        v = (v - center) * scale;
    }

    m_mesh.init(vertices, triangles);
    m_meshLoaded = true;

    std::cout << "Loaded OBJ: " << path
              << "  verts=" << vertices.size()
              << "  tris="  << triangles.size() << std::endl;
}

// ================== Event Listeners

void GLWidget::mousePressEvent(QMouseEvent *event)
{
    m_lastX = event->position().x();
    m_lastY = event->position().y();

    if (event->button() == Qt::LeftButton) {
        m_rotateCapture = true;
    }
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
    int currX  = event->position().x();
    int currY  = event->position().y();

    int deltaX = currX - m_lastX;
    int deltaY = currY - m_lastY;

    if (m_rotateCapture && (deltaX != 0 || deltaY != 0)) {
        m_camera.rotate(deltaY * ROTATE_SPEED,
                        -deltaX * ROTATE_SPEED);
    }

    m_lastX = currX;
    m_lastY = currY;
}

void GLWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        m_rotateCapture = false;
    }
}

void GLWidget::wheelEvent(QWheelEvent *event)
{
    float zoom = 1 - event->pixelDelta().y() * 0.1f / 120.f;
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
    case Qt::Key_T: m_wireframe = !m_wireframe; update(); break;
    case Qt::Key_Escape: close(); break;
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

// ================== Physics Tick

void GLWidget::tick()
{
    float deltaSeconds = m_deltaTimeProvider.restart() / 1000.f;

    auto look = m_camera.getLook();
    look.y() = 0;
    if (look.squaredNorm() > 1e-8f) look.normalize();
    Eigen::Vector3f perp(-look.z(), 0, look.x());
    Eigen::Vector3f moveVec = m_forward * look
                            + m_sideways * perp
                            + m_vertical * Eigen::Vector3f::UnitY();
    moveVec *= deltaSeconds;
    m_camera.move(moveVec);

    // Flag this view for repainting (Qt will call paintGL() soon after)
    update();
}
