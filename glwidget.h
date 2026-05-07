#pragma once

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif

#include "arap.h"
#include "graphics/camera.h"
#include "graphics/shader.h"
#include "graphics/shape.h"

#include <QOpenGLWidget>
#include <QElapsedTimer>
#include <QPointF>
#include <QTimer>
#include <memory>
#include <string>

class GLWidget : public QOpenGLWidget
{
    Q_OBJECT

public:
    GLWidget(QWidget *parent = nullptr);
    ~GLWidget();

    // Queue an OBJ to display. If GL is already initialized, loads immediately.
    void setMeshPath(const std::string &path);
    // Remove any currently displayed mesh and clear the viewport.
    void clearMesh();
    // Reset the camera to the default centered framing.
    void centerView();

    Eigen::Vector3f MakeArap(Eigen::MatrixXd V, Eigen::MatrixXi T);

    Eigen::Vector3f transformToWorldRay(int x, int y);

    void loadMeshFromFile(const std::string &path);

    // Show/hide the red ARAP vertex/anchor point overlay.
    void setAnchorsVisible(bool visible);
    void toggleAnchorsVisible();
    bool anchorsVisible() const { return m_showAnchors; }

private:
    static const int FRAMES_TO_AVERAGE = 30;

private:
    // Basic OpenGL Overrides
    void initializeGL()         override;
    void paintGL()              override;
    void resizeGL(int w, int h) override;

    // Event Listeners
    void mousePressEvent  (QMouseEvent *event) override;
    void mouseMoveEvent   (QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent       (QWheelEvent *event) override;
    void keyPressEvent    (QKeyEvent   *event) override;
    void keyReleaseEvent  (QKeyEvent   *event) override;

private:

private:
    QElapsedTimer m_deltaTimeProvider; // For measuring elapsed time
    QTimer        m_intervalTimer;     // For triggering timed events

    Camera     m_camera;

    Shader *m_shader;
    Shader *m_pointShader;

    Shape       m_mesh;
    bool        m_meshLoaded;
    bool        m_glInitialized;
    bool        m_wireframe;
    std::string m_pendingMeshPath;

    int m_forward;
    int m_sideways;
    int m_vertical;

    int m_lastX;
    int m_lastY;

    bool m_rotateCapture;

    ARAP    m_arap;
    // Mouse handler stuff
    bool m_leftCapture;
    bool m_rightCapture;
    SelectMode m_rightClickSelectMode;
    int m_lastSelectedVertex = -1;

    float m_movementScaling;
    float m_vertexSelectionThreshold;
    float m_vSize;
    bool m_showAnchors = true;

private slots:

    // Physics Tick
    void tick();
};
