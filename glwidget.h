#pragma once

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif

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
    void loadMeshFromFile(const std::string &path);

private:
    QElapsedTimer m_deltaTimeProvider; // For measuring elapsed time
    QTimer        m_intervalTimer;     // For triggering timed events

    Camera     m_camera;
    Shader    *m_shader;

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

private slots:

    // Physics Tick
    void tick();
};
