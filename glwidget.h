#pragma once

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif

#include "graphics/camera.h"
#include "graphics/shader.h"

#include <QOpenGLWidget>
#include <QElapsedTimer>
#include <QPointF>
#include <QTimer>
#include <memory>

class GLWidget : public QOpenGLWidget
{
    Q_OBJECT

public:
    GLWidget(QWidget *parent = nullptr);
    ~GLWidget();

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
    void screenPointToRay(const QPointF &screenPos, Eigen::Vector3f &rayOrigin, Eigen::Vector3f &rayDir);

private:
    QElapsedTimer m_deltaTimeProvider; // For measuring elapsed time
    QTimer        m_intervalTimer;     // For triggering timed events

    Camera     m_camera;
    Shader    *m_shader;

    int m_forward;
    int m_sideways;
    int m_vertical;

    int m_lastX;
    int m_lastY;

    bool m_rotateCapture;
    bool m_dragCapture;

private slots:

    // Physics Tick
    void tick();
};
