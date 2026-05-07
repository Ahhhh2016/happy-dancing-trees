#pragma once

#include "animation.h"
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
#include <string>
#include "animation.h"

class GLWidget : public QOpenGLWidget
{
    Q_OBJECT

public slots:
    void toggleRecordAnimation();
    void togglePlayAnimation();
    void stopAnimation();
    void saveAnimation();
    void loadAnimation();
public:
    GLWidget(QWidget *parent = nullptr);
    ~GLWidget();

    bool m_isAnimating = false;

    // Queue an OBJ to display. If GL is already initialized, loads immediately.
    void setMeshPath(const std::string &path);
    // Remove any currently displayed mesh and clear the viewport.
    void clearMesh();
    // Reset the camera to the default centered framing.
    void centerView();

    Eigen::Vector3f MakeArap(Eigen::MatrixXd V, Eigen::MatrixXi T);

    Eigen::Vector3f transformToWorldRay(int x, int y);

    void loadMeshFromFile(const std::string &path);


    bool isAnimationRecording() const { return m_animationRecording; }
    bool isAnimationPlaying() const { return m_animationPlaying; }
    void setAnchorsVisible(bool visible);


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

    Animation m_animation;
    bool m_animationRecording = false;
    bool m_animationPlaying = false;
    bool m_anchorsVisible = true;

private slots:

    // Physics Tick
    void tick();
};
