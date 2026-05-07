#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSlider>
#include <QSpinBox>
#include <QRadioButton>
#include <QLabel>
#include <QPushButton>
#include <QBoxLayout>
#include <QSplitter>
#include <QPointer>
#include <QButtonGroup>
#include <QFrame>
#include <QTimer>
#include <QList>
#include <QCheckBox>

#include "canvas2d.h"
#include "glwidget.h"

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    MainWindow();

private:
    void setupCanvas2D();
    Canvas2D *m_canvas;

    void addPushButton(QBoxLayout *layout, QString text,
                       void (MainWindow::*slot)());

private slots:
    void onClearButtonClick();
    void onUploadButtonClick();
    void onSaveButtonClick();
    void onMeshResolutionSliderReleased();

private:
    double maxTriangleAreaFromSlider() const;
    void requestAsyncMeshRebuildIfIdle();
    void startAsyncSliderMeshRebuild();
    void finishAsyncSliderMeshRebuild();
    void applyCanvasViewMode();
    void applyAnimationViewMode();

private:
    QString          m_lastMeshPath;
    QSplitter       *m_mainSplitter;
    QSlider         *m_meshResolutionSlider;
    QRadioButton    *m_brushToolRadio;
    QFrame          *m_paintColorSwatch;
    QSlider         *m_paintRadiusSlider;
    GLWidget        *m_glWidget;
    bool             m_sliderMeshRebuildBusy = false;
    bool             m_sliderMeshRebuildPending = false;
    QTimer           m_meshPreviewDebounceTimer;
    QList<int>       m_cachedSplitterSizes;

    // animation add ons
    QPushButton *m_recordButton;
    QPushButton *m_playButton;
    QPushButton *m_stopButton;
    QPushButton *m_saveAnimButton;
    QPushButton *m_loadAnimButton;
    QPushButton *m_toggleAnchorsButton;
    bool m_anchorsVisible = true;
    void updateAnimationButtonStates();
};
#endif // MAINWINDOW_H
