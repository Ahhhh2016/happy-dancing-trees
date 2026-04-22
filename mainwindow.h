#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSlider>
#include <QSpinBox>
#include <QRadioButton>
#include <QLabel>
#include <QPushButton>
#include <QBoxLayout>
#include <QStackedWidget>
#include <QPointer>

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
    void onRevertButtonClick();
    void onUploadButtonClick();
    void onSaveButtonClick();
    void onToggleMeshViewClick();

private:
    // Runs the mesh inflation pipeline; returns the path to the written OBJ,
    // or an empty string on failure.
    QString buildMeshAndSaveObj();

private:
    QString          m_lastMeshPath;
    QStackedWidget  *m_viewStack;
    int              m_canvasPageIndex;
    int              m_meshPageIndex;
    QPushButton     *m_toggleMeshButton;
    GLWidget        *m_glWidget;
};
#endif // MAINWINDOW_H
