#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSlider>
#include <QSpinBox>
#include <QRadioButton>
#include <QLabel>
#include <QPushButton>
#include <QBoxLayout>

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
    void onBuildMeshButtonClick();

private:
    GLWidget *glWidget;
};
#endif // MAINWINDOW_H
