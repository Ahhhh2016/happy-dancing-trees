#include "mainwindow.h"
#include "settings.h"
#include "monster.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QFileInfo>
#include <QDir>
#include <QLabel>
#include <QGroupBox>
#include <QScrollArea>
#include <QMessageBox>
#include <iostream>

MainWindow::MainWindow()
{
    setWindowTitle("2D Projects");

    settings.loadSettingsOrDefaults();

    QHBoxLayout *hLayout = new QHBoxLayout();
    QVBoxLayout *vLayout = new QVBoxLayout();

    vLayout->setAlignment(Qt::AlignTop);

    hLayout->addLayout(vLayout);
    setLayout(hLayout);

    setupCanvas2D();
    resize(800, 600);

    QScrollArea *scrollArea = new QScrollArea();
    scrollArea->setWidget(m_canvas);
    scrollArea->setWidgetResizable(true);
    hLayout->addWidget(scrollArea, 1);

    QWidget *brushGroup = new QWidget();
    QVBoxLayout *brushLayout = new QVBoxLayout();
    brushLayout->setAlignment(Qt::AlignTop);
    brushGroup->setLayout(brushLayout);

    QScrollArea *controlsScroll = new QScrollArea();
    controlsScroll->setWidget(brushGroup);
    controlsScroll->setWidgetResizable(true);

    vLayout->addWidget(controlsScroll);

    addPushButton(brushLayout, "Load Image", &MainWindow::onUploadButtonClick);
    addPushButton(brushLayout, "Revert Image", &MainWindow::onRevertButtonClick);
    addPushButton(brushLayout, "Clear canvas", &MainWindow::onClearButtonClick);
    addPushButton(brushLayout, "Save Image", &MainWindow::onSaveButtonClick);
    addPushButton(brushLayout, "Build Mesh", &MainWindow::onBuildMeshButtonClick);
    addPushButton(brushLayout, "View 3D Mesh", &MainWindow::onViewMeshButtonClick);

    monster m;
    std::vector<Region> regions = m_canvas->getRegions();
}

void MainWindow::addPushButton(QBoxLayout *layout, QString text,
                               void (MainWindow::*slot)()) {
    QPushButton *button = new QPushButton(text);
    layout->addWidget(button);
    connect(button, &QPushButton::clicked, this, slot);
}

void MainWindow::setupCanvas2D() {
    m_canvas = new Canvas2D();
    m_canvas->init();

    if (!settings.imagePath.isEmpty()) {
        m_canvas->loadImageFromFile(settings.imagePath);
    }
}

void MainWindow::onClearButtonClick() {
    m_canvas->resize(m_canvas->parentWidget()->size().width(), m_canvas->parentWidget()->size().height());
    m_canvas->clearCanvas();
}

void MainWindow::onRevertButtonClick() {
    m_canvas->loadImageFromFile(settings.imagePath);
}

void MainWindow::onUploadButtonClick() {
    QString file = QFileDialog::getOpenFileName(this, tr("Open Image"), QDir::homePath(), tr("Image Files (*.png *.jpg *.jpeg)"));
    if (file.isEmpty()) { return; }
    settings.imagePath = file;

    m_canvas->loadImageFromFile(settings.imagePath);

    m_canvas->settingsChanged();
}

void MainWindow::onSaveButtonClick() {
    QString file = QFileDialog::getSaveFileName(this, tr("Save Image"), QDir::currentPath(), tr("Image Files (*.png *.jpg *.jpeg)"));
    if (file.isEmpty()) { return; }

    m_canvas->saveImageToFile(file);
}

void MainWindow::onBuildMeshButtonClick() {
    monster m;
    StitchedMesh mesh = m.buildMesh(
        m_canvas->getRegions(),
        m_canvas->getAllConnectedRegions()
        );
    // monster::buildMesh currently writes "mesh12.obj" in the cwd.
    m_lastMeshPath = "mesh12.obj";
    std::cout << "Mesh written to: "
              << QFileInfo(m_lastMeshPath).absoluteFilePath().toStdString()
              << std::endl;
}

void MainWindow::onViewMeshButtonClick() {
    // Pick the OBJ to show: last built mesh if it exists, otherwise ask.
    QString path = m_lastMeshPath;
    if (path.isEmpty() || !QFileInfo::exists(path)) {
        QString fallback = "mesh12.obj";
        if (QFileInfo::exists(fallback)) {
            path = fallback;
        } else {
            path = QFileDialog::getOpenFileName(
                this, tr("Open OBJ Mesh"), QDir::currentPath(),
                tr("Wavefront OBJ (*.obj)"));
            if (path.isEmpty()) return;
        }
    }

    if (!m_meshViewerWindow) {
        m_meshViewerWindow = new QWidget(nullptr); // top-level window
        m_meshViewerWindow->setAttribute(Qt::WA_DeleteOnClose, false);
        m_meshViewerWindow->setWindowTitle("3D Mesh Viewer");
        m_meshViewerWindow->resize(800, 600);

        QVBoxLayout *layout = new QVBoxLayout(m_meshViewerWindow);
        layout->setContentsMargins(0, 0, 0, 0);

        m_glWidget = new GLWidget(m_meshViewerWindow);
        layout->addWidget(m_glWidget);
    }

    m_glWidget->setMeshPath(path.toStdString());
    m_meshViewerWindow->show();
    m_meshViewerWindow->raise();
    m_meshViewerWindow->activateWindow();
}
