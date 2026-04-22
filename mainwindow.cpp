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
    : m_canvas(nullptr),
      m_viewStack(nullptr),
      m_canvasPageIndex(-1),
      m_meshPageIndex(-1),
      m_toggleMeshButton(nullptr),
      m_glWidget(nullptr)
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

    // Main viewing area: swap between the 2D canvas and the 3D mesh viewer.
    m_viewStack = new QStackedWidget();

    QScrollArea *scrollArea = new QScrollArea();
    scrollArea->setWidget(m_canvas);
    scrollArea->setWidgetResizable(true);
    m_canvasPageIndex = m_viewStack->addWidget(scrollArea);

    m_glWidget = new GLWidget();
    m_meshPageIndex = m_viewStack->addWidget(m_glWidget);

    m_viewStack->setCurrentIndex(m_canvasPageIndex);
    hLayout->addWidget(m_viewStack, 1);

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

    m_toggleMeshButton = new QPushButton("Build && View 3D Mesh");
    brushLayout->addWidget(m_toggleMeshButton);
    connect(m_toggleMeshButton, &QPushButton::clicked,
            this, &MainWindow::onToggleMeshViewClick);
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

QString MainWindow::buildMeshAndSaveObj() {
    monster m;
    StitchedMesh mesh = m.buildMesh(
        m_canvas->getRegions()
        );
    // monster::buildMesh currently writes "mesh12.obj" in the cwd.
    const QString path = "mesh12.obj";
    if (!QFileInfo::exists(path)) {
        std::cerr << "Build Mesh did not produce " << path.toStdString() << std::endl;
        return {};
    }
    std::cout << "Mesh written to: "
              << QFileInfo(path).absoluteFilePath().toStdString()
              << std::endl;
    m_lastMeshPath = path;
    return path;
}

void MainWindow::onToggleMeshViewClick() {
    const bool showingMesh = (m_viewStack->currentIndex() == m_meshPageIndex);

    if (showingMesh) {
        m_viewStack->setCurrentIndex(m_canvasPageIndex);
        m_toggleMeshButton->setText("Build && View 3D Mesh");
        return;
    }

    // Always (re)build from the current canvas state, then display the OBJ.
    QString path = buildMeshAndSaveObj();
    if (path.isEmpty()) return;

    m_glWidget->setMeshPath(path.toStdString());
    m_viewStack->setCurrentIndex(m_meshPageIndex);
    m_glWidget->setFocus();
    m_toggleMeshButton->setText("Back to 2D Canvas");
}
