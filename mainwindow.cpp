#include "mainwindow.h"
#include "settings.h"
#include "monster.h"

#include <QApplication>
#include <QFutureWatcher>
#include <QtConcurrent/QtConcurrentRun>

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
      m_meshResolutionSlider(nullptr),
      m_meshResolutionLabel(nullptr),
      m_brushToolRadio(nullptr),
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

    brushLayout->addWidget(new QLabel(tr("Draw tool:")));
    auto *toolGroup = new QButtonGroup(this);
    m_brushToolRadio = new QRadioButton(tr("Brush"));
    QRadioButton *eraserRadio = new QRadioButton(tr("Eraser (click stroke)"));
    m_brushToolRadio->setChecked(true);
    toolGroup->addButton(m_brushToolRadio);
    toolGroup->addButton(eraserRadio);
    toolGroup->setExclusive(true);
    brushLayout->addWidget(m_brushToolRadio);
    brushLayout->addWidget(eraserRadio);
    connect(m_brushToolRadio, &QRadioButton::toggled, this, [this](bool on) {
        if (on) {
            m_canvas->setTool(Canvas2D::Tool::Brush);
        }
    });
    connect(eraserRadio, &QRadioButton::toggled, this, [this](bool on) {
        if (on) {
            m_canvas->setTool(Canvas2D::Tool::Eraser);
        }
    });

    addPushButton(brushLayout, "Load Image", &MainWindow::onUploadButtonClick);
    addPushButton(brushLayout, "Revert Image", &MainWindow::onRevertButtonClick);
    addPushButton(brushLayout, "Clear canvas", &MainWindow::onClearButtonClick);
    addPushButton(brushLayout, "Save Image", &MainWindow::onSaveButtonClick);

    m_meshResolutionLabel = new QLabel();
    m_meshResolutionSlider = new QSlider(Qt::Horizontal);
    m_meshResolutionSlider->setRange(0, 100);
    m_meshResolutionSlider->setValue(73); // ~same density as former fixed a100
    m_meshResolutionSlider->setToolTip(
        tr("Smaller triangles (right) increase face count and usually smooth the inflated 3D shape."));
    brushLayout->addWidget(new QLabel(tr("2D triangulation (affects 3D smoothness):")));
    brushLayout->addWidget(m_meshResolutionSlider);
    brushLayout->addWidget(m_meshResolutionLabel);
    updateMeshResolutionLabel();
    connect(m_meshResolutionSlider, &QSlider::valueChanged,
            this, &MainWindow::onMeshResolutionSliderChanged);
    connect(m_meshResolutionSlider, &QSlider::sliderReleased,
            this, &MainWindow::onMeshResolutionSliderReleased);

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
    if (m_brushToolRadio) {
        m_brushToolRadio->setChecked(true);
    }
    m_canvas->setTool(Canvas2D::Tool::Brush);
}

void MainWindow::onRevertButtonClick() {
    m_canvas->loadImageFromFile(settings.imagePath);
    if (m_brushToolRadio) {
        m_brushToolRadio->setChecked(true);
    }
    m_canvas->setTool(Canvas2D::Tool::Brush);
}

void MainWindow::onUploadButtonClick() {
    QString file = QFileDialog::getOpenFileName(this, tr("Open Image"), QDir::homePath(), tr("Image Files (*.png *.jpg *.jpeg)"));
    if (file.isEmpty()) { return; }
    settings.imagePath = file;

    m_canvas->loadImageFromFile(settings.imagePath);

    if (m_brushToolRadio) {
        m_brushToolRadio->setChecked(true);
    }
    m_canvas->setTool(Canvas2D::Tool::Brush);
    m_canvas->settingsChanged();
}

void MainWindow::onSaveButtonClick() {
    QString file = QFileDialog::getSaveFileName(this, tr("Save Image"), QDir::currentPath(), tr("Image Files (*.png *.jpg *.jpeg)"));
    if (file.isEmpty()) { return; }

    m_canvas->saveImageToFile(file);
}

double MainWindow::maxTriangleAreaFromSlider() const {
    // Left = coarse (few faces), right = fine (many faces). Matches Triangle's -a max area.
    constexpr double kCoarse = 350.0;
    constexpr double kFine = 6.0;
    const double t = m_meshResolutionSlider->value() / 100.0;
    return kCoarse + (kFine - kCoarse) * t;
}

void MainWindow::updateMeshResolutionLabel() {
    const double a = maxTriangleAreaFromSlider();
    m_meshResolutionLabel->setText(
        tr("Max triangle area: %1 (coarse to fine)").arg(a, 0, 'f', 1));
}

void MainWindow::onMeshResolutionSliderChanged(int) {
    updateMeshResolutionLabel();
}

void MainWindow::onMeshResolutionSliderReleased() {
    if (m_viewStack->currentIndex() != m_meshPageIndex)
        return;
    if (m_sliderMeshRebuildBusy) {
        m_sliderMeshRebuildPending = true;
        return;
    }
    startAsyncSliderMeshRebuild();
}

void MainWindow::startAsyncSliderMeshRebuild() {
    m_sliderMeshRebuildBusy = true;
    m_meshResolutionSlider->setEnabled(false);
    QApplication::setOverrideCursor(Qt::BusyCursor);

    std::vector<Region> regions = m_canvas->getRegions();
    const double cw = static_cast<double>(m_canvas->m_width);
    const double ch = static_cast<double>(m_canvas->m_height);
    const double maxA = maxTriangleAreaFromSlider();

    QFuture<void> future = QtConcurrent::run(
        [regions = std::move(regions), cw, ch, maxA]() mutable {
            monster m;
            (void)m.buildMesh(regions, cw, ch, maxA);
        });

    auto *watcher = new QFutureWatcher<void>(this);
    connect(watcher, &QFutureWatcher<void>::finished, this, [this, watcher]() {
        watcher->deleteLater();
        finishAsyncSliderMeshRebuild();
    });
    watcher->setFuture(future);
}

void MainWindow::finishAsyncSliderMeshRebuild() {
    QApplication::restoreOverrideCursor();
    m_meshResolutionSlider->setEnabled(true);
    m_sliderMeshRebuildBusy = false;

    const QString path = QStringLiteral("mesh12.obj");
    if (!QFileInfo::exists(path)) {
        std::cerr << "Build Mesh did not produce " << path.toStdString() << std::endl;
    } else {
        std::cout << "Mesh written to: "
                  << QFileInfo(path).absoluteFilePath().toStdString()
                  << std::endl;

        const QString texPath =
            QFileInfo(path).absolutePath() + QStringLiteral("/mesh12_texture.png");
        if (!m_canvas->saveMeshTextureToFile(texPath)) {
            std::cerr << "Failed to save texture: " << texPath.toStdString() << std::endl;
        } else {
            std::cout << "Texture written to: " << texPath.toStdString() << std::endl;
        }

        if (m_viewStack->currentIndex() == m_meshPageIndex)
            m_glWidget->setMeshPath(path.toStdString());
        m_lastMeshPath = path;
    }

    if (m_viewStack->currentIndex() != m_meshPageIndex) {
        m_sliderMeshRebuildPending = false;
    } else if (m_sliderMeshRebuildPending) {
        m_sliderMeshRebuildPending = false;
        startAsyncSliderMeshRebuild();
    }
}

QString MainWindow::buildMeshAndSaveObj() {
    monster m;
    StitchedMesh mesh = m.buildMesh(
        m_canvas->getRegions(),
        static_cast<double>(m_canvas->m_width),
        static_cast<double>(m_canvas->m_height),
        maxTriangleAreaFromSlider());
    // monster::buildMesh currently writes "mesh12.obj" in the cwd.
    const QString path = "mesh12.obj";
    if (!QFileInfo::exists(path)) {
        std::cerr << "Build Mesh did not produce " << path.toStdString() << std::endl;
        return {};
    }
    std::cout << "Mesh written to: "
              << QFileInfo(path).absoluteFilePath().toStdString()
              << std::endl;

    const QString texPath =
        QFileInfo(path).absolutePath() + QStringLiteral("/mesh12_texture.png");
    if (!m_canvas->saveMeshTextureToFile(texPath)) {
        std::cerr << "Failed to save texture: " << texPath.toStdString() << std::endl;
    } else {
        std::cout << "Texture written to: " << texPath.toStdString() << std::endl;
    }

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
