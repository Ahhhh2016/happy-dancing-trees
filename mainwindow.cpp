#include "mainwindow.h"
#include "settings.h"
#include "monster.h"

#include <QApplication>
#include <QFutureWatcher>
#include <QtConcurrent/QtConcurrentRun>

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QDir>
#include <QLabel>
#include <QGroupBox>
#include <QScrollArea>
#include <QSplitter>
#include <QMessageBox>
#include <QFrame>
#include <QColorDialog>
#include <iostream>

MainWindow::MainWindow()
    : m_canvas(nullptr),
      m_mainSplitter(nullptr),
      m_meshResolutionSlider(nullptr),
      m_brushToolRadio(nullptr),
      m_paintColorSwatch(nullptr),
      m_paintRadiusSlider(nullptr),
      m_glWidget(nullptr),
      m_meshPreviewDebounceTimer(this)
{
    setWindowTitle("2D Projects");

    settings.loadSettingsOrDefaults();

    QHBoxLayout *hLayout = new QHBoxLayout();
    QVBoxLayout *vLayout = new QVBoxLayout();

    vLayout->setAlignment(Qt::AlignTop);

    hLayout->addLayout(vLayout);
    setLayout(hLayout);

    setupCanvas2D();
    m_meshPreviewDebounceTimer.setSingleShot(true);
    m_meshPreviewDebounceTimer.setInterval(180);
    connect(&m_meshPreviewDebounceTimer, &QTimer::timeout,
            this, &MainWindow::requestAsyncMeshRebuildIfIdle);
    connect(m_canvas, &Canvas2D::meshPreviewDirty, this, [this]() {
        m_meshPreviewDebounceTimer.start();
    });
    resize(1100, 700);

    // Split view: 2D canvas and 3D mesh visible together (draggable divider).
    m_mainSplitter = new QSplitter(Qt::Horizontal);

    auto *scrollArea = new QScrollArea();
    scrollArea->setWidget(m_canvas);
    scrollArea->setWidgetResizable(true);

    m_glWidget = new GLWidget();

    m_mainSplitter->addWidget(scrollArea);
    m_mainSplitter->addWidget(m_glWidget);
    m_mainSplitter->setStretchFactor(0, 1);
    m_mainSplitter->setStretchFactor(1, 1);
    m_mainSplitter->setSizes({550, 550});
    m_cachedSplitterSizes = m_mainSplitter->sizes();

    hLayout->addWidget(m_mainSplitter, 1);

    QWidget *brushGroup = new QWidget();
    QVBoxLayout *brushLayout = new QVBoxLayout();
    brushLayout->setAlignment(Qt::AlignTop);
    brushGroup->setLayout(brushLayout);

    QScrollArea *controlsScroll = new QScrollArea();
    controlsScroll->setWidget(brushGroup);
    controlsScroll->setWidgetResizable(true);

    vLayout->addWidget(controlsScroll);
    addPushButton(brushLayout, "Load Image", &MainWindow::onUploadButtonClick);
    addPushButton(brushLayout, "Save Image", &MainWindow::onSaveButtonClick);


    brushLayout->addWidget(new QLabel(tr("View:")));
    auto *viewGroup = new QButtonGroup(this);
    QRadioButton *canvasViewRadio = new QRadioButton(tr("Canvas view"));
    QRadioButton *animationViewRadio = new QRadioButton(tr("Animation view"));
    canvasViewRadio->setToolTip(
        tr("2D canvas and 3D mesh together (draggable divider)."));
    animationViewRadio->setToolTip(tr("3D view only, full width."));
    viewGroup->addButton(canvasViewRadio);
    viewGroup->addButton(animationViewRadio);
    viewGroup->setExclusive(true);
    canvasViewRadio->setChecked(true);
    brushLayout->addWidget(canvasViewRadio);
    brushLayout->addWidget(animationViewRadio);
    QPushButton *center3DButton = new QPushButton(tr("Center 3D view"));
    center3DButton->setToolTip(tr("Reset camera to the default centered mesh view."));
    brushLayout->addWidget(center3DButton);

    QCheckBox *showAnchorsCheck = new QCheckBox(tr("Show ARAP points (P)"));
    showAnchorsCheck->setToolTip(
        tr("Toggle the red ARAP vertex/anchor overlay in the 3D view."));
    showAnchorsCheck->setChecked(true);
    brushLayout->addWidget(showAnchorsCheck);
    connect(showAnchorsCheck, &QCheckBox::toggled, this, [this](bool on) {
        if (m_glWidget) m_glWidget->setAnchorsVisible(on);
    });
    connect(canvasViewRadio, &QRadioButton::toggled, this, [this](bool on) {
        if (on) {
            applyCanvasViewMode();
        }
    });
    connect(animationViewRadio, &QRadioButton::toggled, this, [this](bool on) {
        if (on) {
            applyAnimationViewMode();
        }
    });
    connect(center3DButton, &QPushButton::clicked, this, [this]() {
        if (m_glWidget) {
            m_glWidget->centerView();
            m_glWidget->setFocus(Qt::OtherFocusReason);
        }
    });

    brushLayout->addWidget(new QLabel(tr("Draw tool:")));
    auto *toolGroup = new QButtonGroup(this);
    m_brushToolRadio = new QRadioButton(tr("Pen"));
    QRadioButton *paintRadio = new QRadioButton(tr("Paint"));
    QRadioButton *bucketRadio = new QRadioButton(tr("Bucket fill"));
    brushLayout->addWidget(m_brushToolRadio);
    brushLayout->addWidget(paintRadio);
    brushLayout->addWidget(bucketRadio);

    brushLayout->addWidget(new QLabel(tr("Erase tool:")));
    QRadioButton *eraserRadio = new QRadioButton(tr("Pen (stroke)"));
    QRadioButton *paintEraserRadio = new QRadioButton(tr("Paint (pixels)"));
    brushLayout->addWidget(eraserRadio);
    brushLayout->addWidget(paintEraserRadio);
    QPushButton *clearPaintButton = new QPushButton(tr("Clear paint"));
    clearPaintButton->setToolTip(
        tr("Remove all paint-layer color while keeping strokes and filled regions."));
    brushLayout->addWidget(clearPaintButton);
    paintEraserRadio->setToolTip(
        tr("Drag to clear painted pixels from the texture layer."));
    m_brushToolRadio->setChecked(true);
    toolGroup->addButton(m_brushToolRadio);
    toolGroup->addButton(eraserRadio);
    toolGroup->addButton(paintRadio);
    toolGroup->addButton(bucketRadio);
    toolGroup->addButton(paintEraserRadio);
    toolGroup->setExclusive(true);
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
    connect(paintRadio, &QRadioButton::toggled, this, [this](bool on) {
        if (on) {
            m_canvas->setTool(Canvas2D::Tool::Paint);
        }
    });
    connect(bucketRadio, &QRadioButton::toggled, this, [this](bool on) {
        if (on) {
            m_canvas->setTool(Canvas2D::Tool::BucketFill);
        }
    });
    connect(paintEraserRadio, &QRadioButton::toggled, this, [this](bool on) {
        if (on) {
            m_canvas->setTool(Canvas2D::Tool::PaintEraser);
        }
    });
    connect(clearPaintButton, &QPushButton::clicked, this, [this]() {
        m_canvas->clearPaint();
    });

    brushLayout->addWidget(new QLabel(tr("Paint color:")));
    m_paintColorSwatch = new QFrame();
    m_paintColorSwatch->setFixedSize(44, 26);
    m_paintColorSwatch->setFrameShape(QFrame::StyledPanel);
    auto updatePaintSwatch = [this](const QColor &c) {
        m_paintColorSwatch->setStyleSheet(
            QStringLiteral("background-color: rgba(%1,%2,%3,%4); border: 1px solid #888;")
                .arg(c.red())
                .arg(c.green())
                .arg(c.blue())
                .arg(c.alpha()));
    };
    const QColor initialPaint(Qt::black);
    m_canvas->setPaintColor(initialPaint);
    updatePaintSwatch(initialPaint);
    auto *paintColorRow = new QHBoxLayout();
    paintColorRow->addWidget(m_paintColorSwatch);
    QPushButton *pickPaintColorButton = new QPushButton(tr("Choose color…"));
    paintColorRow->addWidget(pickPaintColorButton);
    paintColorRow->addStretch();
    brushLayout->addLayout(paintColorRow);
    connect(pickPaintColorButton, &QPushButton::clicked, this, [this, updatePaintSwatch]() {
        const QColor c = QColorDialog::getColor(
            m_canvas->paintColor(),
            this,
            tr("Paint color"),
            QColorDialog::ShowAlphaChannel | QColorDialog::DontUseNativeDialog);
        if (c.isValid()) {
            m_canvas->setPaintColor(c);
            updatePaintSwatch(c);
        }
    });

    brushLayout->addWidget(new QLabel(tr("Paint brush size:")));
    m_paintRadiusSlider = new QSlider(Qt::Horizontal);
    m_paintRadiusSlider->setRange(2, 40);
    m_paintRadiusSlider->setValue(8);
    brushLayout->addWidget(m_paintRadiusSlider);
    connect(m_paintRadiusSlider, &QSlider::valueChanged, this, [this](int v) {
        m_canvas->setPaintBrushRadius(static_cast<float>(v));
    });
    m_canvas->setPaintBrushRadius(static_cast<float>(m_paintRadiusSlider->value()));

    m_meshResolutionSlider = new QSlider(Qt::Horizontal);
    m_meshResolutionSlider->setRange(0, 100);
    m_meshResolutionSlider->setValue(73); // ~same density as former fixed a100
    m_meshResolutionSlider->setToolTip(
        tr("Smaller triangles (right) increase face count and usually smooth the inflated 3D shape."));
    brushLayout->addWidget(new QLabel(tr("Triangle size (smoothness):")));
    brushLayout->addWidget(m_meshResolutionSlider);
    connect(m_meshResolutionSlider, &QSlider::sliderReleased,
            this, &MainWindow::onMeshResolutionSliderReleased);
    addPushButton(brushLayout, "Clear canvas", &MainWindow::onClearButtonClick);
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
    // clearCanvas() emits meshPreviewDirty -> debounce timer; cancel it so we
    // don't ressurect the previous mesh from disk after the user cleared.
    m_canvas->clearCanvas();
    m_meshPreviewDebounceTimer.stop();

    if (m_glWidget) {
        m_glWidget->clearMesh();
    }
    m_lastMeshPath.clear();

    // Remove cached mesh artifacts so a stale buildMesh() result can't be
    // re-loaded by finishAsyncSliderMeshRebuild() if a rebuild was already in
    // flight when the user clicked Clear.
    QFile::remove(QStringLiteral("mesh12.obj"));
    QFile::remove(QStringLiteral("mesh12_texture.png"));
    QFile::remove(QStringLiteral("mesh12_arapl.txt"));

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

void MainWindow::onMeshResolutionSliderReleased() {
    requestAsyncMeshRebuildIfIdle();
}

void MainWindow::requestAsyncMeshRebuildIfIdle() {
    if (m_sliderMeshRebuildBusy) {
        m_sliderMeshRebuildPending = true;
        return;
    }
    startAsyncSliderMeshRebuild();
}

void MainWindow::startAsyncSliderMeshRebuild() {
    std::vector<Region> regions = m_canvas->getRegions();
    if (regions.empty()) {
        // Nothing to mesh (canvas just got cleared, etc.); make sure no stale
        // mesh remains in the 3D view.
        if (m_glWidget) {
            m_glWidget->clearMesh();
        }
        return;
    }

    m_sliderMeshRebuildBusy = true;
    m_meshResolutionSlider->setEnabled(false);
    QApplication::setOverrideCursor(Qt::BusyCursor);

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

    // If the canvas was cleared while the rebuild was running, refuse to
    // reload an old mesh from disk — keep the 3D view empty.
    if (m_canvas->getRegions().empty()) {
        if (m_glWidget) {
            m_glWidget->clearMesh();
        }
        m_lastMeshPath.clear();
        if (m_sliderMeshRebuildPending) {
            m_sliderMeshRebuildPending = false;
        }
        return;
    }

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

        m_glWidget->setMeshPath(path.toStdString());
        m_lastMeshPath = path;
    }

    if (m_sliderMeshRebuildPending) {
        m_sliderMeshRebuildPending = false;
        startAsyncSliderMeshRebuild();
    }
}

void MainWindow::applyCanvasViewMode() {
    QWidget *canvasPane = m_mainSplitter->widget(0);
    canvasPane->setVisible(true);
    if (m_cachedSplitterSizes.size() == m_mainSplitter->count()) {
        m_mainSplitter->setSizes(m_cachedSplitterSizes);
    } else {
        m_mainSplitter->setSizes({550, 550});
    }
}

void MainWindow::applyAnimationViewMode() {
    m_cachedSplitterSizes = m_mainSplitter->sizes();
    m_mainSplitter->widget(0)->setVisible(false);
    if (m_glWidget) {
        m_glWidget->setFocus(Qt::OtherFocusReason);
    }
}
