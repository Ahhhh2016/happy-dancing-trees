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



    // animation interface
    brushLayout->addSpacing(10);
    brushLayout->addWidget(new QLabel(tr("Animation:")));

    auto *animRow1 = new QHBoxLayout();
    m_recordButton = new QPushButton(tr("📸 Record"));
    m_recordButton->setCheckable(true);

    m_playButton = new QPushButton(tr("▶ Play"));
    m_playButton->setCheckable(true);

    m_stopButton = new QPushButton(tr("⏹ Stop"));

    animRow1->addWidget(m_recordButton);
    animRow1->addWidget(m_playButton);
    animRow1->addWidget(m_stopButton);
    brushLayout->addLayout(animRow1);

    auto *animRow2 = new QHBoxLayout();
    m_saveAnimButton = new QPushButton(tr("💾 Save Animation"));

    m_loadAnimButton = new QPushButton(tr("📂 Import Animation"));

    animRow2->addWidget(m_saveAnimButton);
    animRow2->addWidget(m_loadAnimButton);
    brushLayout->addLayout(animRow2);

    auto *animRow3 = new QHBoxLayout();
    m_toggleAnchorsButton = new QPushButton(tr("Hide Anchors"));
    m_toggleAnchorsButton->setCheckable(true);
    m_toggleAnchorsButton->setChecked(false);
    animRow3->addWidget(m_toggleAnchorsButton);
    brushLayout->addLayout(animRow3);



    connect(m_recordButton, &QPushButton::clicked, this, [this]() {
        if (m_glWidget) {
            m_glWidget->toggleRecordAnimation();
            updateAnimationButtonStates();
        }
    });

    connect(m_playButton, &QPushButton::clicked, this, [this]() {
        if (m_glWidget) {
            m_glWidget->togglePlayAnimation();
            updateAnimationButtonStates();
        }
    });

    connect(m_stopButton, &QPushButton::clicked, this, [this]() {
        if (m_glWidget) {
            m_glWidget->stopAnimation();
            updateAnimationButtonStates();
        }
    });

    connect(m_saveAnimButton, &QPushButton::clicked, this, [this]() {
        if (m_glWidget) {
            m_glWidget->saveAnimation();
        }
    });

    connect(m_loadAnimButton, &QPushButton::clicked, this, [this]() {
        if (m_glWidget) {
            m_glWidget->loadAnimation();
            updateAnimationButtonStates();
        }
    });

    // anchors visible/invisible
    connect(m_toggleAnchorsButton, &QPushButton::clicked, this, [this](bool checked) {
        m_anchorsVisible = !checked;
        m_toggleAnchorsButton->setText(m_anchorsVisible ? tr("Hide Anchors") : tr("Show Anchors"));
        if (m_glWidget) {
            m_glWidget->setAnchorsVisible(m_anchorsVisible);
        }
    });

    updateAnimationButtonStates();
}

void MainWindow::updateAnimationButtonStates() {
    if (!m_glWidget) return;

    bool isRecording = m_glWidget->isAnimationRecording();
    bool isPlaying = m_glWidget->isAnimationPlaying();

    m_recordButton->setChecked(isRecording);
    m_recordButton->setText(isRecording ? tr("⏹ Recording...") : tr("📸 Record"));
    m_recordButton->setStyleSheet(isRecording ? "background-color: #ff4444; color: white; font-weight: bold;" : "");

    m_playButton->setChecked(isPlaying);
    m_playButton->setText(isPlaying ? tr("⏸ Playing...") : tr("▶ Play"));
    m_playButton->setStyleSheet(isPlaying ? "background-color: #44aa44; color: white; font-weight: bold;" : "");

    m_stopButton->setEnabled(isRecording || isPlaying);
    m_recordButton->setEnabled(!isPlaying);
    m_playButton->setEnabled(!isRecording);
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
    m_meshPreviewDebounceTimer.stop();

    disconnect(m_canvas, &Canvas2D::meshPreviewDirty, this, nullptr);

    m_canvas->resize(m_canvas->parentWidget()->size().width(), m_canvas->parentWidget()->size().height());
    m_canvas->clearCanvas();
    if (m_glWidget) {
        m_glWidget->clearMesh();
    }
    m_lastMeshPath.clear();
    if (m_brushToolRadio) {
        m_brushToolRadio->setChecked(true);
    }
    m_canvas->setTool(Canvas2D::Tool::Brush);

    connect(m_canvas, &Canvas2D::meshPreviewDirty, this, [this]() {
        m_meshPreviewDebounceTimer.start();
    });
}

void MainWindow::onUploadButtonClick() {
    QString file = QFileDialog::getOpenFileName(this, tr("Open Image"), QDir::homePath(), tr("Image Files (*.png *.jpg *.jpeg)"));
    if (file.isEmpty()) { return; }
    settings.imagePath = file;

    m_meshPreviewDebounceTimer.stop();
    if (m_glWidget) {
        m_glWidget->clearMesh();
    }
    m_lastMeshPath.clear();
    disconnect(m_canvas, &Canvas2D::meshPreviewDirty, this, nullptr);

    m_canvas->loadImageFromFile(settings.imagePath);

    if (m_brushToolRadio) {
        m_brushToolRadio->setChecked(true);
    }
    m_canvas->setTool(Canvas2D::Tool::Brush);
    m_canvas->settingsChanged();

    connect(m_canvas, &Canvas2D::meshPreviewDirty, this, [this]() {
        m_meshPreviewDebounceTimer.start();
    });
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
    if (m_canvas->getRegions().empty()) {
        // Canvas was cleared (e.g. user erased the last stroke) — the OBJ
        // monster wrote is empty, so drop the stale mesh from the 3D view too.
        if (m_glWidget) {
            m_glWidget->clearMesh();
        }
        m_lastMeshPath.clear();
    } else if (!QFileInfo::exists(path)) {
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


