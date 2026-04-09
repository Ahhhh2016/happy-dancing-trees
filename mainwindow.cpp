#include "mainwindow.h"
#include "settings.h"
#include "monster.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QLabel>
#include <QGroupBox>
#include <QScrollArea>
#include <iostream>

MainWindow::MainWindow()
{
    setWindowTitle("2D Projects: Brush");

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

    addHeading(brushLayout, "Brush");
    addSpinBox(brushLayout, "red", 0, 255, 1, settings.brushColor.r, [this](int value){ setUIntVal(settings.brushColor.r, value); });
    addSpinBox(brushLayout, "green", 0, 255, 1, settings.brushColor.g, [this](int value){ setUIntVal(settings.brushColor.g, value); });
    addSpinBox(brushLayout, "blue", 0, 255, 1, settings.brushColor.b, [this](int value){ setUIntVal(settings.brushColor.b, value); });
    addSpinBox(brushLayout, "alpha", 0, 255, 1, settings.brushColor.a, [this](int value){ setUIntVal(settings.brushColor.a, value); });
    addSpinBox(brushLayout, "radius", 0, 100, 1, settings.brushRadius, [this](int value){ setIntVal(settings.brushRadius, value); });

    addPushButton(brushLayout, "Load Image", &MainWindow::onUploadButtonClick);
    addPushButton(brushLayout, "Revert Image", &MainWindow::onRevertButtonClick);
    addPushButton(brushLayout, "Clear canvas", &MainWindow::onClearButtonClick);
    addPushButton(brushLayout, "Save Image", &MainWindow::onSaveButtonClick);
<<<<<<< HEAD

    // filters
    addHeading(filterLayout, "Filter");
    addRadioButton(filterLayout, "Edge detect", settings.filterType == FILTER_EDGE_DETECT,  [this]{ setFilterType(FILTER_EDGE_DETECT); });
    addDoubleSpinBox(filterLayout, "sensitivity", 0.01, 1, 0.01, settings.edgeDetectSensitivity, 2, [this](float value){ setFloatVal(settings.edgeDetectSensitivity, value); });

    addRadioButton(filterLayout, "Blur", settings.filterType == FILTER_BLUR, [this]{ setFilterType(FILTER_BLUR); });
    addSpinBox(filterLayout, "radius", 0, 100, 1, settings.blurRadius, [this](int value){ setIntVal(settings.blurRadius, value); });

    addRadioButton(filterLayout, "Scale", settings.filterType == FILTER_SCALE, [this]{ setFilterType(FILTER_SCALE); });
    addDoubleSpinBox(filterLayout, "x", 0.1, 10, 0.1, settings.scaleX, 2, [this](float value){ setFloatVal(settings.scaleX, value); });
    addDoubleSpinBox(filterLayout, "y", 0.1, 10, 0.1, settings.scaleY, 2, [this](float value){ setFloatVal(settings.scaleY, value); });

    // extra credit filters
    addHeading(filterLayout, "Extra Credit Filters");
    addRadioButton(filterLayout, "Median", settings.filterType == FILTER_MEDIAN,  [this]{ setFilterType(FILTER_MEDIAN); });
    addSpinBox(filterLayout, "radius", 1, 100, 1, settings.medianRadius, [this](int value){ setIntVal(settings.medianRadius, value); });

    addRadioButton(filterLayout, "Chromatic aberration", settings.filterType == FILTER_CHROMATIC,  [this]{ setFilterType(FILTER_CHROMATIC); });
    addSpinBox(filterLayout, "red shift", -100, 100, 0, settings.rShift, [this](int value){ setIntVal(settings.rShift, value); });
    addSpinBox(filterLayout, "green shift", -100, 100, 0, settings.gShift, [this](int value){ setIntVal(settings.gShift, value); });
    addSpinBox(filterLayout, "blue shift", -100, 100, 0, settings.bShift, [this](int value){ setIntVal(settings.bShift, value); });

    addRadioButton(filterLayout, "Tone mapping", settings.filterType == FILTER_MAPPING,  [this]{ setFilterType(FILTER_MAPPING); });
    addCheckBox(filterLayout, "Non linear function", settings.nonLinearMap, [this](bool value){ setBoolVal(settings.nonLinearMap, value); });
    addDoubleSpinBox(filterLayout, "gamma", 0.1, 2, 0.1, settings.gamma, 2, [this](float value){ setFloatVal(settings.gamma, value); });

    addRadioButton(filterLayout, "Rotation", settings.filterType == FILTER_ROTATION,  [this]{ setFilterType(FILTER_ROTATION); });
    addDoubleSpinBox(filterLayout, "angle", -360, 360, 0.1, settings.rotationAngle, 2, [this](float value){ setFloatVal(settings.rotationAngle, value); });

    addRadioButton(filterLayout, "Bilteral smooth", settings.filterType == FILTER_BILATERAL,  [this]{ setFilterType(FILTER_BILATERAL); });
    addSpinBox(filterLayout, "radius", 1, 100, 1, settings.bilateralRadius, [this](int value){ setIntVal(settings.bilateralRadius, value); });

    // filter push buttons
    addPushButton(filterLayout, "Load Image", &MainWindow::onUploadButtonClick);
    addPushButton(filterLayout, "Apply Filter", &MainWindow::onFilterButtonClick);
    addPushButton(filterLayout, "Revert Image", &MainWindow::onRevertButtonClick);
    addPushButton(filterLayout, "Save Image", &MainWindow::onSaveButtonClick);

    // build dummy strokes
    monster m;
    Stroke body = m.makeDummyBody();
    Stroke leg  = m.makeDummyLeg();
=======
>>>>>>> yixuan
}

void MainWindow::setupCanvas2D() {
    m_canvas = new Canvas2D();
    m_canvas->init();

    if (!settings.imagePath.isEmpty()) {
        m_canvas->loadImageFromFile(settings.imagePath);
    }
}

void MainWindow::addHeading(QBoxLayout *layout, QString text) {
    QFont font;
    font.setPointSize(16);
    font.setBold(true);

    QLabel *label = new QLabel(text);
    label->setFont(font);
    layout->addWidget(label);
}

void MainWindow::addLabel(QBoxLayout *layout, QString text) {
    layout->addWidget(new QLabel(text));
}

void MainWindow::addSpinBox(QBoxLayout *layout, QString text, int min, int max, int step, int val, auto function) {
    QSpinBox *box = new QSpinBox();
    box->setMinimum(min);
    box->setMaximum(max);
    box->setSingleStep(step);
    box->setValue(val);
    QHBoxLayout *subLayout = new QHBoxLayout();
    addLabel(subLayout, text);
    subLayout->addWidget(box);
    layout->addLayout(subLayout);
    connect(box, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
            this, function);
}

void MainWindow::addPushButton(QBoxLayout *layout, QString text, auto function) {
    QPushButton *button = new QPushButton(text);
    layout->addWidget(button);
    connect(button, &QPushButton::clicked, this, function);
}

void MainWindow::setUIntVal(std::uint8_t &setValue, int newValue) {
    setValue = newValue;
    m_canvas->settingsChanged();
}

void MainWindow::setIntVal(int &setValue, int newValue) {
    setValue = newValue;
    m_canvas->settingsChanged();
}

void MainWindow::setFloatVal(float &setValue, float newValue) {
    setValue = newValue;
    m_canvas->settingsChanged();
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
