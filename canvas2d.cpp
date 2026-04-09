#include "canvas2d.h"
#include <QPainter>
#include <QMessageBox>
#include <QFileDialog>
#include <algorithm>
#include <iostream>
#include <cmath>
#include "settings.h"

namespace {
constexpr float kMinStrokePointSpacing = 2.0f; // minimum distance between points in a stroke
constexpr float kBrushStampSpacing = 1.0f; // minimum distance between brush stamps

bool isInsideCanvas(const QPointF &point, int width, int height) {
    return point.x() >= 0 && point.x() < width && point.y() >= 0 && point.y() < height;
}

QPointF toQPointF(const Eigen::Vector2f &point) {
    return QPointF(point.x(), point.y());
}
}

void Canvas2D::init() {
    setMouseTracking(true);
    m_width = 500;
    m_height = 500;
    clearCanvas();
    changeMask();
}

int Canvas2D::posToIndex(int x, int y, int width) {
    return y * width + x;
}

void Canvas2D::clearCanvas() {
    m_data.assign(m_width * m_height, RGBA{255, 255, 255, 255});
    m_strokes.clear();
    m_activeStroke.reset();
    m_nextDepthOrder = 0;
    settings.imagePath = "";
    displayImage();
}

bool Canvas2D::loadImageFromFile(const QString &file) {
    QImage myImage;
    if (!myImage.load(file)) {
        std::cout<<"Failed to load in image"<<std::endl;
        return false;
    }
    myImage = myImage.convertToFormat(QImage::Format_RGBX8888);
    m_width = myImage.width();
    m_height = myImage.height();
    QByteArray arr = QByteArray::fromRawData((const char*) myImage.bits(), myImage.sizeInBytes());

    m_data.clear();
    m_data.reserve(m_width * m_height);
    for (int i = 0; i < arr.size() / 4; i++){
        m_data.push_back(RGBA{(std::uint8_t) arr[4*i], (std::uint8_t) arr[4*i+1], (std::uint8_t) arr[4*i+2], (std::uint8_t) arr[4*i+3]});
    }
    m_strokes.clear();
    m_activeStroke.reset();
    m_nextDepthOrder = 0;
    displayImage();
    return true;
}

bool Canvas2D::saveImageToFile(const QString &file) {
    QImage myImage = QImage(m_width, m_height, QImage::Format_RGBX8888);
    for (int i = 0; i < m_data.size(); i++){
        myImage.setPixelColor(i % m_width, i / m_width, QColor(m_data[i].r, m_data[i].g, m_data[i].b, m_data[i].a));
    }
    if (!myImage.save(file)) {
        std::cout<<"Failed to save image"<<std::endl;
        return false;
    }
    return true;
}

void Canvas2D::displayImage() {
    QByteArray img(reinterpret_cast<const char *>(m_data.data()), 4 * m_data.size());
    QImage now = QImage((const uchar*)img.data(), m_width, m_height, QImage::Format_RGBX8888);
    setPixmap(QPixmap::fromImage(now));
    setFixedSize(m_width, m_height);
    update();
}

void Canvas2D::resize(int w, int h) {
    m_width = w;
    m_height = h;
    m_data.resize(w * h);
    displayImage();
}

void Canvas2D::settingsChanged() {
    settings.saveSettings();
    changeMask();
}

Eigen::Vector2f Canvas2D::toVector2D(const QPointF &point) const {
    return Eigen::Vector2f(static_cast<float>(point.x()), static_cast<float>(point.y()));
}

void Canvas2D::beginStroke(const QPointF &point) {
    Stroke stroke;
    stroke.depthOrder = m_nextDepthOrder++;
    stroke.points.push_back(toVector2D(point));
    m_activeStroke = stroke;
}

// interpolate between points to control the density of the stroke saved
void Canvas2D::appendPointToActiveStroke(const QPointF &point) {
    if (!m_activeStroke.has_value()) {
        return;
    }

    const QPointF lastPoint = toQPointF(m_activeStroke->points.back());
    const float dx = static_cast<float>(point.x() - lastPoint.x());
    const float dy = static_cast<float>(point.y() - lastPoint.y());
    const float distance = std::sqrt(dx * dx + dy * dy);
    if (distance < kMinStrokePointSpacing) {
        return;
    }

    // interpolate between points to make the stroke smoother
    const int steps = std::max(1, static_cast<int>(std::floor(distance / kMinStrokePointSpacing)));
    // add points between the last point and the new point
    for (int i = 1; i <= steps; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(steps);
        const QPointF interpolated = lastPoint + (point - lastPoint) * t;
        m_activeStroke->points.push_back(toVector2D(interpolated));
    }
}

void Canvas2D::finishStroke() {
    if (!m_activeStroke.has_value()) {
        return;
    }

    if (!m_activeStroke->points.empty()) {
        m_strokes.push_back(*m_activeStroke);
    }
    m_activeStroke.reset();
}

void Canvas2D::stampMask(int x, int y) {
    int x_left = x - settings.brushRadius;
    int x_right = x + settings.brushRadius;
    int y_top = y - settings.brushRadius;
    int y_bottom = y + settings.brushRadius;

    int mask_width = 2 * settings.brushRadius + 1;

    for (int i = x_left; i <= x_right; i++)
    {
        if (i < 0 || i >= m_width) continue;
        for (int j = y_top; j <= y_bottom; j++)
        {
            if (j < 0 || j >= m_height) continue;

            float opacity = mask_data.at(posToIndex(i - x_left, j - y_top, mask_width));
            RGBA canvasColor = m_data.at(posToIndex(i, j, m_width));
            m_data.at(posToIndex(i, j, m_width)) = colorBlending(settings.brushColor, canvasColor, opacity);
        }
    }
}

void Canvas2D::drawMask(int x, int y) {
    stampMask(x, y);
    displayImage();
}

// interpolate between points to control the density of the brush stamps to better show the stroke on canvas
void Canvas2D::drawInterpolatedSegment(const QPointF &from, const QPointF &to) {
    const float dx = static_cast<float>(to.x() - from.x());
    const float dy = static_cast<float>(to.y() - from.y());
    const float distance = std::sqrt(dx * dx + dy * dy);
    if (distance < kMinStrokePointSpacing) {
        return;
    }

    const int steps = std::max(1, static_cast<int>(std::ceil(distance / kBrushStampSpacing)));
    for (int i = 1; i <= steps; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(steps);
        const QPointF interpolated = from + (to - from) * t;
        stampMask(static_cast<int>(std::lround(interpolated.x())),
                  static_cast<int>(std::lround(interpolated.y())));
    }

    displayImage();
}

void Canvas2D::changeMask() {
    int mask_width = 2 * settings.brushRadius + 1;
    mask_data.assign(mask_width * mask_width, 0);

    for (int i = 0; i < mask_width; i++)
    {
        for (int j = 0; j < mask_width; j++)
        {
            float distance = std::sqrt((i - settings.brushRadius) * (i - settings.brushRadius) + (j - settings.brushRadius) * (j -settings.brushRadius));

            if (distance <= settings.brushRadius)
            {
                mask_data.at(posToIndex(i, j, mask_width)) = 1;
            }
            else mask_data.at(posToIndex(i, j, mask_width)) = 0;
        }
    }
}

void Canvas2D::mouseDown(const QPointF &point) {
    if (isInsideCanvas(point, m_width, m_height))
    {
        beginStroke(point);
        int x = static_cast<int>(point.x());
        int y = static_cast<int>(point.y());
        drawMask(x, y);
        m_isDown = true;
    }
}

void Canvas2D::mouseDragged(const QPointF &point) {
    if (isInsideCanvas(point, m_width, m_height) && m_isDown)
    {
        if (!m_activeStroke.has_value() || m_activeStroke->points.empty()) {
            return;
        }

        const QPointF lastPoint = toQPointF(m_activeStroke->points.back());
        appendPointToActiveStroke(point);
        drawInterpolatedSegment(lastPoint, point);
    }
}

void Canvas2D::mouseUp(const QPointF &point) {
    if (m_isDown && isInsideCanvas(point, m_width, m_height)) {
        if (m_activeStroke.has_value() && !m_activeStroke->points.empty()) {
            const QPointF lastPoint = toQPointF(m_activeStroke->points.back());
            appendPointToActiveStroke(point);
            drawInterpolatedSegment(lastPoint, point);
        } else {
            appendPointToActiveStroke(point);
        }
    }

    m_isDown = false;
    finishStroke();
}

RGBA Canvas2D::colorBlending(RGBA brush, RGBA canvas, float opacity) {
    RGBA res;
    float a = float(brush.a / 255.0);
    res = brush * (opacity * a) + canvas * (1 - opacity * a) + 0.5f;
    res.a = brush.a;
    return res;
}
