#include "canvas2d.h"
#include <QPainter>
#include <QPainterPath>
#include <QMessageBox>
#include <QFileDialog>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <set>
#include "settings.h"

namespace {
constexpr float kMinStrokePointSpacing = 2.0f; // minimum distance between points in a stroke
constexpr float kIntersectionEpsilon = 1e-4f;
const QColor kFillColor(245, 245, 245);
const QColor kOutlineColor(0, 0, 0);

bool isInsideCanvas(const QPointF &point, int width, int height) {
    return point.x() >= 0 && point.x() < width && point.y() >= 0 && point.y() < height;
}

QPointF toQPointF(const Eigen::Vector2f &point) {
    return QPointF(point.x(), point.y());
}

float cross2D(const Eigen::Vector2f &a, const Eigen::Vector2f &b, const Eigen::Vector2f &c) {
    const Eigen::Vector2f ab = b - a;
    const Eigen::Vector2f ac = c - a;
    return ab.x() * ac.y() - ab.y() * ac.x();
}

bool isPointOnSegment(const Eigen::Vector2f &point,
                      const Eigen::Vector2f &segmentStart,
                      const Eigen::Vector2f &segmentEnd) {
    if (std::abs(cross2D(segmentStart, segmentEnd, point)) > kIntersectionEpsilon) {
        return false;
    }

    const float minX = std::min(segmentStart.x(), segmentEnd.x()) - kIntersectionEpsilon;
    const float maxX = std::max(segmentStart.x(), segmentEnd.x()) + kIntersectionEpsilon;
    const float minY = std::min(segmentStart.y(), segmentEnd.y()) - kIntersectionEpsilon;
    const float maxY = std::max(segmentStart.y(), segmentEnd.y()) + kIntersectionEpsilon;
    return point.x() >= minX && point.x() <= maxX && point.y() >= minY && point.y() <= maxY;
}

bool segmentsIntersect(const Eigen::Vector2f &aStart,
                       const Eigen::Vector2f &aEnd,
                       const Eigen::Vector2f &bStart,
                       const Eigen::Vector2f &bEnd) {
    const float d1 = cross2D(aStart, aEnd, bStart);
    const float d2 = cross2D(aStart, aEnd, bEnd);
    const float d3 = cross2D(bStart, bEnd, aStart);
    const float d4 = cross2D(bStart, bEnd, aEnd);

    const bool straddlesA = (d1 > kIntersectionEpsilon && d2 < -kIntersectionEpsilon) ||
                            (d1 < -kIntersectionEpsilon && d2 > kIntersectionEpsilon);
    const bool straddlesB = (d3 > kIntersectionEpsilon && d4 < -kIntersectionEpsilon) ||
                            (d3 < -kIntersectionEpsilon && d4 > kIntersectionEpsilon);
    if (straddlesA && straddlesB) {
        return true;
    }

    return isPointOnSegment(bStart, aStart, aEnd) ||
           isPointOnSegment(bEnd, aStart, aEnd) ||
           isPointOnSegment(aStart, bStart, bEnd) ||
           isPointOnSegment(aEnd, bStart, bEnd);
}

bool strokesIntersect(const Stroke &first, const Stroke &second) {
    if (first.points.size() < 2 || second.points.size() < 2) {
        return false;
    }

    for (std::size_t i = 1; i < first.points.size(); ++i) {
        const Eigen::Vector2f &aStart = first.points[i - 1];
        const Eigen::Vector2f &aEnd = first.points[i];
        for (std::size_t j = 1; j < second.points.size(); ++j) {
            const Eigen::Vector2f &bStart = second.points[j - 1];
            const Eigen::Vector2f &bEnd = second.points[j];
            if (segmentsIntersect(aStart, aEnd, bStart, bEnd)) {
                return true;
            }
        }
    }
    return false;
}

QPainterPath makeClosedFillPath(const Stroke &stroke) {
    QPainterPath fillPath;
    if (stroke.points.size() < 2) {
        return fillPath;
    }

    fillPath.moveTo(toQPointF(stroke.points.front()));
    for (std::size_t i = 1; i < stroke.points.size(); ++i) {
        fillPath.lineTo(toQPointF(stroke.points[i]));
    }
    fillPath.closeSubpath();
    return fillPath;
}
}

void Canvas2D::init() {
    setMouseTracking(true);
    m_width = 500;
    m_height = 500;
    clearCanvas();
}

void Canvas2D::clearCanvas() {
    m_data.assign(m_width * m_height, RGBA{255, 255, 255, 255});
    m_strokes.clear();
    m_regions.clear();
    m_connectedRegions.clear();
    m_regionToComponent.clear();
    m_activeStroke.reset();
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
    m_regions.clear();
    m_connectedRegions.clear();
    m_regionToComponent.clear();
    m_activeStroke.reset();
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
}

Eigen::Vector2f Canvas2D::toVector2D(const QPointF &point) const {
    return Eigen::Vector2f(static_cast<float>(point.x()), static_cast<float>(point.y()));
}

void Canvas2D::beginStroke(const QPointF &point) {
    Stroke stroke;
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

    Stroke stroke = *m_activeStroke;
    if (!stroke.points.empty()) {
        commitStrokeAsRegion(stroke);
    }
    m_activeStroke.reset();
}

std::vector<int> Canvas2D::findOverlappingRegions(const Region &region) const {
    std::vector<int> overlapping;
    if (region.boundaries.empty()) {
        return overlapping;
    }

    const QPainterPath currentFillPath = makeClosedFillPath(region.boundaries.front());
    if (currentFillPath.isEmpty()) {
        return overlapping;
    }

    for (std::size_t idx = 0; idx < m_regions.size(); ++idx) {
        const Region &existingRegion = m_regions[idx];
        if (existingRegion.boundaries.empty()) {
            continue;
        }

        bool overlaps = false;
        const QPainterPath existingFillPath = makeClosedFillPath(existingRegion.boundaries.front());
        if (!currentFillPath.intersected(existingFillPath).isEmpty()) {
            overlaps = true;
        } else {
            for (const Stroke &currentBoundary : region.boundaries) {
                for (const Stroke &existingBoundary : existingRegion.boundaries) {
                    if (strokesIntersect(currentBoundary, existingBoundary)) {
                        overlaps = true;
                        break;
                    }
                }
                if (overlaps) {
                    break;
                }
            }
        }

        if (overlaps) {
            overlapping.push_back(static_cast<int>(idx));
        }
    }
    return overlapping;
}

std::vector<int> Canvas2D::getConnectedRegions(int regionIdx) const {
    if (regionIdx < 0 ||
        static_cast<std::size_t>(regionIdx) >= m_regionToComponent.size()) {
        return {};
    }
    return m_connectedRegions[m_regionToComponent[regionIdx]];
}

Stroke Canvas2D::makeClosingCurve(const Stroke &openStroke) const {
    Stroke closing;
    if (openStroke.points.size() < 2) {
        return closing;
    }

    closing.points.push_back(openStroke.points.back());
    closing.points.push_back(openStroke.points.front());
    closing.isClosingCurve = true;
    closing.depthOrder = openStroke.depthOrder;
    return closing;
}

Region Canvas2D::makeRegionFromStroke(const Stroke &openStroke, const Stroke &closingCurve) const {
    Region region;
    region.depthOrder = openStroke.depthOrder;
    region.boundaries.push_back(openStroke);
    if (!closingCurve.points.empty()) {
        region.boundaries.push_back(closingCurve);
    }
    return region;
}

int Canvas2D::computeDepthOrderForStroke(const Stroke &stroke) const {
    int maxIntersectingDepth = -1;
    for (const Stroke &existingStroke : m_strokes) {
        if (strokesIntersect(stroke, existingStroke)) {
            maxIntersectingDepth = std::max(maxIntersectingDepth, existingStroke.depthOrder);
        }
    }

    return maxIntersectingDepth + 1;
}

void Canvas2D::commitStrokeAsRegion(const Stroke &stroke) {
    if (stroke.points.size() < 2) {
        return;
    }

    Stroke depthAssignedStroke = stroke;
    depthAssignedStroke.isMergingBoundary = false;
    depthAssignedStroke.depthOrder = computeDepthOrderForStroke(stroke);

    Stroke closingCurve;
    if (!depthAssignedStroke.isClosed()) {
        closingCurve = makeClosingCurve(depthAssignedStroke);
    }

    Region region = makeRegionFromStroke(depthAssignedStroke, closingCurve);
    const std::vector<int> overlapping = findOverlappingRegions(region);
    if (!closingCurve.points.empty() &&
        closingCurve.isClosingCurve &&
        !overlapping.empty()) {
        closingCurve.isMergingBoundary = true;
        region.boundaries.back().isMergingBoundary = true;
    }

    m_strokes.push_back(depthAssignedStroke);
    m_regions.push_back(region);

    const int newRegionIdx = static_cast<int>(m_regions.size()) - 1;

    // Collect (via the reverse-index map) the indices of every existing
    // component that touches any overlapping region.
    std::set<int> touchingComponentIdxs;
    for (int r : overlapping) {
        touchingComponentIdxs.insert(m_regionToComponent[r]);
    }

    // Merge all touching components + the new region into one new component.
    std::vector<int> mergedComponent;
    mergedComponent.push_back(newRegionIdx);
    for (int ci : touchingComponentIdxs) {
        const std::vector<int> &component = m_connectedRegions[ci];
        mergedComponent.insert(mergedComponent.end(), component.begin(), component.end());
    }

    // Erase merged components from the back so earlier indices stay valid.
    for (auto it = touchingComponentIdxs.rbegin(); it != touchingComponentIdxs.rend(); ++it) {
        m_connectedRegions.erase(m_connectedRegions.begin() + *it);
    }

    m_connectedRegions.push_back(std::move(mergedComponent));

    // Rebuild the reverse index. Component indices may have shifted after the
    // erases above, so walk every component and remap all of its regions.
    m_regionToComponent.assign(m_regions.size(), -1);
    for (int ci = 0; ci < static_cast<int>(m_connectedRegions.size()); ++ci) {
        for (int r : m_connectedRegions[ci]) {
            m_regionToComponent[r] = ci;
        }
    }

    renderRegion(region);
}

QImage Canvas2D::makeImageFromCanvasData() const {
    QImage image(m_width, m_height, QImage::Format_RGBX8888);
    for (int i = 0; i < static_cast<int>(m_data.size()); ++i) {
        image.setPixelColor(
            i % m_width,
            i / m_width,
            QColor(m_data[i].r, m_data[i].g, m_data[i].b, m_data[i].a)
        );
    }
    return image;
}

void Canvas2D::paintEvent(QPaintEvent *event) {
    QLabel::paintEvent(event);

    if (!m_activeStroke.has_value()) {
        return;
    }

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    paintStrokePreview(painter, *m_activeStroke);
}

void Canvas2D::paintStrokePreview(QPainter &painter, const Stroke &stroke) const {
    if (stroke.points.size() < 2) {
        return;
    }

    QPainterPath path;
    path.moveTo(toQPointF(stroke.points.front()));
    for (std::size_t i = 1; i < stroke.points.size(); ++i) {
        path.lineTo(toQPointF(stroke.points[i]));
    }
    painter.setPen(QPen(kOutlineColor, 2));
    painter.drawPath(path);
}

void Canvas2D::loadCanvasDataFromImage(const QImage &image) {
    QImage converted = image.convertToFormat(QImage::Format_RGBX8888);
    m_data.clear();
    m_data.reserve(converted.width() * converted.height());

    QByteArray arr = QByteArray::fromRawData(
        reinterpret_cast<const char *>(converted.bits()),
        converted.sizeInBytes()
    );

    for (int i = 0; i < arr.size() / 4; ++i) {
        m_data.push_back(RGBA{
            static_cast<std::uint8_t>(arr[4 * i]),
            static_cast<std::uint8_t>(arr[4 * i + 1]),
            static_cast<std::uint8_t>(arr[4 * i + 2]),
            static_cast<std::uint8_t>(arr[4 * i + 3])
        });
    }
}

void Canvas2D::renderRegion(const Region &region) {
    if (region.boundaries.empty()) {
        return;
    }

    const Stroke &openStroke = region.boundaries.front();
    if (openStroke.points.size() < 2) {
        return;
    }

    QImage image = makeImageFromCanvasData();
    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing, true);

    QPainterPath fillPath = makeClosedFillPath(openStroke);

    painter.fillPath(fillPath, kFillColor);
    painter.setPen(QPen(kOutlineColor, 2));
    for (std::size_t i = 1; i < openStroke.points.size(); ++i) {
        painter.drawLine(
            toQPointF(openStroke.points[i - 1]),
            toQPointF(openStroke.points[i])
        );
    }

    painter.end();
    loadCanvasDataFromImage(image);
    displayImage();
}

void Canvas2D::mouseDown(const QPointF &point) {
    if (isInsideCanvas(point, m_width, m_height))
    {
        beginStroke(point);
        m_isDown = true;
        update();
    }
}

void Canvas2D::mouseDragged(const QPointF &point) {
    if (isInsideCanvas(point, m_width, m_height) && m_isDown)
    {
        if (!m_activeStroke.has_value() || m_activeStroke->points.empty()) {
            return;
        }
        appendPointToActiveStroke(point);
        update();
    }
}

void Canvas2D::mouseUp(const QPointF &point) {
    if (m_isDown && isInsideCanvas(point, m_width, m_height)) {
        appendPointToActiveStroke(point);
    }

    m_isDown = false;
    finishStroke();
    update();
}
