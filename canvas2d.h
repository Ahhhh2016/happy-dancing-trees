#ifndef CANVAS2D_H
#define CANVAS2D_H

#include <QLabel>
#include <QMouseEvent>
#include <array>
#include <chrono>
#include <optional>
#include "rgba.h"
#include "monster.h"

class QImage;
class QPainter;
class QPaintEvent;

class Canvas2D : public QLabel {
    Q_OBJECT
public:
    int m_width = 0;
    int m_height = 0;

    // Mouse parameters
    bool m_isDown = false;

    void init();
    void clearCanvas();
    bool loadImageFromFile(const QString &file);
    bool saveImageToFile(const QString &file);
    void displayImage();
    void resize(int w, int h);

    // This will be called when the settings have changed
    void settingsChanged();

    std::vector<RGBA> &getCanvasData() {return m_data;}
    const std::vector<Stroke> &getStrokes() const { return m_strokes; }
    const std::optional<Stroke> &getActiveStroke() const { return m_activeStroke; }
    const std::vector<Region> &getRegions() const { return m_regions; }

private:
    std::vector<RGBA> m_data;
    std::vector<Stroke> m_strokes; // Only contains user drawn Dp
    std::vector<Region> m_regions; // Contains all the regions, and in boundaries containing closing curves
    std::optional<Stroke> m_activeStroke; // Currently active stroke being drawn

    void mouseDown(const QPointF &point);
    void mouseDragged(const QPointF &point);
    void mouseUp(const QPointF &point);

    // These are functions overriden from QWidget that we've provided
    // to prevent you from having to interact with Qt's mouse events.
    // These will pass the mouse coordinates to the above mouse functions
    // that you will have to fill in.
    virtual void mousePressEvent(QMouseEvent* event) override {
        mouseDown(event->position());
    }
    virtual void mouseMoveEvent(QMouseEvent* event) override {
        mouseDragged(event->position());
    }
    virtual void mouseReleaseEvent(QMouseEvent* event) override {
        mouseUp(event->position());
    }
    virtual void paintEvent(QPaintEvent *event) override;

    // TODO: add any member variables or functions you need
    Eigen::Vector2f toVector2D(const QPointF &point) const;
    void beginStroke(const QPointF &point);
    void appendPointToActiveStroke(const QPointF &point);
    void finishStroke();
    bool overlapsExistingRegions(const Region &region) const;
    Stroke makeClosingCurve(const Stroke &openStroke) const;
    Region makeRegionFromStroke(const Stroke &openStroke, const Stroke &closingCurve) const;
    int computeDepthOrderForStroke(const Stroke &stroke) const;
    void commitStrokeAsRegion(const Stroke &stroke);
    QImage makeImageFromCanvasData() const;
    void loadCanvasDataFromImage(const QImage &image);
    void renderRegion(const Region &region);
    void paintStrokePreview(QPainter &painter, const Stroke &stroke) const;
};

#endif // CANVAS2D_H
