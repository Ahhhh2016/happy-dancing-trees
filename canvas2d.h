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
    enum class Tool { Brush, Eraser };

    int m_width = 0;
    int m_height = 0;

    // Mouse parameters
    bool m_isDown = false;

    void setTool(Tool t);
    Tool tool() const { return m_tool; }

    void init();
    void clearCanvas();
    bool loadImageFromFile(const QString &file);
    bool saveImageToFile(const QString &file);
    /// Writes pixels for 3D texturing: imported template + region fills, never stroke outlines.
    bool saveMeshTextureToFile(const QString &file);
    void displayImage();
    void resize(int w, int h);

    // This will be called when the settings have changed
    void settingsChanged();

    std::vector<RGBA> &getCanvasData() {return m_data;}
    const std::vector<Stroke> &getStrokes() const { return m_strokes; }
    const std::optional<Stroke> &getActiveStroke() const { return m_activeStroke; }
    const std::vector<Region> &getRegions() const { return m_regions; }

    // Each inner vector is one connected component and holds the indices (of m_regions) of every region in that component. 
    // If there are n regions partitioned into m connected components, m_connectedRegions has size m and the union of all inner vectors is {0, ..., n-1}.
    const std::vector<std::vector<int>> &getAllConnectedRegions() const { return m_connectedRegions; }

    /// True after loading an external image; region commits skip gray fill so art stays visible for tracing.
    bool hasImportedTemplate() const { return m_hasImportedTemplate; }

    // Returns the component (list of region indices) containing regionIdx,
    // or an empty vector if regionIdx is not connected to any other region.
    std::vector<int> getConnectedRegions(int regionIdx) const;

private:
    std::vector<RGBA> m_data;
    /// Same dimensions as m_data; used only when exporting mesh textures (no black strokes).
    std::vector<RGBA> m_textureNoStroke;
    std::vector<Stroke> m_strokes; // Only contains user drawn Dp
    std::vector<Region> m_regions; // Contains all the regions, and in boundaries containing closing curves
    std::optional<Stroke> m_activeStroke; // Currently active stroke being drawn

    // Partition of m_regions into connected components. 
    // Each inner vector is one connected component and holds the indices (of m_regions) of every region in that component. 
    // Maintained incrementally as new regions are committed.
    std::vector<std::vector<int>> m_connectedRegions;

    // Reverse index: m_regionToComponent[r] is the index into
    // m_connectedRegions of the component that contains region r. 
    std::vector<int> m_regionToComponent;

    bool m_hasImportedTemplate = false;
    Tool m_tool = Tool::Brush;

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
    // Returns indices into m_regions of every existing region that the given
    // region overlaps (via fill-area intersection or boundary-stroke
    // intersection). Empty vector means the region is disjoint from all others.
    std::vector<int> findOverlappingRegions(const Region &region) const;
    Stroke makeClosingCurve(const Stroke &openStroke) const;
    Region makeRegionFromStroke(const Stroke &openStroke, const Stroke &closingCurve) const;
    int computeDepthOrderForStroke(const Stroke &stroke) const;
    void commitStrokeAsRegion(const Stroke &stroke);
    QImage makeImageFromCanvasData() const;
    QImage makeImageFromTextureNoStrokeData() const;
    void loadCanvasDataFromImage(const QImage &image);
    void loadTextureNoStrokeFromImage(const QImage &image);
    void renderRegion(const Region &region, bool updateDisplay = true);
    void paintStrokePreview(QPainter &painter, const Stroke &stroke) const;

    float pointToPolylineDistSq(const Eigen::Vector2f &p, const Stroke &stroke) const;
    int pickStrokeIndexAt(const Eigen::Vector2f &p, float maxDistPx) const;
    void eraseStrokeAtIndex(int strokeIdx);
    bool regionsOverlap(const Region &a, const Region &b) const;
    void recomputeConnectedComponents();
    void rebuildCanvasFromRegions();
};

#endif // CANVAS2D_H
