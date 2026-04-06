#ifndef CANVAS2D_H
#define CANVAS2D_H

#include <QLabel>
#include <QMouseEvent>
#include <array>
#include <chrono>
#include "rgba.h"

class Canvas2D : public QLabel {
    Q_OBJECT
public:
    int m_width = 0;
    int m_height = 0;

    // Mouse parameters
    bool m_isDown = false;

    // For speed brush
    int m_lastX = -1;
    int m_lastY = -1;
    double m_mouseSpeed = 0.0; // pixel speed per ms
    std::chrono::steady_clock::time_point m_lastTime;
    int m_speedRadius;

    void init();
    void clearCanvas();
    bool loadImageFromFile(const QString &file);
    bool saveImageToFile(const QString &file);
    void displayImage();
    void resize(int w, int h);

    // This will be called when the settings have changed
    void settingsChanged();

    // Filter TODO: implement
    void filterImage();

    // fucntions added
    std::vector<RGBA> &getCanvasData() {return m_data;}

private:
    std::vector<RGBA> m_data;
    std::vector<float> mask_data;
    std::vector<RGBA> pre_data;
    std::vector<std::pair<int, int> > points;

    // For fixAlphaBlending
    std::vector<float> m_accumulatedAlpha;

    void mouseDown(int x, int y);
    void mouseDragged(int x, int y);
    void mouseUp(int x, int y);

    // These are functions overriden from QWidget that we've provided
    // to prevent you from having to interact with Qt's mouse events.
    // These will pass the mouse coordinates to the above mouse functions
    // that you will have to fill in.
    virtual void mousePressEvent(QMouseEvent* event) override {
        auto [x, y] = std::array{ event->position().x(), event->position().y() };
        mouseDown(static_cast<int>(x), static_cast<int>(y));
    }
    virtual void mouseMoveEvent(QMouseEvent* event) override {
        auto [x, y] = std::array{ event->position().x(), event->position().y() };
        mouseDragged(static_cast<int>(x), static_cast<int>(y));
    }
    virtual void mouseReleaseEvent(QMouseEvent* event) override {
        auto [x, y] = std::array{ event->position().x(), event->position().y() };
        mouseUp(static_cast<int>(x), static_cast<int>(y));
    }

    // TODO: add any member variables or functions you need
    int posToIndex(int x, int y, int width);
    void drawMask(int x, int y);
    RGBA colorBlending(RGBA brush, RGBA canvas, float opacity);
    RGBA colorBlendingSmudge(RGBA brush, RGBA canvas, float opacity);
    void changeMask();
    void changePreDataType();
    void saveData(int x, int y);

    // Spray paint brush
    void initSrand();
    void drawSprayPaint(int x, int y);
    void spray();

    // Speed brush
    double calculateMouseSpeed(int x, int y);
    int mapSpeedToRadius(double speed);
};

#endif // CANVAS2D_H
