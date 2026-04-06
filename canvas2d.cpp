#include "canvas2d.h"
#include <QPainter>
#include <QMessageBox>
#include <QFileDialog>
#include <iostream>
#include "settings.h"

/**
 * @brief Initializes new 500x500 canvas
 */
void Canvas2D::init() {
    setMouseTracking(true);
    m_width = 500;
    m_height = 500;
    clearCanvas();

    changePreDataType();
    changeMask();

    if (settings.brushType == BRUSH_SPRAY)
    {
        initSrand();
    }
}

// Call this function when settings are changed
void Canvas2D::changePreDataType() {
    int mask_width = settings.brushRadius * 2 + 1;
    pre_data.assign(mask_width * mask_width, RGBA{255, 255, 255, 255});
}

// Convert an (x, y) position
//         into an index you can use to index into the array of canvas data.
int Canvas2D::posToIndex(int x, int y, int width) {
    int index = y * width + x;
    return index;
}


/**
 * @brief Canvas2D::clearCanvas sets all canvas pixels to blank white
 */
void Canvas2D::clearCanvas() {
    m_data.assign(m_width * m_height, RGBA{255, 255, 255, 255});
    settings.imagePath = "";
    displayImage();
}

/**
 * @brief Stores the image specified from the input file in this class's
 * `std::vector<RGBA> m_image`.
 * Also saves the image width and height to canvas width and height respectively.
 * @param file: file path to an image
 * @return True if successfully loads image, False otherwise.
 */
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
    displayImage();
    return true;
}

/**
 * @brief Saves the current canvas image to the specified file path.
 * @param file: file path to save image to
 * @return True if successfully saves image, False otherwise.
 */
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


/**
 * @brief Get Canvas2D's image data and display this to the GUI
 */
void Canvas2D::displayImage() {
    QByteArray img(reinterpret_cast<const char *>(m_data.data()), 4 * m_data.size());
    QImage now = QImage((const uchar*)img.data(), m_width, m_height, QImage::Format_RGBX8888);
    setPixmap(QPixmap::fromImage(now));
    setFixedSize(m_width, m_height);
    update();
}

/**
 * @brief Canvas2D::resize resizes canvas to new width and height
 * @param w
 * @param h
 */
void Canvas2D::resize(int w, int h) {
    m_width = w;
    m_height = h;
    m_data.resize(w * h);
    displayImage();
}

/**
 * @brief Called when the filter button is pressed in the UI
 */
void Canvas2D::filterImage() {
    // Filter TODO: apply the currently selected filter to the loaded image
}

/**
 * @brief Called when any of the parameters in the UI are modified.
 */
void Canvas2D::settingsChanged() {
    // this saves your UI settings locally to load next time you run the program
    settings.saveSettings();

    // TODO: fill in what you need to do when brush or filter parameters change
    changeMask();
    changePreDataType();

    if (settings.brushType == BRUSH_SPRAY) initSrand();
}


// Called when mouse down and mouse moved.
// Used to save the previous data on canvas for smudge brush and fix alpha blending
void Canvas2D::saveData(int x, int y) {
    int x_left = x - settings.brushRadius;
    int x_right = x + settings.brushRadius;
    int y_top = y - settings.brushRadius;
    int y_bottom = y + settings.brushRadius;

    int mask_width = 2 * settings.brushRadius + 1;


    for (int i = x_left; i <= x_right; i++)
    {
        for (int j = y_top; j <= y_bottom; j++)
        {
            if (i < 0 || i >= m_width || j < 0 || j >= m_width)
            {
                pre_data.at(posToIndex(i - x_left, j - y_top, mask_width)) = RGBA(255, 255, 255, 255);
                continue;
            }

            pre_data.at(posToIndex(i - x_left, j - y_top, mask_width)) = m_data.at(posToIndex(i, j, m_width));
        }
    }
}


// Draw on the canvas. The inputs of this function are
//         the x and y coordinates of the center of the drawing point
void Canvas2D::drawMask(int x, int y) {
    int x_left = x - settings.brushRadius;
    int x_right = x + settings.brushRadius;
    int y_top = y - settings.brushRadius;
    int y_bottom = y + settings.brushRadius;

    int mask_width = 2 * settings.brushRadius + 1;

    if (settings.fixAlphaBlending) { // Save the canvas data before drawing
        saveData(x, y);
    }

    for (int i = x_left; i <= x_right; i++)
    {
        if (i < 0 || i >= m_width) continue;
        for (int j = y_top; j <= y_bottom; j++)
        {
            if (j < 0 || j >= m_width) continue;

            float opacity = mask_data.at(posToIndex(i - x_left, j - y_top, mask_width));
            RGBA canvasColor = m_data.at(posToIndex(i, j, m_width));
            RGBA baseColor = pre_data.at(posToIndex(i - x_left, j - y_top, mask_width));
            if (settings.brushType == BRUSH_SMUDGE)
            {
                m_data.at(posToIndex(i, j, m_width)) = colorBlendingSmudge(baseColor, canvasColor, opacity);
            }
            else if (settings.fixAlphaBlending == true)
            {
                float alpha_to_add = opacity * (1.0f - m_accumulatedAlpha.at(posToIndex(i, j, m_width)));
                m_accumulatedAlpha.at(posToIndex(i, j, m_width)) += alpha_to_add;
                m_data.at(posToIndex(i, j, m_width)) = colorBlending(settings.brushColor, baseColor, alpha_to_add);
            }
            else
            {
                m_data.at(posToIndex(i, j, m_width)) = colorBlending(settings.brushColor, canvasColor, opacity);
            }
        }
    }

    if (settings.fixAlphaBlending == false) {
        saveData(x, y);
    }

    displayImage();
}

// Change the mask color distribution when settings are changed or in speed brush, everytime mouse is dragged
void Canvas2D::changeMask() {
    int mask_width = 2 * settings.brushRadius + 1;
    mask_data.assign(mask_width * mask_width, 0);

    for (int i = 0; i < mask_width; i++)
    {
        for (int j = 0; j < mask_width; j++)
        {
            float distance = std::sqrt((i - settings.brushRadius) * (i - settings.brushRadius) + (j - settings.brushRadius) * (j -settings.brushRadius));

            if (settings.brushType == BRUSH_CONSTANT)
            {
                if (distance <= settings.brushRadius)
                {
                    mask_data.at(posToIndex(i, j, mask_width)) = 1;
                }
                else mask_data.at(posToIndex(i, j, mask_width)) = 0;
            }
            else if (settings.brushType == BRUSH_LINEAR || settings.brushType == BRUSH_SMUDGE)
            {
                if (distance <= settings.brushRadius)
                {
                    mask_data.at(posToIndex(i, j, mask_width)) = 1.0 - (float)(distance / settings.brushRadius);
                }
                else mask_data.at(posToIndex(i, j, mask_width)) = 0;
            }
            else if (settings.brushType == BRUSH_QUADRATIC)
            {
                if (distance <= settings.brushRadius)
                {
                    mask_data.at(posToIndex(i, j, mask_width)) = (distance * distance) / (float)(settings.brushRadius * settings.brushRadius) -
                                                                 (2.0 * distance / (float)settings.brushRadius) + 1.0;
                }
                else mask_data.at(posToIndex(i, j, mask_width)) = 0;
            }
            else if (settings.brushType == BRUSH_SPEED)
            {
                if (distance <= m_speedRadius) // Only draw within the speed radius
                {
                    mask_data.at(posToIndex(i, j, mask_width)) = 1;
                }
                else mask_data.at(posToIndex(i, j, mask_width)) = 0;
            }
        }
    }
}

/**
 * @brief These functions are called when the mouse is clicked and dragged on the canvas
 */
void Canvas2D::mouseDown(int x, int y) {
    // Brush TODO
    if (x >= 0 && x < m_width && y >= 0 && y < m_height)
    {
        // Initialise speed
        m_lastX = x;
        m_lastY = y;
        m_lastTime = std::chrono::steady_clock::now();

        // Put accumulated alpha back to 0
        m_accumulatedAlpha.assign(m_width * m_height, 0.0f);

        saveData(x, y);
        if (settings.brushType != BRUSH_SPRAY) drawMask(x, y);
        else drawSprayPaint(x, y);
        m_isDown = true;
    }
}

void Canvas2D::mouseDragged(int x, int y) {
    // Brush TODO
    if (x >= 0 && x < m_width && y >= 0 && y < m_height && m_isDown)
    {
        // Calculate brush radius for brush speed
        if (settings.brushType == BRUSH_SPEED) {
            double speed = calculateMouseSpeed(x, y);
            m_speedRadius = mapSpeedToRadius(speed);
            changeMask();
            // std::cout << x << ", " << y << " speed=" << speed << " radius=" << m_speedRadius << " " << settings.brushRadius << std::endl;
        }

        if (settings.brushType != BRUSH_SPRAY) drawMask(x, y);
        else drawSprayPaint(x, y);
    }
}

void Canvas2D::mouseUp(int x, int y) {
    // Brush TODO
    m_isDown = false;
    saveData(x, y);
}


RGBA Canvas2D::colorBlendingSmudge(RGBA brush, RGBA canvas, float opacity) {
    RGBA res;
    res = brush * opacity + canvas * (1 - opacity);
    res.a = brush.a;
    return res;
}


// mixing colors
RGBA Canvas2D::colorBlending(RGBA brush, RGBA canvas, float opacity) {
    RGBA res;
    float a = float(brush.a / 255.0);
    res = brush * (opacity * a) + canvas * (1 - opacity * a) + 0.5f;
    res.a = brush.a;
    // std::cout << a << " " << opacity<< std::endl;
    // std::cout << (int)res.r << " " << (int)res.g << " " << (int)res.b << std::endl;
    // std::cout << (int)brush.r << " " << (int)brush.g << " " << (int)brush.b << std::endl;
    // std::cout << (int)canvas.r << " " << (int)canvas.g << " " << (int)canvas.b << std::endl;
    return res;
}


// =============================== Spray Paint Brush ===============================

// Initialize the random number generator
void Canvas2D::initSrand() {
    std::srand(static_cast<unsigned>(std::time(nullptr)));
    spray();
}

void Canvas2D::spray() {
    points.clear();

    int density = 0;
    double pi = M_PI;

    if (settings.brushDensity == 100)
    {
        density = (int) ((double)settings.brushDensity / 100 * (settings.brushRadius * settings.brushRadius * pi)) * 100;
    }
    else
    {
        density = (int) ((double)settings.brushDensity / 100 * (settings.brushRadius * settings.brushRadius * pi));
    }

    for (int i = 0; i < density; ++i) {
        int x = rand() % (2 * settings.brushRadius) - settings.brushRadius; // Randomly generate x coordinates
        int y = rand() % (2 * settings.brushRadius) - settings.brushRadius; // Randomly generate y coordinates

        // Check if the point is within the circle
        if (x * x + y * y <= settings.brushRadius * settings.brushRadius) {
            points.push_back(std::make_pair(x, y));
        }
    }
}

void Canvas2D::drawSprayPaint(int x, int y) {
    spray();

    for (int i = 0; i < points.size(); i++)
    {
        if (x + points[i].first < 0 || x + points[i].first >= m_width
            || y + points[i].second < 0 || y + points[i].second >= m_width) continue;
        m_data.at(posToIndex(x + points[i].first, y + points[i].second, m_width)) = settings.brushColor;
    }

    saveData(x, y);

    displayImage();
}


// =============================== Speed Brush ===============================

double Canvas2D::calculateMouseSpeed(int x, int y) {
    using namespace std::chrono;

    auto now = steady_clock::now();

    if (m_lastX >= 0 && m_lastY >= 0) {
        double dx = x - m_lastX;
        double dy = y - m_lastY;
        double distance = std::sqrt(dx * dx + dy * dy);

        auto dt = duration_cast<milliseconds>(now - m_lastTime).count();
        if (dt > 0) {
            m_mouseSpeed = distance / dt; // pixel per ms
        }
    }

    m_lastX = x;
    m_lastY = y;
    m_lastTime = now;

    return m_mouseSpeed;
}

int Canvas2D::mapSpeedToRadius(double speed) {
    // speed: pixel / ms
    const int baseRadius = 1;
    const int maxRadius = settings.brushRadius;

    // Avoid speed = 0
    double adjustedSpeed = std::max(speed, 0.1);

    // inverse relationship: radius = maxRadius / (1 + k * speed)
    double k = 0.5;  // Control sensitivity
    double radius = static_cast<double>(maxRadius) / (1.0 + k * adjustedSpeed);

    if (radius < baseRadius) radius = baseRadius;
    return (int)(radius);
}

