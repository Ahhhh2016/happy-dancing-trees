#include "canvas2d.h"
#include <QPainter>
#include <QMessageBox>
#include <QFileDialog>
#include <iostream>
#include "settings.h"

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

void Canvas2D::drawMask(int x, int y) {
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

void Canvas2D::mouseDown(int x, int y) {
    if (x >= 0 && x < m_width && y >= 0 && y < m_height)
    {
        drawMask(x, y);
        m_isDown = true;
    }
}

void Canvas2D::mouseDragged(int x, int y) {
    if (x >= 0 && x < m_width && y >= 0 && y < m_height && m_isDown)
    {
        drawMask(x, y);
    }
}

void Canvas2D::mouseUp(int x, int y) {
    (void)x;
    (void)y;
    m_isDown = false;
}

RGBA Canvas2D::colorBlending(RGBA brush, RGBA canvas, float opacity) {
    RGBA res;
    float a = float(brush.a / 255.0);
    res = brush * (opacity * a) + canvas * (1 - opacity * a) + 0.5f;
    res.a = brush.a;
    return res;
}
