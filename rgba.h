#pragma once

#include <cstdint>
#include <algorithm>

struct RGBA {
    std::uint8_t r;
    std::uint8_t g;
    std::uint8_t b;
    std::uint8_t a = 255;


    RGBA() : r(0), g(0), b(0), a(0) {}
    RGBA(int r, int g, int b, int a) : r(r), g(g), b(b), a(a) {}

    // RGBA colorBlending()

    RGBA operator*(const float opacity)
    {
        RGBA color;
        color.r = std::min((uint8_t)255, (uint8_t)(this->r * opacity + 0.5f));
        color.g = std::min((uint8_t)255, (uint8_t)(this->g * opacity + 0.5f));
        color.b = std::min((uint8_t)255, (uint8_t)(this->b * opacity + 0.5f));
        return color;
    }

    RGBA operator*(const RGBA& anotherColor)
    {
        RGBA color;
        color.r = std::min(255, this->r * anotherColor.r);
        color.g = std::min(255, this->g * anotherColor.g);
        color.b = std::min(255, this->b * anotherColor.b);
        return color;
    }

    RGBA operator+(const RGBA &anotherColor) {
        RGBA color;
        color.r = std::min(255, this->r + anotherColor.r);
        color.g = std::min(255, this->g + anotherColor.g);
        color.b = std::min(255, this->b + anotherColor.b);
        return color;
    }

    RGBA operator+(const float opacity)
    {
        RGBA color;
        color.r = std::min((uint8_t)255, (uint8_t)(this->r + opacity));
        color.g = std::min((uint8_t)255, (uint8_t)(this->g + opacity));
        color.b = std::min((uint8_t)255, (uint8_t)(this->b + opacity));
        return color;
    }
};
