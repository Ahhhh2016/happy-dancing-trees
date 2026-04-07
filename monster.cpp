#include "monster.h"

using namespace Eigen;
using namespace std;

monster::monster() {}

Stroke monster::makeDummyBody() {
    std::vector<Eigen::Vector2d> points = {
                                           {-1.0,  0.0},
                                           {-0.7,  0.7},
                                           { 0.0,  1.0},
                                           { 0.7,  0.7},
                                           { 1.0,  0.0},
                                           { 0.7, -0.7},
                                           { 0.0, -1.0},
                                           {-0.7, -0.7},
                                           };
    return { points, true, 1 };
}

Stroke monster::makeDummyLeg() {
    std::vector<Eigen::Vector2d> points = {
        {-0.3, -0.7},  // top left, sits on body boundary
        {-0.3, -1.5},  // bottom left
        { 0.3, -1.5},  // bottom right
        { 0.3, -0.7},  // top right, sits on body boundary
    };
    return { points, false, 0 };
}
