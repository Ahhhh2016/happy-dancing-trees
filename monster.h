#ifndef MONSTER_H
#define MONSTER_H

#include <Eigen/Core>
#include <vector>

struct Stroke {
    std::vector<Eigen::Vector2f> points;
    bool isClosingCurve = false;
    bool isMergingBoundary = false;
    int depthOrder = 0;
};

struct Region {
    std::vector<Stroke> boundaries;
    int depthOrder = 0;
};

class monster {
public:
    monster();
    Stroke makeDummyBody();
    Stroke makeDummyLeg();
};


#endif // MONSTER_H
