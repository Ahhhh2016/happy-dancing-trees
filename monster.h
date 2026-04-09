#ifndef MONSTER_H
#define MONSTER_H

#include <Eigen/Core>
#include <vector>

struct Stroke {
    std::vector<Eigen::Vector2f> points;
    bool isClosingCurve = false;
    bool isMergingBoundary = false;
    int depthOrder = 0;

    bool isClosed(float eps = 1e-3f) const {
        if (points.size() < 3) {
            return false;
        }

        const Eigen::Vector2f delta = points.front() - points.back();
        return delta.squaredNorm() <= eps * eps;
    }
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
