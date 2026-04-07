#ifndef MONSTER_H
#define MONSTER_H

#include <Eigen/Dense>
#include <vector>

struct Stroke {
    std::vector<Eigen::Vector2d> D;
    bool isClosed;
    int depthOrder;
};

class monster {
public:
    monster();
    Stroke makeDummyBody();
    Stroke makeDummyLeg();
};


#endif // MONSTER_H
