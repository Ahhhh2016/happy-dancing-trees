#ifndef BUILDLAYERCONSTRAINTS_H
#define BUILDLAYERCONSTRAINTS_H

#include "Eigen/Core"
#include "arap.h"
#include "monster.h"
class buildLayerConstraints
{
public:
    buildLayerConstraints();
    static std::vector<LayerConstraint> buildiLayerConstraints(
        const std::vector<Eigen::Vector3f>  &vertices,
        const StitchedMesh                  &mesh);
};

#endif // BUILDLAYERCONSTRAINTS_H
