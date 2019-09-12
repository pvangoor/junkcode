#pragma once

#include <vector>
#include "eigen3/Eigen/Core"

using namespace Eigen;

struct PlotPoint3 {
    Vector3d coords = Vector3d::Zero();
    Vector4d color = Vector4d(0,0,1,1);
    int size = 1;
};

struct PlotLine3 {
    std::vector<Vector3d> coordsVec;
    Vector4d color = Vector4d(0,0,1,1);
    int size = 1;
};
