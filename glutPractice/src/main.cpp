#include "Plotter.h"

#include "eigen3/Eigen/Core"

#include <vector>
#include <iostream>
#include <chrono>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {
    cout << "Hello GLUT!" << endl;

    // plot some random points
    int n = 50;
    vector<Vector3d> pts(n);

    for (int i=0; i<n; ++i) {
        pts[i] = 3*Vector3d::Random()+ Vector3d(1,1,1);
    }

    Plotter p;
    p.drawPoints(pts);
    p.hold = true;
    p.drawAxes(Matrix4d::Identity(), 1);
    p.hold = false;

    p.maintain();

    return 0;

}