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

    vector<Vector3d> line(n);
    for (int i=0; i<n; ++i) {
        line[i] = Vector3d(cos(0.5*i),0.1*i,sin(0.5*i));
    }

    Plotter p;
    p.hold = true;
    p.drawPoints(pts, Vector4d(0,0,1,0), 30);
    p.drawAxes(Matrix4d::Identity(), 1, 4);
    p.drawLine(line, Vector4d(0,0,0,1), 2);
    p.hold = false;

    p.maintain();

    return 0;

}