#pragma once

#include <vector>
#include "eigen3/Eigen/Core"
#include "GL/gl.h"

using namespace Eigen;
using namespace std;

class PlotData3 {
private:
    vector<Vector3d> coords;
    GLenum drawType = GL_POINTS;
    Vector4d color = Vector4d(0,0,1,1);
    int size = 2;

public:
    PlotData3(const vector<Vector3d>& dataPoints, const GLenum drawType = GL_POINTS, const Vector4d& color = Vector4d(0,0,1,1), const int size = 2) {
        this->coords = dataPoints;
        this->drawType = drawType;
        this->color = color;
        this->size = size;
    }
    void draw() const {
        glPointSize(size);
        glLineWidth(size);
        glColor4f(color[0],color[1],color[2],color[3]);
        glBegin(drawType);
        for (const Vector3d& coord : coords) {
            glVertex3f(coord[0], coord[1], coord[2]);
        }
        glEnd();
    }
};
