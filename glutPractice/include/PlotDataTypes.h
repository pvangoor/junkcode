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
    vector<Vector4d> colors;
    bool useMultipleColors = false;
    int size = 2;

public:
    PlotData3(const vector<Vector3d>& dataPoints, const GLenum drawType = GL_POINTS, const Vector4d& color = Vector4d(0,0,1,1), const int size = 2) {
        this->coords = dataPoints;
        this->drawType = drawType;
        this->color = color;
        this->useMultipleColors = false;
        this->size = size;
    }

    PlotData3(const vector<Vector3d>& dataPoints, const GLenum drawType = GL_POINTS, const vector<Vector4d>& colors = {}, const int size = 2) {
        assert(colors.size() == dataPoints.size());
        this->coords = dataPoints;
        this->drawType = drawType;
        this->colors = colors;
        this->useMultipleColors = true;
        this->size = size;
    }

    void draw() const {
        glPointSize(size);
        glLineWidth(size);
        glBegin(drawType);
        if (!useMultipleColors) {
            glColor4f(color[0],color[1],color[2],color[3]);
            for (const Vector3d& coord : coords) {
                glVertex3f(coord[0], coord[1], coord[2]);
            }
        }
        else {
            for (int i = 0; i < coords.size(); ++i) {
                glColor4f(colors[i][0],colors[i][1],colors[i][2],colors[i][3]);
                glVertex3f(coords[i][0], coords[i][1], coords[i][2]);
            }
        }

        glEnd();
    }
};
