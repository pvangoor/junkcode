#pragma once

#include "GL/glut.h"
#include <thread>

#include <vector>
#include "eigen3/Eigen/Core"

#include "PlotDataTypes.h"

using namespace std;
using namespace Eigen;

enum class PlotMotion {NONE, ROTATE, TRANSLATE, SCALE};

class Plotter {
protected:
    // Technical
    static Plotter* currentInstance;
    thread plottingThread;

    // Plot view interaction
    float ratio = 1;
    float angleX = 0;
    float angleY = 0;
    float zoom = 10.0;

    int mouseOriginX = -1;
    int mouseOriginY = -1;

    float deltaAngle = 0;

    PlotMotion plotMotionType = PlotMotion::NONE;

    Vector3d plotOrigin = Vector3d::Zero();

    // Data
    vector<PlotPoint3> points;
    vector<PlotLine3> lines;

public:

    Plotter();
    void maintain() { plottingThread.join(); };

    void drawPoints(const vector<Vector3d>& newPoints, const Vector4d& color = Vector4d(0,0,1,1), const int& size=1);
    void drawLine(const vector<Vector3d>& newLine, const Vector4d& color = Vector4d(0,0,1,1), const int& size=1);
    void drawAxes(const Matrix4d& pose, const double& length = 1.0, const int& size=1);

    bool hold = false;

private:
    static void startThread();

    // Plotting functionality wrappers
    static void wrap_renderPoints() { currentInstance->renderPoints(); };
    static void wrap_changeWindowSize(int w, int h) { currentInstance->changeWindowSize(w, h); };
    static void wrap_mouseButton(int button, int state, int x, int y) { currentInstance->mouseButton(button, state, x, y); };
    static void wrap_mouseMove(int x, int y) { currentInstance->mouseMove(x, y); };
    static void wrap_mouseWheel(int button, int dir, int x, int y) { currentInstance->mouseWheel(button, dir, x, y); };

    // Plotting functionality functions
    void changeWindowSize(int w, int h);
    void renderPoints();
    void mouseButton(int button, int state, int x, int y);
    void mouseMove(int x, int y);
    void mouseWheel(int button, int dir, int x, int y);

    // Utility
    template<class T>
    T constrain(const T& value, const T& minValue, const T& maxValue) {
        T result = value;
        result = (result < minValue) ? minValue : result;
        result = (result > maxValue) ? maxValue : result;
        return result;
    }


};