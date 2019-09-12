#include "Plotter.h"
// #include "GL/glut.h"
#include "GL/freeglut.h"
#include "eigen3/Eigen/Geometry"
#include <iostream>

using namespace std;

# define PI           3.14159265358979323846  /* pi */

Plotter* Plotter::currentInstance;

Plotter::Plotter() {
    currentInstance = this;

    this->ratio = 1280 * 1.0 / 720;
    this->plottingThread = thread(currentInstance->startThread);
}

void Plotter::startThread() {
    // Initialise glut
    char * my_argv[1];
    int my_argc = 1;
    my_argv[0] = strdup("Plotter");

    glutInit(&my_argc, my_argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowPosition(100,100);
    glutInitWindowSize(1280, 720);
    glutCreateWindow("Plotter window");

    // register callbacks
    glutDisplayFunc(&Plotter::wrap_renderPoints);
    glutReshapeFunc(&Plotter::wrap_changeWindowSize);
    glutIdleFunc(&Plotter::wrap_renderPoints);

    // respond to mouse
    glutMouseFunc(&Plotter::wrap_mouseButton);
	glutMotionFunc(&Plotter::wrap_mouseMove);

    // Enter glut main loop
    glutMainLoop();
}

void Plotter::drawPoints(const vector<Vector3d>& newPoints, const Vector4d& color, const int& size) {
    if (!this->hold) {
        this->plotsData.clear();
    }

    this->plotsData.emplace_back(PlotData3(newPoints, GL_POINTS, color, size));
}

void Plotter::drawLine(const vector<Vector3d>& newLine, const Vector4d& color, const int& size) {
    if (!this->hold) {
        this->plotsData.clear();
    }

    this->plotsData.emplace_back(PlotData3(newLine, GL_LINE_STRIP, color, size));
}

void Plotter::drawAxes(const Matrix4d& pose, const double& length, const int& size) {
    if (!this->hold) {
        this->plotsData.clear();
    }

    Vector3d origin = pose.block<3,1>(0,3);

    PlotData3 xAxis({origin, origin+length*pose.block<3,1>(0,0)}, GL_LINE_STRIP, Vector4d(1,0,0,1), size);
    PlotData3 yAxis({origin, origin+length*pose.block<3,1>(0,1)}, GL_LINE_STRIP, Vector4d(0,1,0,1), size);
    PlotData3 zAxis({origin, origin+length*pose.block<3,1>(0,2)}, GL_LINE_STRIP, Vector4d(0,0,1,1), size);
    
    this->plotsData.emplace_back(xAxis);
    this->plotsData.emplace_back(yAxis);
    this->plotsData.emplace_back(zAxis);
}


void Plotter::changeWindowSize(int w, int h) {

	// Prevent a divide by zero, when window is too short
	// (you cant make a window of zero width).
	if (h == 0)
		h = 1;

	this->ratio =  w * 1.0 / h;

	// Use the Projection Matrix
	glMatrixMode(GL_PROJECTION);

	// Reset Matrix
	glLoadIdentity();

	// Set the viewport to be the entire window
	glViewport(0, 0, w, h);

	// Set the correct perspective.
	gluPerspective(45,ratio,1,100);

	// Get Back to the Modelview
	glMatrixMode(GL_MODELVIEW);
}


void Plotter::renderPoints() {
    glClearColor(0.75f, 0.75f, 0.75f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45,ratio,1,100);

    Vector3d eye = Vector3d(0, 0, -zoom);
    eye = plotOrigin + Quaterniond(sin(angleY*PI/360), 0, cos(angleY*PI/360), 0) *
        Quaterniond(sin(angleX*PI/360), cos(angleX*PI/360), 0, 0) * eye;

    gluLookAt(	eye.x(), eye.y(), eye.z(), // eye xyz 
            plotOrigin.x(), plotOrigin.y(), plotOrigin.z(), // scene centre
            0.0f, 1.0f,  0.0f // 'Up' vector
            );
    
    

    // Draw things
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_LINE_SMOOTH);
	for (const PlotData3& plot : plotsData) {
        plot.draw();
    }

    glutSwapBuffers();
}

void Plotter::mouseButton(int button, int state, int x, int y) {
    // move when left mouse button is pressed.
    if (button == GLUT_LEFT_BUTTON) {
        if (state == GLUT_UP) {
            this->mouseOriginX = -1;
            this->mouseOriginY = -1;
            this->plotMotionType = PlotMotion::NONE;
        } else {
            this->mouseOriginX = x;
            this->mouseOriginY = y;
            this->plotMotionType = PlotMotion::ROTATE;
        }
    } else if (button == GLUT_MIDDLE_BUTTON) {
        if (state == GLUT_UP) {
            this->mouseOriginX = -1;
            this->mouseOriginY = -1;
            this->plotMotionType = PlotMotion::NONE;
        } else {
            this->mouseOriginX = x;
            this->mouseOriginY = y;
            this->plotMotionType = PlotMotion::SCALE;
        }
    } else if (button == GLUT_RIGHT_BUTTON) {
        if (state == GLUT_UP) {
            this->mouseOriginX = -1;
            this->mouseOriginY = -1;
            this->plotMotionType = PlotMotion::NONE;
        } else {
            this->mouseOriginX = x;
            this->mouseOriginY = y;
            this->plotMotionType = PlotMotion::TRANSLATE;
        }
    }
}

void Plotter::mouseMove(int x, int y) {
    if (plotMotionType == PlotMotion::ROTATE) {
        if (this->mouseOriginX >= 0) {
            // update rotation
            this->angleY += (x - this->mouseOriginX) * 0.5f;
            this->mouseOriginX = x;

            this->angleX = constrain(this->angleX+(y - this->mouseOriginY) * 0.5f, -90.0f, 90.0f);
            this->mouseOriginY = y;
        }
    } else if (plotMotionType == PlotMotion::SCALE) {
        if (this->mouseOriginY >= 0) {
            this->zoom *= 1.0f + (y - this->mouseOriginY) * 0.01f;
            this->mouseOriginY = y;
        }
    } else if (plotMotionType == PlotMotion::TRANSLATE && allowTranslation) {
        if (this->mouseOriginX >= 0 && this->mouseOriginY >= 0) {
            float transX = -(x - this->mouseOriginX) * 0.01f;
            this->mouseOriginX = x;

            float transY = -(y - this->mouseOriginY) * 0.01f;
            this->mouseOriginY = y;

            Vector3d trans = Quaterniond(sin(angleY*PI/360), 0, cos(angleY*PI/360), 0) *
            Quaterniond(sin(angleX*PI/360), cos(angleX*PI/360), 0, 0) * Vector3d(transX, transY, 0);

            this->plotOrigin += trans;
        }
    }
}