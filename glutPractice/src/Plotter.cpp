#include "Plotter.h"
// #include "GL/glut.h"
#include "GL/freeglut.h"
#include "eigen3/Eigen/Geometry"

# define PI           3.14159265358979323846  /* pi */

Plotter* Plotter::currentInstance;

Plotter::Plotter() {
    currentInstance = this;

    // Initialise glut
    char * my_argv[1];
    int my_argc = 1;
    my_argv[0] = strdup("Plotter");

    glutInit(&my_argc, my_argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowPosition(100,100);
    glutInitWindowSize(1280, 720);
    glutCreateWindow("Plotter window");

    this->ratio = 1280 * 1.0 / 720;

    // register callbacks
    glutDisplayFunc(&Plotter::wrap_renderPoints);
    glutReshapeFunc(&Plotter::wrap_changeWindowSize);
    glutIdleFunc(&Plotter::wrap_renderPoints);

    // respond to mouse
    glutMouseFunc(&Plotter::wrap_mouseButton);
	glutMotionFunc(&Plotter::wrap_mouseMove);

    // Enter glut main loop

    this->plottingThread = thread(glutMainLoop);
}

void Plotter::drawPoints(const vector<Vector3d>& newPoints, const Vector4d& color, const int& size) {
    if (!this->hold) {
        this->points.clear();
        this->lines.clear();
    }

    for (const Vector3d& newPoint : newPoints) {
        PlotPoint3 point;
        point.coords = {newPoint.x(), newPoint.y(), newPoint.z()};
        point.color = color;
        point.size = size;
        this->points.emplace_back(point);
    }
    
    
    int n = points.size();
    Vector3d sum = Vector3d::Zero();
    for (const Vector3d& newPoint : newPoints) {
        sum += newPoint;
    }
    
    plotOrigin = sum / max(n,1);
}

void Plotter::drawLine(const vector<Vector3d>& newLine, const Vector4d& color, const int& size) {
    if (!this->hold) {
        this->points.clear();
        this->lines.clear();
    }

    PlotLine3 line;
    for (const Vector3d& linePoint : newLine) {
        line.coordsVec.emplace_back(linePoint);
    }
    line.color = color;
    line.size = size;
    
    this->lines.emplace_back(line);
}

void Plotter::drawAxes(const Matrix4d& pose, const double& length, const int& size) {
    if (!this->hold) {
        this->points.clear();
        this->lines.clear();
    }

    Vector3d origin = pose.block<3,1>(0,3);

    PlotLine3 xAxis;
    xAxis.color = Vector4d(1,0,0,1);
    xAxis.coordsVec.emplace_back(origin);
    xAxis.coordsVec.emplace_back(origin+length*pose.block<3,1>(0,0));
    xAxis.size = size;
    this->lines.emplace_back(xAxis);
    
    PlotLine3 yAxis;
    yAxis.color = Vector4d(0,1,0,1);
    yAxis.coordsVec.emplace_back(origin);
    yAxis.coordsVec.emplace_back(origin+length*pose.block<3,1>(0,1));
    yAxis.size = size;
    this->lines.emplace_back(yAxis);

    PlotLine3 zAxis;
    zAxis.color = Vector4d(0,0,1,1);
    zAxis.coordsVec.emplace_back(origin);
    zAxis.coordsVec.emplace_back(origin+length*pose.block<3,1>(0,2));
    zAxis.size = size;
    this->lines.emplace_back(zAxis);
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

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45,ratio,1,100);

    Vector3d eye = plotOrigin + Quaterniond(sin(angleY*PI/360), 0, cos(angleY*PI/360), 0) *
        Quaterniond(sin(angleX*PI/360), cos(angleX*PI/360), 0, 0) * Vector3d(0, 0, -zoom);

    gluLookAt(	eye.x(), eye.y(), eye.z(), // eye xyz 
            plotOrigin.x(), plotOrigin.y(), plotOrigin.z(), // scene centre
            // 0.0f, 0.0f, 0.0f, // scene centre
            0.0f, 1.0f,  0.0f // 'Up' vector
            );
    // glRotatef(this->angleX, 1.0, 0.0, 0.0);
    // glRotatef(this->angleY, 0.0, 1.0, 0.0);
    
    

    // Draw points
    glEnable(GL_POINT_SMOOTH);
	glPointSize(3);
    glBegin(GL_POINTS);
    
    for(const PlotPoint3& point : this->points) {
        
        glColor4f(point.color[0], point.color[1], point.color[2], point.color[3]);
        glVertex3f(point.coords[0], point.coords[1], point.coords[2]);
    }
	glEnd();

    // Draw lines
    glEnable(GL_LINE_SMOOTH);
    glLineWidth(4);
    for (const PlotLine3& line : this->lines) {
        glBegin(GL_LINE_STRIP);
        
        glColor4f(line.color[0], line.color[1], line.color[2], line.color[3]);
        for (const Vector3d& coords : line.coordsVec) {
            glVertex3f(coords[0], coords[1], coords[2]);
        }
        glEnd();
    }
    


    // this->angleY += 0.1;

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
    } else if (button == GLUT_RIGHT_BUTTON) {
        if (state == GLUT_UP) {
            this->mouseOriginX = -1;
            this->mouseOriginY = -1;
            this->plotMotionType = PlotMotion::NONE;
        } else {
            this->mouseOriginX = x;
            this->mouseOriginY = y;
            this->plotMotionType = PlotMotion::SCALE;
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
    }
}