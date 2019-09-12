#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

using namespace std;

const char* vertex_shader =
"#version 400\n"
"in vec3 vp;"
"void main() {"
"  gl_Position = vec4(vp, 1.0);"
"}";

const char* fragment_shader =
"#version 400\n"
"out vec4 frag_colour;"
"void main() {"
"  frag_colour = vec4(0.5, 0.0, 0.5, 1.0);"
"}";


int main(int argc, char** argv) {
    cout << "Hello glfw!" << endl;

	// Initialise GLFW
	glfwInit();

	// Open a window and create its OpenGL context
	GLFWwindow* window = glfwCreateWindow( 1280, 720, "Modern OpenGL Practice", NULL, NULL);
	glfwMakeContextCurrent(window);

	// Initialize GLEW
	glewExperimental = true; // Needed for core profile
	glewInit();

	glEnable(GL_DEPTH_TEST); // enable depth-testing
  	glDepthFunc(GL_LESS); // depth-testing interprets a smaller value as "closer"	

	// Actual graphics code
	// ----------------------------
	
	float points[] = {
	0.0f,  0.5f,  0.0f,
	0.5f, -0.5f,  0.0f,
	-0.5f, -0.5f,  0.0f
	};

	GLuint vbo = 0; // vertex buffer object
	glGenBuffers(1, &vbo); // assigns a buffer name to vbo
	glBindBuffer(GL_ARRAY_BUFFER, vbo); // bind vbo to the GL_ARRAY_BUFFER
	glBufferData(GL_ARRAY_BUFFER, sizeof(points), points, GL_STATIC_DRAW); // initialise the data store of vbo with points.
	// The final argument makes a promise about how data will be handled in the buffer.
	// STATIC means 'modified once and used many times'

	GLuint vao = 0; //vertex array object
	glGenVertexArrays(1, &vao); // assigns an array name to vao
	glBindVertexArray(vao); // binds vao with the name vao
	glEnableVertexAttribArray(0); // enables vertex attributes
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);

	GLuint vs = glCreateShader(GL_VERTEX_SHADER); // create a vertex shader
	glShaderSource(vs, 1, &vertex_shader, NULL); // attach the vertex shader source
	glCompileShader(vs); // compile the vertex shader
	GLuint fs = glCreateShader(GL_FRAGMENT_SHADER); // create a fragment shader
	glShaderSource(fs, 1, &fragment_shader, NULL); // attach the fragment shader source
	glCompileShader(fs); // compile the fragment shader

	// Create a shader program
	GLuint shader_programme = glCreateProgram();
	glAttachShader(shader_programme, fs);
	glAttachShader(shader_programme, vs);
	glLinkProgram(shader_programme);
	glDeleteShader(vs);
	glDeleteShader(fs);  

	while(!glfwWindowShouldClose(window)) {
		// wipe the drawing surface clear
		glClearColor(0.75f, 0.75f, 0.75f, 0.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glUseProgram(shader_programme);
		glBindVertexArray(vao);
		// draw points 0-3 from the currently bound VAO with current in-use shader
		glDrawArrays(GL_LINE_STRIP, 0, 3);
		// update other events like input handling 
		glfwPollEvents();
		// put the stuff we've been drawing onto the display
		glfwSwapBuffers(window);
	}

	// ----------------------------

	glfwTerminate();
	return 0;
}