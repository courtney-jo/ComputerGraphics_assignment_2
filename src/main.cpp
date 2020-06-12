#include "Helpers.h"	// OpenGL Helpers to reduce the clutter
#include <GLFW/glfw3.h>	// GLFW is necessary to handle the OpenGL context
#include <Eigen/Core>	// Linear Algebra Library
#include <Eigen/Dense>
#include <chrono>		// Timer
#include <iostream>

using namespace Eigen;
/*
VBO contains object data
VAO discribes how the data is interprted
*/
// VertexBufferObject wrapper
VertexBufferObject VBO;
VertexBufferObject VBO_C;

// Contains the vertex positions
Eigen::MatrixXf V(2,3);
Eigen::MatrixXf V_tmp(2,3);

//Contains color
Eigen::MatrixXf C(3, 3);
Eigen::MatrixXf C_tmp(3, 3);

// Contains the view transformation
Eigen::Matrix4f view(4, 4);

int V_Size = 0;
int closestVIndex = 0;
double center_x = 0.0;
double center_y = 0.0;

/*Function declareation*/
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);

//function to find closest vertex from last mouse click
int closestVertex(double x, double y) {
	int index = 0;
	double delta = sqrt((abs(V(0, 0) - x)*abs(V(0, 0) - x)) + (abs(V(1, 0) - y)*abs(V(1, 0) - y)));

	for (int i = 0; i < V_Size; ++i) {
		double tmpDelta = sqrt((abs(V(0, i) - x)*abs(V(0, i) - x))+(abs(V(1, i) - y)*abs(V(1, i) - y)));

		if (tmpDelta < delta) {
			delta = tmpDelta;
			index = i;
		}
	}

	return index;
}

//function to calculate center of triangle
void triangleCenter(double& center_x, double& center_y, int loc) {
	center_x = (V(0, loc) + V(0, loc + 1) + V(0, loc + 2))/3;
	center_y = (V(1,loc)+V(1,loc+1)+V(1,loc+2))/3;
}

//Finds area of triangle using coordinates
double triArea(double x1, double y1, double x2, double y2, double x3, double y3) {
	double a = x1 * (y2 - y3);
	double b = x2 * (y3 - y1);
	double c = x3 * (y1 - y2);

	return abs((a + b + c) / 2.0);
}

//returns true if point is within a triangle
bool selectedTri(double x, double y,int& index) {
	bool found = false;
	for (int i = 0; i < V_Size-2; i=i+3) {
		double a = triArea(V(0,i),V(1,i),V(0,i+1),V(1,i+1),V(0,i+2),V(1,i+2));
		double a1 = triArea(x, y, V(0, i), V(1, i), V(0, i + 1), V(1, i + 1));
		double a2 = triArea(x, y, V(0, i + 1), V(1, i + 1), V(0, i + 2), V(1, i + 2));
		double a3 = triArea(x, y, V(0, i), V(1, i), V(0, i + 2), V(1, i + 2));
		if (a == (a1 + a2 + a3)) {
			found = true;
			index = i;
			break;
		}
	}
	return found;
}

//function to keep track of new points added for triangle formations
void insert_I(GLFWwindow* window, int button, int action, int mods) {
	// Get the position of the mouse in the window
	double xpos, ypos;
	glfwGetCursorPos(window, &xpos, &ypos);

	// Get the size of the window
	int width, height;
	glfwGetWindowSize(window, &width, &height);

	// Convert screen position to world coordinates
	Eigen::Vector4f p_screen(xpos, height - 1 - ypos, 0, 1);
	Eigen::Vector4f p_canonical((p_screen[0] / width) * 2 - 1, (p_screen[1] / height) * 2 - 1, 0, 1);
	Eigen::Vector4f p_world = view.inverse()*p_canonical;
	double xworld = p_world[0];
	double yworld = p_world[1]; // NOTE: y axis is flipped in glfw

	// add new vertex to matrix
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
		//copy over elements before resizing
		V_tmp.resize(2, V_Size);
		C_tmp.resize(3, V_Size);
		for (int i = 0; i < V_Size; ++i) {
			V_tmp.col(i) = V.col(i);
			C_tmp.col(i) = C.col(i);
		}

		//resize and add new point to V
		V.resize(2, V_Size+1);
		C.resize(3, V_Size + 1);
		for (int i = 0; i < V_Size; ++i) {
			V.col(i) = V_tmp.col(i);
			C.col(i) = C_tmp.col(i);
		}
		V.col(V_Size) << xworld, yworld;
		C.col(V_Size) << 0.0, 0.0, 0.0;

		++V_Size;
	}
	VBO.update(V);
	VBO_C.update(C);
}

void rotate(double a) {
	double x1 = V(0, closestVIndex);
	double x2 = V(0, closestVIndex+1);
	double x3 = V(0, closestVIndex+2);
	double y1 = V(1, closestVIndex);
	double y2 = V(1, closestVIndex+1);
	double y3 = V(1, closestVIndex+2);

	V(0, closestVIndex) = center_x + (center_x - x1) * cos(a) + (y1 - center_y) * sin(a);
	V(1, closestVIndex) = center_y + (center_x - x1) * sin(a) - (y1 - center_y) * cos(a);

	V(0, closestVIndex+1) = center_x + (center_x - x2) * cos(a) + (y2 - center_y) * sin(a);
	V(1, closestVIndex+1) = center_y + (center_x - x2) * sin(a) - (y2 - center_y) * cos(a);

	V(0, closestVIndex+2) = center_x + (center_x - x3) * cos(a) + (y3 - center_y) * sin(a);
	V(1, closestVIndex+2) = center_y + (center_x - x3) * sin(a) - (y3 - center_y) * cos(a);

	VBO.update(V);
}

void scale(double s) {
	double x1 = V(0, closestVIndex);
	double x2 = V(0, closestVIndex + 1);
	double x3 = V(0, closestVIndex + 2);
	double y1 = V(1, closestVIndex);
	double y2 = V(1, closestVIndex + 1);
	double y3 = V(1, closestVIndex + 2);

	V(0, closestVIndex) = (x1 - center_x)*s + center_x;
	V(1, closestVIndex) = (y1 - center_y)*s + center_y;

	V(0, closestVIndex + 1) = (x2 - center_x)*s + center_x;
	V(1, closestVIndex + 1) = (y2 - center_y)*s + center_y;

	V(0, closestVIndex + 2) = (x3 - center_x)*s + center_x;
	V(1, closestVIndex + 2) = (y3 - center_y)*s + center_y;

	VBO.update(V);
}

//counter_clockwise is positive angle	glRotatef()		glScalef()	***NEEDS WORK***
void rotate_scale(GLFWwindow* window, int key, int scancode, int action, int mods) {
	switch (key) {
		//rotation & scale	h=clockwise	j=counter-clockwise	k=scale up	l=scale down
	case  GLFW_KEY_H:	rotate((-10*180)/3.14);		break;
	case  GLFW_KEY_J:	rotate((10 * 180) / 3.14);		break;
	case  GLFW_KEY_K:	scale(.75);		break;
	case  GLFW_KEY_L:	scale(1.25);		break;
	default:	glfwSetKeyCallback(window, key_callback); break;
	}
}

void cursorCall(GLFWwindow* window, double xpos, double ypos) {
	// Get the size of the window
	int width, height;
	glfwGetWindowSize(window, &width, &height);

	// Convert screen position to world coordinates
	Eigen::Vector4f p_screen(xpos, height - 1 - ypos, 0, 1);
	Eigen::Vector4f p_canonical((p_screen[0] / width) * 2 - 1, (p_screen[1] / height) * 2 - 1, 0, 1);
	Eigen::Vector4f p_world = view.inverse()*p_canonical;
	double xmove = p_world[0];
	double ymove = p_world[1]; // NOTE: y axis is flipped in glfw

	//move object with cursor
	V(0, closestVIndex) += (xmove - center_x);			V(1, closestVIndex) += (ymove - center_y);
	V(0, closestVIndex + 1) += (xmove - center_x);		V(1, closestVIndex + 1) += (ymove - center_y);
	V(0, closestVIndex + 2) += (xmove - center_x);		V(1, closestVIndex + 2) += (ymove - center_y);

	center_x = xmove;
	center_y = ymove;

	VBO.update(V);
}
void translate_O(GLFWwindow* window, int button, int action, int mods) {
	// Get the position of the mouse in the window
	double xpos, ypos;
	glfwGetCursorPos(window, &xpos, &ypos);

	// Get the size of the window
	int width, height;
	glfwGetWindowSize(window, &width, &height);

	// Convert screen position to world coordinates
	Eigen::Vector4f p_screen(xpos, height - 1 - ypos, 0, 1);
	Eigen::Vector4f p_canonical((p_screen[0] / width) * 2 - 1, (p_screen[1] / height) * 2 - 1, 0, 1);
	Eigen::Vector4f p_world = view.inverse()*p_canonical;
	double xworld = p_world[0];
	double yworld = p_world[1]; // NOTE: y axis is flipped in glfw

	// Update the position of selected triangle
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
		if (selectedTri(xworld, yworld, closestVIndex)) {
			center_x = xworld;
			center_y = yworld;
			glfwSetCursorPosCallback(window, cursorCall);
		}
	}

	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
			glfwSetCursorPosCallback(window, 0);
	}

	triangleCenter(center_x, center_y, closestVIndex);
	glfwSetKeyCallback(window, rotate_scale);
}

//deletes triangle closest to selected point
void delete_P(GLFWwindow* window, int button, int action, int mods) {
	// Get the position of the mouse in the window
	double xpos, ypos;
	glfwGetCursorPos(window, &xpos, &ypos);

	// Get the size of the window
	int width, height;
	glfwGetWindowSize(window, &width, &height);

	// Convert screen position to world coordinates
	Eigen::Vector4f p_screen(xpos, height - 1 - ypos, 0, 1);
	Eigen::Vector4f p_canonical((p_screen[0] / width) * 2 - 1, (p_screen[1] / height) * 2 - 1, 0, 1);
	Eigen::Vector4f p_world = view.inverse()*p_canonical;
	double xworld = p_world[0];
	double yworld = p_world[1]; // NOTE: y axis is flipped in glfw

	// Find triangle
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
		int index = 0;
		if (selectedTri(xworld, yworld,index)) {
			//copy over elements minus deleted triangle
			V_tmp.resize(2, V_Size - 3);
			C_tmp.resize(3, V_Size - 3);
			int cnt = 0;
			for (int i = 0; i < V_Size; ++i) {
				if (i == index)	i += 2;
				else
				{
					V_tmp.col(cnt) = V.col(i);
					C_tmp.col(cnt) = C.col(i);
					++cnt;
				}
			}

			//resize and copy over
			V_Size = V_Size - 3;
			V.resize(2, V_Size);
			C.resize(3, V_Size);
			for (int i = 0; i < V_Size; ++i) {
				V.col(i) = V_tmp.col(i);
				C.col(i) = C_tmp.col(i);
			}

			// Upload the change to the GPU
			VBO.update(V);
			VBO_C.update(C);
		}
	}
}

//select a color for vertex
void color_C(GLFWwindow* window, int key, int scancode, int action, int mods) {
	switch (key) {
	case  GLFW_KEY_1:	C.col(closestVIndex) <<  1.0, 0.0, 0.0; break;
	case  GLFW_KEY_2:	C.col(closestVIndex) <<	 0.0, 1.0, 0.0; break;
	case  GLFW_KEY_3:	C.col(closestVIndex) <<  0.0, 0.0, 1.0; break;
	case  GLFW_KEY_4:	C.col(closestVIndex) <<  1.0, 1.0, 0.0; break;
	case  GLFW_KEY_5:	C.col(closestVIndex) <<  0.0, 1.0, 1.0; break;
	case  GLFW_KEY_6:	C.col(closestVIndex) <<  1.0, 0.0, 1.0; break;
	case  GLFW_KEY_7:	C.col(closestVIndex) <<  0.3, 0.0, 0.3; break;
	case  GLFW_KEY_8:	C.col(closestVIndex) <<  0.0, 0.3, 0.3; break;
	case  GLFW_KEY_9:	C.col(closestVIndex) <<  0.0, 0.0, 0.0; break;
	default:	break;
	}
	// Upload the change to the GPU
	VBO_C.update(C);
	glfwSetKeyCallback(window, key_callback);
}

//find closest vertex to color
void color_pick(GLFWwindow* window, int button, int action, int mods) {
	// Get the position of the mouse in the window
	double xpos, ypos;
	glfwGetCursorPos(window, &xpos, &ypos);

	// Get the size of the window
	int width, height;
	glfwGetWindowSize(window, &width, &height);

	// Convert screen position to world coordinates
	// Convert screen position to world coordinates
	Eigen::Vector4f p_screen(xpos, height - 1 - ypos, 0, 1);
	Eigen::Vector4f p_canonical((p_screen[0] / width) * 2 - 1, (p_screen[1] / height) * 2 - 1, 0, 1);
	Eigen::Vector4f p_world = view.inverse()*p_canonical;
	double xworld = p_world[0];
	double yworld = p_world[1]; // NOTE: y axis is flipped in glfw

	// Update the position of the first vertex if the left button is pressed
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
		closestVIndex = closestVertex(xworld, yworld);
		glfwSetKeyCallback(window, color_C);
	}
}

void mouse_button_view(GLFWwindow* window, int button, int action, int mods)
{
	// Get the position of the mouse in the window
	double xpos, ypos;
	glfwGetCursorPos(window, &xpos, &ypos);

	// Get the size of the window
	int width, height;
	glfwGetWindowSize(window, &width, &height);

	// Convert screen position to world coordinates
	Eigen::Vector4f p_screen(xpos, height - 1 - ypos, 0, 1);
	Eigen::Vector4f p_canonical((p_screen[0] / width) * 2 - 1, (p_screen[1] / height) * 2 - 1, 0, 1);
	Eigen::Vector4f p_world = view.inverse()*p_canonical;

	// Update the position of the first vertex if the left button is pressed
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
		V.col(0) << p_world[0], p_world[1];

	// Upload the change to the GPU
	VBO.update(V);
}

/*void viewZoom(char z) {
	if (z == 'a') {

	}
	if (z == 'm') {

	}
	// Upload the change to the GPU
	VBO.update(V);
}

void viewMove(GLFWwindow* window, char m) {
	// Get the size of the window
	int width, height;
	glfwGetWindowSize(window, &width, &height);

	// position
	Vector3d position (vX, vY, 0);

	// Compute new orientation
	float horizontalAngle = width * .2 * float(1024 / 2 - vX);
	float verticalAngle = height * .2 * float(768 / 2 - vY);

	// Direction : Spherical coordinates to Cartesian coordinates conversion
	Vector3d direction(
		cos(verticalAngle) * sin(horizontalAngle),
		sin(verticalAngle),
		cos(verticalAngle) * cos(horizontalAngle)
	);

	// Right vector
	Vector3d right (
		sin(horizontalAngle - 3.14f / 2.0f),
		0,
		cos(horizontalAngle - 3.14f / 2.0f)
	);

	if (m == 'w') {
		position -= direction;
	}
	if (m == 'a') {
		position += right;
	}
	if (m == 's') {
		position += direction;
	}
	if (m == 'd') {
		position -= right;
	}

	// Upload the change to the GPU
	VBO.update(V);
}*/

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
	glfwSetMouseButtonCallback(window, 0);

	// Get the position of the mouse in the window
	double xpos, ypos;
	glfwGetCursorPos(window, &xpos, &ypos);

	// Get the size of the window
	int width, height;
	glfwGetWindowSize(window, &width, &height);

	// Convert screen position to world coordinates
	Eigen::Vector4f p_screen(xpos, height - 1 - ypos, 0, 1);
	Eigen::Vector4f p_canonical((p_screen[0] / width) * 2 - 1, (p_screen[1] / height) * 2 - 1, 0, 1);
	Eigen::Vector4f p_world = view.inverse()*p_canonical;

	switch (key) {
	//soup editor	i=insert	o=translation	p=delete
	case  GLFW_KEY_I:	glfwSetMouseButtonCallback(window, insert_I);		break;
	case  GLFW_KEY_O:	glfwSetMouseButtonCallback(window, translate_O);	break;
	case  GLFW_KEY_P:	glfwSetMouseButtonCallback(window, delete_P);		break;
	//colors	enabled by pressing c
	case GLFW_KEY_C:	glfwSetMouseButtonCallback(window, color_pick);		break;
	//view control	+=zoom in	-=zoom out	w=down	a=right	s=up	d=left
/*		case  GLFW_KEY_KP_ADD:	viewZoom('a');	break;
        case  GLFW_KEY_MINUS:	viewZoom('m');	break;*/
        case  GLFW_KEY_W:		view(3, 3) = p_world[1] * -.2;	break;
		case  GLFW_KEY_A:		view(2, 2) = p_world[0] * -.2;	break;
		case  GLFW_KEY_S:		view(3, 3) = p_world[1] * .2;	break;
		case  GLFW_KEY_D:		view(2,2) = p_world[0]*.2;	break;
        default:	glfwSetMouseButtonCallback(window, 0); break;
    }
}

int main(void) {
    GLFWwindow* window;

    // Initialize the library
    if (!glfwInit())
        return -1;

    // Activate supersampling
    glfwWindowHint(GLFW_SAMPLES, 8);

    // Ensure that we get at least a 3.2 context
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);

    // On apple we have to load a core profile with forward compatibility
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // Create a windowed mode window and its OpenGL context
    window = glfwCreateWindow(1900, 1450, "Hello World", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }

    // Make the window's context current
    glfwMakeContextCurrent(window);

    #ifndef __APPLE__
      glewExperimental = true;
      GLenum err = glewInit();
      if(GLEW_OK != err) {
        /* Problem: glewInit failed, something is seriously wrong. */
       fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
      }
      glGetError(); // pull and savely ignonre unhandled errors like GL_INVALID_ENUM
      fprintf(stdout, "Status: Using GLEW %s\n", glewGetString(GLEW_VERSION));
    #endif

    int major, minor, rev;
    major = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MAJOR);
    minor = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MINOR);
    rev = glfwGetWindowAttrib(window, GLFW_CONTEXT_REVISION);
    printf("OpenGL version recieved: %d.%d.%d\n", major, minor, rev);
    printf("Supported OpenGL is %s\n", (const char*)glGetString(GL_VERSION));
    printf("Supported GLSL is %s\n", (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

    // Initialize the VAO
    // A Vertex Array Object (or VAO) is an object that describes how the vertex
    // attributes are stored in a Vertex Buffer Object (or VBO). This means that
    // the VAO is not the actual object storing the vertex data,
    // but the descriptor of the vertex data.
    VertexArrayObject VAO;
    VAO.init();
    VAO.bind();

    // Initialize the VBO with the vertices data
    // A VBO is a data container that lives in the GPU memory
    VBO.init();
	VBO.update(V);

	VBO_C.init();
	VBO_C.update(C);

    // Initialize the OpenGL Program
    // A program controls the OpenGL pipeline and it must contains
    // at least a vertex shader and a fragment shader to be valid
    Program program;
	//indivinual vertex colored
    const GLchar* vertex_shader =
            "#version 150 core\n"
                    "in vec2 position;"
					"in vec3 color;"
					"out vec3 Color;"
                    "void main()"
                    "{"
                    "    gl_Position = vec4(position, 0.0, 1.0);"
					"	 Color = color;"
                    "}";
    const GLchar* fragment_shader =
            "#version 150 core\n"
					"in vec3 Color;"
                    "out vec4 outColor;"
                    "void main()"
                    "{"
                    "    outColor = vec4(Color, 1.0);"
                    "}";

    // Compile the two shaders and upload the binary to the GPU
    // Note that we have to explicitly specify that the output "slot" called outColor
    // is the one that we want in the fragment buffer (and thus on screen)
    program.init(vertex_shader,fragment_shader,"outColor");
    program.bind();


    // The vertex shader wants the position of the vertices as an input.
    // The following line connects the VBO we defined above with the position "slot"
    // in the vertex shader
    program.bindVertexAttribArray("position",VBO);
	program.bindVertexAttribArray("color", VBO_C);

    // Save the current time --- it will be used to dynamically change the triangle color
    auto t_start = std::chrono::high_resolution_clock::now();

    // Register the keyboard callback
    glfwSetKeyCallback(window, key_callback);

	// Get size of the window
	int width, height;
	glfwGetWindowSize(window, &width, &height);
	float aspect_ratio = float(height) / float(width); // corresponds to the necessary width scaling

	view <<
		aspect_ratio, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

    // Loop until the user closes the window
    while (!glfwWindowShouldClose(window)) {
        // Bind your VAO (not necessary if you have only one)
        VAO.bind();

        // Bind your program
        program.bind();

        // Set the uniform value depending on the time difference
        auto t_now = std::chrono::high_resolution_clock::now();
        float time = std::chrono::duration_cast<std::chrono::duration<float>>(t_now - t_start).count();

		// Get size of the window
		int width, height;
		glfwGetWindowSize(window, &width, &height);
		float aspect_ratio = float(height) / float(width); // corresponds to the necessary width scaling
		view(0,0) = aspect_ratio;

		//glUniformMatrix4fv(program.uniform("view"), 1, GL_FALSE, view.data());

        // Clear the framebuffer
        glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // Draw triangles		little glitchy but can draw multiple triangles
		if (V_Size / 3 != 0) {	//triangles
			for(int i=0; i<V_Size-2; i=i+3)
				glDrawArrays(GL_TRIANGLES, i, i+3);
		}
		if (V_Size % 3 == 2) {	//line
			glDrawArrays(GL_LINE_STRIP, V_Size-1, 2);
		}
		else if (V_Size % 3 == 1) {	//point
			glDrawArrays(GL_POINTS, V_Size, 1);
		}
		else {}

		//part_5
		if (glfwGetKey(window,GLFW_KEY_RIGHT) == GLFW_PRESS) {
			// Clear the framebuffer
			glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT);

			V.resize(2, 3);
			C.resize(3, 3);
			V_Size = 0;
			V << 0, 0.5, -0.5, 0.5, -0.5, -0.5;
			C << 0, 0, 0, 0, 0, 0, 0, 0, 0;

			glDrawArrays(GL_TRIANGLES, 0, 3);
			rotate((float)(sin(time * 4.0f) + 1.0f) / 2.0f);
		}

        // Swap front and back buffers
        glfwSwapBuffers(window);

        // Poll for and process events
        glfwPollEvents();
    }

    // Deallocate opengl memory
    program.free();
    VAO.free();
    VBO.free();
	VBO_C.free();

    // Deallocate glfw internals
    glfwTerminate();
    return 0;
}