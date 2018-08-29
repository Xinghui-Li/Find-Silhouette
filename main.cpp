// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <sstream>  

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>
GLFWwindow* window;
 
// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
using namespace glm;

#include <common/shader.hpp>
#include <common/objloader.hpp>

int main( int argc, char *argv[] )
{
	
	// Initialise GLFW
	if( !glfwInit() )
	{
		fprintf( stderr, "Failed to initialize GLFW\n" );
		getchar();
		return -1;
	}

	glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);  // We want OpenGL 3.3
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // We don't want the old OpenGL
	// The GLFW_VISIBLE command cannot be used. if window is not pop out, no value in frame buffer.
	glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE); // Suppress the pop out of the window. 

	// Open a window and create its OpenGL context
	window = glfwCreateWindow( 640, 480, "", NULL, NULL);
	if( window == NULL ){
		fprintf( stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n" );
		getchar();
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);

	// Initialize GLEW
	glewExperimental = true; // Needed for core profile
	if (glewInit() != GLEW_OK) {
		fprintf(stderr, "Failed to initialize GLEW\n");
		getchar();
		glfwTerminate();
		return -1;
	}

	// Ensure we can capture the escape key being pressed below
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

	// Dark blue background
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

	// Enable depth test
	glEnable(GL_DEPTH_TEST);
	// Accept fragment if it closer to the camera than the former one
	glDepthFunc(GL_LESS); 

    // generate vertex array. 
	GLuint VertexArrayID;
	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);

	// Create and compile our GLSL program from the shaders
    std::string VertexShader_Path;
    std::string FragmentShader_Path;

    VertexShader_Path = "/home/xinghui/Find-Silhouette/SimpleVertexShader.vertexshader";
    FragmentShader_Path = "/home/xinghui/Find-Silhouette/SimpleFragmentShader.fragmentshader";
	GLuint programID = LoadShaders( VertexShader_Path.c_str(), FragmentShader_Path.c_str() );

	// Get a handle for our "MVP" uniform
	GLuint MatrixID = glGetUniformLocation(programID, "MVP");

	// Intrinsic matrix : 10° Field of View, 4:3 ratio, display range : 0.1 unit <-> 300000 units
	glm::mat4 Intrinsic = glm::perspective(glm::radians(10.0f), 4.0f / 3.0f, 0.1f, 300000.0f);
    // Or, for an ortho camera :
	//glm::mat4 Projection = glm::ortho(-10.0f,10.0f,-10.0f,10.0f,0.0f,100.0f); // In world coordinates
	
	// Camera's pose
	glm::mat4 Camera_pose       = glm::lookAt(
								glm::vec3(0,0,0), // Camera is at (0,0,0), in World Space
								glm::vec3(0,0,-1), // and looks at the negative direction of the z axis
								glm::vec3(0,1,0)  // Vertical direction is the positive direction
						   );

	// Model's pose 
    float roll, pitch, yaw;

	float object_pose[16] = {
		0.0f, -1.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, -200000.0f,
        0.0f, 0.0f, 0.0f, 1.0f
	};
	glm::mat4 ModelT = glm::make_mat4(object_pose);
    glm::mat4 Model = glm::transpose(ModelT);

	// Our ModelViewProjection : multiplication of our 3 matrices
	glm::mat4 MVP        = Intrinsic * Camera_pose * Model; // Remember, matrix multiplication is the other way around

    //Load the satellite model
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> VertexMember;
    std::string Model_Path;
    Model_Path = "/home/xinghui/Find-Silhouette/satellite_model.obj";
    bool res = loadOBJ( Model_Path.c_str(), vertices, VertexMember);
    std::cout << "The size of vertices is " << vertices.size() << std:: endl;
    std::cout << "The size of VertexMember is " << VertexMember.size() << std:: endl;

    // Here is to define the color of the silhouette image
    std::vector<glm::vec3> color (vertices.size(), glm::vec3(1.0f, 1.0f, 1.0f));
    std::cout << "The size of color is " << color.size() << std:: endl;
    

    // create a new frame buffer so the subsequent operations are regarding to the new buffer frame. 
    GLuint frameBuffer;
    glGenFramebuffers(1, &frameBuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, frameBuffer);

    // This will identify our vertex buffer
	GLuint vertexbuffer;
	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3)*vertices.size(), &vertices[0], GL_STATIC_DRAW);

    // create the color buffer
	GLuint colorbuffer;
	glGenBuffers(1, &colorbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3)*color.size(), &color[0], GL_STATIC_DRAW);
    
    // generate the texture attachment to the framebuffer (follow the openGL tutorial, not so sure why )
    GLuint texColorBuffer;
    glGenTextures(1, &texColorBuffer);
    glBindTexture(GL_TEXTURE_2D, texColorBuffer);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glFramebufferTexture2D( GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texColorBuffer, 0 );

	// do{

		// Clear the screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Use our shader
		glUseProgram(programID);

		glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

		// 1rst attribute buffer : vertices
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
		glVertexAttribPointer(
			0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
			3,                  // size
			GL_FLOAT,           // type
			GL_FALSE,           // normalized?
			0,                  // stride
			(void*)0            // array buffer offset
		);

		// 2nd attribute buffer : colors
		glEnableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
		glVertexAttribPointer(
			1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
			3,                                // size
			GL_FLOAT,                         // type
			GL_FALSE,                         // normalized?
			0,                                // stride
			(void*)0                          // array buffer offset
		);

		// Draw the model
		glDrawArrays(GL_TRIANGLES, 0, vertices.size()*3); // 3 indices starting at 0 -> 1 triangle
        
        // Save the frame as an 3-channel image
        int width, height;
        glfwGetWindowSize(window, &width, &height);
        cv::Mat image(height, width, CV_8UC3);
        std::string out;
        out = "/home/xinghui/Find-Silhouette/image.png";
        glReadPixels(0 ,0 ,width ,height ,GL_RGB ,GL_UNSIGNED_BYTE, image.data);
        cv::flip(image, image, 0);
        cv::imwrite(out, image);
        
        // delete the framebuffer
        glDeleteFramebuffers(1, &frameBuffer);

		glDisableVertexAttribArray(0);

		// Swap buffers
		glfwSwapBuffers(window);
		glfwPollEvents();

	// } // Check if the ESC key was pressed or the window was closed
	// while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
	// 	   glfwWindowShouldClose(window) == 0 );

    
	// Cleanup VBO
	glDeleteBuffers(1, &vertexbuffer);
	glDeleteBuffers(1, &colorbuffer);
	glDeleteVertexArrays(1, &VertexArrayID);
	glDeleteProgram(programID);

	// Close OpenGL window and terminate GLFW
	glfwTerminate();

	return 0;
}