// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <sstream>  
#include <fstream>

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
#include <tools.hpp>

using namespace std;
using namespace cv;
using namespace Eigen;

int main( int argc, char *argv[] )
{
    //--------------------------- input section ---------------------------

    string original_path = "/home/xinghui/Find-Silhouette/0001.png";
    string noise_path = "/home/xinghui/Find-Silhouette/noise.png";

    // Intrinsic matrix : 10° Field of View, 4:3 ratio, display range : 0.1 unit <-> 300000 units
    glm::mat4 perspective = glm::perspective(glm::radians(10.0f), 4.0f / 3.0f, 0.1f, 300000.0f);
    // This is the camera intrinsic matrix for using opencv projection
    Eigen::Matrix3f K; K << 2743.21, 0, 320,0, 2743.21, 240, 0, 0, 1;
    // Camera's pose
    glm::mat4 Camera_pose       = glm::lookAt(
                                glm::vec3(0,0,0), // Camera is at (0,0,0), in World Space
                                glm::vec3(0,0,-1), // and looks at the negative direction of the z axis
                                glm::vec3(0,1,0)  // Vertical direction is the positive direction
                           );
    // estimated nitial position of the satellite
    // float object_pose[16] = {
    //     0.0f, -1.0f, 0.0f, -1000.0f,
    //     1.0f, 0.0f, 0.0f, 1000.0f,
    //     0.0f, 0.0f, 1.0f, -203073.0f,
    //     0.0f, 0.0f, 0.0f, 1.0f
    //     };
    float object_pose[16] = {
        0.0f, -1.0f, 0.0f, -1800.0f,
        1.0f, 0.0f, 0.0f, 2000.0f,
        0.0f, 0.0f, 1.0f, -203000.0f,
        0.0f, 0.0f, 0.0f, 1.0f
        };

    glm::mat4 ModelT = glm::make_mat4(object_pose);
    glm::mat4 Model = glm::transpose(ModelT);
    Matrix4f E_perspective = ConvertGlmToEigenMat4f(perspective);
    Matrix4f E_Camera_pose = ConvertGlmToEigenMat4f(Camera_pose);
    cout << "E_Camera_pose" << E_Camera_pose << endl;

    Matrix4f E_Model = ConvertGlmToEigenMat4f(Model);

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

    int width = 640;
    int height = 480;

    // parameter used by LM algorithm
    float lambda = 1;
    float nu = 1.4;

    //---------------------------------- start of the algorithm ---------------------------------------
    
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
    window = glfwCreateWindow( width, height, "", NULL, NULL);
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

    // Black background
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

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glFramebufferTexture2D( GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texColorBuffer, 0 );

    Mat original = imread(original_path.c_str());
    Mat noise = imread(noise_path.c_str());

    Mat distmap = DistanceMap(original, noise);

    // string filename = "/home/xinghui/Find-Silhouette/distmap.csv";
    // ofstream myfile;
    // myfile.open(filename.c_str());
    // myfile<< cv::format(distmap, cv::Formatter::FMT_CSV) << std::endl;
    // myfile.close();

    Mat dist8u1;
    normalize(distmap, dist8u1, 0.0, 255, NORM_MINMAX, CV_8UC3);



   
    // cout << endl;

    // return 0;

    imwrite("/home/xinghui/Find-Silhouette/dist.png", distmap);
    // do{
    for (int i = 0; i < 100; i++){
        
        glm::mat4 glmT = Camera_pose * Model;
        glm::mat4 MVP = perspective * glmT;    
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
        cv::Mat image(height, width, CV_8UC3);
        // out = "/home/xinghui/Find-Silhouette/image.png";
        glReadPixels(0 ,0 ,width ,height ,GL_RGB ,GL_UNSIGNED_BYTE, image.data);
        cv::flip(image, image, 0);
        // cv::imwrite(out, image);

        vector<Vector3f> v_silhouette3d = SelectSilhouettePoint (image, E_perspective, E_Camera_pose, E_Model, VertexMember);

        Mat temp, temp2;
        cvtColor(dist8u1, temp, CV_GRAY2RGB);
        // imwrite("dist.png", dist8u1);
        temp.copyTo(temp2);

        for (int j = 0; j < v_silhouette3d.size(); j++){

            Vector3f v = v_silhouette3d[j];

            Vector2i pixel = ProjectOnCVimage(width, height, E_perspective, E_Camera_pose, E_Model, v);
            DrawPoint(temp, pixel[0], pixel[1]);
        }

        imshow(" OpenGL ", temp);
        // imwrite("/home/xinghui/Find-Silhouette/projected.png", temp);

        optimizer opt(v_silhouette3d, K, E_Camera_pose, E_Model, distmap);
        opt.Draw(temp2);
        imshow(" OpenCV ", temp2);
        Eigen::VectorXf dev = opt.GetDev().transpose();
        
        cout << "Current derivative is \n" << dev << " \n" << endl;

        Matrix3f rot = opt.GetT().block<3,3>(0,0);
        VectorXf trans = opt.GetT().block<3,1>(0,3);
        Sophus::SO3f rotation(rot);

        cout << "Current rotation matrix is \n" << rotation.matrix() << " \n" << endl;

        VectorXf rot_delta = rotation.log(); 
        VectorXf delta(6);
        delta << trans, rot_delta;

        cout << "Current Rodrigues vector is \n" << delta << " \n"<< endl;
        
        VectorXf phi(6);
        phi << 30,0,0,0,0,0;

        VectorXf product = phi.cwiseProduct(dev);
        cout << "Current product of scaling vector and derivative is \n" << product << " \n" << endl;
        delta = delta-product;

        VectorXf after_trans = delta.head(3);
        VectorXf after_rot = delta.tail(3);

        MatrixXf new_model(4,4);
        Sophus::SO3f r = Sophus::SO3f::exp(after_rot);
        new_model.block<3,3>(0,0) = r.matrix();
        new_model.block<3,1>(0,3) = after_trans;
        MatrixXf last_row(1,4);
        last_row << 0,0,0,1;
        new_model.block<1,4>(3,0) = last_row;


        // Sophus::SE3f after_transform = Sophus::SE3f::exp(delta);

        // MatrixXf new_model = after_transform.matrix();
        cout << "New T is \n" << new_model << " \n"<< endl;



        // Eigen::MatrixXf deltaT = Sophus::SE3f::exp(dev).matrix();


  //       cout << "------------------------------------------------------------------" << endl;
  //       // cout << " The " << i << " loop" << endl;
  //       cout << "------------------------------------------------------------------" << endl;
  //       cout << "E_Model is " << endl;
  //       cout << E_Model << endl;
  //       cout << "------------------------------------------------------------------" << endl;
  //       cout << "E_Camera_pose * E_Model is " << endl;
  //       cout << E_Camera_pose * E_Model << endl;
  //       cout << "------------------------------------------------------------------" << endl;
  //       cout << "T is " << endl;
  //       cout << CVGLConversion(E_Camera_pose * E_Model) << endl;
        // cout << "------------------------------------------------------------------" << endl;
  //       cout << "Jacobian is " << endl;
  //       cout << opt.GetJ() << endl;
        // cout << "------------------------------------------------------------------" << endl;
  //       cout << "delta is " << endl;
  //       cout << delta << endl;
  //       cout << "------------------------------------------------------------------" << endl;
  //       cout << "deltaT is " << endl;
  //       cout << deltaT << endl;
        cout << "------------------------------------------------------------------" << endl;
        cout << "Total Energy is " << endl;
        cout << opt.GetE0().sum()  << endl;
        cout << "------------------------------------------------------------------" << endl;
  //       cout << "------------------------------------------------------------------" << endl;

        // cout << v_silhouette3d[0] << endl;
        // Vector3f p = v_silhouette3d[0];
        // Vector3f x_hat = K * pi4to3f( CVGLConversion(E_Model) * pi3to4f(p));
        // Vector2f x = pi3to2f(x_hat);
        // int pixelx = FloatRoundToInt(x[0]);
        // int pixely = FloatRoundToInt(x[1]); 

        // cout << (float) distmap.at<uchar>(pixely, pixelx) << endl;
        // cout << (float) distmap.at<uchar>(pixely+1, pixelx) << endl;
        // cout << (float) distmap.at<uchar>(pixely-1, pixelx) << endl;
        // cout << (float) distmap.at<uchar>(pixely, pixelx+1) << endl;
        // cout << (float) distmap.at<uchar>(pixely, pixelx-1) << endl;
        // cout << "------------------------------------------------------------------" << endl;
        // cout <<  temp2.at<Vec3b>(pixely, pixelx) << endl;
        // cout <<  temp2.at<Vec3b>(pixely+1, pixelx) << endl;
        // cout <<  temp2.at<Vec3b>(pixely-1, pixelx) << endl;
        // cout <<  temp2.at<Vec3b>(pixely, pixelx+1) << endl;
        // cout <<  temp2.at<Vec3b>(pixely, pixelx-1) << endl;
        // cout << pixelx << " " << pixely << endl;
        // cout << temp2.channels() << endl;
        // DrawPoint(temp2, pixelx, pixely);

        // imshow("temp2", temp2);




        E_Model = CVGLConversion(new_model);
        Model = ConvertEigenMat4fToGlm(E_Model);


        cvWaitKey(0);
    }  
        // delete the framebuffer
        glDeleteFramebuffers(1, &frameBuffer);

        glDisableVertexAttribArray(0);

        // Swap buffers
        glfwSwapBuffers(window);
        glfwPollEvents();
    

    // } // Check if the ESC key was pressed or the window was closed
    // while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
    //        glfwWindowShouldClose(window) == 0 );

    // Cleanup VBO
    glDeleteBuffers(1, &vertexbuffer);
    glDeleteBuffers(1, &colorbuffer);
    glDeleteVertexArrays(1, &VertexArrayID);
    glDeleteProgram(programID);

    //-----------------------------------------End of the loop

    // Close OpenGL window and terminate GLFW
    glfwTerminate();

    return 0;
}