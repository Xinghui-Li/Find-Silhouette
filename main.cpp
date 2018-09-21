// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <sstream>  
#include <fstream>
#include <iomanip>

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

// void process(){

//     Matrix3d K; K << 2743.21, 0, 320,
//                      0, 2743.21, 240, 
//                      0, 0, 1;
    
//     Matrix4d camera_pose; camera_pose << 1,0,0,0,
//                                          0,1,0,0,
//                                          0,0,1,0,
//                                          0,0,0,1;



//     optimizer opt(picked_vertices, K, E_Camera_pose, E_Model, distmap);


// }

int main( int argc, char *argv[] )
{
    //--------------------------- input section ---------------------------

    string original_path = "/home/xinghui/Find-Silhouette/0001.png";
    string noise_path = "/home/xinghui/Find-Silhouette/noise.png";

    // Intrinsic matrix : 10° Field of View, 4:3 ratio, display range : 0.1 unit <-> 300000 units
    glm::mat4 perspective = glm::perspective(glm::radians(10.0f), 4.0f / 3.0f, 0.1f, 300000.0f);
    // This is the camera intrinsic matrix for using opencv projection
    Eigen::Matrix3d K; K << 2743.21, 0, 320,0, 2743.21, 240, 0, 0, 1;
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
        0.0f, -1.0f, 0.0f, -1500.0f,
        1.0f, 0.0f, 0.0f, 1500.0f,
        0.0f, 0.0f, 1.0f, -203000.0f,
        0.0f, 0.0f, 0.0f, 1.0f
        };
    
    Matrix4d object_pose2; object_pose2 << 0,-1,0,-1000,1,0,0,1000,0,0,1,-203000,0,0,0,1;
    Vector3d disturb; disturb << 0.0, 0.0, 0.0;
    Sophus::SO3d dis = Sophus::SO3d::exp(disturb);

    object_pose2.block<3,3>(0,0) = dis.matrix() * object_pose2.block<3,3>(0,0);

    // glm::mat4 ModelT = glm::make_mat4(object_pose2);
    glm::mat4 Model = ConvertEigenMat4fToGlm(object_pose2);
    Matrix4d E_perspective = ConvertGlmToEigenMat4f(perspective);
    Matrix4d E_Camera_pose = ConvertGlmToEigenMat4f(Camera_pose);
    cout << "E_Camera_pose \n" << E_Camera_pose << endl;

    Matrix4d E_Model = ConvertGlmToEigenMat4f(Model);

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

    // select the corners of the solar panels
    vector<triangle> triangles;

    for (int i = 0; i < vertices.size(); i += 3 ){

        triangle tri;
        tri.vertex1 = ConvertGlmToEigen3f(vertices[i]);
        tri.vertex2 = ConvertGlmToEigen3f(vertices[i+1]);
        tri.vertex3 = ConvertGlmToEigen3f(vertices[i+2]);

        triangles.push_back(tri);

    }

    vector<double> area;

    for (int i = 0; i < triangles.size(); i++){

        double A = Area(triangles[i]);
        area.push_back(A);
    }

    vector<int> sorted_index;
    sorted_index = indexSort(area);

    vector<Vector3d> picked_vertices;

    for (int i = 0; i < 8; i++){

        int number = sorted_index.size()-1-i;
        int index = sorted_index[number];
        
        picked_vertices.push_back(triangles[index].vertex1);
        picked_vertices.push_back(triangles[index].vertex2);
        picked_vertices.push_back(triangles[index].vertex3);
        
    }

    // Try to interpolate more point on the main skeleton of the satellite, the body and the solar panel

    std::vector<Vector3d> vert;
    std::vector< vector<int> > face;

    LoadModelQuad("/home/xinghui/Find-Silhouette/simple_sat.obj", vert, face);

    std::vector<edge> edges;

    // cout << 4 % 4 << endl;

    for (int i = 0; i < face.size(); i++){
    
        for ( int j = 0; j < 4; j++){

            int index1 = j % 4;
            int index2 = (j+1) % 4;
            
            edge ed;

            ed.vertex1 = vert[face[i][index1]-1];
            ed.vertex2 = vert[face[i][index2]-1];
            ed.index1 = face[i][index1];
            ed.index2 = face[i][index2];


            if (edges.size() == 0){

                edges.push_back(ed);

            }else{

                int count = 0;

                for ( int k = 0; k < edges.size(); k++){

                    if ( c_edge(ed, edges[k]) ){

                        count ++;

                    }

                }

                if (count == 0){

                    edges.push_back(ed);
                }
            }

        }
    }

    std::vector<glm::vec3> extra;

    for (int i = 0; i < edges.size(); i++){

        Vector3d a = edges[i].vertex1;
        Vector3d b = edges[i].vertex2;
        double len = (a-b).norm();
        Vector3d n = (b-a)/len;
        double d = 400;

        Vector3d current = a;
        double res = (b-current).norm();
        
        while( res > d ){

            Vector3d New = current + d*n;
            extra.push_back(ConvertEigen3dToGlm(New));
            current = New;
            res = (b-current).norm();

        }

    }


    cout << "The size of extra is " << extra.size() << endl;

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

    // imwrite("tmp.exr", distmap);
    // cout << distmap.rowRange(30, 40).colRange(40, 50) << endl;

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
    for (int i = 0; i < 200; i++){
        
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
        glReadPixels(0 ,0 ,width ,height ,GL_RGB ,GL_UNSIGNED_BYTE, image.data);
        cv::flip(image, image, 0);
        // cv::imwrite("/home/xinghui/Find-Silhouette/silhouette.png", image);

        // Mat depth(height, width, CV_32FC1);
        // glReadPixels(0,0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, depth.data);
        // flip(depth, depth, 0);

        std::vector<glm::vec3> all_point;
        all_point.reserve(VertexMember.size()+extra.size());
        all_point.insert(all_point.end(), VertexMember.begin(), VertexMember.end());
        all_point.insert(all_point.end(), extra.begin(), extra.end());
        vector<Vector3d> v_silhouette3d = SelectSilhouettePoint (image, E_perspective, E_Camera_pose, E_Model, all_point);

        Mat temp, temp2;
        cvtColor(dist8u1, temp, CV_GRAY2RGB);
        // imwrite("dist.png", dist8u1);
        temp.copyTo(temp2);

        for (int j = 0; j < v_silhouette3d.size(); j++){

            Vector3d v = v_silhouette3d[j];

            Vector2i pixel = ProjectOnCVimage(width, height, E_perspective, E_Camera_pose, E_Model, v);
            DrawPoint(temp, pixel[0], pixel[1]);
        }

        imshow(" OpenGL ", temp);
        // imwrite("/home/xinghui/Find-Silhouette/projected.png", temp);

        optimizer opt(v_silhouette3d, K, E_Camera_pose, E_Model, distmap);
        opt.Draw(temp2);
        imshow(" OpenCV ", temp2);
        // Eigen::VectorXd dev = opt.desperate();

        // VectorXd current_p = pseudo_log(opt.GetT());
        // current_p = current_p - 1*dev;
        // Matrix4d current_T = pseudo_exp(current_p);
        // cout << "Current derivative is \n" << dev << " \n" << endl;

        MatrixXd delta = opt.GetDelta();
        Sophus::SE3d change = Sophus::SE3d::exp(-delta);
        Matrix4d current_T = change.matrix() * opt.GetT();



    
        // Matrix3d rot = opt.GetT().block<3,3>(0,0);
        // VectorXd trans = opt.GetT().block<3,1>(0,3);
        // Sophus::SO3f rotation(rot);

        // cout << "Current rotation matrix is \n" << rotation.matrix() << " \n" << endl;

        // VectorXd rot_delta = rotation.log(); 
        // VectorXd delta(6);
        // delta << trans, rot_delta;

        // cout << "Current Rodrigues vector is \n" << delta << " \n"<< endl;
        
        // VectorXd phi(6);
        // phi << 0.01,0.01,0.01,0.000001,0.000001,0.000001;

        // VectorXd product = phi.cwiseProduct(dev);
        // cout << "Current product of scaling vector and derivative is \n" << product << " \n" << endl;
        // delta = delta-product;

        // VectorXd after_trans = delta.head(3);
        // VectorXd after_rot = delta.tail(3);

        // MatrixXf new_model(4,4);
        // Sophus::SO3f r = Sophus::SO3f::exp(after_rot);
        // new_model.block<3,3>(0,0) = r.matrix();
        // new_model.block<3,1>(0,3) = after_trans;
        // MatrixXf last_row(1,4);
        // last_row << 0,0,0,1;
        // new_model.block<1,4>(3,0) = last_row;


        // Sophus::SE3f after_transform = Sophus::SE3f::exp(delta);

        // MatrixXf new_model = after_transform.matrix();
        // cout << "New T is \n" << new_model << " \n"<< endl;



        // Eigen::MatrixXf deltaT = Sophus::SE3f::exp(dev).matrix();


 
        cout << "------------------------------------------------------------------" << endl;
        cout << "Total Energy is " << endl;
        cout <<  opt.GetE0().sum()  << endl;
        cout << "------------------------------------------------------------------" << endl;
        cout << "Pose is " << endl;
        cout << CVGLConversion(opt.GetT())  << endl;
        cout << "------------------------------------------------------------------" << endl;

    




        E_Model = CVGLConversion(current_T);
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