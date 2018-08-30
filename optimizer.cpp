#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <sstream>  
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <Eigen/Eigen>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

#include <common/objloader.hpp>

// Convert glm vec3 to eigen vector3f
Eigen::Vector3f ConvertGlmToEigen3f( glm::vec3 v ) {
    
    Eigen::Vector3f out_v;
    
    out_v[0] = v.x;
    out_v[1] = v.y;
    out_v[2] = v.z;

    return out_v;

}

// Convert glm mat4 to eigen matrix4f
Eigen::Matrix4f ConvertGlmToEigenMat4f ( glm::mat4 mat ) {
    
    Eigen::Matrix4f out_mat;

    for ( int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
           out_mat(i,j) = mat[j][i];  // In glm::mat4, the first index indicating column and the second indicating row, mat[column][row]. 
    	}                             // In Eigen::Matrix, mat(row, column). 
    }

    return out_mat;

}

// Dehomogeneous the vector from 4 element down to 3.
Eigen::Vector3f pi4to3f ( Eigen::Vector4f v ) {
    
    Eigen::Vector3f out_v;

    out_v[0] = v[0]/v[3];
    out_v[1] = v[1]/v[3];
    out_v[2] = v[2]/v[3];

    return out_v; 

}

// Dehomogeneous the vector from 3 element down to 2.
Eigen::Vector2f pi3to2f ( Eigen::Vector3f v ) {
    
    Eigen::Vector2f out_v;

    out_v[0] = v[0]/v[2];
    out_v[1] = v[1]/v[2];

    return out_v; 

}

// convert a 3D vector to homogeneous vector. 
Eigen::Vector4f pi3to4f ( Eigen::Vector3f v ) {
    
    Eigen::Vector4f out_v;

    out_v[0] = v[0];
    out_v[1] = v[1];
    out_v[2] = v[2];
    out_v[3] = 1;

    return out_v; 

}

// Convert float tp integer based on rounding up if decimals > 0.5 and rounding down if decimals < 0.5
int FloatRoundToInt(float a){
    
    int a_integer = static_cast<int>(a);

    if (a>0) {   	
        if ( a - a_integer < 0.5){

    	    return static_cast<int> (a);

        }else{

    	    return static_cast<int> (a+1);

    	}
    }else{ 
    	if ( a - a_integer < -0.5){

    	    return static_cast<int> (a-1);

        }else{

    	    return static_cast<int> (a);

        }
    }

}

// Search the 5*5 neighbour of the selected pixel (x,y) to see if any of its neighbours is background, return true if it have background in its neighbour. 
// Quite stupid, could find a better way. 
bool SearchNeighbour (cv::Mat image, int x, int y ) {
 
    int count = 0;

	for (int i = -2; i < 3; i++) {
		for (int j = -2; j < 3; j++) {            
            // in opencv index of mat is as (row, column), in another words (y,x)
			if ( image.at<cv::Vec3b>(y+i,x+j)[0] < 50 && 0 <= x+j && x+j < image.cols && 0 <= y+i && y+i < image.rows){ 

				count++;

			}
		}
	}

	if ( count > 0 ){	

		return true;

	}else{

		return false;

	} 

}

// Draw a point on the image at coordinate (x,y)
void DrawPoint (cv::Mat image, int x , int y ){

	cv::Vec3b Red (0, 0, 255);

    for (int i = -1; i < 2; i++) {
		for (int j = -1; j < 2; j++) {            
            if (0 <= x+j && x+j < image.cols && 0 <= y+i && y+i < image.rows){

			image.at<cv::Vec3b>( y+i, x+j ) = Red;

            }
		}
	}

}

// project a 3D point to the image and check whether it is on the silhouette  
bool OnSilhouette (cv::Mat image, Eigen::Vector4f point, Eigen::Matrix4f perspective, Eigen::Matrix4f camera_pose, Eigen::Matrix4f model_pose){
    
    Eigen::Vector4f projection = perspective * camera_pose * model_pose * point;
    Eigen::Vector3f dehomo_projection = pi4to3f (projection);
    

    int image_x = FloatRoundToInt(dehomo_projection[0]);
    int image_y = FloatRoundToInt(dehomo_projection[1]);

    bool result = SearchNeighbour(image, image_x, image_y);

    return result;

}

int main(int argc, char const *argv[])
{
    cv::Mat image;

    std::string path = "/home/xinghui/Find-Silhouette/image.png";

    image = cv::imread( path.c_str() );

    glm::mat4 perspective = glm::perspective(glm::radians(10.0f), 4.0f / 3.0f, 0.1f, 300000.0f);
    glm::mat4 Camera_pose       = glm::lookAt(
								glm::vec3(0,0,0), // Camera is at (0,0,0), in World Space
								glm::vec3(0,0,1), // and looks at the negative direction of the z axis
								glm::vec3(0,-1,0)  // Vertical direction is the positive direction
						   );
    float object_pose[16] = {
		0.0f, 1.0f, 0.0f, 0.0f,
        -1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 200000.0f,
        0.0f, 0.0f, 0.0f, 1.0f
	};
	glm::mat4 ModelT = glm::make_mat4(object_pose);
    glm::mat4 Model = glm::transpose(ModelT);

    glm::mat4 MVP = perspective * Camera_pose * Model;

    Eigen::Matrix4f Eigen_perspective = ConvertGlmToEigenMat4f(perspective);
    Eigen::Matrix4f Eigen_Camera_pose = ConvertGlmToEigenMat4f(Camera_pose);
    Eigen::Matrix4f Eigen_Model = ConvertGlmToEigenMat4f(Model);

    Eigen::Matrix4f transform = Eigen_perspective * Eigen_Camera_pose * Eigen_Model;

    std::cout << "perspective" << std::endl;
    std::cout << Eigen_perspective << std::endl;
    std::cout << "Camera_pose" << std::endl;
    std::cout << Eigen_Camera_pose << std::endl;
    std::cout << "Model" << std::endl; 
    std::cout << Eigen_Model << std::endl;
    std::cout << "transform" << std::endl;
    std::cout << transform << std::endl;
    std::cout << "MVP" << std::endl;
    std::cout << ConvertGlmToEigenMat4f(MVP) << std::endl; 

    std::cout << "Camera*Model" << std::endl; 
    std::cout << Eigen_Camera_pose * Eigen_Model << std::endl;

    // Eigen::Vecto3f vertex (-299.761108, -1790.010132, -1401.049927);

    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> VertexMember;
    std::string Model_Path;
    Model_Path = "/home/xinghui/Find-Silhouette/satellite_model.obj";
    bool res = loadOBJ( Model_Path.c_str(), vertices, VertexMember);

    std::vector<Eigen::Vector3f> v;

    for (int i=0; i < VertexMember.size(); i++){
        
        Eigen::Vector3f v_temp = ConvertGlmToEigen3f(VertexMember[i]);
        v.push_back(v_temp);
    }

    Eigen::Affine3f M;
    M.matrix() = Eigen_Camera_pose * Eigen_Model;

    Eigen::Vector3f Tw = M.translation();
    Eigen::Matrix3f Rw = M.linear();
    for (int r = 1; r < 3; r++){
        for    (int c = 0; c < 3; c++){
            Rw(r,c) = -Rw(r,c);
        }
        Tw(r) = -Tw(r);
    }

    Eigen::Matrix3f K;

    K << 2743, 0, 320,0, 2743, 240, 0, 0, 1;
    std::cout << K << std::endl;
    
    for (int i=0; i < v.size(); i++){

        Eigen::Vector3f vertex = v[i];
        Eigen::Vector3f x_hat = K*(Rw*vertex+Tw);
    
    
        Eigen::Vector2f x = pi3to2f(x_hat);
        int image_x = FloatRoundToInt(x[0]);
        int image_y = FloatRoundToInt(x[1]);

        // std::cout << x << std::endl;  
        DrawPoint(image, image_x, image_y );
    }
    cv::imshow(" ", image);
    cvWaitKey(0);

    // std::vector<glm::vec3> vertices;
    // std::vector<glm::vec3> VertexMember;
    // std::string Model_Path;
    // Model_Path = "/home/xinghui/Find-Silhouette/satellite_model.obj";
    // bool res = loadOBJ( Model_Path.c_str(), vertices, VertexMember);

    // std::vector<Eigen::Vector3f> v;
    // for (int i=0; i < VertexMember.size(); i++){
        
    //     Eigen::Vector3f v_temp = ConvertGlmToEigen3f(VertexMember[i]);
    //     v.push_back(v_temp);
    // }

    // for (int i=0; i < v.size(); i++){

    // Eigen::Vector4f vertex4f = pi3to4f(v[i]);

    // Eigen::Vector4f projection = transform * vertex4f;
    // Eigen::Vector3f dehomo_projection = pi4to3f (projection);

    // int image_x = FloatRoundToInt(dehomo_projection[0]);
    // int image_y = FloatRoundToInt(dehomo_projection[1]);
    
    // DrawPoint(image, image_x, image_y ); 
    // }

    // Eigen::Vector4f vertex4f = pi3to4f(v[0]);
    // Eigen::Vector4f projection = transform * vertex4f;
    // std::cout << projection << std::endl;
    
    // DrawPoint(image, 200, 300 );
    // cv::imshow(" ", image);
    // cvWaitKey(0);
    


	return 0;
}