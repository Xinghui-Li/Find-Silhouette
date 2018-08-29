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

           out_mat(i,j) = mat[j][i];  // In glm::mat4, the first index indicating column and the second indicating row. 

    	}

    }

    return out_mat;
}

// Dehomogeneous the vector from 4 element down to 3.
Eigen::Vector3f pi4to3 ( Eigen::Vector4f v ) {
    
    Eigen::Vector3f out_v;

    out_v[0] = v[0]/v[3];
    out_v[1] = v[1]/v[3];
    out_v[2] = v[2]/v[3];

    return out_v; 
}

// Dehomogeneous the vector from 3 element down to 2.
Eigen::Vector2f pi3to2 ( Eigen::Vector3f v ) {
    
    Eigen::Vector2f out_v;

    out_v[0] = v[0]/v[2];
    out_v[1] = v[1]/v[2];

    return out_v; 
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

    for (int i = -2; i < 3; i++) {

		for (int j = -2; j < 3; j++) {
            
            if (0 <= x+j && x+j < image.cols && 0 <= y+i && y+i < image.rows){

			image.at<cv::Vec3b>( y+i, x+j ) = Red;

            }
		}
	}

}

// project a 3D point to the image and check whether it is on the silhouette  
// bool OnSilhouette (cv::Mat image, Eigen::Vector4f point, Eigen::Matrix4f perspective, Eigen::Matrix4f camera_pose, Eigen::Matrix4f model_pose){
    
//     Eigen::Vector4f = perspective * camera_pose * model_pose * 

// }

int main(int argc, char const *argv[])
{
    cv::Mat image;

    std::string path = "/home/xinghui/Find-Silhouette/image.png";

    image = cv::imread( path.c_str() );

    Eigen::Vector2i point (320,240);
    
    bool result = SearchNeighbour(image, point[0], point[1]);

    std::cout << " the pixel value at " << "( " << point[0] << "," << point[1] << " )" << " is " << int(image.at<cv::Vec3b>( point[1], point[0] )[0]) << std::endl;
    
    DrawPoint(image, point[0], point[1] );
    DrawPoint(image, 0, 0);
    cv::imshow(" ", image);
    cvWaitKey(0);

    std::cout << result << std::endl;

	return 0;
}