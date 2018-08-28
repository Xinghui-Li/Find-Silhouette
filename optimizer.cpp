#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <sstream>  
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <Eigen/Eigen>

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

// Dehomogeneous the vector from 4 element down to 3
Eigen::Vector3f pi4to3 ( Eigen::Vector4f v ) {
    
    Eigen::Vector3f out_v;

    out_v[0] = v[0]/v[3];
    out_v[1] = v[1]/v[3];
    out_v[2] = v[2]/v[3];

    return out_v; 
}

int main(int argc, char const *argv[])
{
	// glm::mat4 Intrinsic = glm::perspective(glm::radians(10.0f), 640.0f / 480.0f, 0.5f, 10000.0f);
	// float object_pose[16] = {
	// 	0.0f, -1.0f, 0.0f, 0.0f,
 //        1.0f, 0.0f, 0.0f, 0.0f,
 //        0.0f, 0.0f, 1.0f, -200000.0f,
 //        0.0f, 0.0f, 0.0f, 1.0f
	// };
	// glm::mat4 ModelT = glm::make_mat4(object_pose);
 //    glm::mat4 Model = glm::transpose(ModelT);
 //    glm::mat4 Camera_pose       = glm::lookAt(
	// 							glm::vec3(0,0,0), // Camera is at (0,0,0), in World Space
	// 							glm::vec3(0,0,-1), // and looks at the negative direction of the z axis
	// 							glm::vec3(0,1,0)  // Vertical direction is the positive direction
	// 					   );
	// Eigen::Matrix4f Matrix = ConvertGlmToEigenMat4f(Intrinsic);
	// Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

	Eigen::Vector4f v (1.0f, 2.0f, 3.0f, 4.0f);
	Eigen::Vector3f out_v = pi4to3(v);
	std::cout << out_v << std::endl ;
	return 0;
}