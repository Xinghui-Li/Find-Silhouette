#pragma once 
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

#include <sophus/se3.hpp>

using namespace Eigen;
using namespace std;
using namespace cv;

Eigen::Vector3f ConvertGlmToEigen3f( glm::vec3 v );
Eigen::Matrix4f ConvertGlmToEigenMat4f ( glm::mat4 mat );
glm::mat4 ConvertEigenMat4fToGlm (Eigen::Matrix4f mat);
Eigen::Vector3f pi4to3f ( Eigen::Vector4f v );
Eigen::Vector2f pi3to2f ( Eigen::Vector3f v );
Eigen::Vector4f pi3to4f ( Eigen::Vector3f v );
int FloatRoundToInt(float a);
bool SearchNeighbour (cv::Mat image, int x, int y );
void DrawPoint (cv::Mat image, int x , int y );
Eigen::Matrix4f CVGLConversion ( Eigen::Matrix4f mat );
Eigen::Vector2i ProjectOnCVimage (int width, int height, Eigen::Matrix4f perspective, Eigen::Matrix4f camera_pose, Eigen::Matrix4f model_pose, Eigen::Vector3f vertex);
vector<Vector3f> SelectSilhouettePoint (Mat image, Matrix4f perspective, Matrix4f camera_pose, Matrix4f model_pose, vector<glm::vec3> vertex);
Mat DistanceMap ( Mat original, Mat noise );
Eigen::MatrixXf dev_dist( Mat image, int image_x, int image_y);
Eigen::MatrixXf dev_pi3to2(float x, float y, float z);

class optimizer {

	public:
    
        optimizer(vector<Eigen::Vector3f> v_silhouette3d, Eigen::Matrix3f K, Eigen::Matrix4f camera_pose, Eigen::Matrix4f Model, Mat dist);

        Eigen::MatrixXf GetE0();

        Eigen::MatrixXf GetJ();

        Eigen::MatrixXf GetDelta();

        Eigen::Matrix4f GetT();

        Eigen::MatrixXf LMalgorithm(float lambda);

        Eigen::MatrixXf GetDev();



    
    private:

        vector<Eigen::Vector3f> v_silhouette3d;
        Eigen::Matrix3f K;
        Eigen::Matrix4f camera_pose;
        Eigen::Matrix4f Model;
        Eigen::Matrix4f T;
        Mat dist;
};

