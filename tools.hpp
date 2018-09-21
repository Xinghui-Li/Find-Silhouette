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

Eigen::Vector3d ConvertGlmToEigen3f( const glm::vec3& v );
Eigen::Matrix4d ConvertGlmToEigenMat4f ( const glm::mat4& mat );
glm::vec3 ConvertEigen3dToGlm (const Vector3d& v );
glm::mat4 ConvertEigenMat4fToGlm (const Eigen::Matrix4d& mat);
Eigen::Vector3d pi4to3f ( const Eigen::Vector4d& v );
Eigen::Vector2d pi3to2f ( const Eigen::Vector3d& v );
Eigen::Vector4d pi3to4f ( const Eigen::Vector3d& v );
Matrix4d pseudo_exp( const VectorXd& delta );
VectorXd pseudo_log( const Matrix4d& transform);
int FloatRoundToInt(float a);
bool SearchNeighbour (const cv::Mat& image, int x, int y );
void DrawPoint (cv::Mat image, int x , int y );
Eigen::Matrix4d CVGLConversion ( const Eigen::Matrix4d& mat );
Eigen::Vector2i ProjectOnCVimage (int width, int height, Eigen::Matrix4d perspective, Eigen::Matrix4d camera_pose, Eigen::Matrix4d model_pose, const Eigen::Vector3d& vertex);
vector<Vector3d> SelectSilhouettePoint (Mat image, Matrix4d perspective, Matrix4d camera_pose, Matrix4d model_pose, const vector<glm::vec3>& vertex);
Mat DistanceMap ( const Mat& original, const Mat& noise );
Eigen::MatrixXd dev_dist(const Mat& image, int image_x, int image_y);
Eigen::MatrixXd dev_dist_double( const Mat& image, double image_x, double image_y);
Eigen::MatrixXd dev_pi3to2(double x, double y, double z);

struct triangle{
    Vector3d vertex1;
    Vector3d vertex2;
    Vector3d vertex3; 
};

struct edge {

    Vector3d vertex1;
    Vector3d vertex2;

    int index1;
    int index2;

};
double Area(triangle tri);
vector<int> indexSort( vector<double> x );

void LoadModelQuad(std::string Filename, vector<Vector3d> & Vertices, vector< vector<int> >& face);
bool c_vertex( const Vector3d& a, const Vector3d& b);
bool c_edge( const edge& a, const edge& b);


class optimizer {

	public:
    
        optimizer(vector<Eigen::Vector3d> v_silhouette3d, Eigen::Matrix3d K, Eigen::Matrix4d camera_pose, Eigen::Matrix4d Model, Mat dist);

        Eigen::MatrixXd GetE0();

        Eigen::MatrixXd GetJ();

        Eigen::MatrixXd GetDelta();

        Eigen::Matrix4d GetT();

        Eigen::MatrixXd LMalgorithm(float lambda);

        Eigen::MatrixXd GetDev();

        void Draw( Mat image );

        VectorXd desperate();

        vector<Vector3d> GetPoint();

    
    private:

        vector<Eigen::Vector3d> v_silhouette3d;
        Eigen::Matrix3d K;
        Eigen::Matrix4d camera_pose;
        Eigen::Matrix4d Model;
        Eigen::Matrix4d T;
        Mat dist;
};

template< typename Tp >
Tp bilinearInterpolate(const Mat& img, Tp x, Tp y)
{
    if( x < Tp(0) || x > Tp(639) || y < Tp(0) || y > Tp(479) )
        return Tp(1.f/0.f);
    int px = (int)x; // floor of x
    int py = (int)y; // floor of y
    const int stride = img.cols;
    const float* data = (const float*) img.data;
    const float* p0 = data + px + py * stride; // pointer to first pixel

    // load the four neighboring pixels
    const float& p1 = p0[0 + 0 * stride];
    const float& p2 = p0[1 + 0 * stride];
    const float& p3 = p0[0 + 1 * stride];
    const float& p4 = p0[1 + 1 * stride];

    // Calculate the weights for each pixel
    Tp fx = x - px;
    Tp fy = y - py;
    Tp fx1 = Tp(1.) - fx;
    Tp fy1 = Tp(1.) - fy;

    Tp w1 = fx1 * fy1;
    Tp w2 = fx  * fy1;
    Tp w3 = fx1 * fy ;
    Tp w4 = fx  * fy ;

    // Calculate the weighted sum of pixels (for each color channel)
    Tp out = p1 * w1 + p2 * w2 + p3 * w3 + p4 * w4;

    return out;
}