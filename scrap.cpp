#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <sstream>  
#include <fstream>
#include <math.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
 
// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
using namespace glm;

#include <common/objloader.hpp>
#include <tools.hpp>

using namespace std;
using namespace cv;
using namespace Eigen;

#include <sophus/se3.hpp>
#include <vector>
#include <algorithm>


struct triangle {

    Vector3f vertex1;
    Vector3f vertex2;
    Vector3f vertex3; 

};

float Area (triangle tri){
    
    Vector3f a = tri.vertex1;
    Vector3f b = tri.vertex2;
    Vector3f c = tri.vertex3;

    Vector3f ab = a-b;
    Vector3f ac = a-c;
    
    float theta = acos(ab.dot(ac)/(ab.norm()* ac.norm()));
    
    float area = 0.5 * ab.norm() * ac.norm() * sin(theta);


    return area;
}

vector<int> indexSort( vector<float> x ){

    vector<int> index;

    index.resize(x.size());
    std::size_t n(0);
    std::generate(std::begin(index), std::end(index), [&]{ return n++; });

    std::sort(  std::begin(index), 
                std::end(index),
                [&](int i1, int i2) { return x[i1] < x[i2]; } );

    return index;

}

bool compare(int a, int b, vector<float> * data)
{
    return data[a] < data[b];
}

int main() {
    // //so2
    // {
    //     double theta = 0.1;
    //     Sophus::SO2d rot = Sophus::SO2d::exp(theta);
    //     cout << rot.matrix() << endl;
    // }
    // //so3
    // {
    //     Sophus::Vector3<double> theta(0.1, 0.2, 0.3);
    //     Sophus::SO3d rot = Sophus::SO3d::exp(theta);
    //     cout << rot.matrix() << endl;
    // }

    // //se3
    // {
    //     Sophus::Vector<double, 6> theta;
    //     theta << 0.5, 0.6, 0.7, 0.1, 0.2, 0.3; //translation rotation
    //     Sophus::SE3d transform = Sophus::SE3d::exp(theta);
    //     cout << transform.so3().matrix() << endl;
    //     cout << transform.translation() << endl;

    //     Eigen::Vector3d X(0,0,1);
    //     cout << "transform \n" << transform.matrix() << endl;
    //     cout << "delta \n" << transform.log() << endl;
    //     cout << transform*X << endl;
    // }

    //     Eigen::Matrix<double, 4,4> test;
    //     test << 0,-1,0,-1800,-1,0,0,-2000,0,0,-1,203000,0,0,0,1;
    //     cout <<"Current test transform matirx is \n" << test << endl;
    //     Eigen::Matrix3d rot = test.block<3,3>(0,0);
    //     cout << "rotation matrix is \n" << test.block<3,3>(0,0)<< endl;
    //     Eigen::MatrixXd trans = test.block<3,1>(0,3);
    //     cout << "translation vector is \n" << test.block<3,1>(0,3)<< endl;
    //     Sophus::SO3d rotation (rot);
    //     cout << "SO3 matrix is \n" << rotation.matrix() << endl;

    //     VectorXd rot_delta = rotation.log(); 

    //     cout << "Current so3 vector is \n" << rot_delta << endl;

    //     vector<float> v = {1245,3468,1234,463,-75432,-54,762,5789,1324,-6789,-3211,34681,452,457};

    //     sort(v.begin(), v.end());
        
    //     for (int i = 0; i < v.size(); i++){
        
    //         cout << v[i] << " " ;

    //     }
        // VectorXd phi(3);
        // phi << 0.1,0,0,0,0,0;

        // VectorXd product = phi.cwiseProduct(delta);
        // cout << "the gradient vector is \n" << product << endl;
        // delta = delta-product;

        // Sophus::SE3d after_transform = Sophus::SE3d::exp(delta);

        // MatrixXd new_model = after_transform.matrix();
        // cout << "New T is \n" << new_model << " \n"<< endl;
//---------------------------------------------------------------------------------------------------------------------------------------

    // Find the corner of the solar panel

    //Load the satellite model
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> VertexMember;
    std::string Model_Path;
    Model_Path = "/home/xinghui/Find-Silhouette/satellite_model.obj";
    bool res = loadOBJ( Model_Path.c_str(), vertices, VertexMember);
    std::cout << "The size of vertices is " << vertices.size() << std:: endl;
    std::cout << "The size of VertexMember is " << VertexMember.size() << std:: endl;
    
    vector<triangle> triangles;

    for (int i = 0; i < vertices.size(); i += 3 ){

        triangle tri;
        tri.vertex1 = ConvertGlmToEigen3f(vertices[i]);
        tri.vertex2 = ConvertGlmToEigen3f(vertices[i+1]);
        tri.vertex3 = ConvertGlmToEigen3f(vertices[i+2]);

        triangles.push_back(tri);

    }
    
    cout << "The size of triangles is " << triangles.size() << endl;

    vector<float> area;

    for (int i = 0; i < triangles.size(); i++){

        float A = Area(triangles[i]);
        area.push_back(A);
    }

    cout << "The size of area is " << area.size() << endl;

    vector<float> test = {3,0,1,4,5};
    vector<int> order = indexSort(test);

    for (int i; i < order.size(); i++){

        cout << order[i] << " ";
    }

    // Vector3f a;
    // Vector3f b;
    // Vector3f c;

    // a << 2,0,0;
    // b << 0,0,0;
    // c << 0,2,0;

    // triangle tri;
    // tri.vertex1 = a;
    // tri.vertex2 = b;
    // tri.vertex3 = c;

    // float area = Area(tri);

    // cout << area << endl;

    return 0;
}