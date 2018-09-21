#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <sstream>  
#include <fstream>
#include <math.h>
#include <iomanip>

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


// struct triangle {

//     Vector3d vertex1;
//     Vector3d vertex2;
//     Vector3d vertex3; 

// };

// double Area (triangle tri){
    
//     Vector3d a = tri.vertex1;
//     Vector3d b = tri.vertex2;
//     Vector3d c = tri.vertex3;

//     Vector3d ab = a-b;
//     Vector3d ac = a-c;
    
//     double theta = acos(ab.dot(ac)/(ab.norm()* ac.norm()));
    
//     double area = 0.5 * ab.norm() * ac.norm() * sin(theta);


//     return area;
// }

// vector<int> indexSort( vector<double> x ){

//     vector<int> index;

//     index.resize(x.size());
//     std::size_t n(0);
//     std::generate(std::begin(index), std::end(index), [&]{ return n++; });

//     std::sort(  std::begin(index), 
//                 std::end(index),
//                 [&](int i1, int i2) { return x[i1] < x[i2]; } );

//     return index;

// }

// bool compare(int a, int b, vector<float> * data)
// {
//     return data[a] < data[b];
// }



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

    // std::vector<glm::vec3> vertices;
    // std::vector<glm::vec3> VertexMember;
    // std::string Model_Path;
    // Model_Path = "/home/xinghui/Find-Silhouette/simple_sat.obj";
    // bool res = loadOBJ( Model_Path.c_str(), vertices, VertexMember);
    // std::cout << "The size of vertices is " << vertices.size() << std:: endl;
    // std::cout << "The size of VertexMember is " << VertexMember.size() << std:: endl;
    
    // vector<triangle> triangles;

    // for (int i = 0; i < vertices.size(); i += 3 ){

    //     triangle tri;
    //     tri.vertex1 = ConvertGlmToEigen3f(vertices[i]);
    //     tri.vertex2 = ConvertGlmToEigen3f(vertices[i+1]);
    //     tri.vertex3 = ConvertGlmToEigen3f(vertices[i+2]);

    //     triangles.push_back(tri);

    // }
    
    // cout << "The size of triangles is " << triangles.size() << endl;

    // vector<double> area;

    // for (int i = 0; i < triangles.size(); i++){

    //     double A = Area(triangles[i]);
    //     area.push_back(A);
    // }

    // cout << "The size of area is " << area.size() << endl;

    // vector<int> sorted_index;
    // sorted_index = indexSort(area);

    // cout << "The size of sorted_index is " << sorted_index.size() << endl;

    // vector<Vector3d> picked_vertices;

    // for (int i = 0; i < 16; i++){

    //     int number = sorted_index.size()-1-i;
    //     int index = sorted_index[number];
        
    //     picked_vertices.push_back(triangles[index].vertex1);
    //     picked_vertices.push_back(triangles[index].vertex2);
    //     picked_vertices.push_back(triangles[index].vertex3);
        
    // }

    // cout << "The size of picked_vertices is " << picked_vertices.size() << endl;    

    // Mat dummy(480,640, CV_8UC3);

    // Matrix3d K; K << 2743.21, 0, 320,
    //                  0, 2743.21, 240, 
    //                  0, 0, 1;
    
    // Matrix4d camera_pose; camera_pose << 1,0,0,0,
    //                                      0,1,0,0,
    //                                      0,0,1,0,
    //                                      0,0,0,1;
    
    // Matrix4d model_pose; model_pose << 1,0,0,0,
    //                                    0,0,-1,0,
    //                                    0,1,0,-203000,
    //                                    0,0,0,1;

    // for (int i = 0; i < picked_vertices.size(); i++){

    //     cout << picked_vertices[i] << "\n" << endl;
    // }

    // optimizer opt(picked_vertices, K, camera_pose, model_pose, dummy);
    // Mat scratch(480,640, CV_8UC3);
    
    // cout << "The number of channel of scratch is " << scratch.channels() << endl;
    // opt.Draw(scratch);
    // imshow(" ", scratch);

    // cvWaitKey(0);



    // vector<int> test = {1,2,3,4,5,6};
    // int a = 10;
    // if (find(test.begin(), test.end(), a) != test.end()){

    //     cout << " the element a is in the vector " << endl;

    // }else{

    //     cout << " the element a is not in the vector " << endl;
    // }

//-------------------------------------------------------------------------------------------------------------

    // Generate fill for David

    std::vector<Vector3d> vertices;
    std::vector< vector<int> > face;
    string filename = "/home/xinghui/Find-Silhouette/simple_sat.obj";

    LoadModelQuad(filename, vertices, face);

    std::vector<edge> edges;

    // cout << 4 % 4 << endl;

    for (int i = 0; i < face.size(); i++){
    
        for ( int j = 0; j < 4; j++){

            int index1 = j % 4;
            int index2 = (j+1) % 4;
            
            edge ed;

            ed.vertex1 = vertices[face[i][index1]-1];
            ed.vertex2 = vertices[face[i][index2]-1];
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

    cout << "The number of vertex is " << vertices.size() << endl;
    cout << "The number of face is " << face.size() << endl;
    cout << "The number of edge is " << edges.size() << endl;

    for (int i = 0; i < edges.size(); i++){
        
        cout << "-------------------------------------------------------------"<< endl;
        cout << " index of the edge is " << i + 1 << endl;
        cout << " vertices of the edge are " << edges[i].vertex1.transpose() << " " << edges[i].vertex2.transpose() << endl;
        cout << " indices of the vertices of edge are " << edges[i].index1 << " " << edges[i].index2 << endl;
        cout << " length of the edge is " << (edges[i].vertex1 - edges[i].vertex2).norm() << endl;

    }

    for (int i = 0; i < edges.size(); ++i)
    {
        cout << edges[i].index1 << " " << edges[i].index2 << endl;
    }

    // cout << edges[0].vertex1 << " \n" << endl;
    // cout << edges[0].vertex2 << " \n" << endl;
    // cout << (edges[0].vertex1 - edges[0].vertex2).norm() << endl;
    // cout << "-------------------------------------------------------------"<< endl;

    // cout << edges[1].vertex1 << " \n" << endl;
    // cout << edges[1].vertex2 << " \n" << endl;
    // cout << (edges[1].vertex1 - edges[1].vertex2).norm() << endl;


    // Vector3d a;
    // Vector3d b;
    // Vector3d c;

    // a << 2,0,0;
    // b << 0,0,0;
    // c << 0,2,0;

    // triangle tri;
    // tri.vertex1 = a;
    // tri.vertex2 = b;
    // tri.vertex3 = c;

    // float area = Area(tri);

    // cout << area << endl;

    Matrix3d mat; mat << 0,-1,0,1,0,0,0,0,1;

    Vector3d so3; so3 << 0.2, 0.2,0;
    Sophus::SO3d delta = Sophus::SO3d::exp(so3);

    Vector3d trans; trans << -1500,1500,203000;
    cout <<  delta.matrix()* mat << "\n " << endl;
    cout << delta.matrix() * trans << "\n"<< endl;




    return 0;
}