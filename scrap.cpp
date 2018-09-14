#include <sophus/se3.hpp>
#include <iostream>
using namespace std;


int main() {
    //so2
    {
        double theta = 0.1;
        Sophus::SO2d rot = Sophus::SO2d::exp(theta);
        cout << rot.matrix() << endl;
    }
    //so3
    {
        Sophus::Vector3<double> theta(0.1, 0.2, 0.3);
        Sophus::SO3d rot = Sophus::SO3d::exp(theta);
        cout << rot.matrix() << endl;
    }

    //se3
    {
        Sophus::Vector<double, 6> theta;
        theta << 0.5, 0.6, 0.7, 0.1, 0.2, 0.3; //translation rotation
        Sophus::SE3d transform = Sophus::SE3d::exp(theta);
        cout << transform.so3().matrix() << endl;
        cout << transform.translation() << endl;

        Eigen::Vector3d X(0,0,1);
        cout << "transform \n" << transform.matrix() << endl;
        cout << "delta \n" << transform.log() << endl;
        cout << transform*X << endl;
    }

        Eigen::Matrix<double, 4,4> test;
        test << 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16;
        cout << test << endl;
        cout << test.block<3,3>(0,0)<< endl;
        cout << test.block<3,1>(0,3)<< endl;

    return 0;
}