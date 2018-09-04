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
        cout << "transform " << transform.matrix() << endl;
        cout << transform*X << endl;
    }

    return 0;
}