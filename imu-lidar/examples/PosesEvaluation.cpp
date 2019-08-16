#include <iostream>
#include <eigen3/Eigen/Dense>

void comparePoses(const Eigen::Matrix4d& pose1, const Eigen::Matrix4d& pose2, double & rotError, double & transError)
{
    Eigen::Matrix4d temp = pose1.inverse() * pose2;

    transError = temp.block<3,1>(0,3).norm();
    Eigen::AngleAxisd aa( temp.block<3,3>(0,0));
    rotError = aa.angle() * 180 / M_PI;

}


int main(int argc, char** argv)
{
    Eigen::Matrix4d pose1, pose2;

    pose1 << 0.711448, -0.691639,  0.124405,   1.03647,
             -0.535743,  -0.64838, -0.540909,   1.86047,
              0.454775,  0.318179,  -0.83183,    4.6181,
                    0,         0,         0,         1;

    pose2 << 0.9998,  0.0199987,          0,    1.52935,
             -0.0199987,     0.9998,         -0,      1.255,
                     -0,          0,          1,   -1.06567,
                      0,          0,          0,          1;
    double rotError, transError;

    comparePoses(pose1, pose2, rotError, transError);

    std::cout << "Trans error: "<< transError << "  Rotation error: " << rotError << std::endl;
    return 0;
}

