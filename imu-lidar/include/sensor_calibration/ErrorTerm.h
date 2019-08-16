#ifndef _ERROR_TERM_H_
#define _ERROR_TERM_H

#include <pcl/point_types.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace SensorCalibration
{

class ErrorTerm
{
public:
    ErrorTerm(const pcl::PointXYZI &pt1, const pcl::PointXYZI &pt2)
        :mPoint1(pt1.x, pt1.y, pt1.z), mPoint2(pt2.x, pt2.y, pt2.z)
    {}

    template<typename T>
    bool operator()(const T* const rotation1, const T* const translation1, const T* const rotation2,
                    const T* const translation2, T* residuals) const
    {
        T point1[3];
        T point2[3];

        for(int i=0; i<3; i++)
        {
            point1[i] = T(mPoint1[i]);
            point2[i] = T(mPoint2[i]);
        }

        T transformedPoint1[3];
        T transformedPoint2[3];

        ceres::QuaternionRotatePoint(rotation1, point1, transformedPoint1);
        ceres::QuaternionRotatePoint(rotation2, point2, transformedPoint2);

        for(int i=0; i<3; i++)
        {
            transformedPoint1[i] = transformedPoint1[i] + translation1[i];
            transformedPoint2[i] = transformedPoint2[i] + translation2[i];

            residuals[i] = transformedPoint1[i] - transformedPoint2[i];
        }

        return true;
    }
private:
    Eigen::Vector3d mPoint1;
    Eigen::Vector3d mPoint2;
};

}

#endif
