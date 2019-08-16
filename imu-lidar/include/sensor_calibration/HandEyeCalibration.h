/*
 * Descriptiong: Modified version of hand eye calibration algorithm implementation from github
 * Author: Ran Tang
 * Date: May 09, 2018
*/


#ifndef HANDEYE_CALIBRATION_HEADER
#define HANDEYE_CALIBRATION_HEADER

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <ceres/ceres.h>
#include "DualQuaternion.h"

namespace SensorCalibration {
namespace HandEye{

enum SolverType
{
    defaultSolver = 0,
    dualQuaternionSolver = 1
};

std::string solverTypeString [] = {"defaultSolver", "dualQuaternionSolver"};

class HandEyeCalibration {
  public:
    HandEyeCalibration();

    static void estimateHandEyeScrew(
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& rvecs1,
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& tvecs1,
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& rvecs2,
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& tvecs2,
        Eigen::Matrix4d& H_12, bool planarMotion = false);

    static void estimateHandEyeScrew(
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& rvecs1,
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& tvecs1,
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& rvecs2,
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& tvecs2,
        Eigen::Matrix4d& H_12, ceres::Solver::Summary& summary,
        bool planarMotion = false, bool useInitialGuess=false);

    static void estimateHandEyeStandalone(
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& rvecs1,
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& tvecs1,
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& rvecs2,
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& tvecs2,
        Eigen::Matrix4d& H_12, ceres::Solver::Summary& summary,
        bool planarMotion = false, bool useInitialGuess=false);

    static void handEyeTransformRefinement(
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& rvecs1,
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& tvecs1,
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& rvecs2,
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& tvecs2,
        Eigen::Matrix4d& H_12, ceres::Solver::Summary& summary);

    static void setVerbose(bool on = true);

  private:
    /// @brief solve ax^2 + bx + c = 0
    static bool solveQuadraticEquation(double a, double b, double c, double& x1,
                                       double& x2);

    /// @brief Initial hand-eye screw estimate using fast but coarse
    /// Eigen::JacobiSVD
    static SensorCalibration::DualQuaterniond estimateHandEyeScrewInitial(Eigen::MatrixXd& T,
                                                       bool planarMotion);

    /// @brief Refine hand-eye screw estimate using initial coarse estimate and
    /// Ceres Solver Library.
    static void estimateHandEyeScrewRefine(
        SensorCalibration::DualQuaterniond& dq,
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& rvecs1,
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& tvecs1,
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& rvecs2,
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& tvecs2);

    static void estimateHandEyeScrewRefine(
        SensorCalibration::DualQuaterniond& dq,
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& rvecs1,
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& tvecs1,
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& rvecs2,
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& tvecs2,
        ceres::Solver::Summary& summary);

    static double computeResidual(
            double q[7],
            const Eigen::Vector3d& rvecs1,
            const Eigen::Vector3d& tvecs1,
            const Eigen::Vector3d& rvecs2,
            const Eigen::Vector3d& tvecs2);

    static bool mVerbose;
    static double mResidualThreshold;
};

}
}

#endif

