/*
 * Descriptiong: Multi-lidar optimization core
 * Author: Ran Tang
 * Date: June 06, 2018
*/

#include "sensor_calibration/MultiLidarOptimization.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

#include <algorithm>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <g2o/types/icp/types_icp.h>


namespace SensorCalibration
{
    MultiLidarOptimization::MultiLidarOptimization()
        :mLeafSize(0.1), mRadius(1), mMaxIterations(50), mMaxNeighbors(5), mRotationEpsilon(0.5), mTranslationEpsilon(0.02)
    {

    }

    MultiLidarOptimization::MultiLidarOptimization(const std::string& configFile)
    {

    }

    MultiLidarOptimization::MultiLidarOptimization(double leafSize, double mRadius, int mMaxIterations, int mMaxNeighbors)
        :mLeafSize(leafSize), mRadius(mRadius), mMaxIterations(mMaxIterations), mMaxNeighbors(mMaxNeighbors)
    {

    }

    MultiLidarOptimization::~MultiLidarOptimization()
    {
        for(size_t i=0; i<mErrorTerms.size(); i++)
        {
            delete []mErrorTerms[i];
        }
    }

    pcl::PointCloud<pcl::PointXYZI> MultiLidarOptimization::optimize(std::vector<pcl::PointCloud<pcl::PointXYZI> > &lidarPoints, std::vector<Eigen::Matrix4d> &transformations, bool isSummary)
    {
        // downsample
        pcl::VoxelGrid<pcl::PointXYZI> voxelGrid;
        voxelGrid.setLeafSize(mLeafSize, mLeafSize, mLeafSize);
        std::vector<pcl::PointCloud<pcl::PointXYZI> > lidarPointsVoxel;

        for(int i=0; i<lidarPoints.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointsPtr(new pcl::PointCloud<pcl::PointXYZI>(lidarPoints[i]));
            voxelGrid.setInputCloud(pointsPtr);
            pcl::PointCloud<pcl::PointXYZI> currPoints;
            voxelGrid.filter(currPoints);
            lidarPointsVoxel.push_back(currPoints);
        }

        // compute normals
//        std::vector<pcl::PointCloud<pcl::Normal>> normals = computeSurfaceNormal(lidarPointsVoxel);
        std::vector<pcl::PointCloud<pcl::Normal>> normals = computeSurfaceNormal(lidarPoints);
        // gicp
        std::vector<std::vector<int>> indices = ceresOptimization(lidarPoints, normals, transformations, isSummary);
//        g2oOptimization(lidarPoints, normals, transformations, isSummary);

        // transform points
        pcl::PointCloud<pcl::PointXYZI> registeredPoints;
        for(int i=0; i<lidarPoints.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZI> transformedPoints;
            pcl::transformPointCloud(lidarPoints[i], transformedPoints, transformations[i].cast<float>());
            registeredPoints += transformedPoints;
        }

//        for(int i=0; i<indices.size(); i++)
//        {
//            const std::vector<int>& matchedIndices = indices[i];
//            if(matchedIndices.size() != 0)
//            {
//                registeredPoints.push_back(lidarPoints[0][i]);
//                registeredPoints.push_back(lidarPoints[1][matchedIndices[0]]);
//            }
//        }


        return registeredPoints;
    }

    std::vector<std::vector<int>> MultiLidarOptimization::ceresOptimization(const std::vector<pcl::PointCloud<pcl::PointXYZI> > &lidarPoints, const std::vector<pcl::PointCloud<pcl::Normal>>& normals, std::vector<Eigen::Matrix4d> &transformations, bool isSummary)
    {
        Eigen::Matrix4d previousTransformation1, previousTransformation2;
        previousTransformation1.setIdentity();
        previousTransformation2.setIdentity();

        // copy to array
        copyTransformationToArray(transformations[0], mRotationA, mTranslationA);
        copyTransformationToArray(transformations[1], mRotationB, mTranslationB);

        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

        const pcl::PointCloud<pcl::PointXYZI> &sourcePoints = lidarPoints[0];
        const pcl::PointCloud<pcl::PointXYZI> &targetPoints = lidarPoints[1];
        const pcl::PointCloud<pcl::Normal> &sourceNormals = normals[0];
        const pcl::PointCloud<pcl::Normal> &targetNormals = normals[1];

        std::cout << "----------------------------------------------------------------------" << std::endl;
        for(int i=0; i<mMaxIterations; i++)
        {
            // transform points, assume 2 clouds
            std::vector<pcl::PointCloud<pcl::PointXYZI>> transformedPoints;
            // todo: convert back to eigen matrix
            for(int j=0; j< lidarPoints.size(); j++)
            {
                pcl::PointCloud<pcl::PointXYZI> currTransformedPoints;
                pcl::transformPointCloud(lidarPoints[j], currTransformedPoints, transformations[j].cast<float>());
                transformedPoints.push_back(currTransformedPoints);
            }

            // find nearest points with kdtree
            std::vector<std::vector<int>> neighbors;

            pcl::PointCloud<pcl::PointXYZI>::Ptr targetPointsPtr(new pcl::PointCloud<pcl::PointXYZI>(transformedPoints[1]));
            kdtree.setInputCloud(targetPointsPtr);

            for(int j=0; j<transformedPoints[0].size(); j++)
            {
                std::vector<float> distances;
                std::vector<int> indices;
                kdtree.radiusSearch(transformedPoints[0][j], mRadius, indices, distances, mMaxNeighbors);
                neighbors.push_back(indices);
            }

            // build solver
            ceres::Problem problem;

//            std::cout << "neighbors size: " << neighbors.size() << std::endl;

            for(int l=0; l<sourcePoints.size(); l++)
            {
                const std::vector<int> &currNeighbors = neighbors[l];

//                if(currNeighbors.size() != 0)
//                    std::cout << "current neighbors size: " << currNeighbors.size() << std::endl;

                for(int h=0; h<currNeighbors.size(); h++)
                {
                    int index = currNeighbors[h];
                    ErrorTerm *error = new ErrorTerm(sourcePoints[l], targetPoints[index]);
                    ceres::LossFunction *lossFunction = NULL;
                    ceres::CostFunction *costFunction = new ceres::AutoDiffCostFunction<ErrorTerm, 3, 4, 3, 4, 3>(error);
                    problem.AddResidualBlock(costFunction, lossFunction, mRotationA, mTranslationA, mRotationB, mTranslationB);
                    break;
                }
            }

            ceres::Solver::Options options;
            configureOptions(options);

            ceres::Solver::Summary summary;

            ceres::Solve(options, &problem, &summary);

            if(isSummary)
                std::cout << summary.BriefReport() << std::endl;

            Eigen::Matrix4d currentTransformation1 = computeTransformationFromArray(mRotationA, mTranslationA);
            Eigen::Matrix4d currentTransformation2 = computeTransformationFromArray(mRotationB, mTranslationB);


            // check convergence
            double rotationError1, translationError1;
            double rotationError2, translationError2;

            comparePoses(currentTransformation1, previousTransformation1, rotationError1, translationError1);
            comparePoses(currentTransformation2, previousTransformation2, rotationError2, translationError2);
            std::cout << "rotation error1: " << rotationError1 << " translation error1: " << translationError1 << std::endl;
            std::cout << "rotation error2: " << rotationError2 << " translation error2: " << translationError2 << std::endl;

            if(rotationError1 < mRotationEpsilon && rotationError2 < mRotationEpsilon &&
                    translationError1 < mTranslationEpsilon && translationError2 < mTranslationEpsilon)
            {
                transformations[0] = currentTransformation1;
                transformations[1] = currentTransformation2;
                return neighbors;
            }

            previousTransformation1 = currentTransformation1;
            previousTransformation2 = currentTransformation2;

            // todo: convert back to eigen matrix
            transformations[0] = currentTransformation1;
            transformations[1] = currentTransformation2;
        }
        std::cout << "----------------------------------------------------------------------" << std::endl;
        {
            // transform points, assume 2 clouds
            std::vector<pcl::PointCloud<pcl::PointXYZI>> transformedPoints;
            // todo: convert back to eigen matrix
            for(int j=0; j< lidarPoints.size(); j++)
            {
                pcl::PointCloud<pcl::PointXYZI> currTransformedPoints;
                pcl::transformPointCloud(lidarPoints[j], currTransformedPoints, transformations[j].cast<float>());
                transformedPoints.push_back(currTransformedPoints);
            }

            // find nearest points with kdtree
            std::vector<std::vector<int>> neighbors;

            pcl::PointCloud<pcl::PointXYZI>::Ptr targetPointsPtr(new pcl::PointCloud<pcl::PointXYZI>(transformedPoints[1]));
            kdtree.setInputCloud(targetPointsPtr);

            for(int i=0; i<transformedPoints[0].size(); i++)
            {
                std::vector<float> distances;
                std::vector<int> indices;
                kdtree.radiusSearch(transformedPoints[0][i], mRadius, indices, distances, mMaxNeighbors);
                neighbors.push_back(indices);
            }

            return neighbors;
        }
    }

    void MultiLidarOptimization::g2oOptimization(const std::vector<pcl::PointCloud<pcl::PointXYZI> > &lidarPoints, const std::vector<pcl::PointCloud<pcl::Normal>> &normals, std::vector<Eigen::Matrix4d> &transformations, bool isSummary)
    {
        Eigen::Isometry3d previousTransformation1, previousTransformation2;
        previousTransformation1.setIdentity();
        previousTransformation2.setIdentity();

        // find nearest points with kdtree
        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

        const pcl::PointCloud<pcl::PointXYZI> &sourcePoints = lidarPoints[0];
        const pcl::PointCloud<pcl::PointXYZI> &targetPoints = lidarPoints[1];
        const pcl::PointCloud<pcl::Normal> &sourceNormals = normals[0];
        const pcl::PointCloud<pcl::Normal> &targetNormals = normals[1];

        std::vector<Eigen::Isometry3d> posesEigenIsometry;
        for(size_t i=0; i<transformations.size(); i++)
        {
            Eigen::Isometry3d currPose;
            Eigen::Quaterniond q(transformations[i].block<3,3>(0,0));
            currPose = q;
            currPose.translation() = transformations[i].block<3,1>(0,3);

            posesEigenIsometry.push_back(currPose);
        }

        std::cout << "----------------------------------------------------------------------" << std::endl;
        for(int i=0; i<mMaxIterations; i++)
        {
            // transform points, assume 2 clouds
            std::vector<pcl::PointCloud<pcl::PointXYZI>> transformedPoints;
            for(int j=0; j< lidarPoints.size(); j++)
            {
                pcl::PointCloud<pcl::PointXYZI> currTransformedPoints;
                pcl::transformPointCloud(lidarPoints[j], currTransformedPoints, transformations[j].cast<float>());
                transformedPoints.push_back(currTransformedPoints);
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr targetPointsPtr(new pcl::PointCloud<pcl::PointXYZI>(transformedPoints[1]));
            kdtree.setInputCloud(targetPointsPtr);
            std::vector<std::vector<int>> neighbors;

            for(int j=0; j<transformedPoints[0].size(); j++)
            {
                std::vector<float> distances;
                std::vector<int> indices;
                kdtree.radiusSearch(transformedPoints[0][j], mRadius, indices, distances, mMaxNeighbors);
                neighbors.push_back(indices);
            }

//            std::cout << "neighbors size: " << neighbors.size() << std::endl;

            // build solver
            g2o::SparseOptimizer optimizer;

            auto linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>();
            auto blockSolver = g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver));
            g2o::OptimizationAlgorithmLevenberg* optimizationAlgorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
            optimizer.setAlgorithm(optimizationAlgorithm);

            // build graph
            std::vector<g2o::VertexSE3*> vertices;
            std::vector<g2o::Edge_V_V_GICP*> edges;

            // set up two poses
            for(int j=0; j<2; j++)
            {
                g2o::VertexSE3 * vertex = new g2o::VertexSE3();
                vertex->setEstimate(posesEigenIsometry[j]);
                vertex->setId(j);
                vertex->setFixed(false);
                optimizer.addVertex(vertex);
                vertices.push_back(vertex);
            }

            // set up edges
            for(int l=0; l<sourcePoints.size(); l++)
            {
                const std::vector<int> &currNeighbors = neighbors[l];

                Eigen::Vector3d pt0(sourcePoints[l].x, sourcePoints[l].y, sourcePoints[l].z);
                Eigen::Vector3d nm0(sourceNormals[l].normal_x, sourceNormals[l].normal_y, sourceNormals[l].normal_z);

                for(int h=0; h<currNeighbors.size(); h++)
                {
                    g2o::Edge_V_V_GICP * edge = new g2o::Edge_V_V_GICP();
                    g2o::VertexSE3* vp0 = dynamic_cast<g2o::VertexSE3*>(optimizer.vertices().find(0)->second);
                    g2o::VertexSE3* vp1 = dynamic_cast<g2o::VertexSE3*>(optimizer.vertices().find(1)->second);
                    edge->setVertex(0, vp0);
                    edge->setVertex(1, vp1);

                    int currIndex = currNeighbors[h];
                    Eigen::Vector3d pt1(targetPoints[currIndex].x, targetPoints[currIndex].y, targetPoints[currIndex].z);
                    Eigen::Vector3d nm1(targetNormals[currIndex].normal_x,targetNormals[currIndex].normal_y, targetNormals[currIndex].normal_z);

                    g2o::EdgeGICP meas;
                    meas.pos0 = pt0;
                    meas.pos1 = pt1;
//                    meas.normal0 = nm0;
//                    meas.normal1 = nm1;

                    edge->setMeasurement(meas);
//                    edge->information() =  meas.prec0(0.01);
                    edge->information().setIdentity();

                    optimizer.addEdge(edge);
                    edges.push_back(edge);
                    break;
                }
            }

            optimizer.initializeOptimization();
            optimizer.computeActiveErrors();
            std::cout << "Initial chi2 = " << FIXED(optimizer.chi2()) << std::endl;
            optimizer.setVerbose(true);
            optimizer.optimize(5);

            // retrieve poses from optimizer
            for(size_t j=0; j<posesEigenIsometry.size(); j++)
            {
                posesEigenIsometry[j] =  dynamic_cast<g2o::VertexSE3*>(optimizer.vertices().find(j)->second)->estimate();
            }

            // check convergence
            double rotationError1, translationError1;
            double rotationError2, translationError2;

            comparePoses(posesEigenIsometry[0].matrix(), previousTransformation1.matrix(), rotationError1, translationError1);
            comparePoses(posesEigenIsometry[1].matrix(), previousTransformation2.matrix(), rotationError2, translationError2);
            std::cout << "rotation error1: " << rotationError1 << " translation error1: " << translationError1 << std::endl;
            std::cout << "rotation error2: " << rotationError2 << " translation error2: " << translationError2 << std::endl;

            if(rotationError1 < mRotationEpsilon && rotationError2 < mRotationEpsilon &&
                    translationError1 < mTranslationEpsilon && translationError2 < mTranslationEpsilon)
            {
                transformations[0] = posesEigenIsometry[0].matrix();
                transformations[1] = posesEigenIsometry[1].matrix();
                return;
            }

            previousTransformation1 = posesEigenIsometry[0];
            previousTransformation2 = posesEigenIsometry[1];
        }

        std::cout << "----------------------------------------------------------------------" << std::endl;
        // todo: convert back to eigen matrix
        transformations[0] = posesEigenIsometry[0].matrix();
        transformations[1] = posesEigenIsometry[1].matrix();
    }

    void MultiLidarOptimization::configureOptions(ceres::Solver::Options &options)
    {
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.use_nonmonotonic_steps = false;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;
        options.max_num_iterations = 100;
        options.num_threads = 8;
    }

    void MultiLidarOptimization::copyTransformationToArray(const Eigen::Matrix4d &transform, double rotation[], double translation[])
    {
        Eigen::Matrix3d rotationMatrix = transform.block(0, 0, 3, 3);
        Eigen::Quaterniond qA(rotationMatrix);

        rotation[0] = qA.w();
        rotation[1] = qA.x();
        rotation[2] = qA.y();
        rotation[3] = qA.z();

        translation[0] = transform(0,3);
        translation[1] = transform(1,3);
        translation[2] = transform(2,3);
    }

    Eigen::Matrix4d MultiLidarOptimization::computeTransformationFromArray(const double rotation[], const double translation[])
    {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

        Eigen::Quaternion<double> qA(rotation[0], rotation[1], rotation[2], rotation[3]);
        qA.normalize();
        transform.block(0,0,3,3) = qA.toRotationMatrix();

        transform(0,3) = translation[0];
        transform(1,3) = translation[1];
        transform(2,3) = translation[2];

        return transform;
    }

    void MultiLidarOptimization::comparePoses(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2, double &rotError, double &transError)
    {
        Eigen::Matrix4d temp = pose1.inverse() * pose2;

        transError = temp.block<3,1>(0,3).norm();
        Eigen::AngleAxisd aa( temp.block<3,3>(0,0));
        rotError = aa.angle() * 180 / M_PI;
    }

    std::vector< pcl::PointCloud<pcl::Normal>> MultiLidarOptimization::computeSurfaceNormal(const std::vector<pcl::PointCloud<pcl::PointXYZI>> &points)
    {
        std::vector< pcl::PointCloud<pcl::Normal>> normals;
        pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;

        for(size_t i=0; i<points.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZI>(points[i]));
            ne.setInputCloud(cloudPtr);

            pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
            ne.setSearchMethod(tree);

            pcl::PointCloud<pcl::Normal>::Ptr normalPtr(new pcl::PointCloud<pcl::Normal>());
            ne.setRadiusSearch(3);
            ne.compute(*normalPtr);

            normals.push_back(*normalPtr);
        }

        return normals;
    }

}


