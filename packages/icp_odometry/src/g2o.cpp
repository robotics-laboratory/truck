#include "icp_odometry/g2o.h"

#include "icp_odometry/common.h"
#include "icp_odometry/config.h"
#include "icp_odometry/icp.h"
#include "icp_odometry/visualization.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <iostream>
#include <vector>


namespace truck::icp_odometry {
	typedef g2o::BlockSolver <g2o::BlockSolverTraits<3, 3>> BlockSolverType;
	typedef g2o::LinearSolverDense <BlockSolverType::PoseMatrixType> LinearSolverType;

	bool positionThresholdReached(geometry_msgs::msg::Point a, geometry_msgs::msg::Point b, double threshold) {
		double distance = std::sqrt(std::pow((b.x - a.x), 2) + std::pow((b.y - a.y), 2) + std::pow((b.z - a.z), 2));
		return distance > threshold;
	}

	std::vector <ICPOdometryData> selectReferencePoints(std::vector <ICPOdometryData> icpOdometryData, double odometryThreshold) {
		if (icpOdometryData.size() == 0) {
			return {};
		}

		nav_msgs::msg::Odometry lastOdometryReference = icpOdometryData[0].odometry;
		std::vector <ICPOdometryData> result = {icpOdometryData[0]};

		for (size_t i = 1; i < icpOdometryData.size(); ++i) {
			auto data = icpOdometryData[i];
			if (positionThresholdReached(data.odometry.pose.pose.position, lastOdometryReference.pose.pose.position, odometryThreshold)) {
				result.push_back(data);
				lastOdometryReference = data.odometry;
			}
		}

		return result;
	}

	g2o::SE2 convertTransformationToSE2(const Eigen::Matrix3d &transform) {
		double theta = std::atan2(transform(1, 0), transform(0, 0));

		double tx = transform(0, 2);
		double ty = transform(1, 2);

		return g2o::SE2(tx, ty, theta);
	}

	double quaternionToTheta(const geometry_msgs::msg::Quaternion &q) {
		return 2 * atan2(q.z, q.w);
	}

	nav_msgs::msg::Odometry convertSE2toOdometry(const g2o::SE2 &pose) {
		nav_msgs::msg::Odometry odom_msg;

		odom_msg.header.frame_id = "odom";
		odom_msg.child_frame_id = "base_link";

		odom_msg.header.stamp = rclcpp::Clock().now();

		odom_msg.pose.pose.position.x = pose.translation().x();
		odom_msg.pose.pose.position.y = pose.translation().y();
		odom_msg.pose.pose.position.z = 0.0;

		tf2::Quaternion q;
		q.setRPY(0, 0, pose.rotation().angle());
		odom_msg.pose.pose.orientation = tf2::toMsg(q);

		return odom_msg;
	}

	Eigen::Matrix3d createTransformation(const double angle, const double tx, const double ty) {
		Eigen::Matrix3d T = Eigen::Matrix3d::Identity();

		T(0, 0) = cos(angle);
		T(0, 1) = -sin(angle);
		T(1, 0) = sin(angle);
		T(1, 1) = cos(angle);

		T(0, 2) = tx;
		T(1, 2) = ty;

		return T;
	}

	double extractRotationAngle(const Eigen::Matrix3d &T) {
		return atan2(T(1, 0), T(0, 0));
	}

	Eigen::Matrix3d quaternionToRotationMatrix(const tf2::Quaternion &q) {
		Eigen::Quaterniond eigen_quat(q.w(), q.x(), q.y(), q.z());
		return eigen_quat.toRotationMatrix();
	}

	Eigen::Matrix3d createTransformationMatrix(const nav_msgs::msg::Odometry &odom) {
		double x = odom.pose.pose.position.x;
		double y = odom.pose.pose.position.y;

		tf2::Quaternion q;
		tf2::fromMsg(odom.pose.pose.orientation, q);

		Eigen::Matrix3d R = quaternionToRotationMatrix(q);

		Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
		T.block<2, 2>(0, 0) = R.block<2, 2>(0, 0);
		T(0, 2) = x;
		T(1, 2) = y;

		return T;
	}

	Eigen::Matrix3d computeRelativeTransformation(const nav_msgs::msg::Odometry &odom1, const nav_msgs::msg::Odometry &odom2) {
		double x1 = odom1.pose.pose.position.x;
		double y1 = odom1.pose.pose.position.y;
		double theta1 = quaternionToTheta(odom1.pose.pose.orientation);

		double x2 = odom2.pose.pose.position.x;
		double y2 = odom2.pose.pose.position.y;
		double theta2 = quaternionToTheta(odom2.pose.pose.orientation);

		double dx = x2 - x1;
		double dy = y2 - y1;

		double dtheta = theta2 - theta1;

		Eigen::Matrix3d relativeTransformationMatrix = Eigen::Matrix3d::Identity();
		relativeTransformationMatrix(0, 0) = std::cos(dtheta);
		relativeTransformationMatrix(0, 1) = -std::sin(dtheta);
		relativeTransformationMatrix(1, 0) = std::sin(dtheta);
		relativeTransformationMatrix(1, 1) = std::cos(dtheta);
		relativeTransformationMatrix(0, 2) = dx * std::cos(theta1) + dy * std::sin(theta1);
		relativeTransformationMatrix(1, 2) = -dx * std::sin(theta1) + dy * std::cos(theta1);

		return relativeTransformationMatrix;
	}

	void optimizePoses(Matcher::ICP &icp, std::vector <ICPOdometryData> &icpOdometryData, double icpEdgeThreshold) {
		g2o::SparseOptimizer optimizer;
		auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
		optimizer.setAlgorithm(solver);

		std::vector < g2o::VertexSE2 * > vertices;
		for (size_t i = 0; i < icpOdometryData.size(); ++i) {
			auto *vertex = new g2o::VertexSE2();
			vertex->setId(i);

			auto odometryPose = icpOdometryData[i].odometry.pose.pose;

			double theta = quaternionToTheta(odometryPose.orientation);
			g2o::SE2 pose(odometryPose.position.x, odometryPose.position.y, theta);

			vertex->setEstimate(pose);
			optimizer.addVertex(vertex);
			vertices.push_back(vertex);

			if (globalConfig.verbose) {
				std::cout << "ODOMETRY VERTEX CREATED\n";
				std::cout << "Index: " << i << std::endl;
				std::cout << "x: " << odometryPose.position.x << " y: " << odometryPose.position.y << " theta: " << theta << "\n\n";
			}
		}

		for (size_t i = 1; i < icpOdometryData.size(); ++i) {
			auto *edge = new g2o::EdgeSE2();
			edge->setVertex(0, vertices[i - 1]);
			edge->setVertex(1, vertices[i]);

			Eigen::Matrix3d odomTransform = computeRelativeTransformation(icpOdometryData[i - 1].odometry, icpOdometryData[i].odometry);

			g2o::SE2 relativePose = convertTransformationToSE2(odomTransform);
			edge->setMeasurement(relativePose);
			edge->setInformation(Eigen::Matrix3d::Identity() * globalConfig.odometryEdgeWeight);

			optimizer.addEdge(edge);

			if (globalConfig.verbose) {
				std::cout << "ODOMETRY EDGE CREATED\n";
				std::cout << "Between vertices: " << i - 1 << " " << i << std::endl;
				std::cout << "odomTransform:\n" << odomTransform << "\n\n";
			}
		}

		for (size_t i = 0; i < icpOdometryData.size(); ++i) {
			for (size_t j = i + 1; j < icpOdometryData.size(); ++j) {
				if (!positionThresholdReached(icpOdometryData[i].odometry.pose.pose.position,
											  icpOdometryData[j].odometry.pose.pose.position,
											  icpEdgeThreshold)) {
					auto *edge = new g2o::EdgeSE2();
					edge->setVertex(0, vertices[i]);
					edge->setVertex(1, vertices[j]);

					Eigen::Matrix3d odomTransform = computeRelativeTransformation(icpOdometryData[i].odometry, icpOdometryData[j].odometry);

					TransformationParameters odometryTP = odomTransform.cast<float>();
					DataPoints dataOdomInited(icpOdometryData[j].icpDataPoints);
					icp.transformations.apply(dataOdomInited, odometryTP);

					TransformationParameters transformationParameters = icp(dataOdomInited,
																			icpOdometryData[i].icpDataPoints);

					Eigen::Matrix3d icpTransform = transformationParameters.cast<double>();

					DataPoints dataOut(dataOdomInited);
					icp.transformations.apply(dataOut, transformationParameters);

					if (globalConfig.enableDebugIcpMatches) {
						saveICPTransformationToMcap(icpOdometryData[j].icpDataPoints,
													icpOdometryData[i].icpDataPoints,
													dataOut,
													"transformation_" + std::to_string(i) + "_" + std::to_string(j));
					}

					auto finalTransform = icpTransform * odomTransform;

					g2o::SE2 relativePose = convertTransformationToSE2(finalTransform);
					edge->setMeasurement(relativePose);
					edge->setInformation(Eigen::Matrix3d::Identity() * globalConfig.icpEdgeWeight);

					optimizer.addEdge(edge);

					if (globalConfig.verbose) {
						std::cout << "ICP EDGE CREATED\n";
						std::cout << "Between vertices: " << i << " " << j << "\n";
						std::cout << "odomTransform:\n" << odomTransform << "\n";
						std::cout << "icpTransform:\n" << icpTransform << "\n";
						std::cout << "finalTransform:\n" << finalTransform << "\n\n";
					}
				}
			}
		}

		g2o::VertexSE2 *firstRobotPose = dynamic_cast<g2o::VertexSE2 *>(optimizer.vertex(0));
		firstRobotPose->setFixed(true);
		optimizer.setVerbose(globalConfig.verbose);

		if (globalConfig.verbose) {
			std::cout << "Number of edges: " << optimizer.edges().size() << std::endl;
		}

		optimizer.initializeOptimization();

		optimizer.save("before.g2o");

		if (globalConfig.verbose) {
			std::cout << "Performing optimization:\n";
		}
		optimizer.optimize(10);

		optimizer.save("after.g2o");

		for (size_t i = 0; i < icpOdometryData.size(); ++i) {
			g2o::VertexSE2 *v = dynamic_cast<g2o::VertexSE2 *>(optimizer.vertex(i));
			if (v) {
				g2o::SE2 pose = v->estimate();
				icpOdometryData[i].optimizedOdometry = convertSE2toOdometry(pose);
			}
		}
	}

	Eigen::Matrix4f odometryToTransformation(const nav_msgs::msg::Odometry &odom) {
		Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

		transformation(0, 3) = odom.pose.pose.position.x;
		transformation(1, 3) = odom.pose.pose.position.y;
		transformation(2, 3) = odom.pose.pose.position.z;

		Eigen::Quaternionf quaternion(
				odom.pose.pose.orientation.w,
				odom.pose.pose.orientation.x,
				odom.pose.pose.orientation.y,
				odom.pose.pose.orientation.z
		);
		transformation.block<3, 3>(0, 0) = quaternion.toRotationMatrix();

		return transformation;
	}

	DataPoints getMergedDataPoints(std::vector <ICPOdometryData> &icpOdometryData) {
		if (icpOdometryData.size() == 0) {
			return {};
		}

		DataPoints result = icpOdometryData[0].icpDataPoints;
		for (size_t i = 1; i < icpOdometryData.size(); ++i) {
			Eigen::Matrix4f transformation = odometryToTransformation(icpOdometryData[i].optimizedOdometry);
			DataPoints tempDataPoints = icpOdometryData[i].icpDataPoints;

			for (int i = 0; i < tempDataPoints.features.cols(); ++i) {
				tempDataPoints.features.col(i).head<3>() = transformation.block<3, 3>(0, 0) * tempDataPoints.features.col(i).head<3>() + transformation.col(3).head<3>();
			}

			result.concatenate(tempDataPoints);
		}
		return result;
	}

	std::vector<DataPoints> getTransformedDataPoints(std::vector <ICPOdometryData> &icpOdometryData) {
		std::vector<DataPoints> result;
		for (size_t i = 1; i < icpOdometryData.size(); ++i) {
			Eigen::Matrix4f transformation = odometryToTransformation(icpOdometryData[i].optimizedOdometry);

			DataPoints tempDataPoints = icpOdometryData[i].icpDataPoints;

			for (int i = 0; i < tempDataPoints.features.cols(); ++i) {
				tempDataPoints.features.col(i).head<3>() = transformation.block<3, 3>(0, 0) * tempDataPoints.features.col(i).head<3>() + transformation.col(3).head<3>();
			}

			result.push_back(tempDataPoints);
		}
		return result;
	}
}