#include "icp_odometry/g2o.h"

#include "icp_odometry/common.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/edge_se2.h>

#include <cmath>
#include <iostream>
#include <vector>


namespace truck::icp_odometry {
	typedef g2o::BlockSolver <g2o::BlockSolverTraits<6, 3>> BlockSolverType;
	typedef g2o::LinearSolverDense <BlockSolverType::PoseMatrixType> LinearSolverType;

	g2o::SE2 convertTransformationToSE2(const TransformationParameters &transform) {
		double theta = std::atan2(transform(1, 0), transform(0, 0));

		double tx = transform(0, 2);
		double ty = transform(1, 2);

		return g2o::SE2(tx, ty, theta);
	}

	void optimizePoses(std::vector <TransformationParameters> icpTransformations) {
		g2o::SparseOptimizer optimizer;
		auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
		optimizer.setAlgorithm(solver);

		std::vector < g2o::VertexSE2 * > vertices;
		for (size_t i = 0; i < icpTransformations.size() + 1; ++i) {
			auto *vertex = new g2o::VertexSE2();
			vertex->setId(i);

			g2o::SE2 pose(i, i, 0); // dummy init pose estimation
			vertex->setEstimate(pose);
			optimizer.addVertex(vertex);
			vertices.push_back(vertex);
		}

		for (size_t i = 1; i < vertices.size(); ++i) {
			auto *edge = new g2o::EdgeSE2();
			edge->setVertex(0, vertices[i - 1]);
			edge->setVertex(1, vertices[i]);

			g2o::SE2 relativePose = convertTransformationToSE2(icpTransformations[0]);
			edge->setMeasurement(relativePose);
			edge->setInformation(Eigen::Matrix3d::Identity());

			optimizer.addEdge(edge);
		}

		optimizer.initializeOptimization();
		optimizer.optimize(10);

		optimizer.save("icp_odometry/graph.g2o");
	}
}