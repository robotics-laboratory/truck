#include <cmath>
#include <iostream>

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

using namespace Eigen;
using namespace std;
using namespace g2o;

int main() {
    typedef BlockSolver <BlockSolverTraits<-1, -1>> SlamBlockSolver;
    typedef LinearSolverEigen <SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    // allocating the optimizer
    SparseOptimizer optimizer;
    auto linearSolver = std::make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering(false);
    OptimizationAlgorithmGaussNewton *solver =
            new OptimizationAlgorithmGaussNewton(
                    std::make_unique<SlamBlockSolver>(std::move(linearSolver)));
    optimizer.setAlgorithm(solver);

    return 0;
}
