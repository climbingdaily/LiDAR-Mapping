#pragma once
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>

#include "tool_func.h"
#include "common.h"

using namespace std;
using namespace g2o;
using namespace cv;

typedef BlockSolver_6_3 SlamBlockSolver;
typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

G2O_USE_TYPE_GROUP(slam3d); // use 3d-SLAM types


class OptimizeEnd
{
  public:
    OptimizeEnd();
    ~OptimizeEnd();
    int Run();
    void SetLoopResultFilename(std::string _fileName);
    void SaveOptimizeResult(std::string _optimize_result_path);
    void SetOptimizeIterations(int _iter);
    void SetOutputFile(std::string _out) { outputFile = _out; }
    void setExePath(std::string _path){exepath = _path;};
    
  private:
    OptimizationAlgorithmLevenberg* solver;
    RobustKernel* _robustKernel;
    SparseOptimizer globalOptimizer;
    IOToolFunc io_tool;
    Matrix4Type cali_T;
    std::string loopDetectResult;
    std::string outputFile;
    std::string exepath;
    int iterations;
};
