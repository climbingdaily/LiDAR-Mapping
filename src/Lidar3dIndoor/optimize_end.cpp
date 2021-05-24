#include "optimize_end.h"

/***************************************************************
 * class OptimizeEnd:
 *     graphic optimze end
 * ************************************************************/
OptimizeEnd::OptimizeEnd()
{
    io_tool.makeDir(exepath + "/tmp");
    io_tool.makeDir(exepath + "/output");
    outputFile = "";
}

OptimizeEnd::~OptimizeEnd()
{

}

int OptimizeEnd::Run()
{
    std::ifstream ifs(loopDetectResult.c_str());
    if(!globalOptimizer.load(ifs)){
      std::cout << "Cannot load loop detect result." << std::endl;
      exit(-1);
    }
    consoleProgress(83);
    // create the linear solver
    auto linearSolver = g2o::make_unique<LinearSolverCSparse<BlockSolverX::PoseMatrixType>>();
    // create the block solver on top of the linear solver
    auto blockSolver = g2o::make_unique<BlockSolverX>(std::move(linearSolver));
    solver = new OptimizationAlgorithmLevenberg(std::move(blockSolver));

    _robustKernel = RobustKernelFactory::instance()->construct( "Cauchy" );
    globalOptimizer.setVerbose( false );  // not print optimize information
// #if DEBUG
//     globalOptimizer.setVerbose( true );
// #endif
    globalOptimizer.setAlgorithm(solver);
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize(iterations);
    if(outputFile == ""){
      std::cout << "OptimizeEnd::Run : You haven't set optimize output file!"  << std::endl;
      return -1;
    }
    globalOptimizer.save(outputFile.c_str());
    consoleProgress(85);
    return 0;
}

void OptimizeEnd::SetOptimizeIterations(int _iter)
{
    iterations = _iter;
}


void OptimizeEnd::SetLoopResultFilename(string _fileName)
{
    loopDetectResult = _fileName;
}

void OptimizeEnd::SaveOptimizeResult(string _optimize_result_path)
{

}
