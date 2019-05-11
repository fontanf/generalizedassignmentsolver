#include "gap/lb_linrelax_clp/linrelax_clp.hpp"
#include "gap/opt_branchandcut_cbc/branchandcut_cbc.hpp"

#include <coin/ClpModel.hpp>
#include <coin/OsiClpSolverInterface.hpp>

using namespace gap;

LinRelaxClpOutput gap::lb_linrelax_clp(const Instance& ins, Info info)
{
    VER(info, "*** linrelax_clp ***" << std::endl);

    int loglevel = (info.output->verbose)? 1: 0;

    MilpMatrix mat(ins);

    ClpSimplex model;

    // Reduce printout
    model.messageHandler()->setLogLevel(loglevel);

    // Load problem
    model.loadProblem(mat.matrix, mat.colLower.data(), mat.colUpper.data(),
              mat.objective.data(), mat.rowLower.data(), mat.rowUpper.data());

    // Solve
    model.initialSolve();

    // Get solution
    LinRelaxClpOutput out;
    out.lb = model.getObjValue();
    out.x = std::vector<double>(ins.alternative_number(), 0);
    const double *solution = model.getColSolution();
    for (AltIdx k=0; k<ins.alternative_number(); ++k)
        out.x[k] = solution[k];

    algorithm_end(out.lb, info);
    return out;
}

