#if COINOR_FOUND

#include "generalizedassignment/lb_linrelax_clp/linrelax_clp.hpp"
#include "generalizedassignment/opt_branchandcut_cbc/branchandcut_cbc.hpp"

#include <coin/ClpModel.hpp>
#include <coin/OsiClpSolverInterface.hpp>

using namespace generalizedassignment;

LinRelaxClpOutput& LinRelaxClpOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    //PUT(info, "Algorithm", "Iterations", it);
    //VER(info, "Iterations: " << it << std::endl);
    return *this;
}

LinRelaxClpOutput generalizedassignment::lb_linrelax_clp(const Instance& ins, Info info)
{
    VER(info, "*** linrelax_clp ***" << std::endl);

    LinRelaxClpOutput output(ins, info);

    CoinLP mat(ins);

    ClpSimplex model;

    // Reduce printout
    model.messageHandler()->setLogLevel(0);

    // Load problem
    model.loadProblem(mat.matrix, mat.col_lower.data(), mat.col_upper.data(),
              mat.objective.data(), mat.row_lower.data(), mat.row_upper.data());

    // Solve
    model.initialSolve();

    // Get solution
    Cost lb = std::ceil(model.getObjValue() - TOL);
    output.update_lower_bound(lb, std::stringstream(""), info);
    output.x = std::vector<double>(ins.alternative_number(), 0);
    const double *solution = model.getColSolution();
    for (AltIdx k=0; k<ins.alternative_number(); ++k)
        output.x[k] = solution[k];

    return output.algorithm_end(info);
}

#endif

