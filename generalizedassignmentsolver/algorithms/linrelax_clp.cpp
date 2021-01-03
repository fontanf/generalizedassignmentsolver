#if COINOR_FOUND

#include "generalizedassignmentsolver/algorithms/linrelax_clp.hpp"
#include "generalizedassignmentsolver/algorithms/branchandcut_cbc.hpp"

#include <coin/ClpModel.hpp>
#include <coin/OsiClpSolverInterface.hpp>

using namespace generalizedassignmentsolver;

LinRelaxClpOutput& LinRelaxClpOutput::algorithm_end(Info& info)
{
    //PUT(info, "Algorithm", "Iterations", it);
    Output::algorithm_end(info);
    //VER(info, "Iterations: " << it << std::endl);
    return *this;
}

LinRelaxClpOutput generalizedassignmentsolver::linrelax_clp(const Instance& instance, Info info)
{
    VER(info, "*** linrelax_clp ***" << std::endl);

    ItemIdx  n = instance.item_number();
    AgentIdx m = instance.agent_number();

    LinRelaxClpOutput output(instance, info);

    CoinLP mat(instance);

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
    output.x.resize(n, std::vector<double>(m));
    const double *solution = model.getColSolution();
    for (ItemIdx j = 0; j < n; ++j)
        for (AgentIdx i = 0; i < m; ++i)
            output.x[i][j] = solution[m * j + i];

    return output.algorithm_end(info);
}

#endif

