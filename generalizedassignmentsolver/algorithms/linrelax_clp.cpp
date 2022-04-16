#if COINOR_FOUND

#include "generalizedassignmentsolver/algorithms/linrelax_clp.hpp"
#include "generalizedassignmentsolver/algorithms/milp_cbc.hpp"

#include <coin/ClpModel.hpp>
#include <coin/OsiClpSolverInterface.hpp>

using namespace generalizedassignmentsolver;

LinRelaxClpOutput& LinRelaxClpOutput::algorithm_end(Info& info)
{
    //FFOT_PUT(info, "Algorithm", "Iterations", it);
    Output::algorithm_end(info);
    //FFOT_VER(info, "Iterations: " << it << std::endl);
    return *this;
}

LinRelaxClpOutput generalizedassignmentsolver::linrelax_clp(
        const Instance& instance,
        Info info)
{
    init_display(instance, info);
    FFOT_VER(info,
               "Algorithm" << std::endl
            << "---------" << std::endl
            << "Linear Relaxation (CLP)" << std::endl
            << std::endl);

    ItemIdx n = instance.number_of_items();
    AgentIdx m = instance.number_of_agents();

    LinRelaxClpOutput output(instance, info);

    CoinLP problem(instance);

    ClpSimplex model;

    // Reduce printout
    model.messageHandler()->setLogLevel(0);

    // Load problem
    model.loadProblem(
            problem.matrix,
            problem.column_lower_bounds.data(),
            problem.column_upper_bounds.data(),
            problem.objective.data(),
            problem.row_lower_bounds.data(),
            problem.row_upper_bounds.data());

    // Solve
    model.initialSolve();

    // Get solution
    Cost lb = std::ceil(model.getObjValue() - FFOT_TOL);
    output.update_lower_bound(lb, std::stringstream(""), info);
    output.x.resize(n, std::vector<double>(m));
    const double* solution = model.getColSolution();
    for (ItemIdx j = 0; j < n; ++j)
        for (AgentIdx i = 0; i < m; ++i)
            output.x[i][j] = solution[m * j + i];

    return output.algorithm_end(info);
}

#endif

