#if CBC_FOUND

#include "generalizedassignmentsolver/algorithms/linrelax_clp.hpp"
#include "generalizedassignmentsolver/algorithms/milp_cbc.hpp"

#include <ClpModel.hpp>
#include <OsiClpSolverInterface.hpp>

using namespace generalizedassignmentsolver;

const LinRelaxClpOutput generalizedassignmentsolver::linrelax_clp(
        const Instance& instance,
        const Parameters& parameters = {})
{
    LinRelaxClpOutput output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Linear relaxation (CLP)");
    algorithm_formatter.print_header();

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
    algorithm_formatter.update_bound(lb, "");
    output.x.resize(instance.number_of_items(), std::vector<double>(instance.number_of_agents()));
    const double* solution = model.getColSolution();
    for (ItemIdx item_id = 0;
            item_id < instance.number_of_items();
            ++item_id) {
        for (AgentIdx agent_id = 0;
                agent_id < instance.number_of_agents();
                ++agent_id) {
            output.x[agent_id][item_id] = solution[instance.number_of_agents() * item_id + agent_id];
        }
    }

    algorithm_formatter.end();
    return output;
}

#endif

