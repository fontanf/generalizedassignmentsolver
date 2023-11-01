#include "generalizedassignmentsolver/algorithms/constraint_programming_cplex.hpp"

#if CPLEX_FOUND

#include <ilcp/cp.h>

using namespace generalizedassignmentsolver;

ILOSTLBEGIN

Output generalizedassignmentsolver::constraintprogramming_cplex(
        const Instance& instance,
        ConstraintProgrammingCplexOptionalParameters parameters)
{
    init_display(instance, parameters.info);
    parameters.info.os()
            << "Algorithm" << std::endl
            << "---------" << std::endl
            << "Constraint Programming (CPLEX)" << std::endl
            << std::endl;

    Output output(instance, parameters.info);

    IloEnv env;
    IloModel model(env);

    // Variables
    IloIntVarArray xj(env, instance.number_of_items(), 0, instance.number_of_agents() - 1);
    IloArray<IloBoolVarArray> xij(env, instance.number_of_items());
    for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
        xij[item_id] = IloBoolVarArray(env, instance.number_of_agents());

    // Channel xij == 1 <=> xj == i
    for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id)
        for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
            model.add((xij[item_id][agent_id] == 1) == (xj[item_id] == agent_id));

    // Costs
    IloArray<IloIntArray> cost(env);
    for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id) {
        cost.add(IloIntArray(env, instance.number_of_agents()));
        for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id)
            cost[item_id][agent_id] = instance.cost(item_id, agent_id);
    }

    // Cost variables
    IloIntExprArray cj(env, instance.number_of_items());
    for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
        cj[item_id] = IloElement(cost[item_id], xj[item_id]);

    // Objective
    IloIntExpr c(env);
    for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
        c += cj[item_id];
    model.add(IloMinimize(env, c));

    // Load variables (and capacity constraint)
    IloIntVarArray load(env, instance.number_of_agents());
    for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id) {
        load[agent_id] = IloIntVar(env, 0, instance.capacity(agent_id));
        IloExpr sum(env);
        for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
            sum += instance.weight(item_id, agent_id) * xij[item_id][agent_id];
        model.add(load[agent_id] == sum);
    }

    IloCP cp(model);

    // Display
    cp.setOut(env.getNullStream());

    // Time limit
    if (parameters.info.time_limit != std::numeric_limits<double>::infinity())
        cp.setParameter(IloCP::TimeLimit, parameters.info.time_limit);

    // Solve
    cp.startNewSearch();
    while (cp.next()) {
        Solution sol_curr(instance);
        for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
            sol_curr.set(item_id, cp.getValue(xj[item_id]));
        output.update_solution(sol_curr, std::stringstream(""), parameters.info);
    }

    if (!parameters.info.needs_to_end()) {
        Cost lb = (output.solution.feasible())? output.solution.cost(): instance.bound();
        output.update_bound(lb, std::stringstream(""), parameters.info);
    }

    env.end();

    return output.algorithm_end(parameters.info);
}

#endif

