#if CPLEX_FOUND

#include "generalizedassignmentsolver/algorithms/milp_cplex.hpp"

#include <ilcplex/ilocplex.h>

using namespace generalizedassignmentsolver;

ILOSTLBEGIN

MilpCplexOutput& MilpCplexOutput::algorithm_end(
        optimizationtools::Info& info)
{
    //info.add_to_json("Algorithm", "Iterations", it);
    Output::algorithm_end(info);
    //FFOT_VER(info, "Iterations: " << it << std::endl);
    return *this;
}

ILOMIPINFOCALLBACK4(loggingCallback,
                    const Instance&, instance,
                    MilpCplexOptionalParameters&, parameters,
                    MilpCplexOutput&, output,
                    std::vector<IloNumVarArray>&, x)
{
    Cost lb = std::ceil(getBestObjValue() - FFOT_TOL);
    output.update_lower_bound(lb, std::stringstream(""), parameters.info);

    if (!hasIncumbent())
        return;

    if (!output.solution.feasible() || output.solution.cost() > getIncumbentObjValue() + 0.5) {
        Solution solution(instance);
        for (ItemIdx item_id = 0;
                item_id < instance.number_of_items();
                ++item_id) {
            IloNumArray val(x[item_id].getEnv());
            getIncumbentValues(val, x[item_id]);
            for (AgentIdx agent_id = 0;
                    agent_id < instance.number_of_agents();
                    ++agent_id) {
                if (val[agent_id] > 0.5)
                    solution.set(item_id, agent_id);
            }
        }
        output.update_solution(solution, std::stringstream(""), parameters.info);
    }
}

MilpCplexOutput generalizedassignmentsolver::milp_cplex(
        const Instance& instance,
        MilpCplexOptionalParameters parameters)
{
    init_display(instance, parameters.info);
    parameters.info.os()
            << "Algorithm" << std::endl
            << "---------" << std::endl
            << "MILP (CPLEX)" << std::endl
            << std::endl;

    MilpCplexOutput output(instance, parameters.info);

    AgentIdx m = instance.number_of_agents();

    if (instance.number_of_items() == 0)
        return output.algorithm_end(parameters.info);

    IloEnv env;
    IloModel model(env);

    // Variables
    std::vector<IloNumVarArray> x;
    for (ItemIdx item_id = 0; item_id < instance.number_of_items(); item_id++)
        x.push_back(IloNumVarArray(env, m, 0, 1, ILOBOOL));

    // Objective
    IloExpr expr(env);
    for (ItemIdx item_id = 0; item_id < instance.number_of_items(); item_id++)
        for (AgentIdx agent_id = 0; agent_id < m; agent_id++)
            expr += instance.cost(item_id, agent_id) * x[item_id][agent_id];
    IloObjective obj = IloMinimize(env, expr);
    model.add(obj);

    // Capacity constraints
    for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id) {
        IloExpr expr(env);
        for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
            expr += instance.weight(item_id, agent_id) * x[item_id][agent_id];
        model.add(0 <= expr <= instance.capacity(agent_id));
    }

    // One alternative per item constraint
    for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id) {
        IloExpr expr(env);
        for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id)
            expr += x[item_id][agent_id];
        model.add(expr == 1);
    }

    IloCplex cplex(model);

    // Initial solution
    if (parameters.initial_solution != NULL && parameters.initial_solution->feasible()) {
        IloNumVarArray startVar(env);
        IloNumArray startVal(env);
        for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id) {
            AgentIdx agent_id_curr = parameters.initial_solution->agent(item_id);
            for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id) {
                startVar.add(x[item_id][agent_id]);
                startVal.add(((agent_id == agent_id_curr)? 1: 0));
            }
        }
        cplex.addMIPStart(startVar, startVal);
        startVal.end();
        startVar.end();
    }

    // Redirect standard output to log file
    std::ofstream logfile("cplex.log");
    cplex.setOut(logfile);

    if (parameters.only_linear_relaxation) {
        for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
            for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id)
                model.add(IloConversion(env, x[item_id][agent_id], ILOFLOAT));
        cplex.solve();
        Cost lb = std::ceil(cplex.getObjValue() - FFOT_TOL);
        output.update_lower_bound(lb, std::stringstream("linearrelaxation"), parameters.info);
        for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id) {
            output.x.push_back(std::vector<double>(instance.number_of_agents(), 0));
            for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id)
                if (cplex.getValue(x[item_id][agent_id]) > 0.5)
                    output.x[item_id][agent_id] = 1;
        }
        return output.algorithm_end(parameters.info);
    }

    cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 0.0); // Fix precision issue
    cplex.setParam(IloCplex::Param::MIP::Strategy::File, 2); // Avoid running out of memory

    // Time limit
    if (parameters.info.time_limit != std::numeric_limits<double>::infinity())
        cplex.setParam(IloCplex::TiLim, parameters.info.remaining_time());

    // Callback
    cplex.use(loggingCallback(env, instance, parameters, output, x));

    // Optimize
    cplex.solve();

    if (cplex.getStatus() == IloAlgorithm::Infeasible) {
        output.update_lower_bound(instance.bound(), std::stringstream(""), parameters.info);
    } else if (cplex.getStatus() == IloAlgorithm::Optimal) {
        if (!output.solution.feasible() || output.solution.cost() > cplex.getObjValue() + 0.5) {
            Solution solution(instance);
            for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
                for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id)
                    if (cplex.getValue(x[item_id][agent_id]) > 0.5)
                        solution.set(item_id, agent_id);
            output.update_solution(solution, std::stringstream(""), parameters.info);
        }
        output.update_lower_bound(output.solution.cost(), std::stringstream(""), parameters.info);
    } else if (cplex.isPrimalFeasible()) {
        if (!output.solution.feasible() || output.solution.cost() > cplex.getObjValue() + 0.5) {
            Solution solution(instance);
            for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
                for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id)
                    if (cplex.getValue(x[item_id][agent_id]) > 0.5)
                        solution.set(item_id, agent_id);
            output.update_solution(solution, std::stringstream(""), parameters.info);
        }
        Cost lb = std::ceil(cplex.getBestObjValue() - FFOT_TOL);
        output.update_lower_bound(lb, std::stringstream(""), parameters.info);
    } else {
        Cost lb = std::ceil(cplex.getBestObjValue() - FFOT_TOL);
        output.update_lower_bound(lb, std::stringstream(""), parameters.info);
    }

    env.end();

    return output.algorithm_end(parameters.info);
}

#endif

