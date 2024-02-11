#if CPLEX_FOUND

#include "generalizedassignmentsolver/algorithms/milp_cplex.hpp"

#include <ilcplex/ilocplex.h>

using namespace generalizedassignmentsolver;

ILOSTLBEGIN

ILOMIPINFOCALLBACK4(loggingCallback,
                    const Instance&, instance,
                    MilpCplexParameters&, parameters,
                    MilpCplexOutput&, output,
                    std::vector<IloNumVarArray>&, x)
{
    Cost lb = std::ceil(getBestObjValue() - FFOT_TOL);
    algorithm_formatter.update_bound(lb, "");

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
        algorithm_formatter.update_solution(solution, "");
    }
}

const MilpCplexOutput generalizedassignmentsolver::milp_cplex(
        const Instance& instance,
        const MilpCplexParameters& parameters)
{
    MilpCplexOutput output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("MILP (CPLEX)");
    algorithm_formatter.print_header();

    AgentIdx m = instance.number_of_agents();

    if (instance.number_of_items() == 0) {
        algorithm_formatter.end();
        return output;
    }

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
        algorithm_formatter.update_bound(lb, "linearrelaxation");
        for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id) {
            output.x.push_back(std::vector<double>(instance.number_of_agents(), 0));
            for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id)
                if (cplex.getValue(x[item_id][agent_id]) > 0.5)
                    output.x[item_id][agent_id] = 1;
        }

        algorithm_formatter.end();
        return output;
    }

    cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 0.0); // Fix precision issue
    cplex.setParam(IloCplex::Param::MIP::Strategy::File, 2); // Avoid running out of memory

    // Time limit
    if (parameters.timer.remaining_time() != std::numeric_limits<double>::infinity())
        cplex.setParam(IloCplex::TiLim, parameters.timer.remaining_time());

    // Callback
    cplex.use(loggingCallback(env, instance, parameters, output, x));

    // Optimize
    cplex.solve();

    if (cplex.getStatus() == IloAlgorithm::Infeasible) {
        algorithm_formatter.update_bound(instance.bound(), "");
    } else if (cplex.getStatus() == IloAlgorithm::Optimal) {
        if (!output.solution.feasible() || output.solution.cost() > cplex.getObjValue() + 0.5) {
            Solution solution(instance);
            for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
                for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id)
                    if (cplex.getValue(x[item_id][agent_id]) > 0.5)
                        solution.set(item_id, agent_id);
            algorithm_formatter.update_solution(solution, "");
        }
        algorithm_formatter.update_bound(output.solution.cost(), "");
    } else if (cplex.isPrimalFeasible()) {
        if (!output.solution.feasible() || output.solution.cost() > cplex.getObjValue() + 0.5) {
            Solution solution(instance);
            for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
                for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id)
                    if (cplex.getValue(x[item_id][agent_id]) > 0.5)
                        solution.set(item_id, agent_id);
            algorithm_formatter.update_solution(solution, "");
        }
        Cost lb = std::ceil(cplex.getBestObjValue() - FFOT_TOL);
        algorithm_formatter.update_bound(lb, "");
    } else {
        Cost lb = std::ceil(cplex.getBestObjValue() - FFOT_TOL);
        algorithm_formatter.update_bound(lb, "");
    }

    env.end();

    algorithm_formatter.end();
    return output;
}

#endif

