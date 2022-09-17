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
        AgentIdx m = instance.number_of_agents();
        ItemIdx n = instance.number_of_items();
        Solution solution(instance);
        for (ItemIdx j = 0; j < n; ++j) {
            IloNumArray val(x[j].getEnv());
            getIncumbentValues(val, x[j]);
            for (AgentIdx i = 0; i < m; ++i)
                if (val[i] > 0.5)
                    solution.set(j, i);
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

    ItemIdx n = instance.number_of_items();
    AgentIdx m = instance.number_of_agents();

    if (n == 0)
        return output.algorithm_end(parameters.info);

    IloEnv env;
    IloModel model(env);

    // Variables
    std::vector<IloNumVarArray> x;
    for (ItemIdx j = 0; j < n; j++)
        x.push_back(IloNumVarArray(env, m, 0, 1, ILOBOOL));

    // Objective
    IloExpr expr(env);
    for (ItemIdx j = 0; j < n; j++)
        for (AgentIdx i = 0; i < m; i++)
            expr += instance.cost(j, i) * x[j][i];
    IloObjective obj = IloMinimize(env, expr);
    model.add(obj);

    // Capacity constraints
    for (AgentIdx i = 0; i < m; i++) {
        IloExpr expr(env);
        for (ItemIdx j = 0; j < n; j++)
            expr += instance.weight(j, i) * x[j][i];
        model.add(0 <= expr <= instance.capacity(i));
    }

    // One alternative per item constraint
    for (ItemIdx j = 0; j < n; j++) {
        IloExpr expr(env);
        for (AgentIdx i = 0; i < m; i++)
            expr += x[j][i];
        model.add(expr == 1);
    }

    IloCplex cplex(model);

    // Initial solution
    if (parameters.initial_solution != NULL && parameters.initial_solution->feasible()) {
        IloNumVarArray startVar(env);
        IloNumArray startVal(env);
        for (ItemIdx j = 0; j < n; ++j) {
            AgentIdx i_curr = parameters.initial_solution->agent(j);
            for (AgentIdx i = 0; i < m; ++i) {
                startVar.add(x[j][i]);
                startVal.add(((i == i_curr)? 1: 0));
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
        for (ItemIdx j = 0; j < n; j++)
            for (AgentIdx i = 0; i < m; i++)
                model.add(IloConversion(env, x[j][i], ILOFLOAT));
        cplex.solve();
        Cost lb = std::ceil(cplex.getObjValue() - FFOT_TOL);
        output.update_lower_bound(lb, std::stringstream("linearrelaxation"), parameters.info);
        for (ItemIdx j = 0; j < n; j++) {
            output.x.push_back(std::vector<double>(instance.number_of_agents(), 0));
            for (AgentIdx i = 0; i < m; i++)
                if (cplex.getValue(x[j][i]) > 0.5)
                    output.x[j][i] = 1;
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
            for (ItemIdx j = 0; j < n; j++)
                for (AgentIdx i = 0; i < m; i++)
                    if (cplex.getValue(x[j][i]) > 0.5)
                        solution.set(j, i);
            output.update_solution(solution, std::stringstream(""), parameters.info);
        }
        output.update_lower_bound(output.solution.cost(), std::stringstream(""), parameters.info);
    } else if (cplex.isPrimalFeasible()) {
        if (!output.solution.feasible() || output.solution.cost() > cplex.getObjValue() + 0.5) {
            Solution solution(instance);
            for (ItemIdx j = 0; j < n; j++)
                for (AgentIdx i = 0; i < m; i++)
                    if (cplex.getValue(x[j][i]) > 0.5)
                        solution.set(j, i);
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

