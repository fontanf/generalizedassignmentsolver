#if CPLEX_FOUND

#include "generalizedassignmentsolver/algorithms/branchandcut_cplex.hpp"

#include <ilcplex/ilocplex.h>

using namespace generalizedassignmentsolver;

ILOSTLBEGIN

BranchAndCutCplexOutput& BranchAndCutCplexOutput::algorithm_end(Info& info)
{
    //PUT(info, "Algorithm", "Iterations", it);
    Output::algorithm_end(info);
    //VER(info, "Iterations: " << it << std::endl);
    return *this;
}

ILOMIPINFOCALLBACK4(loggingCallback,
                    const Instance&, instance,
                    BranchAndCutCplexOptionalParameters&, p,
                    BranchAndCutCplexOutput&, output,
                    IloNumVarArray, vars)
{
    Cost lb = std::ceil(getBestObjValue() - TOL);
    output.update_lower_bound(lb, std::stringstream(""), p.info);

    if (!hasIncumbent())
        return;

    if (!output.solution.feasible() || output.solution.cost() > getIncumbentObjValue() + 0.5) {
        Solution sol_curr(instance);
        IloNumArray val(vars.getEnv());
        getIncumbentValues(val, vars);
        for (AltIdx k=0; k<instance.alternative_number(); ++k)
            if (val[k] > 0.5)
                sol_curr.set(k);
        output.update_solution(sol_curr, std::stringstream(""), p.info);
    }
}

BranchAndCutCplexOutput generalizedassignmentsolver::branchandcut_cplex(const Instance& instance, BranchAndCutCplexOptionalParameters p)
{
    VER(p.info, "*** branchandcut_cplex ***" << std::endl);

    BranchAndCutCplexOutput output(instance, p.info);

    ItemIdx n = instance.item_number();
    AgentIdx m = instance.agent_number();
    AltIdx o = instance.alternative_number();

    if (n == 0)
        return output.algorithm_end(p.info);

    IloEnv env;
    IloModel model(env);

    // Variables
    IloNumVarArray x(env, o, 0, 1, ILOBOOL);

    // Objective
    IloExpr expr(env);
    for (ItemIdx j=0; j<n; j++)
        for (AgentIdx i=0; i<m; i++)
            expr += x[instance.alternative_index(j, i)] * instance.alternative(j, i).c;
    IloObjective obj = IloMinimize(env, expr);
    model.add(obj);

    // Capacity constraints
    for (AgentIdx i=0; i<m; i++) {
        IloExpr expr(env);
        for (ItemIdx j=0; j<n; j++)
            expr += x[instance.alternative_index(j, i)] * instance.alternative(j, i).w;
        model.add(0 <= expr <= instance.capacity(i));
    }

    // One alternative per item constraint
    for (ItemIdx j=0; j<n; j++) {
        IloExpr expr(env);
        for (AgentIdx i=0; i<m; i++)
            expr += x[instance.alternative_index(j, i)];
        model.add(expr == 1);
    }

    IloCplex cplex(model);

    // Redirect standard output to log file
    std::ofstream logfile("cplex.log");
    cplex.setOut(logfile);

    cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 0.0); // Fix precision issue
    cplex.setParam(IloCplex::Param::MIP::Strategy::File, 2); // Avoid running out of memory

    // Initial solution
    if (p.initial_solution != NULL && p.initial_solution->feasible()) {
        IloNumVarArray startVar(env);
        IloNumArray startVal(env);
        for (ItemIdx j=0; j<n; ++j) {
            AgentIdx i_curr = p.initial_solution->agent(j);
            for (AgentIdx i=0; i<m; ++i) {
                startVar.add(x[instance.alternative_index(j, i)]);
                startVal.add(((i == i_curr)? 1: 0));
            }
        }
        cplex.addMIPStart(startVar, startVal);
        startVal.end();
        startVar.end();
    }

    // Time limit
    if (p.info.timelimit != std::numeric_limits<double>::infinity())
        cplex.setParam(IloCplex::TiLim, p.info.remaining_time());

    // Callback
    cplex.use(loggingCallback(env, instance, p, output, x));

    // Optimize
    cplex.solve();

    if (cplex.getStatus() == IloAlgorithm::Infeasible) {
        output.update_lower_bound(instance.bound(), std::stringstream(""), p.info);
    } else if (cplex.getStatus() == IloAlgorithm::Optimal) {
        if (!output.solution.feasible() || output.solution.cost() > cplex.getObjValue() + 0.5) {
            Solution sol_curr(instance);
            for (AltIdx k=0; k<o; ++k)
                if (cplex.getValue(x[k]) > 0.5)
                    sol_curr.set(k);
            output.update_solution(sol_curr, std::stringstream(""), p.info);
        }
        output.update_lower_bound(output.solution.cost(), std::stringstream(""), p.info);
    } else if (cplex.isPrimalFeasible()) {
        if (!output.solution.feasible() || output.solution.cost() > cplex.getObjValue() + 0.5) {
            Solution sol_curr(instance);
            for (AltIdx k=0; k<o; ++k)
                if (cplex.getValue(x[k]) > 0.5)
                    sol_curr.set(k);
            output.update_solution(sol_curr, std::stringstream(""), p.info);
        }
        Cost lb = std::ceil(cplex.getBestObjValue() - TOL);
        output.update_lower_bound(lb, std::stringstream(""), p.info);
    } else {
        Cost lb = std::ceil(cplex.getBestObjValue() - TOL);
        output.update_lower_bound(lb, std::stringstream(""), p.info);
    }

    env.end();

    return output.algorithm_end(p.info);
}

#endif

