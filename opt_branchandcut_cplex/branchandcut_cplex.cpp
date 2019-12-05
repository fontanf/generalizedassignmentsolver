#if CPLEX_FOUND

#include "generalizedassignment/opt_branchandcut_cplex/branchandcut_cplex.hpp"

#include <ilcplex/ilocplex.h>

using namespace generalizedassignment;

ILOSTLBEGIN

BranchAndCutCplexOutput& BranchAndCutCplexOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    //PUT(info, "Algorithm", "Iterations", it);
    //VER(info, "Iterations: " << it << std::endl);
    return *this;
}

ILOMIPINFOCALLBACK4(loggingCallback,
                    const Instance&, ins,
                    BranchAndCutCplexOptionalParameters&, p,
                    BranchAndCutCplexOutput&, output,
                    IloNumVarArray, vars)
{
    Cost lb = std::ceil(getBestObjValue() - TOL);
    output.update_lower_bound(lb, std::stringstream(""), p.info);

    if (!hasIncumbent())
        return;

    if (!output.solution.feasible() || output.solution.cost() > getIncumbentObjValue() + 0.5) {
        Solution sol_curr(ins);
        IloNumArray val(vars.getEnv());
        getIncumbentValues(val, vars);
        for (AltIdx k=0; k<ins.alternative_number(); ++k)
            if (val[k] > 0.5)
                sol_curr.set(k);
        output.update_solution(sol_curr, std::stringstream(""), p.info);
    }
}

BranchAndCutCplexOutput generalizedassignment::sopt_branchandcut_cplex(const Instance& ins, BranchAndCutCplexOptionalParameters p)
{
    VER(p.info, "*** branchandcut_cplex ***" << std::endl);

    BranchAndCutCplexOutput output(ins, p.info);

    ItemIdx n = ins.item_number();
    AgentIdx m = ins.agent_number();
    AltIdx o = ins.alternative_number();

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
            expr += x[ins.alternative_index(j, i)] * ins.alternative(j, i).c;
    IloObjective obj = IloMinimize(env, expr);
    model.add(obj);

    // Capacity constraints
    for (AgentIdx i=0; i<m; i++) {
        IloExpr lhs(env);
        for (ItemIdx j=0; j<n; j++)
            lhs += x[ins.alternative_index(j, i)] * ins.alternative(j, i).w;
        model.add(IloRange(env, 0, lhs, ins.capacity(i)));
    }

    // One alternative per item constraint
    for (ItemIdx j=0; j<n; j++) {
        IloExpr lhs(env);
        for (AgentIdx i=0; i<m; i++)
            lhs += x[ins.alternative_index(j, i)];
        model.add(IloRange(env, 1, lhs, 1));
    }

    IloCplex cplex(model);

    cplex.setOut(env.getNullStream()); // Remove standard output
    cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 0.0); // Fix precision issue
    cplex.setParam(IloCplex::Param::MIP::Strategy::File, 2); // Avoid running out of memory

    // Initial solution
    if (p.initial_solution != NULL && p.initial_solution->feasible()) {
        IloNumVarArray startVar(env);
        IloNumArray startVal(env);
        for (ItemIdx j=0; j<n; ++j) {
            AgentIdx i_curr = p.initial_solution->agent(j);
            for (AgentIdx i=0; i<m; ++i) {
                startVar.add(x[ins.alternative_index(j, i)]);
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
    cplex.use(loggingCallback(env, ins, p, output, x));

    // Optimize
    cplex.solve();

    if (cplex.getStatus() == IloAlgorithm::Infeasible) {
        output.update_lower_bound(ins.bound(), std::stringstream(""), p.info);
    } else if (cplex.getStatus() == IloAlgorithm::Optimal) {
        if (!output.solution.feasible() || output.solution.cost() > cplex.getObjValue() + 0.5) {
            Solution sol_curr(ins);
            for (AltIdx k=0; k<o; ++k)
                if (cplex.getValue(x[k]) > 0.5)
                    sol_curr.set(k);
            output.update_solution(sol_curr, std::stringstream(""), p.info);
        }
        output.update_lower_bound(output.solution.cost(), std::stringstream(""), p.info);
    } else if (cplex.isPrimalFeasible()) {
        if (!output.solution.feasible() || output.solution.cost() > cplex.getObjValue() + 0.5) {
            Solution sol_curr(ins);
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

