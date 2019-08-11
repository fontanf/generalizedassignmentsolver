#if CPLEX_FOUND

#include "gap/opt_branchandcut_cplex/branchandcut_cplex.hpp"

#include <ilcplex/ilocplex.h>

using namespace gap;

ILOSTLBEGIN

ILOMIPINFOCALLBACK2(loggingCallback,
                    BranchAndCutCplexData&, d,
                    IloNumVarArray, vars)
{
    if (d.lb < getBestObjValue() - 0.5)
        update_lb(d.lb, getBestObjValue(), d.sol, std::stringstream(""), d.info);

    if (!hasIncumbent())
        return;

    if (!d.sol.feasible() || d.sol.cost() > getIncumbentObjValue() + 0.5) {
        Solution sol_curr(d.ins);
        IloNumArray val(vars.getEnv());
        getIncumbentValues(val, vars);
        for (AltIdx k=0; k<d.ins.alternative_number(); ++k)
            if (val[k] > 0.5)
                sol_curr.set(k);
        d.sol.update(sol_curr, d.lb, std::stringstream(""), d.info);
    }
}

Solution gap::sopt_branchandcut_cplex(BranchAndCutCplexData d)
{
    VER(d.info, "*** branchandcut_cplex ***" << std::endl);

    init_display(d.info);

    ItemIdx n = d.ins.item_number();
    AgentIdx m = d.ins.agent_number();
    AltIdx o = d.ins.alternative_number();

    if (n == 0)
        return algorithm_end(d.sol, d.lb, d.info);

    IloEnv env;
    IloModel model(env);

    // Variables
    IloNumVarArray x(env, o, 0, 1, ILOBOOL);

    // Objective
    IloExpr expr(env);
    for (ItemIdx j=0; j<n; j++)
        for (AgentIdx i=0; i<m; i++)
            expr += x[d.ins.alternative_index(j, i)] * d.ins.alternative(j, i).c;
    IloObjective obj = IloMinimize(env, expr);
    model.add(obj);

    // Capacity constraints
    for (AgentIdx i=0; i<m; i++) {
        IloExpr lhs(env);
        for (ItemIdx j=0; j<n; j++)
            lhs += x[d.ins.alternative_index(j, i)] * d.ins.alternative(j, i).w;
        model.add(IloRange(env, 0, lhs, d.ins.capacity(i)));
    }

    // One alternative per item constraint
    for (ItemIdx j=0; j<n; j++) {
        IloExpr lhs(env);
        for (AgentIdx i=0; i<m; i++)
            lhs += x[d.ins.alternative_index(j, i)];
        model.add(IloRange(env, 1, lhs, 1));
    }

    IloCplex cplex(model);

    cplex.setOut(env.getNullStream()); // Remove standard output
    cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 0.0); // Fix precision issue
    cplex.setParam(IloCplex::Param::MIP::Strategy::File, 2); // Avoid running out of memory

    // Initial solution
    if (d.sol.feasible()) {
        IloNumVarArray startVar(env);
        IloNumArray startVal(env);
        for (ItemIdx j=0; j<n; ++j) {
            AgentIdx i_curr = d.sol.agent(j);
            for (AgentIdx i=0; i<m; ++i) {
                startVar.add(x[d.ins.alternative_index(j, i)]);
                startVal.add(((i == i_curr)? 1: 0));
            }
        }
        cplex.addMIPStart(startVar, startVal);
        startVal.end();
        startVar.end();
    }

    // Time limit
    if (d.info.timelimit != std::numeric_limits<double>::infinity())
        cplex.setParam(IloCplex::TiLim, d.info.timelimit);

    // Callback
    cplex.use(loggingCallback(env, d, x));

    // Optimize
    cplex.solve();

    if (cplex.getStatus() == IloAlgorithm::Infeasible) {
        if (d.lb < d.ins.bound())
            update_lb(d.lb, d.ins.bound(), d.sol, std::stringstream(""), d.info);
    } else if (cplex.getStatus() == IloAlgorithm::Optimal) {
        if (!d.sol.feasible() || d.sol.cost() > cplex.getObjValue() + 0.5) {
            Solution sol_curr(d.ins);
            for (AltIdx k=0; k<o; ++k)
                if (cplex.getValue(x[k]) > 0.5)
                    sol_curr.set(k);
            d.sol.update(sol_curr, d.lb, std::stringstream(""), d.info);
        }
        if (d.lb < d.sol.cost())
            update_lb(d.lb, d.sol.cost(), d.sol, std::stringstream(""), d.info);
    } else if (cplex.isPrimalFeasible()) {
        if (!d.sol.feasible() || d.sol.cost() > cplex.getObjValue() + 0.5) {
            Solution sol_curr(d.ins);
            for (AltIdx k=0; k<o; ++k)
                if (cplex.getValue(x[k]) > 0.5)
                    sol_curr.set(k);
            d.sol.update(sol_curr, d.lb, std::stringstream(""), d.info);
        }
        if (d.lb < cplex.getBestObjValue() + 0.5)
            update_lb(d.lb, cplex.getBestObjValue(), d.sol, std::stringstream(""), d.info);
    } else {
        if (d.lb < cplex.getBestObjValue() + 0.5)
            update_lb(d.lb, cplex.getBestObjValue(), d.sol, std::stringstream(""), d.info);
    }

    env.end();

    return algorithm_end(d.sol, d.lb, d.info);
}

#endif

