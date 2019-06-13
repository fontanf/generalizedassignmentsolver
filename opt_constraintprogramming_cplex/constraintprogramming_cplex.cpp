#include "gap/opt_constraintprogramming_cplex/constraintprogramming_cplex.hpp"

#include <ilcp/cp.h>

using namespace gap;

ILOSTLBEGIN

Solution gap::sopt_constraintprogramming_cplex(ConstraintProgrammingCplexData d)
{
    VER(d.info, "*** constraintprogramming_cplex ***" << std::endl);

    ItemIdx n = d.ins.item_number();
    AgentIdx m = d.ins.agent_number();
    IloEnv env;
    IloModel model(env);

    // Variables
    IloIntVarArray xj(env, n, 0, m - 1);
    IloArray<IloBoolVarArray> xij(env, n);
    for(ItemIdx j=0; j<n; j++)
        xij[j] = IloBoolVarArray(env, m);

    // Channel xij == 1 <=> xj == i
    for(AgentIdx i=0; i<m; i++)
        for(ItemIdx j=0; j<n; j++)
            model.add((xij[j][i] == 1) == (xj[j] == i));

    // Costs
    IloArray<IloIntArray> cost(env);
    for(ItemIdx j=0; j<n; j++) {
        cost.add(IloIntArray(env, m));
        for(AgentIdx i=0; i<m; i++)
            cost[j][i] = d.ins.alternative(j, i).c;
    }

    // Cost variables
    IloIntExprArray cj(env, n);
    for(ItemIdx j=0; j<n; j++)
        cj[j] = IloElement(cost[j], xj[j]);

    // Objective
    IloIntExpr c(env);
    for (ItemIdx j=0; j<n; j++)
        c += cj[j];
    model.add(IloMinimize(env, c));

    // Load variables (and capacity constraint)
    IloIntVarArray load(env, m);
    for(AgentIdx i=0; i<m; i++) {
        load[i] = IloIntVar(env, 0, d.ins.capacity(i));
        IloExpr sum(env);
        for (ItemIdx j=0; j<n; ++j)
            sum += d.ins.alternative(j, i).w * xij[j][i];
        model.add(load[i] == sum);
    }

    IloCP cp(model);

    if (d.info.timelimit != std::numeric_limits<double>::infinity())
        cp.setParameter(IloCP::TimeLimit, d.info.timelimit);

    // Solve
    if (cp.solve())
        for (ItemIdx j=0; j<n; ++j)
            d.sol.set(j, cp.getValue(xj[j]));

    env.end();

    return algorithm_end(d.sol, d.info);
}

