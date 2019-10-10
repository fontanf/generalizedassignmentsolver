#include "gap/opt_constraintprogramming_cplex/constraintprogramming_cplex.hpp"

#if CPLEX_FOUND

#include <ilcp/cp.h>

using namespace gap;

ILOSTLBEGIN

ConstraintProgrammingCplexOutput& ConstraintProgrammingCplexOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    return *this;
}

ConstraintProgrammingCplexOutput gap::sopt_constraintprogramming_cplex(const Instance& ins, ConstraintProgrammingCplexOptionalParameters p)
{
    VER(p.info, "*** constraintprogramming_cplex ***" << std::endl);
    ConstraintProgrammingCplexOutput output(ins, p.info);

    ItemIdx n = ins.item_number();
    AgentIdx m = ins.agent_number();
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
            cost[j][i] = ins.alternative(j, i).c;
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
        load[i] = IloIntVar(env, 0, ins.capacity(i));
        IloExpr sum(env);
        for (ItemIdx j=0; j<n; ++j)
            sum += ins.alternative(j, i).w * xij[j][i];
        model.add(load[i] == sum);
    }

    IloCP cp(model);

    // Display
    cp.setOut(env.getNullStream());

    // Time limit
    if (p.info.timelimit != std::numeric_limits<double>::infinity())
        cp.setParameter(IloCP::TimeLimit, p.info.timelimit);

    // Solve
    cp.startNewSearch();
    while (cp.next()) {
        Solution sol_curr(ins);
        for (ItemIdx j=0; j<n; ++j)
            sol_curr.set(j, cp.getValue(xj[j]));
        output.update_solution(sol_curr, std::stringstream(""), p.info);
    }

    if (p.info.check_time()) {
        Cost lb = (output.solution.feasible())? output.solution.cost(): ins.bound();
        output.update_lower_bound(lb, std::stringstream(""), p.info);
    }

    env.end();

    return output.algorithm_end(p.info);
}

#endif

