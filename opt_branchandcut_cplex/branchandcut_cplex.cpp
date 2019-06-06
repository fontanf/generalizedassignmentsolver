#include "gap/opt_branchandcut_cplex/branchandcut_cplex.hpp"

#include <ilcplex/ilocplex.h>

using namespace gap;

ILOSTLBEGIN

typedef IloArray<IloNumVarArray> NumVarMatrix;
typedef IloArray<IloNumArray>    NumMatrix;

Solution gap::sopt_branchandcut_cplex(BranchAndCutCplexData d)
{
    VER(d.info, "*** branchandcut_cplex ***" << std::endl);

    IloEnv env;
    try {
        ItemIdx n = d.ins.item_number();
        AgentIdx m = d.ins.agent_number();

        IloModel model(env);

        // Variables
        NumVarMatrix x(env, n);
        for (ItemIdx j=0; j<n; ++j)
            x[j] = IloNumVarArray(env, m, 0, 1, ILOBOOL);

        // Objective
        IloExpr expr(env);
        for (ItemIdx j=0; j<n; j++)
            for (AgentIdx i=0; i<m; i++)
                expr += x[j][i] * d.ins.alternative(j, i).c;
        IloObjective obj = IloMinimize(env, expr);
        model.add(obj);

        // Capacity constraints
        for (AgentIdx i=0; i<m; i++) {
            IloExpr lhs(env);
            for (ItemIdx j=0; j<n; j++)
                lhs += x[j][i] * d.ins.alternative(j, i).w;
            model.add(IloRange(env, 0, lhs, d.ins.capacity(i)));
        }

        // One alternative per item constraint
        for (ItemIdx j=0; j<n; j++) {
            IloExpr lhs(env);
            for (AgentIdx i=0; i<m; i++)
                lhs += x[j][i];
            model.add(IloRange(env, 1, lhs, 1));
        }

        IloCplex cplex(model);

        // Initial solution
        if (d.sol.feasible()) {
            IloNumVarArray startVar(env);
            IloNumArray startVal(env);
            for (ItemIdx j=0; j<n; ++j) {
                AgentIdx i_curr = d.sol.agent(j);
                for (AgentIdx i=0; i<m; ++i) {
                    startVar.add(x[j][i]);
                    startVal.add(((i == i_curr)? 1: 0));
                }
            }
            cplex.addMIPStart(startVar, startVal);
            startVal.end();
            startVar.end();
        }

        // Display
        if (!d.info.output->verbose)
            cplex.setOut(env.getNullStream());

        // Precision
        cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 0.0);

        // Time limit
        if (d.info.timelimit != std::numeric_limits<double>::infinity())
            cplex.setParam(IloCplex::TiLim, d.info.timelimit);

        // Optimize
        if (cplex.solve()) {
            // Get solution
            for (AgentIdx i=0; i<m; i++)
                for (ItemIdx j=0; j<n; j++)
                    if (cplex.getValue(x[j][i]) > 0.5)
                        d.sol.set(j, i);
        } else {
        }
    } catch (IloException& ex) {
        std::cerr << "Error: " << ex.getMessage() << std::endl;
    } catch (...) {
        std::cerr << "Error" << std::endl;
    }
    env.end();

    return algorithm_end(d.sol, d.info);
}

