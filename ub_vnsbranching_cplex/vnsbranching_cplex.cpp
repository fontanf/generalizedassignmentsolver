#if CPLEX_FOUND

#include "gap/ub_vnsbranching_cplex/vnsbranching_cplex.hpp"

#include "gap/ub_random/random.hpp"
#include "gap/ub_ls_shiftswap/ls_shiftswap.hpp"

#include <ilcplex/ilocplex.h>

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace gap;

ILOSTLBEGIN

typedef IloArray<IloNumVarArray> NumVarMatrix;
typedef IloArray<IloNumArray>    NumMatrix;

Solution gap::sol_vnsbranching_cplex(const Instance& ins, std::mt19937_64& gen, Info info)
{
    ItemIdx n = ins.item_number();
    AgentIdx m = ins.agent_number();
    AltIdx o = ins.alternative_number();

    init_display(info);
    Solution sol_best(ins);

    Cost lb = 0;
    Solution sol_curr = sol_random(ins, gen);
    sol_curr.update_penalties(std::vector<PCost>(m, 100));

    std::stringstream ss;
    sol_best.update(sol_curr, lb, ss, info);
    IloEnv env;

    std::stringstream ss_tmp;
    auto moves = moves_shiftswap(ins);
    for (AltIdx k_max=4; k_max<o; k_max+=4) {
        bool improved = false;
        if (k_max == 4) {
            improved = shiftswap_iter(sol_curr, moves, gen, ss_tmp);
        } else {
            IloModel model(env);

            // Variables
            NumVarMatrix x(env, n);
            for (ItemIdx j=0; j<n; ++j)
                x[j] = IloNumVarArray(env, m, 0, 1, ILOBOOL);

            // Objective
            IloExpr expr(env);
            for (ItemIdx j=0; j<n; j++)
                for (AgentIdx i=0; i<m; i++)
                    expr += x[j][i] * ins.alternative(j, i).c;
            IloObjective obj = IloMinimize(env, expr);
            model.add(obj);

            // Capacity constraints
            for (AgentIdx i=0; i<m; i++) {
                IloExpr lhs(env);
                for (ItemIdx j=0; j<n; j++)
                    lhs += x[j][i] * ins.alternative(j, i).w;
                model.add(IloRange(env, 0, lhs, ins.capacity(i)));
            }

            // One alternative per item constraint
            for (ItemIdx j=0; j<n; j++) {
                IloExpr lhs(env);
                for (AgentIdx i=0; i<m; i++)
                    lhs += x[j][i];
                model.add(IloRange(env, 1, lhs, 1));
            }

            IloExpr lhs(env);
            for (ItemIdx j=0; j<n; ++j) {
                for (AgentIdx i=0; i<m; ++i) {
                    if (i == sol_best.agent(j)) {
                        lhs -= x[j][i];
                    } else {
                        lhs += x[j][i];
                    }
                }
            }
            model.add(IloRange(env, - n, lhs, k_max - n));

            IloCplex cplex(model);

            // Initial solution
            IloNumVarArray startVar(env);
            IloNumArray startVal(env);
            for (ItemIdx j=0; j<n; ++j) {
                AgentIdx i_curr = sol_best.agent(j);
                for (AgentIdx i=0; i<m; ++i) {
                    startVar.add(x[j][i]);
                    startVal.add(((i == i_curr)? 1: 0));
                }
            }
            cplex.addMIPStart(startVar, startVal);
            startVal.end();
            startVar.end();

            // Display
            cplex.setOut(env.getNullStream());

            // Stop at first improvment
            cplex.setParam(IloCplex::IntSolLim, 2);

            // Time limit
            if (info.timelimit != std::numeric_limits<double>::infinity())
                cplex.setParam(IloCplex::TiLim, info.timelimit - info.elapsed_time());

            // Optimize
            cplex.solve();

            if (sol_best.cost() <= cplex.getObjValue() + 0.5)
                continue;
            improved = true;

            // Get solution
            for (AgentIdx i=0; i<m; i++)
                for (ItemIdx j=0; j<n; j++)
                    if (cplex.getValue(x[j][i]) > 0.5)
                        sol_curr.set(j, i);
        }

        if (improved) {
            std::stringstream ss;
            ss << "k_max " << k_max;
            sol_best.update(sol_curr, lb, ss, info);
            k_max = 0;
        }
    }

    return algorithm_end(sol_best, info);
}

#endif

