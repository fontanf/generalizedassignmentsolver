#include "gap/ub_vlsn_mbp/vlsn_mbp.hpp"

#include "gap/lb_lagrelax_lbfgs/lagrelax_lbfgs.hpp"
#include "gap/ub_random/random.hpp"
#include "gap/ub_repair/repair.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

#include <ilcplex/ilocplex.h>

using namespace gap;

ILOSTLBEGIN

typedef IloArray<IloNumVarArray> NumVarMatrix;
typedef IloArray<IloNumArray>    NumMatrix;

struct ItemInfo
{
    ItemInfo(const Instance& ins, ItemIdx j, const std::vector<double>& lambda)
    {
        AgentIdx m = ins.agent_number();
        i_best_1 = -1;
        i_best_2 = -1;
        for (AgentIdx i=0; i<m; ++i) {
            PCost c = ins.alternative(j, i).c - lambda[i] * ins.alternative(j, i).w;
            if (i_best_1 == -1 || c_best_1 > c) {
                i_best_2 = i_best_1;
                c_best_2 = c_best_1;
                i_best_1 = i;
                c_best_1 = c;
            } else if (i_best_2 == -1 || c_best_2 > c) {
                i_best_2 = i;
                c_best_2 = c;
            }
        }
        //std::cout << "j " << j
            //<< " i_best_1 " << i_best_1 << " c_best_1 " << c_best_1
            //<< " i_best_2 " << i_best_2 << " c_best_2 " << c_best_2
            //<< std::endl;
    }

    AgentIdx i_best_1 = -1;
    PCost    c_best_1 = -1;
    AgentIdx i_best_2 = -1;
    PCost    c_best_2 = -1;
};

bool move_mbp_cplex(const Instance& ins, Solution& sol,
        const std::vector<ItemInfo>& item_info, Info& info)
{
    (void)info;
    ItemIdx n = ins.item_number();
    AgentIdx m = ins.agent_number();
    IloEnv env;
    IloModel model(env);

    // Variables
    IloNumVarArray y(env, n, 0, 1, ILOBOOL);

    // Objective
    IloExpr expr(env);
    for (ItemIdx j=0; j<n; j++) {
        AgentIdx s = sol.agent(j);
        AgentIdx t = item_info[j].i_best_1;
        if (t == s)
            t = item_info[j].i_best_2;
        expr += ins.alternative(j, t).c
            + (ins.alternative(j, s).c - ins.alternative(j, t).c) * y[j];
    }
    IloObjective obj = IloMinimize(env, expr);
    model.add(obj);

    // Constraints
    for (AgentIdx i=0; i<m; i++) {
        IloExpr lhs(env);
        Weight lower = 0;
        for (ItemIdx j=0; j<n; j++) {
            AgentIdx s = sol.agent(j);
            AgentIdx t = item_info[j].i_best_1;
            if (t == s)
                t = item_info[j].i_best_2;
            if (s == i) {
                lhs += ins.alternative(j, s).w * y[j];
            } else if (t == i) {
                lhs -= ins.alternative(j, t).w * y[j];
                lower -= ins.alternative(j, t).w;
            }
        }
        //std::cout << "upper i " << i << " " << ins.capacity(i) + lower << std::endl;
        model.add(IloRange(env, lower, lhs, ins.capacity(i) + lower));
    }

    IloCplex cplex(model);

    // Initial solution
    IloNumVarArray startVar(env);
    IloNumArray startVal(env);
    for (ItemIdx j=0; j<n; ++j) {
        startVar.add(y[j]);
        startVal.add(1);
    }
    cplex.addMIPStart(startVar, startVal);
    startVal.end();
    startVar.end();

    // Display
    cplex.setOut(env.getNullStream());

    // Stop at first improvment
    cplex.setParam(IloCplex::IntSolLim, 2);
    // Time limit
    //cplex.setParam(IloCplex::TiLim, n * m / 1000);

    // Precision
    cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 0.0);

    // Optimize
    cplex.solve();

    //std::cout << "opt " << cplex.getObjValue() << std::endl;
    if (sol.cost() <= cplex.getObjValue() + 0.5)
        return false;

    for (ItemIdx j=0; j<n; j++) {
        if (cplex.getValue(y[j]) < 0.5) {
            AgentIdx t = item_info[j].i_best_1;
            if (t == sol.agent(j))
                t = item_info[j].i_best_2;
            sol.set(j, t);
        }
    }
    //std::cout
        //<< "c " << sol.cost()
        //<< " oc " << sol.overcapacity()
        //<< std::endl;

    env.end();
    return true;
}

Solution gap::sol_vlsn_mbp(const Instance& ins, Solution& sol_best, std::mt19937_64& gen, Info info)
{
    (void)gen;
    ItemIdx n = ins.item_number();

    init_display(info);

    //LinRelaxClpOutput linrelax_output = lb_linrelax_clp(ins);
    //Solution sol_curr = sol_repairlinrelax(ins, linrelax_output);
    Solution sol_curr = sol_random(ins, gen);
    LagRelaxKnapsackLbfgsOutput lagout = lb_lagrelax_knapsack_lbfgs(ins);

    std::stringstream ss;
    if (compare(sol_best, sol_curr))
        sol_best.update(sol_curr, lagout.lb, ss, info);

    std::vector<ItemInfo> item_info;
    for (ItemIdx j=0; j<n; ++j)
        item_info.push_back(ItemInfo(ins, j, lagout.multipliers));

    while (info.check_time() && move_mbp_cplex(ins, sol_curr, item_info, info))
        sol_best.update(sol_curr, lagout.lb, ss, info);

    return algorithm_end(sol_best, info);
}

