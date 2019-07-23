#if CPLEX_FOUND
#if DLIB_FOUND

#include "gap/ub_vlsn_mbp/vlsn_mbp.hpp"

#include "gap/lb_lagrelax_lbfgs/lagrelax_lbfgs.hpp"
#include "gap/ub_random/random.hpp"
#include "gap/ub_repair/repair.hpp"
#include "gap/ub_ls_shiftswap/ls_shiftswap.hpp"

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
        c.resize(m);
        for (AgentIdx i=0; i<m; ++i) {
            c[i] = ins.alternative(j, i).c - lambda[i] * ins.alternative(j, i).w;
            if (i_best_1 == -1 || c_best_1 > c[i]) {
                i_best_2 = i_best_1;
                c_best_2 = c_best_1;
                i_best_1 = i;
                c_best_1 = c[i];
            } else if (i_best_2 == -1 || c_best_2 > c[i]) {
                i_best_2 = i;
                c_best_2 = c[i];
            }
            if (i_worst == -1 || c_worst < c[i]) {
                i_worst = i;
                c_worst = c[i];
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
    AgentIdx i_worst  = -1;
    PCost    c_worst  = -1;
    std::vector<PCost> c;
};

bool move_mbp_cplex(const Instance& ins, Solution& sol,
        const std::vector<ItemInfo>& item_info,
        std::mt19937_64& gen,
        Info& info)
{
    (void)info;
    ItemIdx n = ins.item_number();
    AgentIdx m = ins.agent_number();
    IloEnv env;
    IloModel model(env);

    std::vector<AgentIdx> s(n);
    std::vector<AgentIdx> t(n);
    for (ItemIdx j=0; j<n; j++) {
        s[j] = sol.agent(j);

        t[j] = item_info[j].i_best_1;
        if (t[j] == s[j])
            t[j] = item_info[j].i_best_2;

        //std::vector<PCost> w(m);
        //for (AgentIdx i=0; i<m; ++i) {
            //w[i] = (i == sol.agent(j))? 0:
                //pow((item_info[j].c_worst - item_info[j].c[i] + 1), 8);
        //}
        //std::discrete_distribution<AgentIdx> dis_i(w.begin(), w.end());
        //t[j] = dis_i(gen);
    }

    // Variables
    IloNumVarArray y(env, n, 0, 1, ILOBOOL);

    // Objective
    IloExpr expr(env);
    for (ItemIdx j=0; j<n; j++) {
        expr += ins.alternative(j, t[j]).c
            + (ins.alternative(j, s[j]).c - ins.alternative(j, t[j]).c) * y[j];
    }
    IloObjective obj = IloMinimize(env, expr);
    model.add(obj);

    // Constraints
    for (AgentIdx i=0; i<m; i++) {
        IloExpr lhs(env);
        Weight lower = 0;
        for (ItemIdx j=0; j<n; j++) {
            if (s[j] == i) {
                lhs += ins.alternative(j, s[j]).w * y[j];
            } else if (t[j] == i) {
                lhs -= ins.alternative(j, t[j]).w * y[j];
                lower -= ins.alternative(j, t[j]).w;
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
    //cplex.setParam(IloCplex::IntSolLim, 2);
    // Time limit
    cplex.setParam(IloCplex::TiLim, n * m / 400);

    // Precision
    cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 0.0);

    // Optimize
    cplex.solve();

    //std::cout << "opt " << cplex.getObjValue() << std::endl;
    if (sol.cost() <= cplex.getObjValue() + 0.5)
        return false;

    for (ItemIdx j=0; j<n; j++) {
        if (cplex.getValue(y[j]) < 0.5) {
            sol.set(j, t[j]);
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
    AgentIdx m = ins.agent_number();

    init_display(info);

    //LinRelaxClpOutput linrelax_output = lb_linrelax_clp(ins);
    //Solution sol_curr = sol_repairlinrelax(ins, linrelax_output);
    Solution sol_curr = sol_random(ins, gen);
    sol_curr.update_penalties(std::vector<PCost>(m, 100));
    LagRelaxKnapsackLbfgsOutput lagout = lb_lagrelax_knapsack_lbfgs(ins);

    std::stringstream ss;
    if (compare(sol_best, sol_curr))
        sol_best.update(sol_curr, lagout.lb, ss, info);

    std::vector<ItemInfo> item_info;
    for (ItemIdx j=0; j<n; ++j)
        item_info.push_back(ItemInfo(ins, j, lagout.multipliers));

    auto moves = moves_shiftswap(ins);
    for (; info.check_time();) {
        std::stringstream ss;
        if (shiftswap_iter(sol_curr, moves, gen, ss)) {
        } else if (move_mbp_cplex(ins, sol_curr, item_info, gen, info)) {
        } else {
            break;
        }
        sol_best.update(sol_curr, lagout.lb, ss, info);
    }

    return algorithm_end(sol_best, info);
}

#endif
#endif

