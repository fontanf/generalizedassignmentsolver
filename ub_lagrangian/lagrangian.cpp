#include "gap/ub_lagrangian/lagrangian.hpp"

#include "knapsack/opt_minknap/minknap.hpp"

#include <algorithm>

using namespace gap;

#define DBG(x)
//#define DBG(x) x

LagOut gap::ub_lagrangian(const SubInstance& sub,
        StateIdx it_total, StateIdx a, std::vector<Value>* mult_init, Info* info)
{
    DBG(std::cout << "LAGRANGIAN..." << std::endl;)
    (void)info;
    const Instance& ins = sub.instance();
    assert(ins.objective() == 1);
    LagOut out(ins);
    std::vector<Value> multipliers_curr(ins.item_number(), 0);
    if (mult_init != NULL)
        multipliers_curr = *mult_init;
    for (ItemIdx j=0; j<ins.item_number(); ++j)
        if (sub.reduced_solution()->agent(j) >= 0)
            multipliers_curr[j] = 0;

    Value gamma = 0;
    for (AltIdx k=0; k<ins.alternative_number(); ++k)
        if (gamma < ins.alternative(k).p)
            gamma = ins.alternative(k).p;
    gamma *= 4;
    DBG(std::cout << "GAMMA " << gamma << std::endl;)

    StateIdx it = 1;
    for (; it<it_total && gamma/(it/a+1) >= 1; ++it) {
        DBG(std::cout << "ITERATION " << it << std::endl;)
        DBG(std::cout << "MULT" << std::flush;
        for (Value m: multipliers_curr)
            std::cout << " " << m << std::flush;
        std::cout << std::endl;)

        std::vector<std::vector<int>> xji(ins.item_number(),
                std::vector<int>(ins.agent_number(), 0));
        std::vector<AgentIdx> xj(ins.item_number(), 0);

        Value ub = 0;
        for (ItemIdx j=0; j<ins.item_number(); ++j)
            ub += multipliers_curr[j];

        for (AgentIdx i=0; i<ins.agent_number(); ++i) {
            DBG(std::cout << "AGENT " << i << std::flush;)
            knapsack::Instance ins_kp(sub.agent_alternative_number(i), sub.capacity(i));
            for (ItemIdx j=0; j<ins.item_number(); ++j) {
                AltIdx k = ins.alternative_index(j, i);
                if (sub.reduced_solution()->agent(j) == i) {
                    xj[j]++;
                    xji[j][i] = 1;
                    ub += ins.alternative(k).v;
                }
                if (sub.reduced(k) == -1)
                    continue;
                Value p = ins.alternative(k).v - multipliers_curr[j];
                if (p > 0)
                    ins_kp.add_item(ins.alternative(k).w, p, j);
            }
            knapsack::MinknapParams params;
            knapsack::Solution sol = knapsack::sopt_minknap_list_part(ins_kp, params);
            DBG(std::cout << " KPOPT " << sol.value() << std::endl;)
            ub += sol.profit();
            for (knapsack::ItemPos j_kp=0; j_kp<ins_kp.total_item_number(); ++j_kp) {
                ItemIdx j_gap = ins_kp.item(j_kp).l;
                DBG(std::cout << "JKP " << j_kp << " JGAP " << j_gap << " " << sol.contains_idx(j_kp) << std::endl;)
                if (sol.contains(j_kp)) {
                    xji[j_gap][i] = 1;
                    xj[j_gap]++;
                }
            }
        }

        DBG(std::cout << "UB " << ub << std::flush;
        std::cout << " X" << std::flush;
        for (AgentIdx x: xj)
            std::cout << " " << x << std::flush;
        std::cout << std::endl;)

        if (out.u == -1 || ub < out.u) {
            if (Info::verbose(info))
                std::cout << "NEW UB " << ub << " IT " << it << std::endl;
            out.u    = ub;
            out.mult = multipliers_curr;
            out.xj   = xj;
            out.xji  = xji;
        }
        if (out.u < 0) {
            DBG(std::cout << "LAGRANGIAN... END <= 0" << std::endl;)
            out.u = -1;
            return out;
        }

        // Update multipliers
        for (ItemIdx j=0; j<ins.item_number(); ++j) {
            DBG(std::cout << multipliers_curr[j] << " " << xj[j];)
            if (xj[j] < 1) { // <=> x[j] == 0
                multipliers_curr[j] -= gamma / (it/a+1);
            } else if (xj[j] > 1) {
                multipliers_curr[j] += (gamma * (xj[j]-1)) / (it/a+1);
            }
            DBG(std::cout << " => " << multipliers_curr[j] << std::endl;)
        }
    }

    if (Info::verbose(info))
        std::cout << "TOTAL IT " << it << std::endl;

    DBG(std::cout << "LAGRANGIAN... END" << std::endl;)
    return out;
}

#undef DBG
