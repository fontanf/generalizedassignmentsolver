#include "gap/ub_ls_shiftswap/ls_shiftswap.hpp"
#include "gap/ub_random/random.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace gap;

Solution gap::sol_ls_shiftswap(LSShiftSwapData d)
{
    init_display(d.info);
    Solution sol_best(d.ins);
    AgentIdx m = d.ins.agent_number();
    ItemIdx n = d.ins.item_number();
    std::uniform_int_distribution<Cpt> dis_ss(1, n * m + (n * (n + 1)) / 2);
    std::uniform_int_distribution<ItemIdx> dis_j(0, n - 1);
    std::uniform_int_distribution<ItemIdx> dis_j2(0, n - 2);
    std::uniform_int_distribution<AgentIdx> dis_i(0, m - 2);

    Solution sol_curr = sol_random(d.ins, d.gen);

    Cost v_curr = sol_curr.cost();
    for (; d.info.check_time();) {
        Cpt p = dis_ss(d.gen);
        if (p <= m * n) { // shift
            ItemIdx j = dis_j(d.gen);
            AgentIdx i = dis_i(d.gen);
            AgentIdx i_old = sol_curr.agent(j);
            if (i >= i_old)
                i++;
            sol_curr.set(j, i);
            if (!sol_curr.feasible() || v_curr < sol_curr.cost()) {
                sol_curr.set(j, i_old);
            }
        } else { // swap
            ItemIdx j1 = dis_j(d.gen);
            AgentIdx i1 = sol_curr.agent(j1);
            ItemIdx j2 = dis_j2(d.gen);
            if (j2 >= j1)
                j2++;
            AgentIdx i2 = sol_curr.agent(j2);
            if (i1 == i2)
                continue;
            sol_curr.set(j1, i2);
            sol_curr.set(j2, i1);
            if (!sol_curr.feasible() || v_curr < sol_curr.cost()) {
                sol_curr.set(j1, i1);
                sol_curr.set(j2, i2);
            }
        }
        if (compare(sol_best, sol_curr)) {
            std::stringstream ss;
            sol_best.update(sol_curr, 0, ss, d.info);
        }
    }
    return algorithm_end(sol_best, d.info);
}

/******************************************************************************/

Solution gap::sol_ts_shiftswap(TSShiftSwapData d)
{
    init_display(d.info);
    Solution sol_best(d.ins);
    AgentIdx m = d.ins.agent_number();
    ItemIdx n = d.ins.item_number();
    std::uniform_int_distribution<Cpt> dis_ss(1, n * m + (n * (n + 1)) / 2);
    std::uniform_int_distribution<ItemIdx> dis_j(0, n - 1);
    std::uniform_int_distribution<ItemIdx> dis_j2(0, n - 2);
    std::uniform_int_distribution<AgentIdx> dis_i(0, m - 2);

    Solution sol_curr = sol_random(d.ins, d.gen);
    Cpt l = std::min(d.l, n * m + n * (n + 1) / 4);

    Cost v_curr = sol_curr.cost();
    Cost v_best = -1;
    ItemIdx j_best = -1;
    AgentIdx i_best = -1;
    ItemIdx j1_best = -1;
    ItemIdx j2_best = -1;
    for (Cpt it=0; d.info.check_time();) {
        Cpt p = dis_ss(d.gen);
        if (p <= m * n) { // shift
            ItemIdx j = dis_j(d.gen);
            AgentIdx i = dis_i(d.gen);
            AgentIdx i_old = sol_curr.agent(j);
            if (i >= i_old)
                i++;
            sol_curr.set(j, i);
            if (sol_curr.feasible()) {
                if (compare(sol_best, sol_curr)) {
                    std::stringstream ss;
                    sol_best.update(sol_curr, 0, ss, d.info);
                }
                if (v_curr > sol_curr.cost()) {
                    v_curr = sol_curr.cost();
                    it = 0;
                    v_best = -1;
                    j_best = -1;
                    i_best = -1;
                    j1_best = -1;
                    j2_best = -1;
                    continue;
                }
                if (v_best == -1 || v_best > sol_curr.cost()) {
                    v_best = sol_curr.cost();
                    j_best = j;
                    i_best = i;
                    j1_best = -1;
                    j2_best = -1;
                }
            }
            sol_curr.set(j, i_old);
        } else { // swap
            ItemIdx j1 = dis_j(d.gen);
            AgentIdx i1 = sol_curr.agent(j1);
            ItemIdx j2 = dis_j2(d.gen);
            if (j2 >= j1)
                j2++;
            AgentIdx i2 = sol_curr.agent(j2);
            if (i1 == i2)
                continue;
            sol_curr.set(j1, i2);
            sol_curr.set(j2, i1);

            if (sol_curr.feasible()) {
                if (compare(sol_best, sol_curr)) {
                    std::stringstream ss;
                    sol_best.update(sol_curr, 0, ss, d.info);
                }
                if (v_curr > sol_curr.cost()) {
                    v_curr = sol_curr.cost();
                    it = 0;
                    v_best = -1;
                    j_best = -1;
                    i_best = -1;
                    j1_best = -1;
                    j2_best = -1;
                    continue;
                }
                if (v_best == -1 || v_best > sol_curr.cost()) {
                    v_best = sol_curr.cost();
                    j_best = -1;
                    i_best = -1;
                    j1_best = j1;
                    j2_best = j2;
                }
            }

            sol_curr.set(j1, i1);
            sol_curr.set(j2, i2);
        }

        ++it;
        if (it >= l) {
            if (j_best != -1) {
                sol_curr.set(j_best, i_best);
            } else {
                AgentIdx i1 = sol_curr.agent(j1_best);
                AgentIdx i2 = sol_curr.agent(j2_best);
                sol_curr.set(j1_best, i2);
                sol_curr.set(j2_best, i1);
            }
            it = 0;
            v_best = -1;
            j_best = -1;
            i_best = -1;
            j1_best = -1;
            j2_best = -1;
            v_curr = sol_curr.cost();
        }
    }
    return algorithm_end(sol_best, d.info);
}

/******************************************************************************/

Solution gap::sol_sa_shiftswap(SAShiftSwapData d)
{
    init_display(d.info);
    Solution sol_best(d.ins);
    AgentIdx m = d.ins.agent_number();
    ItemIdx n = d.ins.item_number();
    std::uniform_int_distribution<Cpt> dis_ss(1, n * m + (n * (n + 1)) / 2);
    std::uniform_int_distribution<ItemIdx> dis_j(0, n - 1);
    std::uniform_int_distribution<ItemIdx> dis_j2(0, n - 2);
    std::uniform_int_distribution<AgentIdx> dis_i(0, m - 2);
    std::uniform_real_distribution<double> dis(0, 1);

    Solution sol_curr = sol_random(d.ins, d.gen);

    // Compute initial temperature
    double t0 = 0;
    for (ItemIdx j=0; j<n; ++j)
        for (AgentIdx i=0; i<m; ++i)
            if (t0 < d.ins.alternative(j, i).c)
                t0 = d.ins.alternative(j, i).c;
    t0 /= 100;

    Cpt it_max = 2 * (n * m + (n * (n + 1)) / 2);
    Cpt it_without_change = 0;
    for (double t=t0; d.info.check_time(); t*=d.beta) {
        for (Cpt it=0; it<d.l && d.info.check_time();) {
            Cost v = sol_curr.cost();
            Cpt p = dis_ss(d.gen);
            if (p <= m * n) { // shift
                ItemIdx j = dis_j(d.gen);
                AgentIdx i = dis_i(d.gen);
                AgentIdx i_old = sol_curr.agent(j);
                if (i >= i_old)
                    i++;
                sol_curr.set(j, i);
                if (!sol_curr.feasible() || (v < sol_curr.cost()
                            && dis(d.gen) > exp(((double)v - sol_curr.cost()) / t))) {
                    sol_curr.set(j, i_old);
                }
            } else { // swap
                ItemIdx j1 = dis_j(d.gen);
                AgentIdx i1 = sol_curr.agent(j1);
                ItemIdx j2 = dis_j2(d.gen);
                if (j2 >= j1)
                    j2++;
                AgentIdx i2 = sol_curr.agent(j2);
                if (i1 == i2)
                    continue;
                sol_curr.set(j1, i2);
                sol_curr.set(j2, i1);
                if (!sol_curr.feasible() || (v < sol_curr.cost()
                            && dis(d.gen) > exp(((double)v - sol_curr.cost()) / t))) {
                    sol_curr.set(j1, i1);
                    sol_curr.set(j2, i2);
                }
            }

            // Update it_without_change
            if (sol_curr.cost() == v) {
                it_without_change++;
                if (it_without_change > it_max)
                    return algorithm_end(sol_best, d.info);
            } else {
                it_without_change = 0;
            }

            // Update best solution
            if (compare(sol_best, sol_curr)) {
                std::stringstream ss;
                ss << "T " << t;
                sol_best.update(sol_curr, 0, ss, d.info);
                it = 0;
            }

            ++it;
        }
    }
    return algorithm_end(sol_best, d.info);
}

