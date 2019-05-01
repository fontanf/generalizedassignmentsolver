#include "gap/ub_localsearch/localsearch.hpp"
#include "gap/ub_random/random.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace gap;

bool gap::shift_swap_first(Solution& sol, std::vector<std::pair<ItemIdx, ItemIdx>>& alt,
        std::default_random_engine& gen, Info& info)
{
    std::shuffle(alt.begin(), alt.end(), gen);
    Value v = sol.value();
    for (std::pair<ItemIdx, ItemIdx> p: alt) {
        if (p.second < 0) { // shift
            ItemIdx j = p.first;
            AgentIdx i_old = sol.agent(j);
            AgentIdx i = - p.second - 1;
            sol.set(j, i);
            if (sol.feasible() == 0 && v > sol.value()) {
                std::stringstream ss;
                ss << "shift j " << j << " i " << i;
                sol.update(sol, 0, ss, info);
                return true;
            }
            sol.set(j, i_old);
        } else { // swap
            ItemIdx j1 = p.first;
            ItemIdx j2 = p.second;
            AgentIdx i1 = sol.agent(j1);
            AgentIdx i2 = sol.agent(j2);
            sol.set(j1, i2);
            sol.set(j2, i1);
            if (sol.feasible() == 0 && v > sol.value()) {
                std::stringstream ss;
                ss << "swap j1 " << j1 << " j2 " << j2;
                sol.update(sol, 0, ss, info);
                return true;
            }
            sol.set(j1, i1);
            sol.set(j2, i2);
        }
    }
    return false;
}

Solution gap::sol_localsearch_first(const Instance& ins, Solution& sol, std::default_random_engine& gen, Info info)
{
    if (!sol.is_complete() || sol.feasible() > 0)
        sol = sol_random(ins, gen);

    Cpt k = 0;
    std::vector<std::pair<ItemIdx, ItemIdx>> alt(
            ins.item_number() * (ins.item_number() + 1) / 2 + ins.item_number() * ins.agent_number());
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        for (AgentIdx i=0; i<ins.agent_number(); ++i)
            alt[k++] = {j, - i - 1};
        for (ItemIdx j2=j+1; j2<ins.item_number(); ++j2)
            alt[k++] = {j, j2};
    }
    init_display(sol, 0, info);
    while (shift_swap_first(sol, alt, gen, info));
    return algorithm_end(sol, info);
}

/******************************************************************************/

bool gap::shift_swap_best(const Instance& ins, Solution& sol, Info& info)
{
    Value v_best = sol.value();

    ItemIdx j_best = -1;
    AgentIdx i_best = -1;
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        AgentIdx i_old = sol.agent(j);
        for (AgentIdx i=0; i<ins.agent_number(); ++i) {
            if (i == i_old)
                continue;
            sol.set(j, i);
            if (sol.feasible() == 0 && v_best > sol.value()) {
                v_best = sol.value();
                j_best = j;
                i_best = i;
            }
        }
        sol.set(j, i_old);
    }

    ItemIdx j1_best = -1;
    ItemIdx j2_best = -1;
    for (ItemIdx j1=0; j1<ins.item_number(); ++j1) {
        AgentIdx i1 = sol.agent(j1);
        for (ItemIdx j2=j1+1; j2<ins.item_number(); ++j2) {
            AgentIdx i2 = sol.agent(j2);
            sol.set(j1, i2);
            sol.set(j2, i1);
            if (sol.feasible() == 0 && v_best > sol.value()) {
                v_best = sol.value();
                j_best = -1;
                i_best = -1;
                j1_best = j1;
                j2_best = j2;
            }
            sol.set(j2, i2);
        }
        sol.set(j1, i1);
    }

    if (j_best == -1 && j1_best == -1)
        return false;

    std::stringstream ss;
    if (j1_best != -1) {
        AgentIdx i1 = sol.agent(j1_best);
        AgentIdx i2 = sol.agent(j2_best);
        sol.set(j1_best, i2);
        sol.set(j2_best, i1);
        ss << "swap j1 " << j1_best << " j2 " << j2_best;
    } else {
        sol.set(j_best, i_best);
        ss << "shift j " << j_best << " i " << i_best;
    }
    sol.update(sol, 0, ss, info);
    return true;
}

Solution gap::sol_localsearch_best(const Instance& ins, Solution& sol, std::default_random_engine& gen, Info info)
{
    if (!sol.is_complete() || sol.feasible() > 0)
        sol = sol_random(ins, gen);
    init_display(sol, 0, info);
    while (shift_swap_best(ins, sol, info));
    return algorithm_end(sol, info);
}

