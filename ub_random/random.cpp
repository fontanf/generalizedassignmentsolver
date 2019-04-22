#include "gap/ub_random/random.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace gap;

bool improve(const Instance& ins, Solution& sol, std::vector<ItemIdx>& vec, Info& info)
{
    (void)info;
    std::random_shuffle(vec.begin(), vec.end());
    Weight wf = sol.feasible();
    for (ItemIdx j: vec) {
        AgentIdx i = sol.agent(j);
        if (sol.remaining_capacity(i) >= 0)
            continue;
        ItemIdx i_best = i;
        Weight wf_best = wf;
        for (AgentIdx i_new=0; i_new<ins.agent_number(); ++i_new) {
            if (i_new == i)
                continue;
            sol.set(j, i_new);
            if (sol.feasible() < wf_best) {
                i_best = i_new;
                wf_best = sol.feasible();
            }
        }
        sol.set(j, i_best);
        if (sol.feasible() < wf)
            return sol.feasible() != 0;
    }
    return false;
}

Solution gap::sol_random(const Instance& ins, Info info)
{
    std::default_random_engine gen(0);
    std::uniform_int_distribution<> dis(0, ins.agent_number() - 1);
    for (int s=0;; ++s) {
        Solution sol(ins);
        for (ItemIdx j=0; j<ins.item_number(); ++j) {
            if (s == 0) {
                AgentIdx i_min = 0;
                for (AgentIdx i=1; i<ins.agent_number(); ++i)
                    if (ins.alternative(j, i_min).w > ins.alternative(j, i).w)
                        i_min = i;
                sol.set(j, i_min);
            } else {
                sol.set(j, dis(gen));
            }
        }
        if (sol.feasible() == 0)
            return algorithm_end(sol, info);
        std::vector<ItemIdx> vec(ins.item_number());
        std::iota(vec.begin(), vec.end(), 0);
        while (improve(ins, sol, vec, info));
        if (sol.feasible() == 0)
            return algorithm_end(sol, info);
    }
    return algorithm_end(Solution(ins), info);
}

