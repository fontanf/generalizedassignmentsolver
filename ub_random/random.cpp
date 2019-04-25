#include "gap/ub_random/random.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace gap;

Solution gap::sol_random(const Instance& ins, std::default_random_engine& gen, Info info)
{
    std::uniform_int_distribution<> dis(0, ins.agent_number() - 1);
    Solution sol(ins);
    for (ItemIdx j=0; j<ins.item_number(); ++j)
        sol.set(j, dis(gen));

    while (sol.feasible() > 0) {
        Weight wf_min = sol.feasible();
        ItemIdx j_best = -1;
        ItemIdx i_best = -1;
        for (ItemIdx j=0; j<ins.item_number(); ++j) {
            AgentIdx i_old = sol.agent(j);
            for (AgentIdx i=0; i<ins.agent_number(); ++i) {
                if (i == i_old)
                    continue;
                sol.set(j, i);
                if (wf_min > sol.feasible()) {
                    j_best = j;
                    i_best = i;
                    wf_min = sol.feasible();
                }
            }
            sol.set(j, i_old);
        }

        if (j_best == -1)
            return sol_random(ins, gen);
        sol.set(j_best, i_best);
    }

    return algorithm_end(sol, info);
}

