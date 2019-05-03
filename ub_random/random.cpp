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

    AgentIdx m = ins.agent_number();
    ItemIdx n = ins.item_number();
    std::uniform_int_distribution<Cpt> dis_ss(1, n * m + (n * (n + 1)) / 2);
    std::uniform_int_distribution<ItemIdx> dis_j(0, n - 1);
    std::uniform_int_distribution<ItemIdx> dis_j2(0, n - 2);
    std::uniform_int_distribution<AgentIdx> dis_i1(0, m - 2);
    std::uniform_int_distribution<AgentIdx> dis_i2(0, m - 3);

    Cpt it_max = 2 * (n * m + (n * (n + 1)) / 2);
    Cpt it_without_change = 0;

    while (it_without_change < it_max) {
        Weight wf = sol.overcapacity();
        Cpt p = dis_ss(gen);
        if (p <= m * n) { // shift
            ItemIdx j = dis_j(gen);
            AgentIdx i = dis_i1(gen);
            AgentIdx i_old = sol.agent(j);
            if (i >= i_old)
                i++;
            sol.set(j, i);
            if (sol.overcapacity() > wf) {
                sol.set(j, i_old);
            }
        } else { // swap
            ItemIdx j1 = dis_j(gen);
            ItemIdx j2 = dis_j2(gen);
            if (j2 >= j1)
                j2++;
            AgentIdx i1 = sol.agent(j1);
            AgentIdx i2 = sol.agent(j2);
            sol.set(j1, i2);
            sol.set(j2, i1);
            if (sol.overcapacity() > wf) {
                sol.set(j1, i1);
                sol.set(j2, i2);
            }
        }
        if (sol.overcapacity() == 0)
            return algorithm_end(sol, info);
    }

    return sol_random(ins, gen, info);
}

