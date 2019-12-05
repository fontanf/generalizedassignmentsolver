#include "generalizedassignment/ub_random/random.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace generalizedassignment;

Solution generalizedassignment::sol_random_infeasible(const Instance& ins, std::mt19937_64& gen)
{
    std::uniform_int_distribution<> dis(0, ins.agent_number() - 1);
    Solution sol(ins);
    for (ItemIdx j=0; j<ins.item_number(); ++j)
        sol.set(j, dis(gen));
    return sol;
}

Output generalizedassignment::sol_random(const Instance& ins, std::mt19937_64& gen, Info info)
{
    VER(info, "*** random ***" << std::endl);
    Output output(ins, info);

    Solution sol = sol_random_infeasible(ins, gen);
    AgentIdx m = ins.agent_number();
    ItemIdx n = ins.item_number();
    std::uniform_int_distribution<Cpt> dis_ss(1, n * m + (n * (n + 1)) / 2);
    std::uniform_int_distribution<ItemIdx> dis_j(0, n - 1);
    std::uniform_int_distribution<ItemIdx> dis_j2(0, n - 2);
    std::uniform_int_distribution<AgentIdx> dis_i(0, m - 2);
    std::uniform_real_distribution<double> dis(0, 1);

    Cpt it_max = 10000;
    Cpt it_without_change = 0;

    while (it_without_change < it_max && info.check_time()) {
        Weight wf = sol.overcapacity();
        Cpt p = dis_ss(gen);
        if (p <= m * n) { // shift
            ItemIdx j = dis_j(gen);
            AgentIdx i = dis_i(gen);
            AgentIdx i_old = sol.agent(j);
            if (i >= i_old)
                i++;
            sol.set(j, i);
            if (sol.overcapacity() > wf) {
                sol.set(j, i_old);
                it_without_change++;
            } else {
                it_without_change = 0;
            }
        } else { // swap
            ItemIdx j1 = dis_j(gen);
            ItemIdx j2 = dis_j2(gen);
            if (j2 >= j1)
                j2++;
            AgentIdx i1 = sol.agent(j1);
            AgentIdx i2 = sol.agent(j2);
            if (i1 == i2)
                continue;
            sol.set(j1, i2);
            sol.set(j2, i1);
            if (sol.overcapacity() > wf) {
                sol.set(j1, i1);
                sol.set(j2, i2);
                it_without_change++;
            } else {
                it_without_change = 0;
            }
        }
        if (sol.overcapacity() == 0) {
            output.update_solution(sol, std::stringstream(""), info);
            return output.algorithm_end(info);
        }
    }

    return output.algorithm_end(info);
}

