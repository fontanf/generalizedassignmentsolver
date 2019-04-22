#include "gap/ub_lssimple/lssimple.hpp"
#include "gap/ub_random/random.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace gap;

Solution gap::sol_lssimple(const Instance& ins, Info info)
{
    Solution sol = sol_random(ins);
    Value lb = 0;
    for (ItemIdx j=0; j<ins.item_number(); ++j)
        lb += ins.item(j).v_min;

    Solution sol_best = sol;
    init_display(sol_best, lb, info);
    std::default_random_engine gen(0);
    std::uniform_int_distribution<> dis_j(0, ins.item_number() - 1);
    std::uniform_int_distribution<> dis_i(0, ins.agent_number() - 1);
    for (Cpt it=0, it_without_improvment=0; info.check_time(); ++it, ++it_without_improvment) {

        // Shift
        ItemIdx j = dis_j(gen);
        AgentIdx i_old = sol.agent(j);
        AgentIdx i_new = sol.agent(j);
        while (i_new == sol.agent(j))
            i_new = dis_i(gen);
        Value v = sol.value();
        sol.set(j, i_new);
        if (sol.feasible() > 0 || v < sol.value())
            sol.set(j, i_old);

        // Swap
        ItemIdx j1 = dis_j(gen);
        ItemIdx j2 = j1;
        AgentIdx i1 = sol.agent(j1);
        AgentIdx i2 = sol.agent(j2);
        while (i2 == i1) {
            j2 = dis_j(gen);
            i2 = sol.agent(j2);
        }
        v = sol.value();
        sol.set(j1, i2);
        sol.set(j2, i1);
        if (sol.feasible() > 0 || v < sol.value()) {
            sol.set(j1, i1);
            sol.set(j2, i2);
        }

        if (it % 1000000 == 0 && sol_best.value() > sol.value()) {
            it_without_improvment = 0;
            std::stringstream ss;
            ss << "it " << it;
            sol_best.update(sol, lb, ss, info);
        }

        if (it_without_improvment > 4 * ins.item_number() * ins.item_number()
                && sol_best.value() <= sol.value())
            break;
    }
    return algorithm_end(sol, info);
}

