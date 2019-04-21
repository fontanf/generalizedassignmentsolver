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
    double t = 0;
    std::uniform_int_distribution<> dis_j(0, ins.item_number() - 1);
    std::uniform_int_distribution<> dis_i(0, ins.agent_number() - 1);
    for (Cpt it=0; info.check_time(); ++it) {
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

        if (info.elapsed_time() > t + 10 && sol_best.value() > sol.value()) {
            t = info.elapsed_time();
            std::stringstream ss;
            ss << "it " << it << " (ub)";
            sol_best.update(sol, lb, ss, info);
        }
    }
    return algorithm_end(sol, info);
}

