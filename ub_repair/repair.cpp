#if COINOR_FOUND

#include "gap/ub_repair/repair.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace gap;

void repair(Solution& sol_curr)
{
    const Instance& ins = sol_curr.instance();
    AgentIdx m = ins.agent_number();
    ItemIdx n = ins.item_number();

    while (sol_curr.overcapacity() > 0) {
        //std::cout << "cost " << sol_curr.cost() << " oc " << sol_curr.overcapacity() << std::endl;
        Weight oc = sol_curr.overcapacity();
        Cost c = sol_curr.cost();
        ItemIdx j1_best = -1;
        ItemIdx j2_best = -1;
        ItemIdx i1_best = -1;
        ItemIdx i2_best = -1;
        double v_best = -1;
        for (ItemIdx j1=0; j1<n; ++j1) {
            AgentIdx i1 = sol_curr.agent(j1);
            if (i1 == -1)
                continue;

            // Shift
            for (AgentIdx i=0; i<m; ++i) {
                if (i == i1)
                    continue;
                sol_curr.set(j1, i);
                if (sol_curr.overcapacity() < oc) {
                    double v = (double)(sol_curr.cost() - c) / (oc - sol_curr.overcapacity());
                    if (j1_best < 0 || v_best > v) {
                        v_best = v;
                        j1_best = j1;
                        i1_best = i;
                        j2_best = -1;
                    }
                }
            }

            // Swap
            for (ItemIdx j2=j1+1; j2<n; ++j2) {
                AgentIdx i2 = sol_curr.agent(j2);
                if (i2 == i1 || i2 == -1)
                    continue;
                sol_curr.set(j2, i1);
                sol_curr.set(j1, i2);
                if (sol_curr.overcapacity() < oc) {
                    double v = (double)(sol_curr.cost() - c) / (oc - sol_curr.overcapacity());
                    if (j1_best < 0 || v_best > v) {
                        v_best = v;
                        j1_best = j1;
                        j2_best = j2;
                        i1_best = i2;
                        i2_best = i1;
                    }
                }
                sol_curr.set(j2, i2);
            }

            sol_curr.set(j1, i1);
        }

        if (j1_best == -1)
            break;
        sol_curr.set(j1_best, i1_best);
        if (j2_best != -1)
            sol_curr.set(j2_best, i2_best);
    }
}

Solution gap::sol_repairgreedy(const Instance& ins, Info info)
{
    ItemIdx n = ins.item_number();

    // Initilize current solution
    Solution sol_curr(ins);
    for (ItemIdx j=0; j<n; ++j) {
        sol_curr.set(j, ins.item(j).i_best);
        repair(sol_curr);
    }
    return algorithm_end(sol_curr, info);
}

Solution gap::sol_repaircombrelax(const Instance& ins, Info info)
{
    ItemIdx n = ins.item_number();

    // Initilize current solution
    Solution sol_curr(ins);
    for (ItemIdx j=0; j<n; ++j)
        sol_curr.set(j, ins.item(j).i_best);

    repair(sol_curr);
    return algorithm_end(sol_curr, info);
}

Solution gap::sol_repairlinrelax(const Instance& ins, const LinRelaxClpOutput& linrelax_output, Info info)
{
    AgentIdx m = ins.agent_number();
    ItemIdx n = ins.item_number();

    // Initilize current solution
    Solution sol_curr(ins);
    for (ItemIdx j=0; j<n; ++j) {
        AgentIdx i_best = -1;
        Cost c_best = -1;
        for (AgentIdx i=0; i<m; ++i) {
            double x = linrelax_output.x.at(ins.alternative_index(j, i));
            Cost c = ins.alternative(j, i).c;
            if (x > 0 && (c_best == -1 || c_best > c)) {
                i_best = i;
                c_best = c;
            }
        }
        sol_curr.set(j, i_best);
    }

    repair(sol_curr);
    return algorithm_end(sol_curr, info);
}

#endif

