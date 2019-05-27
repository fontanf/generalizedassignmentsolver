#include "gap/ub_ls_ejectionchains/ls_ejectionchains.hpp"
#include "gap/ub_random/random.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace gap;

void gap::sol_lsfirst_ejectionchain(LSFirstECData d, Solution& sol_best)
{
    ItemIdx n = d.ins.item_number();
    AgentIdx m = d.ins.agent_number();

    //sol_lsfirst_shiftswap(d, sol_best);
    //sol_lsfirst_doubleshift(d, sol_best);

    std::vector<std::pair<ItemIdx, ItemIdx>> moves;
    std::vector<ItemIdx> items(n);
    std::iota(items.begin(), items.end(), 0);
    sol_best.update_penalties(std::vector<PCost>(m, d.alpha));

    for (Cpt it=0; d.info.check_time(); ++it) {
        std::shuffle(items.begin(), items.end(), d.gen);
        bool improved = false;
        for (ItemIdx j0: items) {
            std::vector<bool> used(n, false);
            Solution sol_curr = sol_best;
            ItemIdx i0 = sol_curr.agent(j0);
            std::vector<ItemIdx> j {j0};
            std::vector<AgentIdx> i {i0};
            sol_curr.set(j0, -1);
            used[j0] = true;
            //std::cout << "(" << j0 << "," << i0 << ")";
            for (;;) {
                //std::cout << " " << sol_curr.pcost();
                ItemIdx j_best = -1;
                AgentIdx i_best = -1;
                PCost v_best = -1;
                for (ItemIdx j_suiv: items) {
                    if (used[j_suiv])
                        continue;
                    AgentIdx i_suiv = sol_curr.agent(j_suiv);
                    if (i_suiv == i.back())
                        continue;
                    sol_curr.set(j_suiv, i.back());
                    if (j_best == -1 || v_best > sol_curr.pcost()) {
                        j_best = j_suiv;
                        i_best = i_suiv;
                        v_best = sol_curr.pcost();
                    }
                    sol_curr.set(j_suiv, i_suiv);
                }
                //std::cout << " (" << j_best << "," << i_best << ")";
                if (j_best == -1) {
                    //std::cout << std::endl;
                    break;
                }
                sol_curr.set(j_best, i.back());
                used[j_best] = true;
                j.push_back(j_best);
                i.push_back(i_best);
                for (AgentIdx i=0; i<m; ++i) {
                sol_curr.set(j0, i);
                if (sol_curr.feasible() && ( !sol_best.feasible() || sol_best.cost() > sol_curr.cost())) {
                    //std::cout << std::endl;
                    std::stringstream ss;
                    ss << "ec " << j.size();
                    if (j.size() < 8) {
                        ss << ":";
                        for (ItemIdx j1: j)
                            ss << " " << j1;
                    }
                    sol_best.update(sol_curr, 0, ss, d.info);
                    improved = true;
                    break;
                }
                if (improved)
                    break;
                }
                sol_curr.set(j0, -1);
                if (sol_curr.pcost() > sol_best.pcost()) {
                    //std::cout << std::endl;
                    break;
                }
            }
            if (improved)
                break;
        }
        if (!improved)
            break;
    }
}

/*
Solution gap::sol_lsfirst_ejectionchain(LSFirstShiftSwapData d)
{
    init_display(d.info);
    Solution sol = random_solution(d.ins, d.gen);
    sol_lsfirst_ejectionchain(d, sol);
    return algorithm_end(sol, d.info);
}
*/

