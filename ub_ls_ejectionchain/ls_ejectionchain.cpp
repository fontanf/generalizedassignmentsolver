#include "gap/ub_ls_ejectionchain/ls_ejectionchain.hpp"

#include "gap/ub_ls_shiftswap/ls_shiftswap.hpp"
#include "gap/ub_random/random.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace gap;

std::vector<ItemIdx> gap::ejectionchain_iter(Solution& sol_best, std::vector<ItemIdx>& items, std::mt19937_64& gen, std::stringstream& ss)
{
    ItemIdx n = sol_best.instance().item_number();
    AgentIdx m = sol_best.instance().agent_number();
    for (ItemPos j_pos=0; j_pos<n; ++j_pos) {
        std::uniform_int_distribution<Cpt> dis(j_pos, n - 1);
        Cpt move_idx = dis(gen);
        iter_swap(items.begin() + j_pos, items.begin() + move_idx);
        ItemPos j0 = items[j_pos];

        std::vector<AgentIdx> used(n, -1);
        Solution sol_curr = sol_best;
        ItemIdx i_last = sol_curr.agent(j0);
        used[j0] = i_last;
        std::vector<ItemIdx> chain {j0};
        sol_curr.set(j0, -1);
        //std::cout <<  "* j_pos " << j_pos << " (" << j0 << "," << i_last << ")";
        for (;;) {
            //std::cout << " " << sol_curr.pcost();
            ItemIdx j_best = -1;
            AgentIdx i_best = -1;
            PCost v_best = -1;
            for (ItemIdx j_suiv: items) {
                if (used[j_suiv] >= 0)
                    continue;
                AgentIdx i_suiv = sol_curr.agent(j_suiv);
                if (i_suiv == i_last)
                    continue;
                sol_curr.set(j_suiv, i_last);
                if (j_best == -1 || v_best > sol_curr.pcost()) {
                    j_best = j_suiv;
                    i_best = i_suiv;
                    v_best = sol_curr.pcost();
                }

                sol_curr.set(j0, i_suiv);
                if (compare(sol_best, sol_curr)) {
                    sol_best = sol_curr;
                    ss << "ec " << chain.size();
                    if (chain.size() <= 8) {
                        ss << ":";
                        for (ItemIdx j: chain)
                            ss << " " << j;
                    }
                    return chain;
                }
                sol_curr.set(j0, -1);

                sol_curr.set(j_suiv, i_suiv);
            }
            if (j_best == -1) {
                //std::cout << " j_best -1" << std::endl;
                break;
            }
            sol_curr.set(j_best, i_last);
            used[j_best] = i_last;
            chain.push_back(j_best);
            i_last = i_best;
            for (AgentIdx i=0; i<m; ++i) {
                sol_curr.set(j0, i);
                if (compare(sol_best, sol_curr)) {
                    sol_best = sol_curr;
                    ss << "ec " << chain.size();
                    if (chain.size() <= 8) {
                        ss << ":";
                        for (ItemIdx j: chain)
                            ss << " " << j;
                    }
                    return chain;
                }
            }
            sol_curr.set(j0, -1);
            if (sol_curr.pcost() > sol_best.pcost()) {
                //std::cout << " too high " << sol_curr.pcost() << std::endl;
                break;
            }
        }
    }
    return {};
}

void gap::sol_lsfirst_ejectionchain(LSFirstECData d, Solution& sol_best)
{
    ItemIdx n = d.ins.item_number();
    AgentIdx m = d.ins.agent_number();
    Solution sol_curr = sol_best;
    sol_curr.update_penalties(std::vector<PCost>(m, d.alpha));
    std::vector<ItemIdx> items(n);
    std::iota(items.begin(), items.end(), 0);
    auto moves_ss = moves_shiftswap(d.ins);
    for (; d.info.check_time();) {
        std::stringstream ss;
        if (shiftswap_iter(sol_curr, moves_ss, d.gen, ss)) {
        } else if (ejectionchain_iter(sol_curr, items, d.gen, ss).size() > 0) {
        } else {
            break;
        }
        if (sol_curr.feasible() && (!sol_best.feasible() || sol_best.cost() > sol_curr.cost())) {
            sol_best.update(sol_curr, 0, ss, d.info);
        }
    }
}

Solution gap::sol_lsfirst_ejectionchain(LSFirstECData d)
{
    init_display(d.info);
    Solution sol = sol_random_infeasible(d.ins, d.gen);
    sol_lsfirst_ejectionchain(d, sol);
    return algorithm_end(sol, d.info);
}

