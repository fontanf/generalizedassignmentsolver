#include "gap/ub_lssimple/lssimple.hpp"
#include "gap/ub_random/random.hpp"
#include "gap/opt_milp/milp.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace gap;

bool shift_swap(const Instance& ins, Solution& sol, Info& info)
{
    (void)info;
    ItemIdx j_best = -1;
    ItemIdx i_best = -1;
    Value vshift_min = sol.value();
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        AgentIdx i_old = sol.agent(j);
        for (AgentIdx i=0; i<ins.agent_number(); ++i) {
            if (i == i_old)
                continue;
            sol.set(j, i);
            if (sol.feasible() == 0 && vshift_min > sol.value()) {
                j_best = j;
                i_best = i;
                vshift_min = sol.value();
            }
        }
        sol.set(j, i_old);
    }

    ItemIdx j1_best = -1;
    ItemIdx j2_best = -1;
    Value vswap_min = sol.value();
    for (ItemIdx j1=0; j1<ins.item_number(); ++j1) {
        AgentIdx i1 = sol.agent(j1);
        for (ItemIdx j2=j1+1; j2<ins.item_number(); ++j2) {
            AgentIdx i2 = sol.agent(j2);
            sol.set(j1, i2);
            sol.set(j2, i1);
            if (sol.feasible() == 0 && vswap_min > sol.value()) {
                j1_best = j1;
                j2_best = j2;
                vswap_min = sol.value();
            }
            sol.set(j2, i2);
        }
        sol.set(j1, i1);
    }

    if (j_best == -1 && j2_best == -1) { // No improvment
        return false;
    } else if (j1_best == -1 || (j_best != -1 && vshift_min < vswap_min)) {
        sol.set(j_best, i_best);
    } else {
        AgentIdx i1 = sol.agent(j1_best);
        AgentIdx i2 = sol.agent(j2_best);
        sol.set(j1_best, i2);
        sol.set(j2_best, i1);
    }
    return true;
}

bool move_gap(const Instance& ins, Solution& sol, AgentIdx m, ItemIdx n,
        std::vector<ItemIdx>& items, std::vector<AgentIdx>& agents,
        Info& info)
{
    (void)info;
    if (n != ins.item_number())
        std::random_shuffle(items.begin(), items.end());
    if (m != ins.agent_number())
        std::random_shuffle(agents.begin(), agents.end());
    std::vector<Weight> c(m, 0);
    for (AgentIdx i=0; i<m; ++i)
        c[i] = ins.capacity(agents[i]);
    Instance ins_tmp(m, n);
    std::vector<AgentIdx> sol_vec;
    Value v = 0;
    std::vector<ItemIdx> pos;
    for (ItemIdx j: items) {
        AgentIdx i = -1;
        for (AgentPos i_pos=0; i_pos<m; ++i_pos) {
            if (agents[i_pos] == sol.agent(j)) {
                i = i_pos;
                break;
            }
        }
        if (i == -1)
            continue;
        if (ins_tmp.item_number() == n) {
            c[i] -= ins.alternative(j, sol.agent(j)).w;
            continue;
        }
        v += ins.alternative(j, sol.agent(j)).v;
        ItemIdx j_tmp = ins_tmp.add_item();
        sol_vec.push_back(i);
        for (AgentPos i_pos=0; i_pos<m; ++i_pos)
            ins_tmp.set_alternative(j_tmp, i_pos,
                    ins.alternative(j, agents[i_pos]).w,
                    ins.alternative(j, agents[i_pos]).v);
        pos.push_back(j);
    }
    for (AgentPos i_pos=0; i_pos<m; ++i_pos)
        ins_tmp.set_capacity(i_pos, c[i_pos]);
    Solution sol_tmp(ins_tmp);
    for (ItemIdx j=0; j<(ItemIdx)pos.size(); ++j)
        sol_tmp.set(j, sol_vec[j]);
    sopt_milp(ins_tmp, sol_tmp);
    for (ItemIdx j=0; j<(ItemIdx)pos.size(); ++j)
        sol.set(pos[j], agents[sol_tmp.agent(j)]);
    return (sol_tmp.value() < v);
}

Solution gap::sol_lssimple(const Instance& ins, Solution& sol, Info info)
{
    if (!sol.is_complete() || sol.feasible() > 0)
        sol = sol_random(ins);
    Value lb = 0;
    for (ItemIdx j=0; j<ins.item_number(); ++j)
        lb += ins.item(j).v_min;

    std::vector<ItemIdx> items(ins.item_number(), 0);
    std::vector<AgentIdx> agents(ins.agent_number(), 0);
    std::iota(items.begin(), items.end(), 0);
    std::iota(agents.begin(), agents.end(), 0);

    Solution sol_best = sol;
    init_display(sol_best, lb, info);
    for (Cpt it=0; info.check_time(); ++it) {
        while (shift_swap(ins, sol, info)) {  }
        std::stringstream ss;
        ss << "it " << it << " shift-swap";
        sol_best.update(sol, lb, ss, info);

        bool b = false;
        for (ItemIdx k=0; ; ++k) {
            ItemIdx n = 2;
            if (k <= 10) {
                n = 2;
            } else if (k <= 50) {
                n = (k % 2) + 2;
            } else if (k <= 200) {
                n = (k % 3) + 2;
            } else if (k <= 1000) {
                n = (k % 4) + 2;
            } else if (k <= 5000) {
                n = (k % 5) + 2;
            } else if (k <= 20000) {
                n = (k % 6) + 2;
            } else {
                n = (k % 7) + 2;
            }
            if (n > ins.agent_number())
                n = ins.agent_number();
            if (move_gap(ins, sol, n, ins.item_number(), items, agents, info)) {
                std::stringstream ss;
                ss << "it " << it << " gap " << n;
                sol_best.update(sol, lb, ss, info);
                b = true;
                break;
            }
        }
        if (b)
            continue;
    }
    return algorithm_end(sol, info);
}

