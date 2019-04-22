#include "gap/ub_lssimple/lssimple.hpp"
#include "gap/ub_random/random.hpp"
#include "gap/opt_milp/milp.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace gap;

bool move_shift(Solution& sol,
        std::default_random_engine& gen,
        std::uniform_int_distribution<>& dis_j,
        std::uniform_int_distribution<>& dis_i,
        Info& info)
{
    (void)info;
    ItemIdx j = dis_j(gen);
    AgentIdx i_old = sol.agent(j);
    AgentIdx i_new = sol.agent(j);
    while (i_new == sol.agent(j))
        i_new = dis_i(gen);
    Value v = sol.value();
    sol.set(j, i_new);
    if (sol.feasible() > 0 || v < sol.value()) {
        sol.set(j, i_old);
        return false;
    }
    return true;
}

bool move_swap(Solution& sol,
        std::default_random_engine& gen,
        std::uniform_int_distribution<>& dis_j,
        Info& info)
{
    (void)info;
    ItemIdx j1 = dis_j(gen);
    ItemIdx j2 = j1;
    AgentIdx i1 = sol.agent(j1);
    AgentIdx i2 = sol.agent(j2);
    while (i2 == i1) {
        j2 = dis_j(gen);
        i2 = sol.agent(j2);
    }
    Value v = sol.value();
    sol.set(j1, i2);
    sol.set(j2, i1);
    if (sol.feasible() > 0 || v < sol.value()) {
        sol.set(j1, i1);
        sol.set(j2, i2);
        return false;
    }
    return true;
}

bool move_swap3(Solution& sol,
        std::default_random_engine& gen,
        std::uniform_int_distribution<>& dis_j,
        Info& info)
{
    (void)info;
    ItemIdx j1 = dis_j(gen);
    ItemIdx j2 = j1;
    ItemIdx j3 = j1;
    AgentIdx i1 = sol.agent(j1);
    AgentIdx i2 = sol.agent(j2);
    AgentIdx i3 = sol.agent(j3);
    while (i2 == i1) {
        j2 = dis_j(gen);
        i2 = sol.agent(j2);
    }
    while (i3 == i1 || i3 == i2) {
        j3 = dis_j(gen);
        i3 = sol.agent(j3);
    }
    Value v = sol.value();
    sol.set(j1, i2);
    sol.set(j2, i3);
    sol.set(j3, i1);
    if (sol.feasible() > 0 || v < sol.value()) {
        sol.set(j1, i1);
        sol.set(j2, i2);
        sol.set(j3, i3);
        return false;
    }
    return true;
}

bool move_gap(const Instance& ins, Solution& sol, AgentIdx m, ItemIdx n,
        std::vector<ItemIdx>& items, std::vector<AgentIdx>& agents,
        Info& info)
{
    (void)info;
    std::random_shuffle(items.begin(), items.end());
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
    std::default_random_engine gen(0);
    std::uniform_int_distribution<> dis_j(0, ins.item_number() - 1);
    std::uniform_int_distribution<> dis_i(0, ins.agent_number() - 1);
    std::uniform_int_distribution<> dis_m(2, std::min(ins.agent_number() - 1, (AgentIdx)5));
    for (Cpt it=1, it_without_improvment=0; info.check_time(); ++it, ++it_without_improvment) {
        if (move_shift(sol, gen, dis_j, dis_i, info))
            it_without_improvment = 0;
        if (move_swap(sol, gen, dis_j, info))
            it_without_improvment = 0;
        if ((it + 1) % 10 == 0)
            if (move_swap3(sol, gen, dis_j, info))
                it_without_improvment = 0;
        if ((it > 100000 && it % 10000 == 0) || (it > 1000000 && it % 1000 == 0))
            if (move_gap(ins, sol, 2, ins.item_number(), items, agents, info))
                it_without_improvment = 0;
        if ((it > 1000000 && it % 100000 == 0) || (it > 10000000 && it % 10000 == 0))
            if (move_gap(ins, sol, 3, ins.item_number(), items, agents, info))
                it_without_improvment = 0;
        if ((it > 10000000 && it % 1000000 == 0) || (it > 100000000 && it % 100000 == 0))
            if (move_gap(ins, sol, 4, 1024, items, agents, info))
                it_without_improvment = 0;
        if ((it > 100000000 && it % 10000000 == 0) || (it > 1000000000 && it % 1000000 == 0))
            if (move_gap(ins, sol, 5, 512, items, agents, info))
                it_without_improvment = 0;

        if (it % 100000 == 0 && sol_best.value() > sol.value()) {
            it_without_improvment = 0;
            std::stringstream ss;
            ss << "it " << it;
            sol_best.update(sol, lb, ss, info);
        }
    }
    return algorithm_end(sol, info);
}

