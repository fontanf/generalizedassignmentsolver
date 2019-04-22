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

bool move_milp2(const Instance& ins, Solution& sol,
        std::default_random_engine& gen, std::uniform_int_distribution<>& dis,
        Info& info)
{
    (void)info;
    AgentIdx i1 = dis(gen);
    AgentIdx i2 = i1;
    while (i2 == i1)
        i2 = dis(gen);
    Instance ins_tmp(2);
    ins_tmp.set_capacity(0, ins.capacity(i1));
    ins_tmp.set_capacity(1, ins.capacity(i2));
    Value v = 0;
    std::vector<ItemIdx> pos;
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        if (sol.agent(j) == i1 || sol.agent(j) == i2) {
            v += ins.alternative(j, sol.agent(j)).v;
            ItemIdx j_tmp = ins_tmp.add_item();
            ins_tmp.set_alternative(j_tmp, 0, ins.alternative(j, i1).w, ins.alternative(j, i1).v);
            ins_tmp.set_alternative(j_tmp, 1, ins.alternative(j, i2).w, ins.alternative(j, i2).v);
            pos.push_back(j);
        }
    }
    Solution sol_tmp = sopt_milp(ins_tmp);
    for (ItemIdx j=0; j<(ItemIdx)pos.size(); ++j) {
        if (sol.agent(pos[j]) == i1 && sol_tmp.agent(j) == 1)
            sol.set(pos[j], i2);
        if (sol.agent(pos[j]) == i2 && sol_tmp.agent(j) == 0)
            sol.set(pos[j], i1);
    }
    return (sol_tmp.value() < v);
}

bool move_milp3(const Instance& ins, Solution& sol,
        std::default_random_engine& gen, std::uniform_int_distribution<>& dis,
        Info& info)
{
    (void)info;
    AgentIdx i[3];
    i[0] = dis(gen);
    i[1] = i[0];
    i[2] = i[0];
    while (i[1] == i[0])
        i[1] = dis(gen);
    while (i[2] == i[0] || i[2] == i[1])
        i[2] = dis(gen);
    Instance ins_tmp(3);
    ins_tmp.set_capacity(0, ins.capacity(i[0]));
    ins_tmp.set_capacity(1, ins.capacity(i[1]));
    ins_tmp.set_capacity(2, ins.capacity(i[2]));
    Value v = 0;
    std::vector<ItemIdx> pos;
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        if (sol.agent(j) == i[0] || sol.agent(j) == i[1] || sol.agent(j) == i[2]) {
            v += ins.alternative(j, sol.agent(j)).v;
            ItemIdx j_tmp = ins_tmp.add_item();
            ins_tmp.set_alternative(j_tmp, 0, ins.alternative(j, i[0]).w, ins.alternative(j, i[0]).v);
            ins_tmp.set_alternative(j_tmp, 1, ins.alternative(j, i[1]).w, ins.alternative(j, i[1]).v);
            ins_tmp.set_alternative(j_tmp, 2, ins.alternative(j, i[2]).w, ins.alternative(j, i[2]).v);
            pos.push_back(j);
        }
    }
    Solution sol_tmp = sopt_milp(ins_tmp);
    for (ItemIdx j=0; j<(ItemIdx)pos.size(); ++j) {
        sol.set(pos[j], i[sol_tmp.agent(j)]);
    }
    return (sol_tmp.value() < v);
}

Solution gap::sol_lssimple(const Instance& ins, Solution& sol, Info info)
{
    if (!sol.is_complete() || sol.feasible() > 0)
        sol = sol_random(ins);
    Value lb = 0;
    for (ItemIdx j=0; j<ins.item_number(); ++j)
        lb += ins.item(j).v_min;

    Solution sol_best = sol;
    init_display(sol_best, lb, info);
    std::default_random_engine gen(0);
    std::uniform_int_distribution<> dis_j(0, ins.item_number() - 1);
    std::uniform_int_distribution<> dis_i(0, ins.agent_number() - 1);
    for (Cpt it=0, it_without_improvment=0; info.check_time(); ++it, ++it_without_improvment) {
        if (move_shift(sol, gen, dis_j, dis_i, info))
            it_without_improvment = 0;
        if (move_swap(sol, gen, dis_j, info))
            it_without_improvment = 0;
        if ((it + 1) % 10 == 0)
            if (move_swap3(sol, gen, dis_j, info))
                it_without_improvment = 0;
        if ((it + 1) % 100000 == 0)
            if (move_milp2(ins, sol, gen, dis_i, info))
                it_without_improvment = 0;
        if ((it + 1) % 10000000 == 0)
            if (move_milp3(ins, sol, gen, dis_i, info))
                it_without_improvment = 0;

        if ((it + 1) % 1000000 == 0 && sol_best.value() > sol.value()) {
            it_without_improvment = 0;
            std::stringstream ss;
            ss << "it " << it;
            sol_best.update(sol, lb, ss, info);
        }
    }
    return algorithm_end(sol, info);
}

