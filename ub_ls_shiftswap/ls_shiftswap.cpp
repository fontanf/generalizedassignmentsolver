#include "gap/ub_ls_shiftswap/ls_shiftswap.hpp"
#include "gap/ub_random/random.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace gap;

bool gap::shiftswap_first(Solution& sol, std::vector<std::pair<ItemIdx, ItemIdx>>& alt,
        std::default_random_engine& gen, Info& info)
{
    std::shuffle(alt.begin(), alt.end(), gen);
    Value v = sol.value();
    for (std::pair<ItemIdx, ItemIdx> p: alt) {
        if (p.second < 0) { // shift
            ItemIdx j = p.first;
            AgentIdx i_old = sol.agent(j);
            AgentIdx i = - p.second - 1;
            if (i == i_old)
                continue;
            sol.set(j, i);
            if (sol.feasible() == 0 && v > sol.value()) {
                std::stringstream ss;
                ss << "shift j " << j << " i " << i;
                sol.update(sol, 0, ss, info);
                return true;
            }
            sol.set(j, i_old);
        } else { // swap
            ItemIdx j1 = p.first;
            ItemIdx j2 = p.second;
            AgentIdx i1 = sol.agent(j1);
            AgentIdx i2 = sol.agent(j2);
            if (i1 == i2)
                continue;
            sol.set(j1, i2);
            sol.set(j2, i1);
            if (sol.feasible() == 0 && v > sol.value()) {
                std::stringstream ss;
                ss << "swap j1 " << j1 << " j2 " << j2;
                sol.update(sol, 0, ss, info);
                return true;
            }
            sol.set(j1, i1);
            sol.set(j2, i2);
        }
    }
    return false;
}

Solution gap::sol_ls_shiftswap_first(const Instance& ins, Solution& sol, std::default_random_engine& gen, Info info)
{
    if (!sol.is_complete() || sol.feasible() > 0)
        sol = sol_random(ins, gen);

    Cpt k = 0;
    std::vector<std::pair<ItemIdx, ItemIdx>> alt(
            ins.item_number() * (ins.item_number() + 1) / 2 + ins.item_number() * ins.agent_number());
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        for (AgentIdx i=0; i<ins.agent_number(); ++i)
            alt[k++] = {j, - i - 1};
        for (ItemIdx j2=j+1; j2<ins.item_number(); ++j2)
            alt[k++] = {j, j2};
    }
    init_display(sol, 0, info);
    while (shiftswap_first(sol, alt, gen, info));
    return algorithm_end(sol, info);
}

/******************************************************************************/

bool gap::shiftswap_best(const Instance& ins, Solution& sol, Info& info)
{
    Value v_best = sol.value();

    ItemIdx j_best = -1;
    AgentIdx i_best = -1;
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        AgentIdx i_old = sol.agent(j);
        for (AgentIdx i=0; i<ins.agent_number(); ++i) {
            if (i == i_old)
                continue;
            sol.set(j, i);
            if (sol.feasible() == 0 && v_best > sol.value()) {
                v_best = sol.value();
                j_best = j;
                i_best = i;
            }
        }
        sol.set(j, i_old);
    }

    ItemIdx j1_best = -1;
    ItemIdx j2_best = -1;
    for (ItemIdx j1=0; j1<ins.item_number(); ++j1) {
        AgentIdx i1 = sol.agent(j1);
        for (ItemIdx j2=j1+1; j2<ins.item_number(); ++j2) {
            AgentIdx i2 = sol.agent(j2);
            if (i2 == i1)
                continue;
            sol.set(j1, i2);
            sol.set(j2, i1);
            if (sol.feasible() == 0 && v_best > sol.value()) {
                v_best = sol.value();
                j_best = -1;
                i_best = -1;
                j1_best = j1;
                j2_best = j2;
            }
            sol.set(j2, i2);
        }
        sol.set(j1, i1);
    }

    if (j_best == -1 && j1_best == -1)
        return false;

    std::stringstream ss;
    if (j1_best != -1) {
        AgentIdx i1 = sol.agent(j1_best);
        AgentIdx i2 = sol.agent(j2_best);
        sol.set(j1_best, i2);
        sol.set(j2_best, i1);
        ss << "swap j1 " << j1_best << " j2 " << j2_best;
    } else {
        sol.set(j_best, i_best);
        ss << "shift j " << j_best << " i " << i_best;
    }
    sol.update(sol, 0, ss, info);
    return true;
}

Solution gap::sol_ls_shiftswap_best(const Instance& ins, Solution& sol, std::default_random_engine& gen, Info info)
{
    if (!sol.is_complete() || sol.feasible() > 0)
        sol = sol_random(ins, gen);
    init_display(sol, 0, info);
    while (shiftswap_best(ins, sol, info));
    return algorithm_end(sol, info);
}

/******************************************************************************/

bool gap::doubleshift_best(const Instance& ins, Solution& sol, Info& info)
{
    Value v_best = sol.value();

    ItemIdx j1_best = -1;
    ItemIdx j2_best = -1;
    AgentIdx i2_best = -1;
    for (ItemIdx j1=0; j1<ins.item_number(); ++j1) {
        AgentIdx i1_old = sol.agent(j1);
        for (ItemIdx j2=j1+1; j2<ins.item_number(); ++j2) {
            AgentIdx i2_old = sol.agent(j2);
            if (i2_old == i1_old)
                continue;
            sol.set(j1, i2_old);
            for (AgentIdx i2=0; i2<ins.agent_number(); ++i2) {
                if (i2 == i2_old)
                    continue;
                sol.set(j2, i2);
                if (sol.feasible() == 0 && v_best > sol.value()) {
                    v_best = sol.value();
                    j1_best = j1;
                    j2_best = j2;
                    i2_best = i2;
                }
            }
            sol.set(j2, i2_old);
        }
        sol.set(j1, i1_old);
    }

    if (j1_best == -1)
        return false;

    sol.set(j1_best, sol.agent(j2_best));
    sol.set(j2_best, i2_best);
    std::stringstream ss;
    ss << "doubleshift j1 " << j1_best << " j2 " << j2_best << " i2 " << i2_best;
    sol.update(sol, 0, ss, info);
    return true;
}

bool gap::tripleswap_best(const Instance& ins, Solution& sol, Info& info)
{
    Value v_best = sol.value();

    ItemIdx j1_best = -1;
    ItemIdx j2_best = -1;
    ItemIdx j3_best = -1;
    for (ItemIdx j1=0; j1<ins.item_number(); ++j1) {
        AgentIdx i1 = sol.agent(j1);
        for (ItemIdx j2=j1+1; j2<ins.item_number(); ++j2) {
            AgentIdx i2 = sol.agent(j2);
            if (i1 == i2)
                continue;
            sol.set(j1, i2);
            for (ItemIdx j3=j2+1; j3<ins.item_number(); ++j3) {
                AgentIdx i3 = sol.agent(j3);
                if (i3 == i1 || i3 == i2)
                    continue;
                sol.set(j2, i3);
                sol.set(j3, i1);
                if (sol.feasible() == 0 && v_best > sol.value()) {
                    v_best = sol.value();
                    j1_best = j1;
                    j2_best = j2;
                    j3_best = j3;
                }
                sol.set(j3, i3);
            }
            sol.set(j2, i2);
        }
        sol.set(j1, i1);
    }

    if (j1_best == -1)
        return false;

    std::stringstream ss;
    AgentIdx i1 = sol.agent(j1_best);
    AgentIdx i2 = sol.agent(j2_best);
    AgentIdx i3 = sol.agent(j3_best);
    sol.set(j1_best, i2);
    sol.set(j2_best, i3);
    sol.set(j3_best, i1);
    ss << "swap j1 " << j1_best << " j2 " << j2_best << " j3 " << j3_best;
    sol.update(sol, 0, ss, info);
    return true;
}

/******************************************************************************/

void tabu_shiftswap(const Instance& ins, Solution& sol,
        std::vector<std::vector<Cpt>>& tabu, Cpt tabu_size, Cpt it)
{
    Value v_best = -1;
    ItemIdx j_best = -1;
    AgentIdx i_best = -1;
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        AgentIdx i_old = sol.agent(j);
        for (AgentIdx i=0; i<ins.agent_number(); ++i) {
            if (i == i_old)
                continue;
            if (it - tabu[j][i] < tabu_size)
                continue;
            sol.set(j, i);
            if (sol.feasible() == 0 && (v_best == -1 || v_best > sol.value())) {
                v_best = sol.value();
                j_best = j;
                i_best = i;
            }
        }
        sol.set(j, i_old);
    }

    ItemIdx j1_best = -1;
    ItemIdx j2_best = -1;
    for (ItemIdx j1=0; j1<ins.item_number(); ++j1) {
        AgentIdx i1 = sol.agent(j1);
        for (ItemIdx j2=j1+1; j2<ins.item_number(); ++j2) {
            AgentIdx i2 = sol.agent(j2);
            if (i2 == i1)
                continue;
            if (it - tabu[j1][i2] < tabu_size)
                continue;
            if (it - tabu[j2][i1] < tabu_size)
                continue;
            sol.set(j1, i2);
            sol.set(j2, i1);
            if (sol.feasible() == 0 && (v_best == -1 || v_best > sol.value())) {
                v_best = sol.value();
                j_best = -1;
                i_best = -1;
                j1_best = j1;
                j2_best = j2;
            }
            sol.set(j2, i2);
        }
        sol.set(j1, i1);
    }

    if (v_best == -1) {
        std::cout << "toto" << std::endl;
        return;
    }

    if (j1_best != -1) {
        AgentIdx i1 = sol.agent(j1_best);
        AgentIdx i2 = sol.agent(j2_best);
        if (v_best >= sol.value()) {
            tabu[j1_best][i1] = it;
            tabu[j2_best][i2] = it;
        }
        sol.set(j1_best, i2);
        sol.set(j2_best, i1);
    } else {
        if (v_best >= sol.value())
            tabu[j_best][sol.agent(j_best)] = it;
        sol.set(j_best, i_best);
    }
}

Solution gap::sol_ts_shiftswap(const Instance& ins, Solution& sol, std::default_random_engine& gen, Info info)
{
    if (!sol.is_complete() || sol.feasible() > 0)
        sol = sol_random(ins, gen);
    Solution sol_best = sol;
    init_display(sol_best, 0, info);
    Cpt tabu_size = ins.item_number() / 20;
    std::vector<std::vector<Cpt>> tabu(ins.item_number(), std::vector<Cpt>(ins.agent_number(), - tabu_size - 1));

    for (Cpt it=0; info.check_time(); ++it) {
        tabu_shiftswap(ins, sol, tabu, tabu_size, it);
        if (sol_best.value() > sol.value()) {
            std::stringstream ss;
            ss << "it " << it;
            sol_best.update(sol, 0, ss, info);
        }
    }
    return algorithm_end(sol, info);
}

