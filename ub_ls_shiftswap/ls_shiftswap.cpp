#include "gap/ub_ls_shiftswap/ls_shiftswap.hpp"
#include "gap/ub_random/random.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace gap;

Solution gap::sol_lsfirst_shiftswap(const Instance& ins, std::default_random_engine& gen, double alpha, Info info)
{
    AgentIdx m = ins.agent_number();
    ItemIdx n = ins.item_number();
    std::uniform_int_distribution<Cpt> dis_ss(1, n * m + (n * (n + 1)) / 2);
    std::uniform_int_distribution<ItemIdx> dis_j(0, n - 1);
    std::uniform_int_distribution<ItemIdx> dis_j2(0, n - 2);
    std::uniform_int_distribution<AgentIdx> dis_i(0, m - 1);
    std::uniform_int_distribution<AgentIdx> dis_i1(0, m - 2);
    std::uniform_int_distribution<AgentIdx> dis_i2(0, m - 3);

    Solution sol_best(ins);
    Solution sol_curr(ins);
    for (ItemIdx j=0; j<ins.item_number(); ++j)
        sol_curr.set(j, dis_i(gen));

    Cpt it_max = 2 * (n * m + (n * (n + 1)) / 2);
    Cpt it_without_change = 0;

    while (it_without_change < it_max) {
        Value v = sol_curr.value() + alpha * sol_curr.feasible();
        Cpt p = dis_ss(gen);
        if (p <= m * n) { // shift
            ItemIdx j = dis_j(gen);
            AgentIdx i = dis_i1(gen);
            AgentIdx i_old = sol_curr.agent(j);
            if (i >= i_old)
                i++;
            sol_curr.set(j, i);
            double v_curr = sol_curr.value() + alpha * sol_curr.feasible();
            if (v < v_curr) {
                sol_curr.set(j, i_old);
            }
        } else { // swap
            ItemIdx j1 = dis_j(gen);
            ItemIdx j2 = dis_j2(gen);
            if (j2 >= j1)
                j2++;
            AgentIdx i1 = sol_curr.agent(j1);
            AgentIdx i2 = sol_curr.agent(j2);
            sol_curr.set(j1, i2);
            sol_curr.set(j2, i1);
            double v_curr = sol_curr.value() + alpha * sol_curr.feasible();
            if (v < v_curr) {
                sol_curr.set(j1, i1);
                sol_curr.set(j2, i2);
            }
        }

        // Update it_without_change
        if (sol_curr.feasible() == 0 && (!sol_best.is_complete() || sol_best.value() > sol_curr.value())) {
            std::stringstream ss;
            sol_best.update(sol_curr, 0, ss, info);
            it_without_change = 0;
        } else {
            it_without_change++;
        }
    }

    return algorithm_end(sol_best, info);
}

/******************************************************************************/

bool lsbest_shiftswap_move(const Instance& ins, Solution& sol, Info& info)
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

Solution gap::sol_lsbest_shiftswap(const Instance& ins, Solution& sol, std::default_random_engine& gen, Info info)
{
    if (!sol.is_complete() || sol.feasible() > 0)
        sol = sol_random(ins, gen);
    init_display(sol, 0, info);
    while (lsbest_shiftswap_move(ins, sol, info));
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

Solution gap::sol_ts_shiftswap(const Instance& ins, Solution& sol, std::default_random_engine& gen, Info info)
{
    if (!sol.is_complete() || sol.feasible() > 0)
        sol = sol_random(ins, gen);
    Solution sol_best = sol;
    init_display(sol_best, 0, info);

    Cpt neig_size = 10000;
    Cpt tabu_size = ins.item_number() / 20;

    AgentIdx m = ins.agent_number();
    ItemIdx n = ins.item_number();
    Cpt neigh_totalsize = n * m + (n * (n + 1)) / 2;
    std::vector<std::vector<Cpt>> tabu(ins.item_number(), std::vector<Cpt>(ins.agent_number(), - tabu_size - 1));
    std::uniform_int_distribution<Cpt> dis_ss(1, neigh_totalsize);
    std::uniform_int_distribution<ItemIdx> dis_j(0, n - 1);
    std::uniform_int_distribution<ItemIdx> dis_j2(0, n - 2);
    std::uniform_int_distribution<AgentIdx> dis_i1(0, m - 2);
    std::uniform_int_distribution<AgentIdx> dis_i2(0, m - 3);
    std::uniform_real_distribution<double> dis(0, 1);

    for (Cpt it=0; info.check_time(); ++it) {
        if (neig_size < neigh_totalsize / 2)
            neig_size++;

        Value v_best = -1;
        ItemIdx j_best = -1;
        AgentIdx i_best = -1;
        ItemIdx j1_best = -1;
        ItemIdx j2_best = -1;

        for (Cpt it2=0; it2<neig_size;) {
            Cpt p = dis_ss(gen);
            if (p <= m * n) { // shift
                ItemIdx j = dis_j(gen);
                AgentIdx i_old = sol.agent(j);
                AgentIdx i = dis_i1(gen);
                if (i >= i_old)
                    i++;

                if (it - tabu[j][i] < tabu_size)
                    continue;

                sol.set(j, i);
                Value v = sol.value();
                if (sol.feasible() == 0 && (v_best == -1 || v_best > v)) {
                    v_best = v;
                    j_best = j;
                    i_best = i;
                    j1_best = -1;
                    j2_best = -1;
                }
                sol.set(j, i_old);
            } else { // swap
                ItemIdx j1 = dis_j(gen);
                AgentIdx i1 = sol.agent(j1);

                ItemIdx j2 = dis_j2(gen);
                if (j2 >= j1)
                    j2++;
                AgentIdx i2 = sol.agent(j2);

                if (i1 == i2)
                    continue;
                if (it - tabu[j1][i2] < tabu_size)
                    continue;
                if (it - tabu[j2][i1] < tabu_size)
                    continue;

                sol.set(j1, i2);
                sol.set(j2, i1);
                Value v = sol.value();
                if (sol.feasible() == 0 && (v_best == -1 || v_best > v)) {
                    v_best = v;
                    j_best = -1;
                    i_best = -1;
                    j1_best = j1;
                    j2_best = j2;
                }
                sol.set(j1, i1);
                sol.set(j2, i2);
            }

            ++it2;
        }

        if (v_best == -1) {
            std::cout << "toto" << std::endl;
            continue;
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

        // Update best solution
        if (sol_best.value() > sol.value()) {
            std::stringstream ss;
            ss << "it " << it;
            sol_best.update(sol, 0, ss, info);
        }
    }
    return algorithm_end(sol, info);
}

/******************************************************************************/

Solution gap::sol_sa_shiftswap(const Instance& ins, Solution& sol, std::default_random_engine& gen, Info info)
{
    if (!sol.is_complete() || sol.feasible() > 0)
        sol = sol_random(ins, gen);
    Solution sol_best = sol;
    init_display(sol_best, 0, info);

    AgentIdx m = ins.agent_number();
    ItemIdx n = ins.item_number();
    std::uniform_int_distribution<Cpt> dis_ss(1, n * m + (n * (n + 1)) / 2);
    std::uniform_int_distribution<ItemIdx> dis_j(0, n - 1);
    std::uniform_int_distribution<ItemIdx> dis_j2(0, n - 2);
    std::uniform_int_distribution<AgentIdx> dis_i1(0, m - 2);
    std::uniform_int_distribution<AgentIdx> dis_i2(0, m - 3);
    std::uniform_real_distribution<double> dis(0, 1);

    double t0 = 0;
    for (ItemIdx j=0; j<ins.item_number(); ++j)
        for (AgentIdx i=0; i<ins.agent_number(); ++i)
            if (t0 < ins.alternative(j, i).v)
                t0 = ins.alternative(j, i).v;

    double alpha = 0.99;
    Cpt l = 100000;

    Cpt it_max = 2 * (n * m + (n * (n + 1)) / 2);
    Cpt it_without_change = 0;

    for (double t=t0; info.check_time(); t*=alpha) {
        for (Cpt it=0; it<l;) {
            Value v = sol.value();
            Cpt p = dis_ss(gen);
            if (p <= m * n) { // shift
                ItemIdx j = dis_j(gen);
                AgentIdx i = dis_i1(gen);
                AgentIdx i_old = sol.agent(j);
                if (i >= i_old)
                    i++;
                sol.set(j, i);
                if (sol.feasible() > 0
                        || (v < sol.value() &&
                            dis(gen) > exp((double)(v - sol.value()) / t))) {
                    sol.set(j, i_old);
                }
            } else { // swap
                ItemIdx j1 = dis_j(gen);
                AgentIdx i1 = sol.agent(j1);

                ItemIdx j2 = dis_j2(gen);
                AgentIdx i2 = sol.agent(j2);

                if (i1 == i2)
                    continue;

                sol.set(j1, i2);
                sol.set(j2, i1);
                if (sol.feasible() > 0
                        || (v < sol.value() &&
                            dis(gen) > exp((double)(v - sol.value()) / t))) {
                    sol.set(j1, i1);
                    sol.set(j2, i2);
                }
            }

            // Update it_without_change
            if (sol.value() == v) {
                it_without_change++;
                if (it_without_change > it_max)
                    return algorithm_end(sol_best, info);
            } else {
                it_without_change = 0;
            }

            // Update best solution
            if (sol_best.value() > sol.value()) {
                std::stringstream ss;
                ss << "t " << t;
                sol_best.update(sol, 0, ss, info);
            }

            ++it;
        }
    }
    return algorithm_end(sol_best, info);
}

/******************************************************************************/

Solution gap::sol_pr_shiftswap(const Instance& ins, Solution& sol, std::default_random_engine& gen, Info info)
{
    (void)ins;
    (void)sol;
    (void)gen;
    (void)info;
    return Solution(ins);
}

