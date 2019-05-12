#include "gap/ub_ls_shiftswap/ls_shiftswap.hpp"
#include "gap/ub_random/random.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace gap;

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

    while (!sol_curr.feasible()) {
        //std::cout << "cost " << sol_curr.cost() << " oc " << sol_curr.overcapacity() << std::endl;
        Weight oc = sol_curr.overcapacity();
        Cost c = sol_curr.cost();
        ItemIdx j1_best = -1;
        ItemIdx j2_best = -1;
        ItemIdx j_best = -1;
        ItemIdx i_best = -1;
        double v_best = -1;
        for (ItemIdx j1=0; j1<n; ++j1) {
            AgentIdx i1 = sol_curr.agent(j1);

            // Shift
            for (AgentIdx i=0; i<m; ++i) {
                if (i == i1)
                    continue;
                sol_curr.set(j1, i);
                if (sol_curr.overcapacity() < oc) {
                    double v = (double)(sol_curr.cost() - c) / (oc - sol_curr.overcapacity());
                    if (j1_best < 0 || v_best > v) {
                        v_best = v;
                        j_best = j1;
                        i_best = i;
                    }
                }
            }

            // Swap
            for (ItemIdx j2=j1+1; j2<n; ++j2) {
                AgentIdx i2 = sol_curr.agent(j2);
                if (i1 == i2)
                    continue;
                sol_curr.set(j2, i1);
                sol_curr.set(j1, i2);
                if (sol_curr.overcapacity() < oc) {
                    double v = (double)(sol_curr.cost() - c) / (oc - sol_curr.overcapacity());
                    if (j1_best < 0 || v_best > v) {
                        v_best = v;
                        j1_best = j1;
                        j2_best = j2;
                        j_best = -1;
                    }
                }
                sol_curr.set(j2, i2);
            }

            sol_curr.set(j1, i1);
        }

        if (j_best != -1) { // Shift
            sol_curr.set(j_best, i_best);
        } else { // Swap
            AgentIdx i1 = sol_curr.agent(j1_best);
            AgentIdx i2 = sol_curr.agent(j2_best);
            sol_curr.set(j1_best, i2);
            sol_curr.set(j2_best, i1);
        }
    }
    return algorithm_end(sol_curr, info);
}

/******************************************************************************/

std::vector<std::pair<ItemIdx, ItemIdx>> get_moves(const Instance& ins)
{
    ItemIdx n = ins.item_number();
    AgentIdx m = ins.agent_number();
    std::vector<std::pair<ItemIdx, ItemIdx>> moves;
    for (ItemIdx j=0; j<n; ++j) {
        for (AgentIdx i=0; i<m-1; ++i)
            moves.push_back({j, -i-1});
        for (ItemIdx j2=j+1; j2<n; ++j2)
            moves.push_back({j, j2});
    }
    return moves;
}

Solution random_solution(const Instance& ins, std::mt19937_64& gen)
{
    Solution sol(ins);
    std::uniform_int_distribution<AgentIdx> dis_i(0, ins.agent_number() - 1);
    for (ItemIdx j=0; j<ins.item_number(); ++j)
        sol.set(j, dis_i(gen));
    return sol;
}

Solution gap::sol_lsfirst_shiftswap(LSFirstShiftSwapData d)
{
    init_display(d.info);
    Solution sol_best(d.ins);
    AgentIdx m = d.ins.agent_number();

    Solution sol_curr = random_solution(d.ins, d.gen);
    sol_curr.update_penalties(std::vector<PCost>(m, d.alpha));
    auto moves = get_moves(d.ins);

    for (Cpt it=0; d.info.check_time();) {
        PCost v_curr = sol_curr.pcost();
        bool improved = false;
        for (Cpt k=0; k<(Cpt)moves.size(); ++k, ++it) {
            std::uniform_int_distribution<Cpt> dis(k, moves.size() - 1);
            Cpt move_idx = dis(d.gen);
            iter_swap(moves.begin() + k, moves.begin() + move_idx);
            if (moves[k].second < 0) { // shift
                ItemIdx j = moves[k].first;
                AgentIdx i_old = sol_curr.agent(j);
                AgentIdx i = - (moves[k].second + 1);
                if (i >= i_old)
                    i++;
                //std::cout << "j " << j << " i_old " << i_old << " i " << i << std::endl;
                sol_curr.set(j, i);
                if (sol_curr.feasible() && (!sol_best.feasible() || sol_best.cost() > sol_curr.cost())) {
                    std::stringstream ss;
                    ss << "it " << it;
                    sol_best.update(sol_curr, 0, ss, d.info);
                }
                if (v_curr > sol_curr.pcost()) {
                    improved = true;
                    break;
                }
                sol_curr.set(j, i_old);
            } else { // swap
                ItemIdx j1 = moves[k].first;
                ItemIdx j2 = moves[k].second;
                AgentIdx i1 = sol_curr.agent(j1);
                AgentIdx i2 = sol_curr.agent(j2);
                if (i1 == i2)
                    continue;
                //std::cout << "j1 " << j1 << " i1 " << i1 << " j2 " << j2 << " i2 " << i2 << std::endl;
                sol_curr.set(j1, i2);
                sol_curr.set(j2, i1);
                if (sol_curr.feasible() && (!sol_best.feasible() || sol_best.cost() > sol_curr.cost())) {
                    std::stringstream ss;
                    ss << "it " << it;
                    sol_best.update(sol_curr, 0, ss, d.info);
                }
                if (v_curr > sol_curr.pcost()) {
                    improved = true;
                    break;
                }
                sol_curr.set(j1, i1);
                sol_curr.set(j2, i2);
            }
        }
        if (!improved)
            break;
    }
    return algorithm_end(sol_best, d.info);
}

/******************************************************************************/

Solution gap::sol_lsbest_shiftswap(LSBestShiftSwapData d)
{
    init_display(d.info);
    AgentIdx m = d.ins.agent_number();
    ItemIdx n = d.ins.item_number();
    std::uniform_int_distribution<AgentIdx> dis_i(0, m - 1);

    Solution sol_best(d.ins);
    Solution sol_curr(d.ins);
    sol_curr.update_penalties(std::vector<PCost>(m, d.alpha));
    for (ItemIdx j=0; j<d.ins.item_number(); ++j)
        sol_curr.set(j, dis_i(d.gen));

    for (Cpt it=0;; ++it) {
        double v_best = sol_curr.pcost();

        ItemIdx j_best = -1;
        AgentIdx i_best = -1;
        for (ItemIdx j=0; j<n; ++j) {
            AgentIdx i_old = sol_curr.agent(j);
            for (AgentIdx i=0; i<m; ++i) {
                if (i == i_old)
                    continue;
                sol_curr.set(j, i);
                if (v_best > sol_curr.pcost()) {
                    v_best = sol_curr.pcost();
                    j_best = j;
                    i_best = i;
                }
            }
            sol_curr.set(j, i_old);
        }

        ItemIdx j1_best = -1;
        ItemIdx j2_best = -1;
        for (ItemIdx j1=0; j1<n; ++j1) {
            AgentIdx i1 = sol_curr.agent(j1);
            for (ItemIdx j2=j1+1; j2<n; ++j2) {
                AgentIdx i2 = sol_curr.agent(j2);
                if (i2 == i1)
                    continue;
                sol_curr.set(j1, i2);
                sol_curr.set(j2, i1);
                if (v_best > sol_curr.pcost()) {
                    v_best = sol_curr.pcost();
                    j_best = -1;
                    i_best = -1;
                    j1_best = j1;
                    j2_best = j2;
                }
                sol_curr.set(j2, i2);
            }
            sol_curr.set(j1, i1);
        }

        if (j_best == -1 && j1_best == -1)
            return algorithm_end(sol_best, d.info);

        std::stringstream ss;
        if (j_best != -1) { // Shift
            sol_curr.set(j_best, i_best);
        } else { // Swap
            AgentIdx i1 = sol_curr.agent(j1_best);
            AgentIdx i2 = sol_curr.agent(j2_best);
            sol_curr.set(j1_best, i2);
            sol_curr.set(j2_best, i1);
        }

        // Update best solution
        if (sol_curr.feasible() && (!sol_best.feasible() || sol_best.cost() > sol_curr.cost())) {
            std::stringstream ss;
            ss << "it " << it;
            sol_best.update(sol_curr, 0, ss, d.info);
        }
    }

    return algorithm_end(sol_best, d.info);
}

/******************************************************************************/

Solution gap::sol_ts_shiftswap(TSShiftSwapData d)
{
    init_display(d.info);
    Solution sol_best(d.ins);
    AgentIdx m = d.ins.agent_number();
    ItemIdx n = d.ins.item_number();

    Solution sol_curr = random_solution(d.ins, d.gen);
    sol_curr.update_penalties(std::vector<PCost>(m, d.alpha));
    auto moves = get_moves(d.ins);

    Cpt tabu_size = 16;
    std::vector<std::vector<Cpt>> tabu(n, std::vector<Cpt>(m, - tabu_size - 1));

    for (Cpt it=0; d.info.check_time();) {
        PCost v_curr = sol_curr.pcost();
        PCost v_best = -1;
        ItemIdx j_best = -1;
        AgentIdx i_best = -1;
        ItemIdx j1_best = -1;
        ItemIdx j2_best = -1;
        bool improved = false;
        for (Cpt k=0; k<(Cpt)moves.size(); ++k) {
            std::uniform_int_distribution<Cpt> dis(k, moves.size() - 1);
            Cpt move_idx = dis(d.gen);
            iter_swap(moves.begin() + k, moves.begin() + move_idx);
            if (moves[k].second < 0) { // shift
                ItemIdx j = moves[k].first;
                AgentIdx i_old = sol_curr.agent(j);
                AgentIdx i = - (moves[k].second + 1);
                if (i >= i_old)
                    i++;
                //std::cout << "j " << j << " i_old " << i_old << " i " << i << std::endl;
                if (it - tabu[j][i] < tabu_size)
                    continue;
                sol_curr.set(j, i);
                if (sol_curr.feasible() && (!sol_best.feasible() || sol_best.cost() > sol_curr.cost())) {
                    std::stringstream ss;
                    ss << "it " << it;
                    sol_best.update(sol_curr, 0, ss, d.info);
                }
                if (v_curr > sol_curr.pcost()) {
                    improved = true;
                    break;
                }
                if (v_best == -1 || v_best > sol_curr.pcost()) {
                    v_best = sol_curr.pcost();
                    j_best = j;
                    i_best = i;
                    j1_best = -1;
                    j2_best = -1;
                }
                sol_curr.set(j, i_old);
            } else { // swap
                ItemIdx j1 = moves[k].first;
                ItemIdx j2 = moves[k].second;
                AgentIdx i1 = sol_curr.agent(j1);
                AgentIdx i2 = sol_curr.agent(j2);
                if (i1 == i2)
                    continue;
                //std::cout << "j1 " << j1 << " i1 " << i1 << " j2 " << j2 << " i2 " << i2 << std::endl;
                if (it - tabu[j1][i2] < tabu_size)
                    continue;
                if (it - tabu[j2][i1] < tabu_size)
                    continue;
                sol_curr.set(j1, i2);
                sol_curr.set(j2, i1);
                if (sol_curr.feasible() && (!sol_best.feasible() || sol_best.cost() > sol_curr.cost())) {
                    std::stringstream ss;
                    ss << "it " << it;
                    sol_best.update(sol_curr, 0, ss, d.info);
                }
                if (v_curr > sol_curr.pcost()) {
                    improved = true;
                    break;
                }
                if (v_best == -1 || v_best > sol_curr.pcost()) {
                    v_best = sol_curr.pcost();
                    j_best = -1;
                    i_best = -1;
                    j1_best = j1;
                    j2_best = j2;
                }
                sol_curr.set(j1, i1);
                sol_curr.set(j2, i2);
            }
        }

        if (!improved) {
            if (j_best != -1) {
                //std::cout << "j_best " << j_best << " i_best " << i_best << std::endl;
                tabu[j_best][sol_curr.agent(j_best)] = it;
                sol_curr.set(j_best, i_best);
            } else {
                //std::cout << "j1_best " << j1_best << " j2_best " << j2_best << std::endl;
                AgentIdx i1 = sol_curr.agent(j1_best);
                AgentIdx i2 = sol_curr.agent(j2_best);
                tabu[j1_best][i1] = it;
                tabu[j2_best][i2] = it;
                sol_curr.set(j1_best, i2);
                sol_curr.set(j2_best, i1);
            }
            ++it;
        }
    }
    return algorithm_end(sol_best, d.info);
}

/******************************************************************************/

Solution gap::sol_sa_shiftswap(SAShiftSwapData d)
{
    init_display(d.info);
    Solution sol_best(d.ins);
    AgentIdx m = d.ins.agent_number();
    ItemIdx n = d.ins.item_number();
    std::uniform_int_distribution<Cpt> dis_ss(1, n * m + (n * (n + 1)) / 2);
    std::uniform_int_distribution<ItemIdx> dis_j(0, n - 1);
    std::uniform_int_distribution<ItemIdx> dis_j2(0, n - 2);
    std::uniform_int_distribution<AgentIdx> dis_i(0, m - 1);
    std::uniform_int_distribution<AgentIdx> dis_i1(0, m - 2);
    std::uniform_int_distribution<AgentIdx> dis_i2(0, m - 3);
    std::uniform_real_distribution<double> dis(0, 1);

    Solution sol_curr = sol_random(d.ins, d.gen);
    sol_curr.update_penalties(std::vector<PCost>(m, d.alpha));

    // Compute initial temperature
    double t0 = 0;
    for (ItemIdx j=0; j<n; ++j)
        for (AgentIdx i=0; i<m; ++i)
            if (t0 < d.ins.alternative(j, i).c)
                t0 = d.ins.alternative(j, i).c;

    Cpt it_max = 2 * (n * m + (n * (n + 1)) / 2);
    Cpt it_without_change = 0;
    for (double t=t0; d.info.check_time(); t*=d.beta) {
        for (Cpt it=0; it<d.l;) {
            PCost v = sol_curr.pcost();
            Cpt p = dis_ss(d.gen);
            if (p <= m * n) { // shift
                ItemIdx j = dis_j(d.gen);
                AgentIdx i = dis_i1(d.gen);
                AgentIdx i_old = sol_curr.agent(j);
                if (i >= i_old)
                    i++;
                sol_curr.set(j, i);
                if ((v < sol_curr.pcost() && dis(d.gen) > exp((double)(v - sol_curr.pcost()) / t))) {
                    sol_curr.set(j, i_old);
                }
            } else { // swap
                ItemIdx j1 = dis_j(d.gen);
                AgentIdx i1 = sol_curr.agent(j1);
                ItemIdx j2 = dis_j2(d.gen);
                if (j2 >= j1)
                    j2++;
                AgentIdx i2 = sol_curr.agent(j2);
                if (i1 == i2)
                    continue;
                sol_curr.set(j1, i2);
                sol_curr.set(j2, i1);
                if ((v < sol_curr.pcost() && dis(d.gen) > exp((double)(v - sol_curr.pcost()) / t))) {
                    sol_curr.set(j1, i1);
                    sol_curr.set(j2, i2);
                }
            }

            // Update it_without_change
            if (sol_curr.pcost() == v) {
                it_without_change++;
                if (it_without_change > it_max)
                    return algorithm_end(sol_best, d.info);
            } else {
                it_without_change = 0;
            }

            // Update best solution
            if (sol_curr.feasible() && (!sol_best.feasible() || sol_best.cost() > sol_curr.cost())) {
                std::stringstream ss;
                ss << "T " << t;
                sol_best.update(sol_curr, 0, ss, d.info);
            }

            ++it;
        }
    }
    return algorithm_end(sol_best, d.info);
}

/******************************************************************************/

void pr_shiftswap(PRShiftSwapData& d,
        std::vector<std::pair<ItemIdx, ItemIdx>>& moves,
        Solution& sol_curr, Solution& sol_best)
{
    for (;;) {
        PCost v_curr = sol_curr.pcost();
        bool improved = false;
        for (Cpt k=0; k<(Cpt)moves.size(); ++k) {
            std::uniform_int_distribution<Cpt> dis(k, moves.size() - 1);
            Cpt move_idx = dis(d.gen);
            iter_swap(moves.begin() + k, moves.begin() + move_idx);
            if (moves[k].second < 0) { // shift
                ItemIdx j = moves[k].first;
                AgentIdx i_old = sol_curr.agent(j);
                AgentIdx i = - (moves[k].second + 1);
                if (i >= i_old)
                    i++;
                //std::cout << "j " << j << " i_old " << i_old << " i " << i << std::endl;
                sol_curr.set(j, i);
                if (sol_curr.feasible() && (!sol_best.feasible() || sol_best.cost() > sol_curr.cost())) {
                    std::stringstream ss;
                    sol_best.update(sol_curr, 0, ss, d.info);
                }
                if (v_curr > sol_curr.pcost()) {
                    improved = true;
                    break;
                }
                sol_curr.set(j, i_old);
            } else { // swap
                ItemIdx j1 = moves[k].first;
                ItemIdx j2 = moves[k].second;
                AgentIdx i1 = sol_curr.agent(j1);
                AgentIdx i2 = sol_curr.agent(j2);
                if (i1 == i2)
                    continue;
                //std::cout << "j1 " << j1 << " i1 " << i1 << " j2 " << j2 << " i2 " << i2 << std::endl;
                sol_curr.set(j1, i2);
                sol_curr.set(j2, i1);
                if (sol_curr.feasible() && (!sol_best.feasible() || sol_best.cost() > sol_curr.cost())) {
                    std::stringstream ss;
                    sol_best.update(sol_curr, 0, ss, d.info);
                }
                if (v_curr > sol_curr.pcost()) {
                    improved = true;
                    break;
                }
                sol_curr.set(j1, i1);
                sol_curr.set(j2, i2);
            }
        }
        if (!improved)
            break;
    }
}

template <class Set>
void add(PRShiftSwapData& d, Set& set, Solution& sol)
{
    if ((Cpt)set.size() < d.gamma ||
            (sol.pcost() < std::prev(set.end())->pcost())) {
        set.insert(sol);
        if ((Cpt)set.size() > d.gamma)
            set.erase(std::prev(set.end()));
    }
}

Solution gap::sol_pr_shiftswap(PRShiftSwapData d)
{
    Solution sol_best(d.ins);
    AgentIdx m = d.ins.agent_number();
    ItemIdx  n = d.ins.item_number();
    std::uniform_int_distribution<Cpt> dis_a(0, d.rho - 1);
    std::uniform_int_distribution<Cpt> dis_b(0, d.rho - 2);
    auto moves = get_moves(d.ins);

    std::vector<Solution> pool(d.rho, Solution(d.ins));
    for (Solution& sol_curr: pool) {
        sol_curr = random_solution(d.ins, d.gen);
        sol_curr.update_penalties(std::vector<PCost>(m, d.alpha));
        pr_shiftswap(d, moves, sol_curr, sol_best);
    }

    auto cmp = [](const Solution& s1, const Solution& s2) { return s1.pcost() < s2.pcost(); };
    for (;;) {
        //for (const Solution& sol: pool)
            //std::cout << sol.pcost() << " ";
        //std::cout << std::endl;

        // Get solutions a and b
        Cpt a = dis_a(d.gen);
        Cpt b = dis_b(d.gen);
        if (b >= a)
            b++;
        Solution sol_a = pool[a];
        Solution sol_b = pool[b];

        // Fill s
        std::multiset<Solution, decltype(cmp)> s(cmp);
        ItemIdx dist = 0;
        for (ItemIdx j=0; j<n; ++j)
            if (sol_a.agent(j) != sol_b.agent(j))
                dist++;
        for (; dist>1; --dist) {
            double v_best = -1;
            ItemIdx j_best = -1;
            for (ItemIdx j=0; j<n; ++j) {
                AgentIdx ia = sol_a.agent(j);
                AgentIdx ib = sol_b.agent(j);
                if (ia == ib)
                    continue;
                sol_a.set(j, ib);
                if (v_best == -1 || v_best > sol_a.pcost()) {
                    v_best = sol_a.pcost();
                    j_best = j;
                }
                sol_a.set(j, ia);
            }
            sol_a.set(j_best, sol_b.agent(j_best));
            add(d, s, sol_a);
        }

        for (Solution sol_curr: s) {
            pr_shiftswap(d, moves, sol_curr, sol_best); // Apply shift-swap
            // Add sol_curr to pool and remove worst solution from pool.
            double v_max = sol_curr.pcost();
            Cpt l_max = -1;
            for (Cpt l=0; l<d.rho; ++l) {
                // If the solution is already in the pool, we don't add it again.
                if (sol_curr == pool[l]) {
                    l_max = -1;
                    break;
                }
                if (v_max <= pool[l].pcost()) {
                    v_max = pool[l].pcost();
                    l_max = l;
                }
            }
            if (l_max != -1)
                pool[l_max] = sol_curr;
        }
    }

    return algorithm_end(sol_best, d.info);
}

