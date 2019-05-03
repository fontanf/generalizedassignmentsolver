#include "gap/ub_ls_shiftswap/ls_shiftswap.hpp"
#include "gap/ub_random/random.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace gap;

Solution gap::sol_lsfirst_shiftswap(LSFirstShiftSwapData d)
{
    init_display(d.info);
    AgentIdx m = d.ins.agent_number();
    ItemIdx n = d.ins.item_number();
    std::uniform_int_distribution<Cpt> dis_ss(1, n * m + (n * (n + 1)) / 2);
    std::uniform_int_distribution<ItemIdx> dis_j(0, n - 1);
    std::uniform_int_distribution<ItemIdx> dis_j2(0, n - 2);
    std::uniform_int_distribution<AgentIdx> dis_i(0, m - 1);
    std::uniform_int_distribution<AgentIdx> dis_i1(0, m - 2);
    std::uniform_int_distribution<AgentIdx> dis_i2(0, m - 3);

    Solution sol_best(d.ins);
    Solution sol_curr(d.ins);
    for (ItemIdx j=0; j<d.ins.item_number(); ++j)
        sol_curr.set(j, dis_i(d.gen));

    Cpt it_max = 2 * (n * m + (n * (n + 1)) / 2);
    Cpt it_without_change = 0;

    while (it_without_change < it_max) {
        double v = sol_curr.value(d.alpha);
        Cpt p = dis_ss(d.gen);
        if (p <= m * n) { // shift
            ItemIdx j = dis_j(d.gen);
            AgentIdx i = dis_i1(d.gen);
            AgentIdx i_old = sol_curr.agent(j);
            if (i >= i_old)
                i++;
            sol_curr.set(j, i);
            double v_curr = sol_curr.value(d.alpha);
            if (v < v_curr) {
                sol_curr.set(j, i_old);
            }
        } else { // swap
            ItemIdx j1 = dis_j(d.gen);
            ItemIdx j2 = dis_j2(d.gen);
            if (j2 >= j1)
                j2++;
            AgentIdx i1 = sol_curr.agent(j1);
            AgentIdx i2 = sol_curr.agent(j2);
            sol_curr.set(j1, i2);
            sol_curr.set(j2, i1);
            double v_curr = sol_curr.value(d.alpha);
            if (v < v_curr) {
                sol_curr.set(j1, i1);
                sol_curr.set(j2, i2);
            }
        }

        // Update it_without_change
        if (sol_curr.feasible() && (!sol_best.feasible() || sol_best.value() > sol_curr.value())) {
            std::stringstream ss;
            sol_best.update(sol_curr, 0, ss, d.info);
            it_without_change = 0;
        } else {
            it_without_change++;
        }
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
    for (ItemIdx j=0; j<d.ins.item_number(); ++j)
        sol_curr.set(j, dis_i(d.gen));

    for (Cpt it=0;; ++it) {
        double v_best = sol_curr.value(d.alpha);

        ItemIdx j_best = -1;
        AgentIdx i_best = -1;
        for (ItemIdx j=0; j<n; ++j) {
            AgentIdx i_old = sol_curr.agent(j);
            for (AgentIdx i=0; i<m; ++i) {
                if (i == i_old)
                    continue;
                sol_curr.set(j, i);
                double v = sol_curr.value(d.alpha);
                if (v_best > v) {
                    v_best = v;
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
                double v = sol_curr.value(d.alpha);
                if (v_best > v) {
                    v_best = v;
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
        if (j1_best != -1) {
            AgentIdx i1 = sol_curr.agent(j1_best);
            AgentIdx i2 = sol_curr.agent(j2_best);
            sol_curr.set(j1_best, i2);
            sol_curr.set(j2_best, i1);
        } else {
            sol_curr.set(j_best, i_best);
        }

        // Update best solution
        if (sol_curr.feasible() && (!sol_best.feasible() || sol_best.value() > sol_curr.value())) {
            std::stringstream ss;
            ss << "it " << it;
            sol_best.update(sol_curr, 0, ss, d.info);
        }
    }

    return algorithm_end(sol_best, d.info);
}

/******************************************************************************/

bool gap::doubleshift_best(const Instance& ins, Solution& sol, Info& info)
{
    double v_best = sol.value();

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
                if (sol.overcapacity() == 0 && v_best > sol.value()) {
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
    double v_best = sol.value();

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
                if (sol.overcapacity() == 0 && v_best > sol.value()) {
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

Solution gap::sol_ts_shiftswap(TSShiftSwapData d)
{
    init_display(d.info);
    AgentIdx m = d.ins.agent_number();
    ItemIdx n = d.ins.item_number();
    std::uniform_int_distribution<Cpt> dis_ss(1, n * m + (n * (n + 1)) / 2);
    std::uniform_int_distribution<ItemIdx> dis_j(0, n - 1);
    std::uniform_int_distribution<ItemIdx> dis_j2(0, n - 2);
    std::uniform_int_distribution<AgentIdx> dis_i(0, m - 1);
    std::uniform_int_distribution<AgentIdx> dis_i1(0, m - 2);
    std::uniform_int_distribution<AgentIdx> dis_i2(0, m - 3);
    std::uniform_real_distribution<double> dis(0, 1);

    Solution sol_best(d.ins);
    Solution sol_curr(d.ins);
    for (ItemIdx j=0; j<n; ++j)
        sol_curr.set(j, dis_i(d.gen));

    Cpt neig_size = 10000;
    Cpt tabu_size = n / 20;

    std::vector<std::vector<Cpt>> tabu(n, std::vector<Cpt>(m, - tabu_size - 1));
    Cpt neigh_totalsize = n * m + (n * (n + 1)) / 2;

    for (Cpt it=0; d.info.check_time(); ++it) {
        if (neig_size < neigh_totalsize / 2)
            neig_size++;

        double v_best = -1;
        ItemIdx j_best = -1;
        AgentIdx i_best = -1;
        ItemIdx j1_best = -1;
        ItemIdx j2_best = -1;

        for (Cpt it2=0; it2<neig_size;) {
            Cpt p = dis_ss(d.gen);
            if (p <= m * n) { // shift
                ItemIdx j = dis_j(d.gen);
                AgentIdx i_old = sol_curr.agent(j);
                AgentIdx i = dis_i1(d.gen);
                if (i >= i_old)
                    i++;

                if (it - tabu[j][i] < tabu_size)
                    continue;

                sol_curr.set(j, i);
                double v = sol_curr.value(d.alpha);
                if (v_best == -1 || v_best > v) {
                    v_best = v;
                    j_best = j;
                    i_best = i;
                    j1_best = -1;
                    j2_best = -1;
                }
                sol_curr.set(j, i_old);
            } else { // swap
                ItemIdx j1 = dis_j(d.gen);
                AgentIdx i1 = sol_curr.agent(j1);

                ItemIdx j2 = dis_j2(d.gen);
                if (j2 >= j1)
                    j2++;
                AgentIdx i2 = sol_curr.agent(j2);

                if (i1 == i2)
                    continue;
                if (it - tabu[j1][i2] < tabu_size)
                    continue;
                if (it - tabu[j2][i1] < tabu_size)
                    continue;

                sol_curr.set(j1, i2);
                sol_curr.set(j2, i1);
                double v = sol_curr.value(d.alpha);
                if (v_best == -1 || v_best > v) {
                    v_best = v;
                    j_best = -1;
                    i_best = -1;
                    j1_best = j1;
                    j2_best = j2;
                }
                sol_curr.set(j1, i1);
                sol_curr.set(j2, i2);
            }

            ++it2;
        }

        if (v_best == -1) {
            std::cout << "toto" << std::endl;
            continue;
        }

        if (j1_best != -1) {
            AgentIdx i1 = sol_curr.agent(j1_best);
            AgentIdx i2 = sol_curr.agent(j2_best);
            if (v_best >= sol_curr.value(d.alpha)) {
                tabu[j1_best][i1] = it;
                tabu[j2_best][i2] = it;
            }
            sol_curr.set(j1_best, i2);
            sol_curr.set(j2_best, i1);
        } else {
            if (v_best >= sol_curr.value(d.alpha))
                tabu[j_best][sol_curr.agent(j_best)] = it;
            sol_curr.set(j_best, i_best);
        }

        // Update best solution
        if (sol_curr.feasible() && (!sol_best.feasible() || sol_best.value() > sol_curr.value())) {
            std::stringstream ss;
            ss << "it " << it;
            sol_best.update(sol_curr, 0, ss, d.info);
        }
    }
    return algorithm_end(sol_best, d.info);
}

/******************************************************************************/

Solution gap::sol_sa_shiftswap(SAShiftSwapData d)
{
    init_display(d.info);
    AgentIdx m = d.ins.agent_number();
    ItemIdx n = d.ins.item_number();
    std::uniform_int_distribution<Cpt> dis_ss(1, n * m + (n * (n + 1)) / 2);
    std::uniform_int_distribution<ItemIdx> dis_j(0, n - 1);
    std::uniform_int_distribution<ItemIdx> dis_j2(0, n - 2);
    std::uniform_int_distribution<AgentIdx> dis_i(0, m - 1);
    std::uniform_int_distribution<AgentIdx> dis_i1(0, m - 2);
    std::uniform_int_distribution<AgentIdx> dis_i2(0, m - 3);
    std::uniform_real_distribution<double> dis(0, 1);

    Solution sol_best(d.ins);
    Solution sol_curr(d.ins);
    for (ItemIdx j=0; j<n; ++j)
        sol_curr.set(j, dis_i(d.gen));

    // Compute initial temperature
    double t0 = 0;
    for (ItemIdx j=0; j<n; ++j)
        for (AgentIdx i=0; i<m; ++i)
            if (t0 < d.ins.alternative(j, i).v)
                t0 = d.ins.alternative(j, i).v;

    Cpt it_max = 2 * (n * m + (n * (n + 1)) / 2);
    Cpt it_without_change = 0;
    for (double t=t0; d.info.check_time(); t*=d.beta) {
        for (Cpt it=0; it<d.l;) {
            double v = sol_curr.value(d.alpha);
            Cpt p = dis_ss(d.gen);
            if (p <= m * n) { // shift
                ItemIdx j = dis_j(d.gen);
                AgentIdx i = dis_i1(d.gen);
                AgentIdx i_old = sol_curr.agent(j);
                if (i >= i_old)
                    i++;
                sol_curr.set(j, i);
                double v_curr = sol_curr.value(d.alpha);
                if ((v < v_curr && dis(d.gen) > exp((double)(v - v_curr) / t))) {
                    sol_curr.set(j, i_old);
                }
            } else { // swap
                ItemIdx j1 = dis_j(d.gen);
                AgentIdx i1 = sol_curr.agent(j1);

                ItemIdx j2 = dis_j2(d.gen);
                AgentIdx i2 = sol_curr.agent(j2);

                if (i1 == i2)
                    continue;

                sol_curr.set(j1, i2);
                sol_curr.set(j2, i1);
                double v_curr = sol_curr.value(d.alpha);
                if ((v < v_curr && dis(d.gen) > exp((double)(v - v_curr) / t))) {
                    sol_curr.set(j1, i1);
                    sol_curr.set(j2, i2);
                }
            }

            // Update it_without_change
            if (sol_curr.value(d.alpha) == v) {
                it_without_change++;
                if (it_without_change > it_max)
                    return algorithm_end(sol_best, d.info);
            } else {
                it_without_change = 0;
            }

            // Update best solution
            if (sol_curr.feasible() && (!sol_best.feasible() || sol_best.value() > sol_curr.value())) {
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

void pr_shiftswap(PRShiftSwapData& d, Solution& sol_curr, Solution& sol_best)
{
    AgentIdx m = d.ins.agent_number();
    ItemIdx n = d.ins.item_number();
    std::uniform_int_distribution<Cpt> dis_ss(1, n * m + (n * (n + 1)) / 2);
    std::uniform_int_distribution<ItemIdx> dis_j(0, n - 1);
    std::uniform_int_distribution<ItemIdx> dis_j2(0, n - 2);
    std::uniform_int_distribution<AgentIdx> dis_i(0, m - 1);
    std::uniform_int_distribution<AgentIdx> dis_i1(0, m - 2);
    std::uniform_int_distribution<AgentIdx> dis_i2(0, m - 3);

    Cpt it_max = n * m;
    Cpt it_without_change = 0;

    while (it_without_change < it_max) {
        double v = sol_curr.value(d.alpha);
        Cpt p = dis_ss(d.gen);
        if (p <= m * n) { // shift
            ItemIdx j = dis_j(d.gen);
            AgentIdx i = dis_i1(d.gen);
            AgentIdx i_old = sol_curr.agent(j);
            if (i >= i_old)
                i++;
            sol_curr.set(j, i);
            double v_curr = sol_curr.value(d.alpha);
            if (v < v_curr) {
                sol_curr.set(j, i_old);
            }
        } else { // swap
            ItemIdx j1 = dis_j(d.gen);
            ItemIdx j2 = dis_j2(d.gen);
            if (j2 >= j1)
                j2++;
            AgentIdx i1 = sol_curr.agent(j1);
            AgentIdx i2 = sol_curr.agent(j2);
            sol_curr.set(j1, i2);
            sol_curr.set(j2, i1);
            double v_curr = sol_curr.value(d.alpha);
            if (v < v_curr) {
                sol_curr.set(j1, i1);
                sol_curr.set(j2, i2);
            }
        }

        // Update it_without_change
        if (sol_curr.feasible() && (!sol_best.feasible() || sol_best.value() > sol_curr.value())) {
            std::stringstream ss;
            sol_best.update(sol_curr, 0, ss, d.info);
            it_without_change = 0;
        } else {
            it_without_change++;
        }
    }
}

template <class Set>
void add(PRShiftSwapData& d, Set& set, Solution& sol)
{
    if ((Cpt)set.size() < d.gamma ||
            (sol.value(d.alpha) < std::prev(set.end())->value(d.alpha))) {
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
    std::uniform_int_distribution<AgentIdx> dis_i(0, m - 1);
    std::uniform_int_distribution<AgentIdx> dis_i1(0, m - 2);
    std::uniform_int_distribution<ItemIdx> dis_j(0, n - 1);
    std::uniform_int_distribution<Cpt> dis_sigma(0, 1);
    std::uniform_int_distribution<Cpt> dis_a(0, d.rho - 1);
    std::uniform_int_distribution<Cpt> dis_b(0, d.rho - 2);

    std::vector<Solution> pool(d.rho, Solution(d.ins));
    for (Solution& sol_curr: pool) {
        for (ItemIdx j=0; j<d.ins.item_number(); ++j)
            sol_curr.set(j, dis_i(d.gen));
        pr_shiftswap(d, sol_curr, sol_best);
    }

    auto cmp = [&d](const Solution& s1, const Solution& s2) { return s1.value(d.alpha) < s2.value(d.alpha); };
    for (;;) {
        //double val = 0;
        //for (Solution& sol_curr: pool) {
            //std::cout << sol_curr.value(d.alpha) << " ";
            //val += sol_curr.value(d.alpha);
        //}
        //std::cout << "average " << val/d.rho << std::endl;

        // Get solutions a and b
        Cpt a = dis_a(d.gen);
        Cpt b = dis_b(d.gen);
        if (b >= a)
            b++;

        // With probability 1/2, replace solution b by a shift neighbor
        Solution sol_a = pool[a];
        Solution sol_b = pool[b];
        if (dis_sigma(d.gen) == 0) {
            ItemIdx j = dis_j(d.gen);
            AgentIdx i = dis_i1(d.gen);
            if (i >= sol_b.agent(j))
                i++;
            sol_b.set(j, i);
        }

        std::multiset<Solution, decltype(cmp)> s(cmp);

        // Fill s with N'shift(sol_a, sol_b)
        {
            double v_best = -1;
            ItemIdx j_best = -1;
            for (ItemIdx j=0; j<n; ++j) {
                AgentIdx i_old = sol_a.agent(j);
                AgentIdx i = sol_b.agent(j);
                if (i == i_old)
                    continue;
                sol_a.set(j, i);
                add(d, s, sol_a);
                double v = sol_a.value(d.alpha);
                if (v_best == -1 || v_best > v) {
                    v_best = v;
                    j_best = j;
                }
                sol_a.set(j, i_old);
            }
            sol_a.set(j_best, sol_b.agent(j_best));
        }

        // Fill s with sol_2 ... sol_{dist - 1}
        ItemIdx dist = 0;
        for (ItemIdx j=0; j<n; ++j)
            if (sol_a.agent(j) != sol_b.agent(j))
                dist++;
        //std::cout << "dist " << dist << std::endl;
        for (; dist>1; --dist) {
            double v_best = -1;
            ItemIdx j_best = -1;
            for (ItemIdx j=0; j<n; ++j) {
                AgentIdx i_old = sol_a.agent(j);
                AgentIdx i = sol_b.agent(j);
                if (i == i_old)
                    continue;
                sol_a.set(j, i);
                double v = sol_a.value(d.alpha);
                if (v_best == -1 || v_best > v) {
                    v_best = v;
                    j_best = j;
                }
                sol_a.set(j, i_old);
            }
            sol_a.set(j_best, sol_b.agent(j_best));
            add(d, s, sol_a);
        }

        for (Solution sol_curr: s) {
            //std::cout << " v before " << sol_curr.value(d.alpha);

            // Apply shift-swap
            pr_shiftswap(d, sol_curr, sol_best);
            //std::cout << " v after " << sol_curr.value(d.alpha);

            // Add sol_curr to pool and remove worst solution from pool.
            double v_max = sol_curr.value(d.alpha);
            Cpt l_max = -1;
            for (Cpt l=0; l<d.rho; ++l) {
                // If the solution is already in the pool, we don't add it again.
                if (sol_curr == pool[l]) {
                    //std::cout << " already";
                    l_max = -1;
                    break;
                }
                double v = pool[l].value(d.alpha);
                if (v_max <= v) {
                    v_max = v;
                    l_max = l;
                }
            }
            //std::cout << " l_max " << l_max << std::endl;
            if (l_max != -1)
                pool[l_max] = sol_curr;
        }
    }

    return algorithm_end(sol_best, d.info);
}

