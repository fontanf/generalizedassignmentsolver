#include "generalizedassignment/ub_ls_shiftswap/ls_shiftswap.hpp"
#include "generalizedassignment/ub_random/random.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace generalizedassignment;

/******************************** Local search ********************************/

LSShiftSwapOutput& LSShiftSwapOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    PUT(info, "Algorithm", "Iterations", it);
    VER(info, "Iterations: " << it << std::endl);
    return *this;
}

LSShiftSwapOutput generalizedassignment::sol_ls_shiftswap(const Instance& ins, std::mt19937_64& gen, LSShiftSwapOptionalParameters p)
{
    LSShiftSwapOutput output(ins, p.info);

    Solution sol_curr(ins);
    while (!sol_curr.feasible()) {
        if (!p.info.check_time())
            return output.algorithm_end(p.info);
        auto output_random = sol_random(ins, gen, Info().set_timelimit(p.info.remaining_time()));
        sol_curr = output_random.solution;
        output.update_solution(sol_curr, std::stringstream("initial solution"), p.info);
    }

    AgentIdx m = ins.agent_number();
    ItemIdx n = ins.item_number();
    std::uniform_int_distribution<Cpt> dis_ss(1, n * m + (n * (n + 1)) / 2);
    std::uniform_int_distribution<ItemIdx> dis_j(0, n - 1);
    std::uniform_int_distribution<ItemIdx> dis_j2(0, n - 2);
    std::uniform_int_distribution<AgentIdx> dis_i(0, m - 2);

    Cost v_curr = sol_curr.cost();
    for (; p.info.check_time();) {
        Cpt x = dis_ss(gen);
        if (x <= m * n) { // shift
            ItemIdx j = dis_j(gen);
            AgentIdx i = dis_i(gen);
            AgentIdx i_old = sol_curr.agent(j);
            if (i >= i_old)
                i++;
            sol_curr.set(j, i);
            if (!sol_curr.feasible() || v_curr < sol_curr.cost()) {
                sol_curr.set(j, i_old);
            }
        } else { // swap
            ItemIdx j1 = dis_j(gen);
            AgentIdx i1 = sol_curr.agent(j1);
            ItemIdx j2 = dis_j2(gen);
            if (j2 >= j1)
                j2++;
            AgentIdx i2 = sol_curr.agent(j2);
            if (i1 == i2)
                continue;
            sol_curr.set(j1, i2);
            sol_curr.set(j2, i1);
            if (!sol_curr.feasible() || v_curr < sol_curr.cost()) {
                sol_curr.set(j1, i1);
                sol_curr.set(j2, i2);
            }
        }
        output.it++;
        if (compare(output.solution, sol_curr)) {
            std::stringstream ss;
            ss << "it " << output.it;
            output.update_solution(sol_curr, ss, p.info);
        }
    }
    return output.algorithm_end(p.info);
}

/******************************** Tabu search *********************************/

TSShiftSwapOutput& TSShiftSwapOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    PUT(info, "Algorithm", "Iterations", it);
    PUT(info, "Algorithm", "Improving", improving_move_number);
    PUT(info, "Algorithm", "Degrading", degrading_move_number);
    VER(info, "Iterations: " << it << std::endl);
    VER(info, "Improving: " << improving_move_number << std::endl);
    VER(info, "Degrading: " << degrading_move_number << std::endl);
    return *this;
}

TSShiftSwapOutput generalizedassignment::sol_ts_shiftswap(const Instance& ins, std::mt19937_64& gen, TSShiftSwapOptionalParameters p)
{
    TSShiftSwapOutput output(ins, p.info);

    Solution sol_curr(ins);
    while (!sol_curr.feasible()) {
        if (!p.info.check_time())
            return output.algorithm_end(p.info);
        auto output_random = sol_random(ins, gen, Info().set_timelimit(p.info.remaining_time()));
        sol_curr = output_random.solution;
        output.update_solution(sol_curr, std::stringstream("initial solution"), p.info);
    }

    AgentIdx m = ins.agent_number();
    ItemIdx n = ins.item_number();
    std::uniform_int_distribution<Cpt> dis_ss(1, n * m + (n * (n + 1)) / 2);
    std::uniform_int_distribution<ItemIdx> dis_j(0, n - 1);
    std::uniform_int_distribution<ItemIdx> dis_j2(0, n - 2);
    std::uniform_int_distribution<AgentIdx> dis_i(0, m - 2);

    Cpt l = std::min(p.l, n * m + n * (n + 1) / 4);
    Cost v_curr = sol_curr.cost();
    Cost v_best = -1;
    ItemIdx j_best = -1;
    AgentIdx i_best = -1;
    ItemIdx j1_best = -1;
    ItemIdx j2_best = -1;
    for (Cpt it=0; p.info.check_time();) {
        Cpt x = dis_ss(gen);
        if (x <= m * n) { // shift
            output.it++;
            ItemIdx j = dis_j(gen);
            AgentIdx i = dis_i(gen);
            AgentIdx i_old = sol_curr.agent(j);
            if (i >= i_old)
                i++;
            sol_curr.set(j, i);
            if (sol_curr.feasible()) {
                if (compare(output.solution, sol_curr)) {
                    std::stringstream ss;
                    ss << "it " << it;
                    output.update_solution(sol_curr, ss, p.info);
                }
                if (v_curr > sol_curr.cost()) {
                    v_curr = sol_curr.cost();
                    it = 0;
                    v_best = -1;
                    j_best = -1;
                    i_best = -1;
                    j1_best = -1;
                    j2_best = -1;
                    output.improving_move_number++;
                    continue;
                }
                if (v_best == -1 || v_best > sol_curr.cost()) {
                    v_best = sol_curr.cost();
                    j_best = j;
                    i_best = i;
                    j1_best = -1;
                    j2_best = -1;
                }
            }
            sol_curr.set(j, i_old);
        } else { // swap
            ItemIdx j1 = dis_j(gen);
            AgentIdx i1 = sol_curr.agent(j1);
            ItemIdx j2 = dis_j2(gen);
            if (j2 >= j1)
                j2++;
            AgentIdx i2 = sol_curr.agent(j2);
            if (i1 == i2)
                continue;
            output.it++;
            sol_curr.set(j1, i2);
            sol_curr.set(j2, i1);

            if (sol_curr.feasible()) {
                if (compare(output.solution, sol_curr)) {
                    std::stringstream ss;
                    ss << "it " << it;
                    output.update_solution(sol_curr, ss, p.info);
                }
                if (v_curr > sol_curr.cost()) {
                    v_curr = sol_curr.cost();
                    it = 0;
                    v_best = -1;
                    j_best = -1;
                    i_best = -1;
                    j1_best = -1;
                    j2_best = -1;
                    output.improving_move_number++;
                    continue;
                }
                if (v_best == -1 || v_best > sol_curr.cost()) {
                    v_best = sol_curr.cost();
                    j_best = -1;
                    i_best = -1;
                    j1_best = j1;
                    j2_best = j2;
                }
            }

            sol_curr.set(j1, i1);
            sol_curr.set(j2, i2);
        }

        ++it;
        if (it >= l) {
            if (j_best != -1) {
                sol_curr.set(j_best, i_best);
            } else {
                AgentIdx i1 = sol_curr.agent(j1_best);
                AgentIdx i2 = sol_curr.agent(j2_best);
                sol_curr.set(j1_best, i2);
                sol_curr.set(j2_best, i1);
            }
            it = 0;
            v_best = -1;
            j_best = -1;
            i_best = -1;
            j1_best = -1;
            j2_best = -1;
            v_curr = sol_curr.cost();
            output.degrading_move_number++;
        }
    }
    return output.algorithm_end(p.info);
}

/**************************** Simulated annealing *****************************/

SAShiftSwapOutput& SAShiftSwapOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    PUT(info, "Algorithm", "Iterations", it);
    VER(info, "Iterations: " << it << std::endl);
    return *this;
}

SAShiftSwapOutput generalizedassignment::sol_sa_shiftswap(const Instance& ins, std::mt19937_64& gen, SAShiftSwapOptionalParameters p)
{
    SAShiftSwapOutput output(ins, p.info);

    Solution sol_curr(ins);
    while (!sol_curr.feasible()) {
        if (!p.info.check_time())
            return output.algorithm_end(p.info);
        auto output_random = sol_random(ins, gen, Info().set_timelimit(p.info.remaining_time()));
        sol_curr = output_random.solution;
        output.update_solution(sol_curr, std::stringstream("initial solution"), p.info);
    }

    AgentIdx m = ins.agent_number();
    ItemIdx n = ins.item_number();
    std::uniform_int_distribution<Cpt> dis_ss(1, n * m + (n * (n + 1)) / 2);
    std::uniform_int_distribution<ItemIdx> dis_j(0, n - 1);
    std::uniform_int_distribution<ItemIdx> dis_j2(0, n - 2);
    std::uniform_int_distribution<AgentIdx> dis_i(0, m - 2);
    std::uniform_real_distribution<double> dis(0, 1);

    // Compute initial temperature
    double t0 = 0;
    for (ItemIdx j=0; j<n; ++j)
        for (AgentIdx i=0; i<m; ++i)
            if (t0 < ins.alternative(j, i).c)
                t0 = ins.alternative(j, i).c;
    t0 /= 100;

    Cpt it_max = 2 * (n * m + (n * (n + 1)) / 2);
    Cpt it_without_change = 0;
    for (double t=t0; p.info.check_time(); t*=p.beta) {
        for (Cpt it=0; it<p.l && p.info.check_time();) {
            Cost v = sol_curr.cost();
            Cpt x = dis_ss(gen);
            if (x <= m * n) { // shift
                ItemIdx j = dis_j(gen);
                AgentIdx i = dis_i(gen);
                AgentIdx i_old = sol_curr.agent(j);
                if (i >= i_old)
                    i++;
                sol_curr.set(j, i);
                if (!sol_curr.feasible() || (v < sol_curr.cost()
                            && dis(gen) > exp(((double)v - sol_curr.cost()) / t))) {
                    sol_curr.set(j, i_old);
                }
            } else { // swap
                ItemIdx j1 = dis_j(gen);
                AgentIdx i1 = sol_curr.agent(j1);
                ItemIdx j2 = dis_j2(gen);
                if (j2 >= j1)
                    j2++;
                AgentIdx i2 = sol_curr.agent(j2);
                if (i1 == i2)
                    continue;
                sol_curr.set(j1, i2);
                sol_curr.set(j2, i1);
                if (!sol_curr.feasible() || (v < sol_curr.cost()
                            && dis(gen) > exp(((double)v - sol_curr.cost()) / t))) {
                    sol_curr.set(j1, i1);
                    sol_curr.set(j2, i2);
                }
            }

            // Update it_without_change
            if (sol_curr.cost() == v) {
                it_without_change++;
                if (it_without_change > it_max)
                    return output.algorithm_end(p.info);
            } else {
                it_without_change = 0;
            }

            // Update best solution
            if (compare(output.solution, sol_curr)) {
                std::stringstream ss;
                ss << "T " << t;
                output.update_solution(sol_curr, ss, p.info);
                it = 0;
            }

            ++it;
            ++output.it;
        }
    }
    return output.algorithm_end(p.info);
}

