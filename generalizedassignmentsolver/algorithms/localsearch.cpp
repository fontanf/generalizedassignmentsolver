#include "generalizedassignmentsolver/algorithms/localsearch.hpp"
#include "generalizedassignmentsolver/algorithms/random.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace generalizedassignmentsolver;

/******************************** Local search ********************************/

LocalSearchOutput& LocalSearchOutput::algorithm_end(Info& info)
{
    PUT(info, "Algorithm", "Iterations", iterations);
    Output::algorithm_end(info);
    VER(info, "Iterations: " << iterations << std::endl);
    return *this;
}

LocalSearchOutput generalizedassignmentsolver::localsearch(
        const Instance& instance,
        std::mt19937_64& generator,
        LocalSearchOptionalParameters parameters)
{
    LocalSearchOutput output(instance, parameters.info);

    Solution sol_curr(instance);
    if (parameters.initial_solution != NULL)
        sol_curr = *parameters.initial_solution;
    while (!sol_curr.feasible()) {
        if (!parameters.info.check_time())
            return output.algorithm_end(parameters.info);
        auto output_random = random(instance, generator, Info().set_timelimit(parameters.info.remaining_time()));
        sol_curr = output_random.solution;
        output.update_solution(sol_curr, std::stringstream("initial solution"), parameters.info);
    }

    AgentIdx m = instance.agent_number();
    ItemIdx  n = instance.item_number();
    std::uniform_int_distribution<Counter> dis_ss(1, n * m + (n * (n + 1)) / 2);
    std::uniform_int_distribution<ItemIdx> dis_j(0, n - 1);
    std::uniform_int_distribution<ItemIdx> dis_j2(0, n - 2);
    std::uniform_int_distribution<AgentIdx> dis_i(0, m - 2);

    Cost v_curr = sol_curr.cost();
    Counter iteration_without_improvment = 0;
    for (; parameters.info.check_time();) {
        Counter x = dis_ss(generator);
        if (x <= m * n) { // shift
            ItemIdx j = dis_j(generator);
            AgentIdx i = dis_i(generator);
            AgentIdx i_old = sol_curr.agent(j);
            if (i >= i_old)
                i++;
            sol_curr.set(j, i);
            if (!sol_curr.feasible() || v_curr < sol_curr.cost()) {
                sol_curr.set(j, i_old);
            }
        } else { // swap
            ItemIdx j1 = dis_j(generator);
            AgentIdx i1 = sol_curr.agent(j1);
            ItemIdx j2 = dis_j2(generator);
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
        if (compare(output.solution, sol_curr)) {
            std::stringstream ss;
            ss << "it " << output.iterations;
            output.update_solution(sol_curr, ss, parameters.info);
            iteration_without_improvment = 0;
        }

        output.iterations++;
        if (parameters.iteration_limit > 0
                && output.iterations >= parameters.iteration_limit)
            break;
        iteration_without_improvment++;
        if (parameters.iteration_without_improvment_limit > 0
                && iteration_without_improvment > parameters.iteration_without_improvment_limit)
            break;
    }
    return output.algorithm_end(parameters.info);
}

/******************************** Tabu search *********************************/

TabuSearchOutput& TabuSearchOutput::algorithm_end(Info& info)
{
    PUT(info, "Algorithm", "Iterations", iterations);
    PUT(info, "Algorithm", "Improving", improving_move_number);
    PUT(info, "Algorithm", "Degrading", degrading_move_number);
    Output::algorithm_end(info);
    VER(info, "Iterations: " << iterations << std::endl);
    VER(info, "Improving: " << improving_move_number << std::endl);
    VER(info, "Degrading: " << degrading_move_number << std::endl);
    return *this;
}

TabuSearchOutput generalizedassignmentsolver::tabusearch(
        const Instance& instance,
        std::mt19937_64& generator,
        TabuSearchOptionalParameters parameters)
{
    TabuSearchOutput output(instance, parameters.info);

    Solution sol_curr(instance);
    if (parameters.initial_solution != NULL)
        sol_curr = *parameters.initial_solution;
    while (!sol_curr.feasible()) {
        if (!parameters.info.check_time())
            return output.algorithm_end(parameters.info);
        auto output_random = random(instance, generator, Info().set_timelimit(parameters.info.remaining_time()));
        sol_curr = output_random.solution;
        output.update_solution(sol_curr, std::stringstream("initial solution"), parameters.info);
    }

    AgentIdx m = instance.agent_number();
    ItemIdx  n = instance.item_number();
    std::uniform_int_distribution<Counter> dis_ss(1, n * m + (n * (n + 1)) / 2);
    std::uniform_int_distribution<ItemIdx> dis_j(0, n - 1);
    std::uniform_int_distribution<ItemIdx> dis_j2(0, n - 2);
    std::uniform_int_distribution<AgentIdx> dis_i(0, m - 2);

    Counter l = std::min(parameters.l, n * m + n * (n + 1) / 4);
    Cost v_curr = sol_curr.cost();
    Cost v_best = -1;
    ItemIdx j_best = -1;
    AgentIdx i_best = -1;
    ItemIdx j1_best = -1;
    ItemIdx j2_best = -1;
    Counter iteration_without_improvment = 0;
    for (Counter it = 0; parameters.info.check_time();) {
        Counter x = dis_ss(generator);
        if (x <= m * n) { // shift
            output.iterations++;
            iteration_without_improvment++;
            ItemIdx j = dis_j(generator);
            AgentIdx i = dis_i(generator);
            AgentIdx i_old = sol_curr.agent(j);
            if (i >= i_old)
                i++;
            sol_curr.set(j, i);
            if (sol_curr.feasible()) {
                if (compare(output.solution, sol_curr)) {
                    std::stringstream ss;
                    ss << "it " << it;
                    output.update_solution(sol_curr, ss, parameters.info);
                    iteration_without_improvment = 0;
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
            ItemIdx j1 = dis_j(generator);
            AgentIdx i1 = sol_curr.agent(j1);
            ItemIdx j2 = dis_j2(generator);
            if (j2 >= j1)
                j2++;
            AgentIdx i2 = sol_curr.agent(j2);
            if (i1 == i2)
                continue;
            output.iterations++;
            iteration_without_improvment++;
            sol_curr.set(j1, i2);
            sol_curr.set(j2, i1);

            if (sol_curr.feasible()) {
                if (compare(output.solution, sol_curr)) {
                    std::stringstream ss;
                    ss << "it " << it;
                    output.update_solution(sol_curr, ss, parameters.info);
                    iteration_without_improvment = 0;
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

        if (parameters.iteration_limit > 0
                && output.iterations >= parameters.iteration_limit)
            break;
        if (parameters.iteration_without_improvment_limit > 0
                && iteration_without_improvment > parameters.iteration_without_improvment_limit)
            break;

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
    return output.algorithm_end(parameters.info);
}

/**************************** Simulated annealing *****************************/

SimulatedAnnealingOutput& SimulatedAnnealingOutput::algorithm_end(Info& info)
{
    PUT(info, "Algorithm", "Iterations", iterations);
    Output::algorithm_end(info);
    VER(info, "Iterations: " << iterations << std::endl);
    return *this;
}

SimulatedAnnealingOutput generalizedassignmentsolver::simulatedannealing(
        const Instance& instance,
        std::mt19937_64& generator,
        SimulatedAnnealingOptionalParameters parameters)
{
    SimulatedAnnealingOutput output(instance, parameters.info);

    Solution sol_curr(instance);
    if (parameters.initial_solution != NULL)
        sol_curr = *parameters.initial_solution;
    while (!sol_curr.feasible()) {
        if (!parameters.info.check_time())
            return output.algorithm_end(parameters.info);
        auto output_random = random(instance, generator, Info().set_timelimit(parameters.info.remaining_time()));
        sol_curr = output_random.solution;
        output.update_solution(sol_curr, std::stringstream("initial solution"), parameters.info);
    }

    AgentIdx m = instance.agent_number();
    ItemIdx  n = instance.item_number();
    std::uniform_int_distribution<Counter> dis_ss(1, n * m + (n * (n + 1)) / 2);
    std::uniform_int_distribution<ItemIdx> dis_j(0, n - 1);
    std::uniform_int_distribution<ItemIdx> dis_j2(0, n - 2);
    std::uniform_int_distribution<AgentIdx> dis_i(0, m - 2);
    std::uniform_real_distribution<double> dis(0, 1);

    // Compute initial temperature
    double t0 = 0;
    for (ItemIdx j = 0; j < n; ++j)
        for (AgentIdx i = 0; i < m; ++i)
            if (t0 < instance.alternative(j, i).c)
                t0 = instance.alternative(j, i).c;
    t0 /= 100;

    Counter it_max = 2 * (n * m + (n * (n + 1)) / 2);
    Counter it_without_change = 0;
    Counter iteration_without_improvment = 0;
    for (double t = t0; parameters.info.check_time(); t *= parameters.beta) {
        for (Counter it = 0; it < parameters.l && parameters.info.check_time();) {
            Cost v = sol_curr.cost();
            Counter x = dis_ss(generator);
            if (x <= m * n) { // shift
                ItemIdx j = dis_j(generator);
                AgentIdx i = dis_i(generator);
                AgentIdx i_old = sol_curr.agent(j);
                if (i >= i_old)
                    i++;
                sol_curr.set(j, i);
                if (!sol_curr.feasible() || (v < sol_curr.cost()
                            && dis(generator) > exp(((double)v - sol_curr.cost()) / t))) {
                    sol_curr.set(j, i_old);
                }
            } else { // swap
                ItemIdx j1 = dis_j(generator);
                AgentIdx i1 = sol_curr.agent(j1);
                ItemIdx j2 = dis_j2(generator);
                if (j2 >= j1)
                    j2++;
                AgentIdx i2 = sol_curr.agent(j2);
                if (i1 == i2)
                    continue;
                sol_curr.set(j1, i2);
                sol_curr.set(j2, i1);
                if (!sol_curr.feasible() || (v < sol_curr.cost()
                            && dis(generator) > exp(((double)v - sol_curr.cost()) / t))) {
                    sol_curr.set(j1, i1);
                    sol_curr.set(j2, i2);
                }
            }

            // Update it_without_change
            if (sol_curr.cost() == v) {
                it_without_change++;
                if (it_without_change > it_max)
                    return output.algorithm_end(parameters.info);
            } else {
                it_without_change = 0;
            }

            // Update best solution
            if (compare(output.solution, sol_curr)) {
                std::stringstream ss;
                ss << "T " << t;
                output.update_solution(sol_curr, ss, parameters.info);
                it = 0;
                iteration_without_improvment = 0;
            }

            it++;
            output.iterations++;
            if (parameters.iteration_limit > 0
                    && output.iterations >= parameters.iteration_limit)
                return output.algorithm_end(parameters.info);
            iteration_without_improvment++;
            if (parameters.iteration_without_improvment_limit > 0
                    && iteration_without_improvment > parameters.iteration_without_improvment_limit)
                return output.algorithm_end(parameters.info);
        }
    }
    return output.algorithm_end(parameters.info);
}

