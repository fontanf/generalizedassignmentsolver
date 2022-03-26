#include "generalizedassignmentsolver/algorithms/random.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace generalizedassignmentsolver;

Solution generalizedassignmentsolver::random_infeasible(
        const Instance& instance,
        std::mt19937_64& generator)
{
    std::uniform_int_distribution<> dis(0, instance.number_of_agents() - 1);
    Solution solution(instance);
    for (ItemIdx j=0; j<instance.number_of_items(); ++j)
        solution.set(j, dis(generator));
    return solution;
}

Output generalizedassignmentsolver::random(
        const Instance& instance,
        std::mt19937_64& generator,
        Info info)
{
    init_display(instance, info);
    VER(info,
               "Algorithm" << std::endl
            << "---------" << std::endl
            << "Random" << std::endl
            << std::endl);

    Output output(instance, info);

    Solution solution = random_infeasible(instance, generator);
    AgentIdx m = instance.number_of_agents();
    ItemIdx n = instance.number_of_items();
    std::uniform_int_distribution<Counter> dis_ss(1, n * m + (n * (n + 1)) / 2);
    std::uniform_int_distribution<ItemIdx> dis_j(0, n - 1);
    std::uniform_int_distribution<ItemIdx> dis_j2(0, n - 2);
    std::uniform_int_distribution<AgentIdx> dis_i(0, m - 2);
    std::uniform_real_distribution<double> dis(0, 1);

    Counter it_max = 2 * (n * m + (n * (n + 1)) / 2);
    Counter it_without_change = 0;

    while (it_without_change < it_max && !info.needs_to_end()) {
        Counter p = dis_ss(generator);
        if (p <= m * n) { // shift
            ItemIdx j = dis_j(generator);
            AgentIdx i = dis_i(generator);
            AgentIdx i_old = solution.agent(j);
            if (i >= i_old)
                i++;
            if (std::max((Weight)0, solution.weight(i_old) - instance.weight(j, i_old) - instance.capacity(i_old))
                    + std::max((Weight)0, solution.weight(i) + instance.weight(j, i) - instance.capacity(i))
                    <= solution.overcapacity(i_old) + solution.overcapacity(i)) {
                solution.set(j, i);
                it_without_change = 0;
            } else {
                it_without_change++;
            }
        } else { // swap
            ItemIdx j1 = dis_j(generator);
            ItemIdx j2 = dis_j2(generator);
            if (j2 >= j1)
                j2++;
            AgentIdx i1 = solution.agent(j1);
            AgentIdx i2 = solution.agent(j2);
            if (i1 == i2)
                continue;
            if (std::max((Weight)0, solution.weight(i1) - instance.weight(j1, i1) + instance.weight(j2, i1) - instance.capacity(i1))
                    + std::max((Weight)0, solution.weight(i2) - instance.weight(j2, i2) + instance.weight(j1, i2) - instance.capacity(i2))
                    <= solution.overcapacity(i1) + solution.overcapacity(i2)) {
                solution.set(j1, i2);
                solution.set(j2, i1);
                it_without_change = 0;
            } else {
                it_without_change++;
            }
        }
        if (solution.overcapacity() == 0) {
            output.update_solution(solution, std::stringstream(""), info);
            return output.algorithm_end(info);
        }
    }

    return output.algorithm_end(info);
}

