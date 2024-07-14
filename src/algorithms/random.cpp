#include "generalizedassignmentsolver/algorithms/random.hpp"

#include "generalizedassignmentsolver/algorithm_formatter.hpp"

#include <random>
#include <algorithm>

using namespace generalizedassignmentsolver;

Solution generalizedassignmentsolver::random_infeasible(
        const Instance& instance,
        std::mt19937_64& generator)
{
    std::uniform_int_distribution<> dis(0, instance.number_of_agents() - 1);
    Solution solution(instance);
    for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
        solution.set(item_id, dis(generator));
    return solution;
}

Output generalizedassignmentsolver::random(
        const Instance& instance,
        std::mt19937_64& generator,
        const Parameters& parameters)
{
    Output output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Random");
    algorithm_formatter.print_header();

    Solution solution = random_infeasible(instance, generator);
    std::uniform_int_distribution<Counter> dis_ss(1, instance.number_of_items() * instance.number_of_agents() + (instance.number_of_items() * (instance.number_of_items() + 1)) / 2);
    std::uniform_int_distribution<ItemIdx> dis_j(0, instance.number_of_items() - 1);
    std::uniform_int_distribution<ItemIdx> dis_item_id_2(0, instance.number_of_items() - 2);
    std::uniform_int_distribution<AgentIdx> dis_i(0, instance.number_of_agents() - 2);
    std::uniform_real_distribution<double> dis(0, 1);

    Counter it_max = 2 * (instance.number_of_items() * instance.number_of_agents() + (instance.number_of_items() * (instance.number_of_items() + 1)) / 2);
    Counter it_without_change = 0;

    while (it_without_change < it_max && !parameters.timer.needs_to_end()) {
        Counter p = dis_ss(generator);
        if (p <= instance.number_of_agents() * instance.number_of_items()) { // shift
            ItemIdx item_id = dis_j(generator);
            AgentIdx agent_id = dis_i(generator);
            AgentIdx agent_id_old = solution.agent(item_id);
            if (agent_id >= agent_id_old)
                agent_id++;
            if (std::max((Weight)0, solution.weight(agent_id_old) - instance.weight(item_id, agent_id_old) - instance.capacity(agent_id_old))
                    + std::max((Weight)0, solution.weight(agent_id) + instance.weight(item_id, agent_id) - instance.capacity(agent_id))
                    <= solution.overcapacity(agent_id_old) + solution.overcapacity(agent_id)) {
                solution.set(item_id, agent_id);
                it_without_change = 0;
            } else {
                it_without_change++;
            }
        } else { // swap
            ItemIdx item_id_1 = dis_j(generator);
            ItemIdx item_id_2 = dis_item_id_2(generator);
            if (item_id_2 >= item_id_1)
                item_id_2++;
            AgentIdx agent_id_1 = solution.agent(item_id_1);
            AgentIdx agent_id_2 = solution.agent(item_id_2);
            if (agent_id_1 == agent_id_2)
                continue;
            if (std::max((Weight)0, solution.weight(agent_id_1) - instance.weight(item_id_1, agent_id_1) + instance.weight(item_id_2, agent_id_1) - instance.capacity(agent_id_1))
                    + std::max((Weight)0, solution.weight(agent_id_2) - instance.weight(item_id_2, agent_id_2) + instance.weight(item_id_1, agent_id_2) - instance.capacity(agent_id_2))
                    <= solution.overcapacity(agent_id_1) + solution.overcapacity(agent_id_2)) {
                solution.set(item_id_1, agent_id_2);
                solution.set(item_id_2, agent_id_1);
                it_without_change = 0;
            } else {
                it_without_change++;
            }
        }
        if (solution.overcapacity() == 0) {
            algorithm_formatter.update_solution(solution, "");

            algorithm_formatter.end();
            return output;
        }
    }

    algorithm_formatter.end();
    return output;
}

