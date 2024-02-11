#include "generalizedassignmentsolver/algorithms/repair.hpp"

#include "generalizedassignmentsolver/algorithm_formatter.hpp"
#include "generalizedassignmentsolver/algorithms/lagrelax_lbfgs.hpp"
#if CLP_FOUND
#include "generalizedassignmentsolver/algorithms/linrelax_clp.hpp"
#endif
#if CPLEX_FOUND
#include "generalizedassignmentsolver/algorithms/milp_cplex.hpp"
#endif

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace generalizedassignmentsolver;

std::istream& generalizedassignmentsolver::operator>>(std::istream& in, RepairInitialSolution& initial_solution)
{
    std::string token;
    in >> token;
    if (token == "combinatorial_relaxation") {
        initial_solution = RepairInitialSolution::CombinatorialRelaxation;
#if CLP_FOUND
    } else if (token == "linrelax_clp") {
        initial_solution = RepairInitialSolution::LinearRelaxationClp;
#endif
#if CPLEX_FOUND
    } else if (token == "linearrelaxation_cplex") {
        initial_solution = RepairInitialSolution::LinearRelaxationCplex;
#endif
    } else if (token == "lagrelax_knapsack_lbfgs") {
        initial_solution = RepairInitialSolution::LagrangianRelaxationKnapsackLbfgs;
    } else  {
        in.setstate(std::ios_base::failbit);
    }
    return in;
}

RepairOutput generalizedassignmentsolver::repair(
        const Instance& instance,
        std::mt19937_64& generator,
        const RepairParameters& parameters)
{
    RepairOutput output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Repair");
    algorithm_formatter.print_header();

    Solution solution(instance);
    switch (parameters.initial_solution) {
    case RepairInitialSolution::CombinatorialRelaxation: {
        for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
            solution.set(item_id, instance.item(item_id).minimum_cost_agent_id);
        algorithm_formatter.update_bound(
                solution.cost(),
                "combinatorial relaxation");
        break;
    } case RepairInitialSolution::LagrangianRelaxationKnapsackLbfgs: {
        auto output_lagrelax_knapsack_lbfgs = lagrelax_knapsack_lbfgs(instance);
        algorithm_formatter.update_bound(
                output_lagrelax_knapsack_lbfgs.bound,
                "lagrangian relaxation knapsack");
        for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id) {
            for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id) {
                if (output_lagrelax_knapsack_lbfgs.x[item_id][agent_id] > 0.5) {
                    solution.set(item_id, agent_id);
                    break;
                }
            }
        }
        break;
#if CLP_FOUND
    } case RepairInitialSolution::LinearRelaxationClp: {
        LinRelaxClpOutput output_linrelax_clp = linrelax_clp(instance);
        for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id) {
            AgentIdx agent_id_best = -1;
            Cost c_best = -1;
            for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id) {
                double x = output_linrelax_clp.x[item_id][agent_id];
                Cost c = instance.cost(item_id, agent_id);
                if (x > 0 && (c_best == -1 || c_best > c)) {
                    agent_id_best = agent_id;
                    c_best = c;
                }
            }
            solution.set(item_id, agent_id_best);
        }
        break;
#endif
#if CPLEX_FOUND
    } case RepairInitialSolution::LinearRelaxationCplex: {
        MilpCplexParameters parameters_linearrelaxation_cplex;
        parameters_linearrelaxation_cplex.only_linear_relaxation = true;
        auto output_linearrelaxation_cplex = milp_cplex(instance, parameters_linearrelaxation_cplex);
        algorithm_formatter.update_bound(
                output_linearrelaxation_cplex.bound,
                "linear relaxation cplex");
        for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id) {
            AgentIdx agent_id_best = -1;
            Cost c_best = -1;
            for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id) {
                double x = output_linearrelaxation_cplex.x[item_id][agent_id];
                Cost c = instance.cost(item_id, agent_id);
                if (x > 0 && (c_best == -1 || c_best > c)) {
                    agent_id_best = agent_id;
                    c_best = c;
                }
            }
            solution.set(item_id, agent_id_best);
        }
        break;
#endif
    }
    }

    if (parameters.l == -1) {
        while (solution.overcapacity() > 0 && !parameters.timer.needs_to_end()) {
            //std::cout << "cost " << solution.cost() << " oc " << solution.overcapacity() << std::endl;
            ItemIdx item_id_1_best = -1;
            ItemIdx item_id_2_best = -1;
            ItemIdx agent_id_1_best = -1;
            ItemIdx agent_id_2_best = -1;
            double v_best = -1;
            for (ItemIdx item_id_1 = 0; item_id_1 < instance.number_of_items(); ++item_id_1) {
                AgentIdx agent_id_1 = solution.agent(item_id_1);
                if (agent_id_1 == -1)
                    continue;

                // Shift
                for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id) {
                    if (agent_id == agent_id_1)
                        continue;
                    Weight diff = solution.overcapacity(agent_id_1) + solution.overcapacity(agent_id)
                        - std::max((Weight)0, solution.weight(agent_id_1) - instance.weight(item_id_1, agent_id_1) - instance.capacity(agent_id_1))
                        - std::max((Weight)0, solution.weight(agent_id) + instance.weight(item_id_1, agent_id) - instance.capacity(agent_id));
                    if (diff > 0) {
                        double v = (double)(instance.cost(item_id_1, agent_id) - instance.cost(item_id_1, agent_id_1)) / diff;
                        if (item_id_1_best < 0 || v_best > v) {
                            v_best = v;
                            item_id_1_best = item_id_1;
                            agent_id_1_best = agent_id;
                            item_id_2_best = -1;
                        }
                    }
                }

                // Swap
                for (ItemIdx item_id_2 = item_id_1 + 1; item_id_2 < instance.number_of_items(); ++item_id_2) {
                    AgentIdx agent_id_2 = solution.agent(item_id_2);
                    if (agent_id_2 == agent_id_1 || agent_id_2 == -1)
                        continue;
                    Weight diff = solution.overcapacity(agent_id_1) + solution.overcapacity(agent_id_2)
                        - std::max((Weight)0, solution.weight(agent_id_1) - instance.weight(item_id_1, agent_id_1) + instance.weight(item_id_2, agent_id_1) - instance.capacity(agent_id_1))
                        - std::max((Weight)0, solution.weight(agent_id_2) - instance.weight(item_id_2, agent_id_2) + instance.weight(item_id_1, agent_id_2) - instance.capacity(agent_id_2));
                    if (diff > 0) {
                        double v = (double)(instance.cost(item_id_1, agent_id_2) + instance.cost(item_id_2, agent_id_1)
                                - instance.cost(item_id_1, agent_id_1) - instance.cost(item_id_2, agent_id_2)) / diff;
                        if (item_id_1_best < 0 || v_best > v) {
                            v_best = v;
                            item_id_1_best = item_id_1;
                            item_id_2_best = item_id_2;
                            agent_id_1_best = agent_id_2;
                            agent_id_2_best = agent_id_1;
                        }
                    }
                }
            }

            if (item_id_1_best == -1)
                break;
            solution.set(item_id_1_best, agent_id_1_best);
            if (item_id_2_best != -1)
                solution.set(item_id_2_best, agent_id_2_best);
        }
    } else {
        std::uniform_int_distribution<Counter> dis_ss(1, instance.number_of_items() * instance.number_of_agents() + (instance.number_of_items() * (instance.number_of_items() + 1)) / 2);
        std::uniform_int_distribution<ItemIdx> dis_j(0, instance.number_of_items() - 1);
        std::uniform_int_distribution<ItemIdx> dis_item_id_2(0, instance.number_of_items() - 2);
        std::uniform_int_distribution<AgentIdx> dis_i(0, instance.number_of_agents() - 2);

        Counter l = std::min(parameters.l, instance.number_of_items() * instance.number_of_agents() + instance.number_of_items() * (instance.number_of_items() + 1) / 4);
        ItemIdx item_id_1_best = -1;
        ItemIdx item_id_2_best = -1;
        ItemIdx agent_id_1_best = -1;
        ItemIdx agent_id_2_best = -1;
        Cost v_best = -1;
        Counter it = 0;
        while (solution.overcapacity() > 0 && !parameters.timer.needs_to_end()) {
            Counter x = dis_ss(generator);
            if (x <= instance.number_of_agents() * instance.number_of_items()) { // shift
                ItemIdx item_id = dis_j(generator);
                AgentIdx agent_id = dis_i(generator);
                AgentIdx agent_id_old = solution.agent(item_id);
                if (agent_id >= agent_id_old)
                    agent_id++;
                Weight diff = solution.overcapacity(agent_id_old) + solution.overcapacity(agent_id)
                    - std::max((Weight)0, solution.weight(agent_id_old) - instance.weight(item_id, agent_id_old) - instance.capacity(agent_id_old))
                    - std::max((Weight)0, solution.weight(agent_id) + instance.weight(item_id, agent_id) - instance.capacity(agent_id));
                if (diff > 0) {
                    double v = (double)(instance.cost(item_id, agent_id) - instance.cost(item_id, agent_id_old)) / diff;
                    if (item_id_1_best < 0 || v_best > v) {
                        v_best = v;
                        item_id_1_best = item_id;
                        agent_id_1_best = agent_id;
                        item_id_2_best = -1;
                    }
                }
            } else { // swap
                ItemIdx item_id_1 = dis_j(generator);
                AgentIdx agent_id_1 = solution.agent(item_id_1);
                ItemIdx item_id_2 = dis_item_id_2(generator);
                if (item_id_2 >= item_id_1)
                    item_id_2++;
                AgentIdx agent_id_2 = solution.agent(item_id_2);
                if (agent_id_1 == agent_id_2)
                    continue;

                Weight diff = solution.overcapacity(agent_id_1) + solution.overcapacity(agent_id_2)
                    - std::max((Weight)0, solution.weight(agent_id_1) - instance.weight(item_id_1, agent_id_1) + instance.weight(item_id_2, agent_id_1) - instance.capacity(agent_id_1))
                    - std::max((Weight)0, solution.weight(agent_id_2) - instance.weight(item_id_2, agent_id_2) + instance.weight(item_id_1, agent_id_2) - instance.capacity(agent_id_2));
                if (diff > 0) {
                    double v = (double)(instance.cost(item_id_1, agent_id_2) + instance.cost(item_id_2, agent_id_1)
                            - instance.cost(item_id_1, agent_id_1) - instance.cost(item_id_2, agent_id_2)) / diff;
                    if (item_id_1_best < 0 || v_best > v) {
                        v_best = v;
                        item_id_1_best = item_id_1;
                        item_id_2_best = item_id_2;
                        agent_id_1_best = agent_id_2;
                        agent_id_2_best = agent_id_1;
                    }
                }
            }

            ++it;
            if (it >= l) {
                it = 0;
                if (item_id_1_best != -1) {
                    solution.set(item_id_1_best, agent_id_1_best);
                    if (item_id_2_best != -1)
                        solution.set(item_id_2_best, agent_id_2_best);
                }
                //std::cout << "cost " << solution.cost() << " oc " << solution.overcapacity() << std::endl;
                item_id_1_best = -1;
                agent_id_1_best = -1;
                item_id_2_best = -1;
                agent_id_2_best = -1;
                v_best = -1;
            }
        }

    }

    algorithm_formatter.update_solution(solution, "");

    algorithm_formatter.end();
    return output;
}
