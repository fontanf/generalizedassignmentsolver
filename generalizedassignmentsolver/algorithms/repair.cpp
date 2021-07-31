#include "generalizedassignmentsolver/algorithms/repair.hpp"

#include "generalizedassignmentsolver/algorithms/lagrelax_lbfgs.hpp"
#if COINOR_FOUND
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
#if COINOR_FOUND
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

Output generalizedassignmentsolver::repair(
        const Instance& instance,
        std::mt19937_64& generator,
        RepairOptionalParameters parameters)
{
    Output output(instance, parameters.info);

    AgentIdx m = instance.number_of_agents();
    ItemIdx  n = instance.number_of_items();

    Solution solution(instance);
    switch (parameters.initial_solution) {
    case RepairInitialSolution::CombinatorialRelaxation: {
        for (ItemIdx j = 0; j < n; ++j)
            solution.set(j, instance.item(j).i_cmin);
        output.update_lower_bound(solution.cost(), std::stringstream("combinatorialrelaxation"), parameters.info);
        break;
    } case RepairInitialSolution::LagrangianRelaxationKnapsackLbfgs: {
        auto output_lagrelax_knapsack_lbfgs = lagrelax_knapsack_lbfgs(instance);
        output.update_lower_bound(output_lagrelax_knapsack_lbfgs.lower_bound, std::stringstream("lagrangianrelaxation_knapsack"), parameters.info);
        for (ItemIdx j = 0; j < n; ++j) {
            for (AgentIdx i = 0; i < m; ++i) {
                if (output_lagrelax_knapsack_lbfgs.x[j][i] > 0.5) {
                    solution.set(j, i);
                    break;
                }
            }
        }
        break;
#if COINOR_FOUND
    } case RepairInitialSolution::LinearRelaxationClp: {
        LinRelaxClpOutput output_linrelax_clp = linrelax_clp(instance);
        for (ItemIdx j = 0; j < n; ++j) {
            AgentIdx i_best = -1;
            Cost c_best = -1;
            for (AgentIdx i=0; i<m; ++i) {
                double x = output_linrelax_clp.x[j][i];
                Cost c = instance.cost(j, i);
                if (x > 0 && (c_best == -1 || c_best > c)) {
                    i_best = i;
                    c_best = c;
                }
            }
            solution.set(j, i_best);
        }
        break;
#endif
#if CPLEX_FOUND
    } case RepairInitialSolution::LinearRelaxationCplex: {
        MilpCplexOptionalParameters parameters_linearrelaxation_cplex;
        parameters_linearrelaxation_cplex.only_linear_relaxation = true;
        auto output_linearrelaxation_cplex = milp_cplex(instance, parameters_linearrelaxation_cplex);
        output.update_lower_bound(output_linearrelaxation_cplex.lower_bound, std::stringstream("linearrelaxation_cplex"), parameters.info);
        for (ItemIdx j = 0; j < n; ++j) {
            AgentIdx i_best = -1;
            Cost c_best = -1;
            for (AgentIdx i=0; i<m; ++i) {
                double x = output_linearrelaxation_cplex.x[j][i];
                Cost c = instance.cost(j, i);
                if (x > 0 && (c_best == -1 || c_best > c)) {
                    i_best = i;
                    c_best = c;
                }
            }
            solution.set(j, i_best);
        }
        break;
#endif
    }
    }

    if (parameters.l == -1) {
        while (solution.overcapacity() > 0 && parameters.info.check_time()) {
            //std::cout << "cost " << solution.cost() << " oc " << solution.overcapacity() << std::endl;
            ItemIdx j1_best = -1;
            ItemIdx j2_best = -1;
            ItemIdx i1_best = -1;
            ItemIdx i2_best = -1;
            double v_best = -1;
            for (ItemIdx j1 = 0; j1 < n; ++j1) {
                AgentIdx i1 = solution.agent(j1);
                if (i1 == -1)
                    continue;

                // Shift
                for (AgentIdx i = 0; i < m; ++i) {
                    if (i == i1)
                        continue;
                    Weight diff = solution.overcapacity(i1) + solution.overcapacity(i)
                        - std::max((Weight)0, solution.weight(i1) - instance.weight(j1, i1) - instance.capacity(i1))
                        - std::max((Weight)0, solution.weight(i) + instance.weight(j1, i) - instance.capacity(i));
                    if (diff > 0) {
                        double v = (double)(instance.cost(j1, i) - instance.cost(j1, i1)) / diff;
                        if (j1_best < 0 || v_best > v) {
                            v_best = v;
                            j1_best = j1;
                            i1_best = i;
                            j2_best = -1;
                        }
                    }
                }

                // Swap
                for (ItemIdx j2 = j1 + 1; j2 < n; ++j2) {
                    AgentIdx i2 = solution.agent(j2);
                    if (i2 == i1 || i2 == -1)
                        continue;
                    Weight diff = solution.overcapacity(i1) + solution.overcapacity(i2)
                        - std::max((Weight)0, solution.weight(i1) - instance.weight(j1, i1) + instance.weight(j2, i1) - instance.capacity(i1))
                        - std::max((Weight)0, solution.weight(i2) - instance.weight(j2, i2) + instance.weight(j1, i2) - instance.capacity(i2));
                    if (diff > 0) {
                        double v = (double)(instance.cost(j1, i2) + instance.cost(j2, i1)
                                - instance.cost(j1, i1) - instance.cost(j2, i2)) / diff;
                        if (j1_best < 0 || v_best > v) {
                            v_best = v;
                            j1_best = j1;
                            j2_best = j2;
                            i1_best = i2;
                            i2_best = i1;
                        }
                    }
                }
            }

            if (j1_best == -1)
                break;
            solution.set(j1_best, i1_best);
            if (j2_best != -1)
                solution.set(j2_best, i2_best);
        }
    } else {
        std::uniform_int_distribution<Counter> dis_ss(1, n * m + (n * (n + 1)) / 2);
        std::uniform_int_distribution<ItemIdx> dis_j(0, n - 1);
        std::uniform_int_distribution<ItemIdx> dis_j2(0, n - 2);
        std::uniform_int_distribution<AgentIdx> dis_i(0, m - 2);

        Counter l = std::min(parameters.l, n * m + n * (n + 1) / 4);
        ItemIdx j1_best = -1;
        ItemIdx j2_best = -1;
        ItemIdx i1_best = -1;
        ItemIdx i2_best = -1;
        Cost v_best = -1;
        Counter it = 0;
        while (solution.overcapacity() > 0 && parameters.info.check_time()) {
            Counter x = dis_ss(generator);
            if (x <= m * n) { // shift
                ItemIdx j = dis_j(generator);
                AgentIdx i = dis_i(generator);
                AgentIdx i_old = solution.agent(j);
                if (i >= i_old)
                    i++;
                Weight diff = solution.overcapacity(i_old) + solution.overcapacity(i)
                    - std::max((Weight)0, solution.weight(i_old) - instance.weight(j, i_old) - instance.capacity(i_old))
                    - std::max((Weight)0, solution.weight(i) + instance.weight(j, i) - instance.capacity(i));
                if (diff > 0) {
                    double v = (double)(instance.cost(j, i) - instance.cost(j, i_old)) / diff;
                    if (j1_best < 0 || v_best > v) {
                        v_best = v;
                        j1_best = j;
                        i1_best = i;
                        j2_best = -1;
                    }
                }
            } else { // swap
                ItemIdx j1 = dis_j(generator);
                AgentIdx i1 = solution.agent(j1);
                ItemIdx j2 = dis_j2(generator);
                if (j2 >= j1)
                    j2++;
                AgentIdx i2 = solution.agent(j2);
                if (i1 == i2)
                    continue;

                Weight diff = solution.overcapacity(i1) + solution.overcapacity(i2)
                    - std::max((Weight)0, solution.weight(i1) - instance.weight(j1, i1) + instance.weight(j2, i1) - instance.capacity(i1))
                    - std::max((Weight)0, solution.weight(i2) - instance.weight(j2, i2) + instance.weight(j1, i2) - instance.capacity(i2));
                if (diff > 0) {
                    double v = (double)(instance.cost(j1, i2) + instance.cost(j2, i1)
                            - instance.cost(j1, i1) - instance.cost(j2, i2)) / diff;
                    if (j1_best < 0 || v_best > v) {
                        v_best = v;
                        j1_best = j1;
                        j2_best = j2;
                        i1_best = i2;
                        i2_best = i1;
                    }
                }
            }

            ++it;
            if (it >= l) {
                it = 0;
                if (j1_best != -1) {
                    solution.set(j1_best, i1_best);
                    if (j2_best != -1)
                        solution.set(j2_best, i2_best);
                }
                //std::cout << "cost " << solution.cost() << " oc " << solution.overcapacity() << std::endl;
                j1_best = -1;
                i1_best = -1;
                j2_best = -1;
                i2_best = -1;
                v_best = -1;
            }
        }

    }

    output.update_solution(solution, std::stringstream(""), parameters.info);
    return output.algorithm_end(parameters.info);
}

