/**
 * The linear programming formulation of the problem based on Dantzig–Wolfe
 * decomposition is written as follows:
 *
 * Variables:
 * - yᵢᵏ ∈ {0, 1} representing a set of items for agent i.
 *   yᵢᵏ = 1 iff the corresponding set of items is assigned to agent i.
 *   xⱼᵢᵏ = 1 iff yᵢᵏ contains item j, otherwise 0.
 *
 * Program:
 *
 * min ∑ᵢ ∑ₖ (∑ⱼ cᵢⱼ xⱼᵢᵏ) yᵢᵏ
 *                                      Note that (∑ⱼ cᵢⱼ xⱼᵏ) is a constant.
 *
 * 0 <= ∑ₖ yᵢᵏ <= 1      for all agents i
 *                          (not more than 1 packing selected for each agent)
 *                                                         Dual variables: uᵢ
 * 1 <= ∑ₖ xⱼᵢᵏ yᵢᵏ <= 1  for all items j
 *                                          (each item selected exactly once)
 *                                                         Dual variables: vⱼ
 *
 * The pricing problem consists in finding a variable of negative reduced cost.
 * The reduced cost of a variable yᵏ is given by:
 * rc(yᵢᵏ) = ∑ⱼ cᵢⱼ xⱼᵢᵏ - uᵢ - ∑ⱼ xⱼᵢᵏ vⱼ
 *         = - ∑ⱼ (vⱼ - cᵢⱼ) xⱼᵢᵏ - uᵢ
 *
 * Therefore, finding a variable of minium reduced cost reduces to solving
 * m Knapsack Problems with items with profit (vⱼ - cᵢⱼ).
 *
 */

#include "generalizedassignmentsolver/algorithms/column_generation.hpp"
#include "generalizedassignmentsolver/algorithm_formatter.hpp"

#include "columngenerationsolver/algorithms/greedy.hpp"
#include "columngenerationsolver/algorithms/limited_discrepancy_search.hpp"

#include "knapsacksolver/knapsack/instance_builder.hpp"
#include "knapsacksolver/knapsack/algorithms/dynamic_programming_primal_dual.hpp"
#include <iostream>
//#include "knapsacksolver/knapsack/algorithms/dynamic_programming_bellman.hpp"

using namespace generalizedassignmentsolver;

typedef columngenerationsolver::RowIdx RowIdx;
typedef columngenerationsolver::ColIdx ColIdx;
typedef columngenerationsolver::Value Value;
typedef columngenerationsolver::Column Column;

class PricingSolver: public columngenerationsolver::PricingSolver
{

public:

    PricingSolver(const Instance& instance):
        instance_(instance),
        fixed_items_(instance.number_of_items()),
        fixed_agents_(instance.number_of_agents())
    {  }

    virtual std::vector<std::shared_ptr<const Column>> initialize_pricing(
            const std::vector<std::pair<std::shared_ptr<const Column>, Value>>& fixed_columns);

    virtual std::vector<std::shared_ptr<const Column>> solve_pricing(
            const std::vector<Value>& duals);

private:

    const Instance& instance_;

    std::vector<int8_t> fixed_items_;

    std::vector<int8_t> fixed_agents_;

    std::vector<ItemIdx> kp2gap_;

};

columngenerationsolver::Model get_model(const Instance& instance)
{
    columngenerationsolver::Model model;

    model.objective_sense = optimizationtools::ObjectiveDirection::Minimize;

    // Rows.
    // Assignment constraints.
    for (AgentIdx agent_id = 0;
            agent_id < instance.number_of_agents();
            ++agent_id) {
        columngenerationsolver::Row row;
        row.lower_bound = 0;
        row.upper_bound = 1;
        row.coefficient_lower_bound = 0;
        row.coefficient_upper_bound = 1;
        model.rows.push_back(row);
    }
    // Knapsack constraints.
    for (ItemIdx item_id = 0;
            item_id < instance.number_of_items();
            ++item_id) {
        columngenerationsolver::Row row;
        row.lower_bound = 1;
        row.upper_bound = 1;
        row.coefficient_lower_bound = 0;
        row.coefficient_upper_bound = 1;
        model.rows.push_back(row);
    }

    // Pricing solver.
    model.pricing_solver = std::unique_ptr<columngenerationsolver::PricingSolver>(
            new PricingSolver(instance));

    return model;
}

Solution columns2solution(
        const Instance& instance,
        const std::vector<std::pair<std::shared_ptr<const Column>, Value>>& columns)
{
    Solution solution(instance);
    for (const auto& pair: columns) {
        const Column& column = *(pair.first);
        Value value = pair.second;
        if (value < 0.5)
            continue;
        AgentIdx agent_id = 0;
        for (const columngenerationsolver::LinearTerm& element: column.elements) {
            if (element.row < instance.number_of_agents()
                    && element.coefficient > 0.5) {
                agent_id = element.row;
            }
        }
        for (const columngenerationsolver::LinearTerm& element: column.elements) {
            if (element.row >= instance.number_of_agents()
                    && element.coefficient > 0.5) {
                ItemIdx item_id = element.row - instance.number_of_agents();
                solution.set(item_id, agent_id);
            }
        }
    }
    return solution;
}

std::vector<std::shared_ptr<const Column>> PricingSolver::initialize_pricing(
            const std::vector<std::pair<std::shared_ptr<const Column>, Value>>& fixed_columns)
{
    std::fill(fixed_items_.begin(), fixed_items_.end(), -1);
    std::fill(fixed_agents_.begin(), fixed_agents_.end(), -1);
    for (auto p: fixed_columns) {
        const Column& column = *(p.first);
        Value value = p.second;
        if (value < 0.5)
            continue;
        for (const columngenerationsolver::LinearTerm& element: column.elements) {
            if (element.coefficient < 0.5)
                continue;
            if (element.row < instance_.number_of_agents()) {
                fixed_agents_[element.row] = 1;
            } else {
                fixed_items_[element.row - instance_.number_of_agents()] = 1;
            }
        }
    }
    return {};
}

std::vector<std::shared_ptr<const Column>> PricingSolver::solve_pricing(
            const std::vector<Value>& duals)
{
    std::vector<std::shared_ptr<const Column>> columns;
    for (AgentIdx agent_id = 0;
            agent_id < instance_.number_of_agents();
            ++agent_id) {
        if (fixed_agents_[agent_id] == 1)
            continue;

        // Build subproblem instance.
        knapsacksolver::knapsack::InstanceFromFloatProfitsBuilder kp_instance_builder;
        kp_instance_builder.set_capacity(instance_.capacity(agent_id));
        kp2gap_.clear();
        for (ItemIdx item_id = 0;
                item_id < instance_.number_of_items();
                ++item_id) {
            if (fixed_items_[item_id] == 1)
                continue;
            double profit
                = duals[instance_.number_of_agents() + item_id]
                - instance_.cost(item_id, agent_id);
            if (profit <= 0
                    || instance_.weight(item_id, agent_id)
                    > instance_.capacity(agent_id))
                continue;
            kp_instance_builder.add_item(
                    profit,
                    instance_.weight(item_id, agent_id));
            kp2gap_.push_back(item_id);
        }
        const knapsacksolver::knapsack::Instance kp_instance = kp_instance_builder.build();

        // Solve subproblem instance.
        knapsacksolver::knapsack::DynamicProgrammingPrimalDualParameters kp_parameters;
        kp_parameters.verbosity_level = 0;
        auto kp_output = knapsacksolver::knapsack::dynamic_programming_primal_dual(
                kp_instance,
                kp_parameters);

        // Retrieve column.
        Column column;
        columngenerationsolver::LinearTerm element;
        element.row = agent_id;
        element.coefficient = 1;
        column.elements.push_back(element);
        for (knapsacksolver::knapsack::ItemId kp_item_id = 0;
                kp_item_id < kp_instance.number_of_items();
                ++kp_item_id) {
            if (kp_output.solution.contains(kp_item_id)) {
                ItemIdx item_id = kp2gap_[kp_item_id];
                columngenerationsolver::LinearTerm element;
                element.row = instance_.number_of_agents() + item_id;
                element.coefficient = 1;
                column.elements.push_back(element);
                column.objective_coefficient += instance_.cost(item_id, agent_id);
            }
        }
        columns.push_back(std::shared_ptr<const Column>(new Column(column)));
    }
    return columns;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

const ColumnGenerationOutput generalizedassignmentsolver::column_generation(
        const Instance& instance,
        const ColumnGenerationParameters& parameters)
{
    ColumnGenerationOutput output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Column generation");
    algorithm_formatter.print_header();

    columngenerationsolver::Model model = get_model(instance);
    columngenerationsolver::ColumnGenerationParameters cgs_parameters;
    cgs_parameters.verbosity_level = 0;
    cgs_parameters.timer = parameters.timer;
    cgs_parameters.linear_programming_solver
        = columngenerationsolver::s2lps(parameters.linear_programming_solver);
    cgs_parameters.internal_diving = true;
    cgs_parameters.self_adjusting_wentges_smoothing = true;
    cgs_parameters.automatic_directional_smoothing = true;
    auto cgscg_output = columngenerationsolver::column_generation(model, cgs_parameters);

    Cost bound = std::ceil(cgscg_output.relaxation_solution.objective_value() - FFOT_TOL);
    algorithm_formatter.update_bound(bound, "");
    output.number_of_added_columns = cgscg_output.columns.size();
    output.number_of_iterations = cgscg_output.number_of_column_generation_iterations;

    algorithm_formatter.end();
    return output;
}

const ColumnGenerationHeuristicGreedyOutput generalizedassignmentsolver::column_generation_heuristic_greedy(
        const Instance& instance,
        const ColumnGenerationParameters& parameters)
{
    ColumnGenerationHeuristicGreedyOutput output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Column generation heuristic - greedy");
    algorithm_formatter.print_header();

    columngenerationsolver::Model model = get_model(instance);
    columngenerationsolver::GreedyParameters cgsg_parameters;
    //cgsg_parameters.verbosity_level = 0;
    cgsg_parameters.timer = parameters.timer;
    cgsg_parameters.column_generation_parameters.linear_programming_solver
        = columngenerationsolver::s2lps(parameters.linear_programming_solver);
    cgsg_parameters.internal_diving = true;
    cgsg_parameters.column_generation_parameters.self_adjusting_wentges_smoothing = true;
    cgsg_parameters.column_generation_parameters.automatic_directional_smoothing = true;
    cgsg_parameters.new_solution_callback = [&instance, &algorithm_formatter](
            const columngenerationsolver::Output& cgs_output)
    {
        Cost bound = std::ceil(cgs_output.bound - FFOT_TOL);
        algorithm_formatter.update_bound(bound, "");

        if (cgs_output.solution.columns().size() > 0) {
            Solution solution = columns2solution(instance, cgs_output.solution.columns());
            algorithm_formatter.update_solution(solution, "");
        }
    };
    auto cgsg_output = columngenerationsolver::greedy(model, cgsg_parameters);

    algorithm_formatter.end();
    return output;
}

const ColumnGenerationHeuristicLimitedDiscrepancySearchOutput generalizedassignmentsolver::column_generation_heuristic_limited_discrepancy_search(
        const Instance& instance,
        const ColumnGenerationParameters& parameters)
{
    ColumnGenerationHeuristicLimitedDiscrepancySearchOutput  output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Column generation heuristic - limited discrepancy search");
    algorithm_formatter.print_header();

    columngenerationsolver::Model model = get_model(instance);
    columngenerationsolver::LimitedDiscrepancySearchParameters cgslds_parameters;
    cgslds_parameters.verbosity_level = 0;
    cgslds_parameters.timer = parameters.timer;
    cgslds_parameters.column_generation_parameters.linear_programming_solver
        = columngenerationsolver::s2lps(parameters.linear_programming_solver);
    cgslds_parameters.column_generation_parameters.self_adjusting_wentges_smoothing = true;
    cgslds_parameters.column_generation_parameters.automatic_directional_smoothing = true;
    cgslds_parameters.new_solution_callback = [&instance, &algorithm_formatter](
                const columngenerationsolver::Output& cgs_output)
        {
            const columngenerationsolver::LimitedDiscrepancySearchOutput& cgslds_output
                = static_cast<const columngenerationsolver::LimitedDiscrepancySearchOutput&>(cgs_output);
            std::stringstream ss;
            ss << "node " << cgslds_output.number_of_nodes;
            if (cgslds_output.solution.feasible()) {
                ss << " discrepancy " << cgslds_output.maximum_discrepancy;
                algorithm_formatter.update_solution(
                        columns2solution(instance, cgslds_output.solution.columns()),
                        ss.str());
            }
            Cost bound = std::ceil(cgslds_output.bound - FFOT_TOL);
            algorithm_formatter.update_bound(bound, ss.str());
        };

    auto cgslds_output = columngenerationsolver::limited_discrepancy_search(
            model,
            cgslds_parameters);

    algorithm_formatter.end();
    return output;
}
