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

#include "generalizedassignmentsolver/algorithms/columngeneration.hpp"

#include "columngenerationsolver/algorithms/greedy.hpp"
#include "columngenerationsolver/algorithms/limited_discrepancy_search.hpp"

#include "knapsacksolver/algorithms/dynamic_programming_primal_dual.hpp"
#include "knapsacksolver/algorithms/dynamic_programming_bellman.hpp"

using namespace generalizedassignmentsolver;

typedef columngenerationsolver::RowIdx RowIdx;
typedef columngenerationsolver::ColIdx ColIdx;
typedef columngenerationsolver::Value Value;
typedef columngenerationsolver::Column Column;

void ColumnGenerationOutput::print_statistics(
        optimizationtools::Info& info) const
{
    if (info.verbosity_level() >= 1) {
        info.os()
            << "Iterations:               " << number_of_iterations << std::endl
            << "Number of columns added:  " << number_of_added_columns << std::endl
            ;
    }
    info.add_to_json("Algorithm", "Iterations", number_of_iterations);
    info.add_to_json("Algorithm", "AddedColumns", number_of_added_columns);
}

class PricingSolver: public columngenerationsolver::PricingSolver
{

public:

    PricingSolver(const Instance& instance):
        instance_(instance),
        fixed_items_(instance.number_of_items()),
        fixed_agents_(instance.number_of_agents())
    {  }

    virtual std::vector<ColIdx> initialize_pricing(
            const std::vector<Column>& columns,
            const std::vector<std::pair<ColIdx, Value>>& fixed_columns);

    virtual std::vector<Column> solve_pricing(
            const std::vector<Value>& duals);

private:

    const Instance& instance_;

    std::vector<int8_t> fixed_items_;
    std::vector<int8_t> fixed_agents_;

    std::vector<ItemIdx> kp2gap_;

};

columngenerationsolver::Parameters get_parameters(const Instance& instance)
{
    ItemIdx n = instance.number_of_items();
    columngenerationsolver::Parameters p(instance.number_of_agents() + n);

    p.objective_sense = columngenerationsolver::ObjectiveSense::Min;
    p.column_lower_bound = 0;
    p.column_upper_bound = 1;
    // Row lower bounds.
    std::fill(
            p.row_lower_bounds.begin(),
            p.row_lower_bounds.begin() + instance.number_of_agents(),
            0);
    std::fill(
            p.row_lower_bounds.begin() + instance.number_of_agents(),
            p.row_lower_bounds.end(),
            1);
    // Row upper bounds.
    std::fill(
            p.row_upper_bounds.begin(),
            p.row_upper_bounds.begin() + instance.number_of_agents(),
            1);
    std::fill(
            p.row_upper_bounds.begin() + instance.number_of_agents(),
            p.row_upper_bounds.end(),
            1);
    // Row coefficent lower bounds.
    std::fill(
            p.row_coefficient_lower_bounds.begin(),
            p.row_coefficient_lower_bounds.end(),
            0);
    // Row coefficent upper bounds.
    std::fill(
            p.row_coefficient_upper_bounds.begin(),
            p.row_coefficient_upper_bounds.end(),
            1);
    // Dummy column objective coefficient.
    p.dummy_column_objective_coefficient = instance.bound();
    // Pricing solver.
    p.pricing_solver = std::unique_ptr<columngenerationsolver::PricingSolver>(
            new PricingSolver(instance));
    return p;
}

Solution columns2solution(
        const Instance& instance,
        const std::vector<std::pair<Column, Value>>& columns)
{
    Solution solution(instance);
    for (const auto& pair: columns) {
        const Column& column = pair.first;
        Value value = pair.second;
        if (value < 0.5)
            continue;
        AgentIdx agent_id = 0;
        for (RowIdx row_pos = 0; row_pos < (RowIdx)column.row_indices.size(); ++row_pos)
            if (column.row_indices[row_pos] < instance.number_of_agents()
                    && column.row_coefficients[row_pos] > 0.5)
                agent_id = column.row_indices[row_pos];
        for (RowIdx row_pos = 0; row_pos < (RowIdx)column.row_indices.size(); ++row_pos) {
            if (column.row_indices[row_pos] >= instance.number_of_agents()
                    && column.row_coefficients[row_pos] > 0.5) {
                ItemIdx item_id = column.row_indices[row_pos] - instance.number_of_agents();
                solution.set(item_id, agent_id);
            }
        }
    }
    return solution;
}

std::vector<ColIdx> PricingSolver::initialize_pricing(
            const std::vector<Column>& columns,
            const std::vector<std::pair<ColIdx, Value>>& fixed_columns)
{
    std::fill(fixed_items_.begin(), fixed_items_.end(), -1);
    std::fill(fixed_agents_.begin(), fixed_agents_.end(), -1);
    for (auto p: fixed_columns) {
        const Column& column = columns[p.first];
        Value value = p.second;
        if (value < 0.5)
            continue;
        for (RowIdx row_pos = 0; row_pos < (RowIdx)column.row_indices.size(); ++row_pos) {
            RowIdx row_index = column.row_indices[row_pos];
            Value row_coefficient = column.row_coefficients[row_pos];
            if (row_coefficient < 0.5)
                continue;
            if (row_index < instance_.number_of_agents()) {
                fixed_agents_[row_index] = 1;
            } else {
                fixed_items_[row_index - instance_.number_of_agents()] = 1;
            }
        }
    }
    return {};
}

std::vector<Column> PricingSolver::solve_pricing(
            const std::vector<Value>& duals)
{
    std::vector<Column> columns;
    knapsacksolver::Profit mult = 10000;
    for (AgentIdx agent_id = 0;
            agent_id < instance_.number_of_agents();
            ++agent_id) {
        if (fixed_agents_[agent_id] == 1)
            continue;

        // Build subproblem instance.
        knapsacksolver::Instance kp_instance;
        kp_instance.set_capacity(instance_.capacity(agent_id));
        kp2gap_.clear();
        for (ItemIdx item_id = 0; item_id < instance_.number_of_items(); ++item_id) {
            if (fixed_items_[item_id] == 1)
                continue;
            knapsacksolver::Profit profit
                = std::floor(mult * duals[instance_.number_of_agents() + item_id])
                - std::ceil(mult * instance_.cost(item_id, agent_id));
            if (profit <= 0
                    || instance_.weight(item_id, agent_id)
                    > instance_.capacity(agent_id))
                continue;
            kp_instance.add_item(instance_.weight(item_id, agent_id), profit);
            kp2gap_.push_back(item_id);
        }

        // Solve subproblem instance.
        auto kp_output = knapsacksolver::dynamic_programming_primal_dual(kp_instance);

        // Retrieve column.
        Column column;
        column.row_indices.push_back(agent_id);
        column.row_coefficients.push_back(1);
        for (knapsacksolver::ItemIdx kp_item_id = 0;
                kp_item_id < kp_instance.number_of_items();
                ++kp_item_id) {
            if (kp_output.solution.contains_idx(kp_item_id)) {
                ItemIdx item_id = kp2gap_[kp_item_id];
                column.row_indices.push_back(instance_.number_of_agents() + item_id);
                column.row_coefficients.push_back(1);
                column.objective_coefficient += instance_.cost(item_id, agent_id);
            }
        }
        columns.push_back(column);
    }
    return columns;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

ColumnGenerationOutput generalizedassignmentsolver::columngeneration(
        const Instance& instance,
        ColumnGenerationOptionalParameters parameters)
{
    init_display(instance, parameters.info);
    parameters.info.os()
            << "Algorithm" << std::endl
            << "---------" << std::endl
            << "Column Generation" << std::endl
            << std::endl
            << "Parameters" << std::endl
            << "----------" << std::endl
            << "Linear programming solver:  " << parameters.linear_programming_solver << std::endl
            << std::endl;

    ColumnGenerationOutput output(instance, parameters.info);

    columngenerationsolver::Parameters p = get_parameters(instance);
    columngenerationsolver::ColumnGenerationOptionalParameters op;
    op.info.set_time_limit(parameters.info.remaining_time());
    op.linear_programming_solver
        = columngenerationsolver::s2lps(parameters.linear_programming_solver);
    auto column_generation_output = columngenerationsolver::column_generation(p, op);

    output.update_bound(
            std::ceil(column_generation_output.solution_value - FFOT_TOL),
            std::stringstream(""),
            parameters.info);
    output.number_of_added_columns = column_generation_output.number_of_added_columns;
    output.number_of_iterations = column_generation_output.number_of_iterations;

    output.algorithm_end(parameters.info);
    return output;
}

ColumnGenerationHeuristicGreedyOutput generalizedassignmentsolver::columngenerationheuristic_greedy(
        const Instance& instance,
        ColumnGenerationOptionalParameters parameters)
{
    init_display(instance, parameters.info);
    parameters.info.os()
            << "Algorithm" << std::endl
            << "---------" << std::endl
            << "Column Generation Heuristic - Greedy" << std::endl
            << std::endl
            << "Parameters" << std::endl
            << "----------" << std::endl
            << "Linear programming solver:  " << parameters.linear_programming_solver << std::endl
            << std::endl;

    ColumnGenerationHeuristicGreedyOutput output(instance, parameters.info);

    columngenerationsolver::Parameters p = get_parameters(instance);
    columngenerationsolver::GreedyOptionalParameters op;
    op.info.set_time_limit(parameters.info.remaining_time());
    op.column_generation_parameters.linear_programming_solver
        = columngenerationsolver::s2lps(parameters.linear_programming_solver);
    auto greedy_output = columngenerationsolver::greedy(p, op);

    output.update_bound(
            std::ceil(greedy_output.bound - FFOT_TOL),
            std::stringstream(""),
            parameters.info);
    if (greedy_output.solution.size() > 0) {
        output.update_solution(
                columns2solution(instance, greedy_output.solution),
                std::stringstream(""),
                parameters.info);
    }

    output.algorithm_end(parameters.info);
    return output;
}

ColumnGenerationHeuristicLimitedDiscrepancySearchOutput generalizedassignmentsolver::columngenerationheuristic_limiteddiscrepancysearch(
        const Instance& instance,
        ColumnGenerationOptionalParameters parameters)
{
    init_display(instance, parameters.info);
    parameters.info.os()
            << "Algorithm" << std::endl
            << "---------" << std::endl
            << "Column Generation Heuristic - Limited Discrepancy Search" << std::endl
            << std::endl
            << "Parameters" << std::endl
            << "----------" << std::endl
            << "Linear programming solver:  " << parameters.linear_programming_solver << std::endl
            << std::endl;

    ColumnGenerationHeuristicLimitedDiscrepancySearchOutput output(instance, parameters.info);

    columngenerationsolver::Parameters p = get_parameters(instance);
    columngenerationsolver::LimitedDiscrepancySearchOptionalParameters op;
    op.column_generation_parameters.linear_programming_solver
        = columngenerationsolver::s2lps(parameters.linear_programming_solver);
    op.new_bound_callback = [&instance, &parameters, &output](
                const columngenerationsolver::LimitedDiscrepancySearchOutput& o)
        {
            std::stringstream ss;
            ss << "node " << o.number_of_nodes;
            if (o.solution.size() > 0) {
                ss << " discrepancy " << o.solution_discrepancy;
                output.update_solution(
                        columns2solution(instance, o.solution),
                        ss,
                        parameters.info);
            }
            output.update_bound(
                    std::ceil(o.bound - FFOT_TOL),
                    ss,
                    parameters.info);
        };
    op.info.set_time_limit(parameters.info.remaining_time());

    auto lds_output = columngenerationsolver::limited_discrepancy_search( p, op);

    output.algorithm_end(parameters.info);
    return output;
}
