#include "generalizedassignmentsolver/algorithms/columngeneration.hpp"

#include "columngenerationsolver/algorithms/greedy.hpp"
#include "columngenerationsolver/algorithms/limited_discrepancy_search.hpp"

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

using namespace generalizedassignmentsolver;

typedef columngenerationsolver::RowIdx RowIdx;
typedef columngenerationsolver::ColIdx ColIdx;
typedef columngenerationsolver::Value Value;
typedef columngenerationsolver::Column Column;

ColumnGenerationOutput& ColumnGenerationOutput::algorithm_end(Info& info)
{
    PUT(info, "Algorithm", "Iterations", number_of_iterations);
    PUT(info, "Algorithm", "AddedColumns", number_of_added_columns);
    Output::algorithm_end(info);
    VER(info, "Iterations: " << number_of_iterations << std::endl);
    VER(info, "Added columns: " << number_of_added_columns << std::endl);
    return *this;
}

ColumnGenerationHeuristicGreedyOutput& ColumnGenerationHeuristicGreedyOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    return *this;
}

ColumnGenerationHeuristicLimitedDiscrepancySearchOutput& ColumnGenerationHeuristicLimitedDiscrepancySearchOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    return *this;
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
    AgentIdx m = instance.number_of_agents();
    ItemIdx n = instance.number_of_items();
    columngenerationsolver::Parameters p(m + n);

    p.objective_sense = columngenerationsolver::ObjectiveSense::Min;
    p.column_lower_bound = 0;
    p.column_upper_bound = 1;
    // Row lower bounds.
    std::fill(p.row_lower_bounds.begin(), p.row_lower_bounds.begin() + m, 0);
    std::fill(p.row_lower_bounds.begin() + m, p.row_lower_bounds.end(), 1);
    // Row upper bounds.
    std::fill(p.row_upper_bounds.begin(), p.row_upper_bounds.begin() + m, 1);
    std::fill(p.row_upper_bounds.begin() + m, p.row_upper_bounds.end(), 1);
    // Row coefficent lower bounds.
    std::fill(p.row_coefficient_lower_bounds.begin(), p.row_coefficient_lower_bounds.end(), 0);
    // Row coefficent upper bounds.
    std::fill(p.row_coefficient_upper_bounds.begin(), p.row_coefficient_upper_bounds.end(), 1);
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
    AgentIdx m = instance.number_of_agents();
    Solution solution(instance);
    for (const auto& pair: columns) {
        const Column& column = pair.first;
        Value value = pair.second;
        if (value < 0.5)
            continue;
        AgentIdx i = 0;
        for (RowIdx row_pos = 0; row_pos < (RowIdx)column.row_indices.size(); ++row_pos)
            if (column.row_indices[row_pos] < m && column.row_coefficients[row_pos] > 0.5)
                i = column.row_indices[row_pos];
        for (RowIdx row_pos = 0; row_pos < (RowIdx)column.row_indices.size(); ++row_pos)
            if (column.row_indices[row_pos] >= m && column.row_coefficients[row_pos] > 0.5)
                solution.set(column.row_indices[row_pos] - m, i);
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
    AgentIdx m = instance_.number_of_agents();
    ItemIdx n = instance_.number_of_items();
    std::vector<Column> columns;
    knapsacksolver::Profit mult = 10000;
    for (AgentIdx i = 0; i < instance_.number_of_agents(); ++i) {
        if (fixed_agents_[i] == 1)
            continue;
        // Build subproblem instance.
        knapsacksolver::Instance instance_kp;
        instance_kp.set_capacity(instance_.capacity(i));
        kp2gap_.clear();
        for (ItemIdx j = 0; j < n; ++j) {
            if (fixed_items_[j] == 1)
                continue;
            knapsacksolver::Profit profit = std::floor(mult * duals[m + j])
                    - std::ceil(mult * instance_.cost(j, i));
            if (profit <= 0 || instance_.weight(j, i) > instance_.capacity(i))
                continue;
            instance_kp.add_item(instance_.weight(j, i), profit);
            kp2gap_.push_back(j);
        }

        // Solve subproblem instance.
        auto output_kp = knapsacksolver::minknap(instance_kp);

        // Retrieve column.
        Column column;
        column.row_indices.push_back(i);
        column.row_coefficients.push_back(1);
        for (knapsacksolver::ItemIdx j = 0; j < instance_kp.number_of_items(); ++j) {
            if (output_kp.solution.contains_idx(j)) {
                column.row_indices.push_back(m + kp2gap_[j]);
                column.row_coefficients.push_back(1);
                column.objective_coefficient += instance_.cost(kp2gap_[j], i);
            }
        }
        columns.push_back(column);
    }
    return columns;
}

/******************************************************************************/

ColumnGenerationOutput generalizedassignmentsolver::columngeneration(
        const Instance& instance, ColumnGenerationOptionalParameters parameters)
{
    VER(parameters.info, "*** columngeneration"
            << " --linear-programming-solver " << parameters.linear_programming_solver
            << " ***" << std::endl);
    ColumnGenerationOutput output(instance, parameters.info);

    columngenerationsolver::Parameters p = get_parameters(instance);
    columngenerationsolver::ColumnGenerationOptionalParameters op;
    op.info.set_time_limit(parameters.info.remaining_time());
    op.linear_programming_solver
        = columngenerationsolver::s2lps(parameters.linear_programming_solver);
    auto column_generation_output = columngenerationsolver::column_generation(p, op);

    output.update_lower_bound(
            std::ceil(column_generation_output.solution_value - TOL),
            std::stringstream(""),
            parameters.info);
    output.number_of_added_columns = column_generation_output.number_of_added_columns;
    output.number_of_iterations = column_generation_output.number_of_iterations;
    return output.algorithm_end(parameters.info);
}

ColumnGenerationHeuristicGreedyOutput generalizedassignmentsolver::columngenerationheuristic_greedy(
        const Instance& instance, ColumnGenerationOptionalParameters parameters)
{
    VER(parameters.info, "*** columngenerationheuristic_greedy"
            << " --linear-programming-solver " << parameters.linear_programming_solver
            << " ***" << std::endl);
    ColumnGenerationHeuristicGreedyOutput output(instance, parameters.info);

    columngenerationsolver::Parameters p = get_parameters(instance);
    columngenerationsolver::GreedyOptionalParameters op;
    op.info.set_time_limit(parameters.info.remaining_time());
    op.column_generation_parameters.linear_programming_solver
        = columngenerationsolver::s2lps(parameters.linear_programming_solver);
    auto output_greedy = columngenerationsolver::greedy(p, op);

    output.update_lower_bound(
            std::ceil(output_greedy.bound - TOL),
            std::stringstream(""),
            parameters.info);
    if (output_greedy.solution.size() > 0)
        output.update_solution(
                columns2solution(instance, output_greedy.solution),
                std::stringstream(""),
                parameters.info);
    return output.algorithm_end(parameters.info);
}

ColumnGenerationHeuristicLimitedDiscrepancySearchOutput generalizedassignmentsolver::columngenerationheuristic_limiteddiscrepancysearch(
        const Instance& instance, ColumnGenerationOptionalParameters parameters)
{
    VER(parameters.info, "*** columngenerationheuristic_limiteddiscrepancysearch"
            << " --linear-programming-solver " << parameters.linear_programming_solver
            << " ***" << std::endl);
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
            output.update_lower_bound(
                    std::ceil(o.bound - TOL),
                    ss,
                    parameters.info);
        };
    op.info.set_time_limit(parameters.info.remaining_time());

    auto output_limited_discrepancy_search = columngenerationsolver::limited_discrepancy_search( p, op);
    return output.algorithm_end(parameters.info);
}
