#include "generalizedassignmentsolver/algorithms/colgenheuristics.hpp"

#include "generalizedassignmentsolver/algorithms/columngeneration.hpp"

#include <set>

using namespace generalizedassignmentsolver;

/**************************** cgh_restrictedmaster ****************************/

CghRestrictedMasterOutput& CghRestrictedMasterOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    return *this;
}

CghRestrictedMasterOutput generalizedassignmentsolver::cgh_restrictedmaster(
        const Instance& instance, CghRestrictedMasterOptionalParameters parameters)
{
    VER(parameters.info, "*** cgh_restrictedmaster " << parameters.lp_solver << " ***" << std::endl);
    CghRestrictedMasterOutput output(instance, parameters.info);

    return output.algorithm_end(parameters.info);
}

/********************************* cgh_greedy *********************************/

CghGreedyOutput& CghGreedyOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    return *this;
}

CghGreedyOutput generalizedassignmentsolver::cgh_greedy(
        const Instance& instance, CghGreedyOptionalParameters parameters)
{
    VER(parameters.info, "*** cgh_greedy " << parameters.lp_solver << " ***" << std::endl);
    CghGreedyOutput output(instance, parameters.info);

    Solution solution(instance);
    std::vector<std::vector<std::vector<ItemIdx>>> columns(instance.agent_number());
    std::vector<std::vector<int>> fixed_alternatives(instance.item_number(), std::vector<int>(instance.agent_number(), -1));
    std::vector<int> fixed_agents(instance.agent_number(), 0);
    ColGenOptionalParameters colgen_parameters;
    colgen_parameters.columns      = &columns;
    colgen_parameters.fixed_alt    = &fixed_alternatives;
    colgen_parameters.fixed_agents = &fixed_agents;
    colgen_parameters.lp_solver    = parameters.lp_solver;
    colgen_parameters.info.set_timelimit(parameters.info.remaining_time());
    while (!solution.feasible()) {

        // Check time
        if (!parameters.info.check_time())
            return output.algorithm_end(parameters.info);

        //std::cout << "n " << solution.item_number()
            //<< " c " << solution.cost()
            //<< std::endl;
        auto colgen_output = columngeneration(instance, colgen_parameters);
        ColIdx col_best = -1;
        for (ColIdx col = 0; col < (ColIdx)colgen_output.column_indices.size(); ++col)
            if (col_best == -1 || colgen_output.solution[col_best] < colgen_output.solution[col])
                col_best = col;
        if (col_best == 0)
            return output.algorithm_end(parameters.info);
        AgentIdx i_best = colgen_output.column_indices[col_best].first;
        //std::cout << "i_best " << i_best << std::endl;
        for (ItemIdx j: columns[i_best][colgen_output.column_indices[col_best].second]) {
            solution.set(j, i_best);
            for (AgentIdx i = 0; i < instance.agent_number(); ++i)
                fixed_alternatives[j][i] = 0;
            fixed_alternatives[j][i_best] = 1;
        }
        fixed_agents[i_best] = 1;
    }
    output.update_solution(solution, std::stringstream(""), parameters.info);

    return output.algorithm_end(parameters.info);
}

/************************ cgh_limiteddiscrepencysearch ************************/

CghLimitedDiscrepencySearchOutput& CghLimitedDiscrepencySearchOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    return *this;
}

struct CghLimitedDiscrepencySearchNode
{
    std::shared_ptr<CghLimitedDiscrepencySearchNode> father = nullptr;
    AgentIdx i = -1;
    ColIdx col = -1;
    double v = 0;
    double value = 0;
};

CghLimitedDiscrepencySearchOutput generalizedassignmentsolver::cgh_limiteddiscrepencysearch(
        const Instance& instance, CghLimitedDiscrepencySearchOptionalParameters parameters)
{
    VER(parameters.info, "*** cgh_limiteddiscrepencysearch --lp-solver " << parameters.lp_solver << " ***" << std::endl);
    CghLimitedDiscrepencySearchOutput output(instance, parameters.info);

    // Initialize column generation parameters.
    std::vector<std::vector<std::vector<ItemIdx>>> columns(instance.agent_number());
    std::vector<std::vector<int>> fixed_alternatives(instance.item_number(), std::vector<int>(instance.agent_number(), -1));
    std::vector<int> fixed_agents(instance.agent_number(), 0);
    ColGenOptionalParameters colgen_parameters;
    colgen_parameters.columns      = &columns;
    colgen_parameters.fixed_alt    = &fixed_alternatives;
    colgen_parameters.fixed_agents = &fixed_agents;
    colgen_parameters.lp_solver    = parameters.lp_solver;
    colgen_parameters.info.set_timelimit(parameters.info.remaining_time());

    // Nodes
    auto comp = [](
            const std::shared_ptr<CghLimitedDiscrepencySearchNode>& node_1,
            const std::shared_ptr<CghLimitedDiscrepencySearchNode>& node_2) {
        return node_1->value < node_2->value; };
    std::multiset<std::shared_ptr<CghLimitedDiscrepencySearchNode>, decltype(comp)> nodes(comp);

    // Root node.
    auto root = std::make_shared<CghLimitedDiscrepencySearchNode>();
    auto colgen_output = columngeneration(instance, colgen_parameters);
    std::stringstream ss;
    ss << "root node";
    output.update_lower_bound(colgen_output.lower_bound, ss, parameters.info);
    nodes.insert(root);

    while (!nodes.empty()) {
        output.node_number++;
        //std::cout << "nodes.size() " << nodes.size() << std::endl;

        // Check time
        if (!parameters.info.check_time())
            return output.algorithm_end(parameters.info);

        // Get node
        auto node = *nodes.begin();
        nodes.erase(nodes.begin());

        // Initialize solution and fixed_agents, fixed_alternatives
        Solution solution(instance);
        std::vector<std::vector<ColIdx>> forbidden_columns(instance.agent_number());
        for (ItemIdx j = 0; j < instance.item_number(); ++j)
            std::fill(fixed_alternatives[j].begin(), fixed_alternatives[j].end(), -1);
        std::fill(fixed_agents.begin(), fixed_agents.end(), -1);
        auto node_tmp = node;
        Counter depth = 0;
        while (node_tmp->father != nullptr) {
            AgentIdx i = node_tmp->i;
            auto     v = node_tmp->v;
            ColIdx col = node_tmp->col;
            if (v == 1) {
                for (ItemIdx j: columns[i][col]) {
                    solution.set(j, i);
                    for (AgentIdx i = 0; i < instance.agent_number(); ++i)
                        fixed_alternatives[j][i] = 0;
                    fixed_alternatives[j][i] = 1;
                }
                fixed_agents[i] = 1;
            } else {
                forbidden_columns[i].push_back(col);
            }
            node_tmp = node_tmp->father;
            depth++;
        }
        //std::cout << "node " << output.node_number
            //<< " depth " << depth
            //<< " i " << node->i
            //<< " col " << node->col
            //<< " v " << node->v
            //<< " val " << node->value
            //<< " n " << solution.item_number()
            //<< " c " << solution.cost();
            //<< " tabu";
        //for (AgentIdx i = 0; i < instance.agent_number(); ++i)
            //for (ColIdx col: forbidden_columns[i])
                //std::cout << " (" << i <<  "," << col << ")";
        //std::cout << std::endl;

        // Run column generation
        auto colgen_output = columngeneration(instance, colgen_parameters);
        //std::cout << "lb " << colgen_output.lower_bound << std::endl;

        // Check bound
        if (output.solution.feasible()
                && output.solution.cost() <= colgen_output.lower_bound)
            continue;

        // Compute next column to branch on.
        ColIdx column_best_1 = 0;
        ColIdx column_best_2 = 0;
        AgentIdx i_best = -1;
        ColIdx col_best = -1;
        for (ColIdx column = 1; column < (ColIdx)colgen_output.column_indices.size(); ++column) {
            AgentIdx i = colgen_output.column_indices[column].first;
            ColIdx col = colgen_output.column_indices[column].second;
            //std::cout << "column " << column << " i " << i << " col " << col << std::endl;
            if(std::find(forbidden_columns[i].begin(), forbidden_columns[i].end(), col) != forbidden_columns[i].end())
                continue;
            if (colgen_output.solution[column] == 0)
                continue;
            if (column_best_1 == 0 || colgen_output.solution[column_best_1] < colgen_output.solution[column]) {
                column_best_2 = column_best_1;
                column_best_1 = column;
                i_best        = i;
                col_best      = col;
            } else if (column_best_2 == 0 || colgen_output.solution[column_best_2] < colgen_output.solution[column]) {
                column_best_2 = column;
            }
        }
        //std::cout << "column_best_1 " << column_best_1
            //<< " x " << colgen_output.solution[column_best_1]
            //<< " column_best_2 " << column_best_2
            //<< " x " << colgen_output.solution[column_best_2]
            //<< std::endl;
        if (column_best_1 == 0)
            continue;

        // Update solution
        for (ItemIdx j: columns[i_best][col_best])
            solution.set(j, i_best);
        if (solution.feasible()) {
            std::stringstream ss;
            ss << "node " << output.node_number;
            output.update_solution(solution, ss, parameters.info);
            if (output.optimal())
                return output.algorithm_end(parameters.info);
            continue;
        }

        // Child 1
        auto child_1 = std::make_shared<CghLimitedDiscrepencySearchNode>();
        child_1->father = node;
        child_1->i      = i_best;
        child_1->col    = col_best;
        child_1->v      = 1;
        child_1->value  = node->value;
        nodes.insert(child_1);

        // Child 0
        if (column_best_2 != 0) {
            auto child_0 = std::make_shared<CghLimitedDiscrepencySearchNode>();
            child_0->father = node;
            child_0->i      = i_best;
            child_0->col    = col_best;
            child_0->v      = 0;
            child_0->value  = node->value + colgen_output.solution[column_best_1] - colgen_output.solution[column_best_2];
            nodes.insert(child_0);
        }

    }

    return output.algorithm_end(parameters.info);
}

