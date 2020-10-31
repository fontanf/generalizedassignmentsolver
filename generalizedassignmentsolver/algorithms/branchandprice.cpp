#include "generalizedassignmentsolver/algorithms/branchandprice.hpp"

#include "generalizedassignmentsolver/algorithms/columngeneration.hpp"
#include "generalizedassignmentsolver/algorithms/greedy.hpp"

#include <queue>
#include <set>

using namespace generalizedassignmentsolver;

BranchAndPriceOutput& BranchAndPriceOutput::algorithm_end(Info& info)
{
    PUT(info, "Algorithm", "NodeNumber", node_number);
    Output::algorithm_end(info);
    VER(info, "Nodes: " << node_number << std::endl);
    return *this;
}

struct BranchAndPriceNode
{
    std::shared_ptr<BranchAndPriceNode> father = nullptr;
    ItemIdx j;
    AgentIdx i;
    int8_t v = 0;
    double value = 0;
};

BranchAndPriceOutput generalizedassignmentsolver::branchandprice(
        const Instance& instance, BranchAndPriceOptionalParameters parameters)
{
    VER(parameters.info, "*** branchandprice"
            << " --lp-solver " << parameters.lp_solver
            << " --tree-search-algorithm " << parameters.tree_search_algorithm
            << " --branching-rule " << parameters.branching_rule
            << " ***" << std::endl);
    BranchAndPriceOutput output(instance, parameters.info);

    // Initialize column generation parameters.
    std::vector<std::vector<std::vector<ItemIdx>>> columns(instance.agent_number());
    std::vector<std::vector<int>> fixed_alternatives(instance.item_number(), std::vector<int>(instance.agent_number(), -1));
    ColGenOptionalParameters colgen_parameters;
    colgen_parameters.columns   = &columns;
    colgen_parameters.fixed_alt = &fixed_alternatives;
    colgen_parameters.lp_solver = parameters.lp_solver;

    // Nodes
    auto comp = [](
            const std::shared_ptr<BranchAndPriceNode>& node_1,
            const std::shared_ptr<BranchAndPriceNode>& node_2) {
        return node_1->value < node_2->value; };
    std::multiset<std::shared_ptr<BranchAndPriceNode>, decltype(comp)> nodes(comp);

    // Root node.
    auto root = std::make_shared<BranchAndPriceNode>();
    colgen_parameters.info.set_timelimit(parameters.info.remaining_time());
    auto colgen_output = columngeneration(instance, colgen_parameters);
    std::stringstream ss;
    ss << "root node";
    output.update_lower_bound(colgen_output.lower_bound, ss, parameters.info);
    nodes.insert(root);

    while (!nodes.empty()) {
        output.node_number++;

        // Check time
        if (!parameters.info.check_time())
            return output.algorithm_end(parameters.info);

        // Get node
        auto node = *nodes.begin();
        nodes.erase(nodes.begin());

        // Initialize solution and fixed_alternatives
        Solution solution(instance);
        for (ItemIdx j = 0; j < instance.item_number(); ++j)
            std::fill(fixed_alternatives[j].begin(), fixed_alternatives[j].end(), -1);
        auto node_tmp = node;
        Counter depth = 0;
        while (node_tmp->father != nullptr) {
            //std::cout << "j " << node_tmp->j << " i " << node_tmp->i << " v " << (int)node_tmp->v << std::endl;
            if (node_tmp->v == 1) {
                solution.set(node_tmp->j, node_tmp->i);
                for (AgentIdx i = 0; i < instance.agent_number(); ++i)
                    fixed_alternatives[node_tmp->j][i] = 0;
            }
            fixed_alternatives[node_tmp->j][node_tmp->i] = node_tmp->v;
            node_tmp = node_tmp->father;
            depth++;
        }
        // Propagation
        bool stop = false;
        for (ItemIdx j = 0; j < instance.item_number(); ++j) {
            AgentPos i_remaining = 0;
            AgentIdx i_last = -1;
            for (AgentIdx i = 0; i < instance.agent_number(); ++i)
                if (fixed_alternatives[j][i] == -1
                        && solution.remaining_capacity(i) < instance.weight(j, i))
                    fixed_alternatives[j][i] = 0;
            if (solution.agent(j) != -1)
                continue;
            for (AgentIdx i = 0; i < instance.agent_number(); ++i) {
                if (fixed_alternatives[j][i] == -1) {
                    i_remaining++;
                    i_last = i;
                }
            }
            if (i_remaining == 0) {
                stop = true;
                break;
            }
            if (i_remaining == 1) {
                solution.set(j, i_last);
                fixed_alternatives[j][i_last] = 1;
            }
        }
        if (stop || solution.overcapacity() > 0)
            continue;
        if (solution.feasible()) {
            std::stringstream ss;
            ss << "node " << output.node_number;
            output.update_solution(solution, ss, parameters.info);
            if (output.optimal())
                return output.algorithm_end(parameters.info);
            continue;
        }

        // Run column generation
        auto colgen_output = columngeneration(instance, colgen_parameters);
        //for (ItemIdx j = 0; j < instance.item_number(); ++j)
            //for (AgentIdx i = 0; i < instance.agent_number(); ++i)
                //std::cout << " " << colgen_output.x[j][i];
        //std::cout << std::endl;
        if (parameters.tree_search_algorithm == "bfs") {
            std::stringstream ss;
            ss << "node " << output.node_number;
            output.update_lower_bound(colgen_output.lower_bound, ss, parameters.info);
        }
        //std::cout << "node_number " << output.node_number
            //<< " depth " << depth
            //<< " n " << solution.item_number()
            //<< " c " << solution.cost()
            //<< " oc " << solution.overcapacity()
            //<< " lb " << colgen_output.lower_bound
            //<< std::endl;

        // Heuristic
        Solution solution_tmp(solution);
        for (ItemIdx j = 0; j < instance.item_number(); ++j) {
            if (solution_tmp.agent(j) != -1)
                continue;
            AgentIdx i_best = -1;
            for (AgentIdx i = 0; i < instance.agent_number(); ++i) {
                if (fixed_alternatives[j][i] == 0)
                    continue;
                if (solution_tmp.remaining_capacity(i) < instance.weight(j, i))
                    continue;
                if (i_best == -1 || colgen_output.x[j][i_best] < colgen_output.x[j][i])
                    i_best = i;
            }
            if (i_best == -1)
                break;
            solution_tmp.set(j, i_best);
        }
        if (solution_tmp.feasible()) {
            std::stringstream ss;
            ss << "node " << output.node_number;
            output.update_solution(solution_tmp, ss, parameters.info);
            if (output.optimal())
                return output.algorithm_end(parameters.info);
        }

        // Check bound
        if (output.solution.feasible()
                && output.solution.cost() <= colgen_output.lower_bound)
            continue;

        // Compute next variable to branch on.
        //std::cout << "compute j_best / i_best" << std::endl;
        ItemIdx  j_best = -1;
        AgentIdx i_best = -1;
        if (parameters.branching_rule == "most-fractional") {
            for (ItemIdx j = 0; j < instance.item_number(); ++j) {
                for (AgentIdx i = 0; i < instance.agent_number(); ++i) {
                    if (fixed_alternatives[j][i] != -1)
                        continue;
                    if (j_best == -1
                            || std::abs(colgen_output.x[j_best][i_best] - 0.5)
                            > std::abs(colgen_output.x[j][i] - 0.5)) {
                        j_best = j;
                        i_best = i;
                    }
                }
            }
        } else {
            for (ItemIdx j = 0; j < instance.item_number(); ++j) {
                for (AgentIdx i = 0; i < instance.agent_number(); ++i) {
                    if (fixed_alternatives[j][i] != -1)
                        continue;
                    if (j_best == -1
                            || colgen_output.x[j_best][i] < colgen_output.x[j][i]) {
                        j_best = j;
                        i_best = i;
                    }
                }
            }
        }
        AgentIdx i_remaining = 0;
        for (AgentIdx i = 0; i < instance.agent_number(); ++i)
            if (fixed_alternatives[j_best][i] == -1)
                i_remaining++;

        //std::cout << "j_best " << j_best << " i_best " << i_best
            //<< " val " << colgen_output.x[j_best][i_best]
            //<< std::endl;
        if (solution.remaining_capacity(i_best) >= instance.weight(j_best, i_best)) {
            // Update solution
            solution.set(j_best, i_best);
            if (solution.feasible()) {
                std::stringstream ss;
                ss << "node " << output.node_number;
                output.update_solution(solution, ss, parameters.info);
                if (output.optimal())
                    return output.algorithm_end(parameters.info);
                continue;
            }

            // Child 1
            auto child_1 = std::make_shared<BranchAndPriceNode>();
            child_1->father = node;
            child_1->j = j_best;
            child_1->i = i_best;
            child_1->v = 1;
            if (parameters.tree_search_algorithm == "dfs") {
                child_1->value = - 0.5 - depth;
            } else if (parameters.tree_search_algorithm == "lds") {
                child_1->value = node->value;
            } else {
                fixed_alternatives[j_best][i_best] = 1;
                auto colgen_output_child = columngeneration(instance, colgen_parameters);
                child_1->value = (double)colgen_output_child.lower_bound - (double)(solution.item_number() + 1) / instance.item_number();
            }
            //std::cout << "add child 1" << std::endl;
            nodes.insert(child_1);
        }

        // Child 0
        //std::cout << "i_remaining " << i_remaining << std::endl;
        if (i_remaining >= 2) {
            auto child_0 = std::make_shared<BranchAndPriceNode>();
            child_0->father = node;
            child_0->j = j_best;
            child_0->i = i_best;
            child_0->v = 0;
            if (parameters.tree_search_algorithm == "dfs") {
                child_0->value = - depth;
            } else if (parameters.tree_search_algorithm == "lds") {
                child_0->value = node->value + colgen_output.x[j_best][i_best];
            } else {
                fixed_alternatives[j_best][i_best] = 0;
                auto colgen_output_child = columngeneration(instance, colgen_parameters);
                child_0->value = (double)colgen_output_child.lower_bound - (double)solution.item_number() / instance.item_number();
            }
            //std::cout << "add child 0" << std::endl;
            nodes.insert(child_0);
        }
    }

    output.update_lower_bound(output.solution.cost(), std::stringstream("tree search completed"), parameters.info);
    return output.algorithm_end(parameters.info);
}

