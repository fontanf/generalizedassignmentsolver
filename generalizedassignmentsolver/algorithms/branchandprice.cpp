#include "generalizedassignmentsolver/algorithms/branchandprice.hpp"

#include "generalizedassignmentsolver/algorithms/columngeneration.hpp"
#include "generalizedassignmentsolver/algorithms/greedy.hpp"

#include <queue>

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

    void filter()
    {
        const Instance& ins = sol_curr.instance();
        for (ItemIdx j = 0; j < ins.item_number(); ++j) {
            AgentIdx i0 = -1;
            for (AgentIdx i = 0; i < ins.agent_number(); ++i) {
                AltIdx k = ins.alternative_index(j, i);
                if ((*p_colgen.fixed_alt)[k] == 1) {
                    i0 = ins.agent_number();
                    break;
                } else if ((*p_colgen.fixed_alt)[k] == -1 && ins.alternative(k).w <= sol_curr.remaining_capacity(i)) {
                    if (i0 < 0) {
                        i0 = i;
                    } else {
                        i0 = ins.agent_number();
                        break;
                    }
                }
            }
            if (i0 < 0) {
                feasible_ = false;
                return;
            }
            if (i0 == ins.agent_number())
                continue;
            //std::cout << "set " << ins.alternative(k).j << " to " << ins.alternative(k).i << std::endl;
            sol_curr.set(j, i0);
            for (AgentIdx i = 0; i < ins.agent_number(); ++i)
                (*p_colgen.fixed_alt)[ins.alternative_index(j, i)] = 0;
            (*p_colgen.fixed_alt)[ins.alternative_index(j, i0)] = 1;
        }
    }

    BranchAndPriceNode(const Instance& ins, BranchAndPriceOptionalParameters& p,
            std::vector<std::vector<std::vector<ItemIdx>>>* columns):
        sol_curr(ins),
        p_colgen({
                .info = Info(),
                .solver = p.solver,
                .columns = columns,
                .fixed_alt = new std::vector<int>(ins.alternative_number(), -1)
                }),
        output_colgen(columngeneration(ins, p_colgen))
    {
        filter();
    }

    Solution init_solution(const Instance& ins, const BranchAndPriceNode& node, AltIdx k, int val)
    {
        (void)ins;
        Solution sol(node.sol_curr);
        if (val == 1)
            sol.set(k);
        return sol;
    }

    ColGenOutput init_output_colgen(const Instance& ins, const BranchAndPriceNode& node, AltIdx k, int val)
    {
        //p_colgen.info      = Info().set_verbose(true);
        p_colgen.solver    = node.p_colgen.solver;
        p_colgen.columns   = node.p_colgen.columns;
        p_colgen.fixed_alt = new std::vector<int>(*(node.p_colgen.fixed_alt));
        (*p_colgen.fixed_alt)[k] = val;
        if (val == 1) {
            ItemIdx j = ins.alternative(k).j;
            AgentIdx i0 = ins.alternative(k).i;
            for (AgentIdx i = 0; i < ins.agent_number(); ++i)
                if (i != i0)
                    (*p_colgen.fixed_alt)[ins.alternative_index(j, i)] = 0;
        }
        //for (ItemIdx j = 0; j < ins.item_number(); ++j) {
            //for (AgentIdx i = 0; i < ins.agent_number(); ++i)
                //std::cout << (*p_colgen.fixed_alt)[ins.alternative_index(j, i)] << " ";
            //std::cout << std::endl;
        //}
        //std::cout << sol_curr << std::endl;
        return columngeneration(ins, p_colgen);
    }

    BranchAndPriceNode(const Instance& ins, const BranchAndPriceNode& node, AltIdx k, int val):
        sol_curr(init_solution(ins, node, k, val)),
        output_colgen(init_output_colgen(ins, node, k, val))
    {
        filter();
    }

    BranchAndPriceNode(const BranchAndPriceNode& node):
        sol_curr(node.sol_curr), p_colgen(node.p_colgen), output_colgen(node.output_colgen)
    {
        p_colgen.fixed_alt = new std::vector<int>(*(node.p_colgen.fixed_alt));
    }
    BranchAndPriceNode& operator=(const BranchAndPriceNode& node)
    {
        if (this != &node) {
            sol_curr = node.sol_curr;
            p_colgen = node.p_colgen;
            output_colgen = node.output_colgen;
            p_colgen.fixed_alt = new std::vector<int>(*(node.p_colgen.fixed_alt));
        }
        return *this;
    }
    ~BranchAndPriceNode() { delete p_colgen.fixed_alt; }

    std::vector<BranchAndPriceNode> children(std::string str)
    {
        const Instance& ins = sol_curr.instance();
        ItemIdx  j_best = -1;
        AgentIdx i_best = -1;
        double val_best = -1;
        for (ItemIdx j = 0; j < ins.item_number(); ++j) {
            for (AgentIdx i = 0; i < ins.agent_number(); ++i) {
                AltIdx k = ins.alternative_index(j, i);
                if ((*p_colgen.fixed_alt)[k] >= 0)
                    continue;
                if (sol_curr.remaining_capacity(i) < ins.alternative(k).w)
                    continue;
                double val = 0;
                if (str == "frac")
                    val = std::abs(output_colgen.x[k] - 0.5);
                if (str == "max")
                    val = - output_colgen.x[k];

                if (j_best == -1 || val_best > val) {
                    j_best = j;
                    i_best = i;
                    val_best = val;
                }
            }
        }
        if (j_best == -1)
            return {};
        AltIdx k = ins.alternative_index(j_best, i_best);
        std::vector<BranchAndPriceNode> vec;
        vec.push_back(BranchAndPriceNode(ins, *this, k, 0));
        vec.push_back(BranchAndPriceNode(ins, *this, k, 1));
        return vec;
    }

    bool feasible_ = true;
    Solution sol_curr;
    ColGenOptionalParameters p_colgen;
    ColGenOutput output_colgen;
};

BranchAndPriceOutput generalizedassignmentsolver::branchandprice_dfs(const Instance& ins, BranchAndPriceOptionalParameters p)
{
    VER(p.info, "*** branchandprice_dfs " << p.solver << " ***" << std::endl);
    BranchAndPriceOutput output(ins, p.info);

    std::vector<std::vector<std::vector<ItemIdx>>> columns(ins.agent_number());

    std::vector<BranchAndPriceNode> q;

    q.push_back(BranchAndPriceNode(ins, p, &columns));
    while (!q.empty()) {
        output.node_number++;

        if (!p.info.check_time())
            return output.algorithm_end(p.info);

        // Get node
        BranchAndPriceNode node = q.back();
        q.pop_back();

        if (output.node_number == 1)
            output.update_lower_bound(node.output_colgen.lower_bound, std::stringstream(), p.info);

        // if found feasible solution
        if (node.sol_curr.feasible()) {
            output.update_solution(node.sol_curr, std::stringstream(""), p.info);
            if (output.optimal())
                return output.algorithm_end(p.info);
        }

        auto children = node.children("max");
        for (BranchAndPriceNode& child: children)
            q.push_back(child);

    }
    return output.algorithm_end(p.info);
}

BranchAndPriceOutput generalizedassignmentsolver::branchandprice_astar(const Instance& ins, BranchAndPriceOptionalParameters p)
{
    VER(p.info, "*** branchandprice_astar " << p.solver << " ***" << std::endl);
    BranchAndPriceOutput output(ins, p.info);

    std::vector<std::vector<std::vector<ItemIdx>>> columns(ins.agent_number());

    auto comp = [](const BranchAndPriceNode& n1, const BranchAndPriceNode& n2)
    {
        return n1.output_colgen.lower_bound >= n2.output_colgen.lower_bound;
    };
    std::priority_queue<BranchAndPriceNode, std::vector<BranchAndPriceNode>, decltype(comp)> q(comp);

    q.push(BranchAndPriceNode(ins, p, &columns));
    while (!q.empty()) {
        //std::cout << "node " << node_number << std::endl;
        output.node_number++;

        if (!p.info.check_time())
            return output.algorithm_end(p.info);

        // Get node
        BranchAndPriceNode node = q.top();
        q.pop();
        //std::cout << node.output_colgen.lower_bound << std::endl;
        //for (AltIdx k = 0; k < ins.alternative_number(); ++k)
            //if ((*node.p_colgen.fixed_alt)[k] >= 0)
                //std::cout << ins.alternative(k).j << "/" << ins.alternative(k).i << " " << (*node.p_colgen.fixed_alt)[k] << "; ";
        //std::cout << std::endl;
        //std::cout << node.sol_curr << std::endl;
        //std::cout << ins.alternative(node.k).j << "/" << ins.alternative(node.k).i << std::endl;;
        output.update_lower_bound(node.output_colgen.lower_bound, std::stringstream(), p.info);

        // if found feasible solution
        if (node.sol_curr.feasible()) {
            output.update_solution(node.sol_curr, std::stringstream(""), p.info);
            return output.algorithm_end(p.info);
        }

        auto children = node.children("frac");
        for (BranchAndPriceNode& child: children)
            q.push(child);

    }
    return output.algorithm_end(p.info);
}

