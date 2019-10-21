#include "gap/opt_branchandprice/branchandprice.hpp"

#include "gap/lb_columngeneration/columngeneration.hpp"
#include "gap/ub_greedy/greedy.hpp"

#include <queue>

using namespace gap;

BranchAndPriceOutput& BranchAndPriceOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    PUT(info, "Algorithm", "NodeNumber", node_number);
    VER(info, "Nodes: " << node_number << std::endl);
    return *this;
}

struct BranchAndPriceNode
{

    AltIdx init_k(const Instance& ins)
    {
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
            if (i0 < 0)
                return -1;
            if (i0 == ins.agent_number())
                continue;
            //std::cout << "set " << ins.alternative(k).j << " to " << ins.alternative(k).i << std::endl;
            sol_curr.set(j, i0);
            for (AgentIdx i = 0; i < ins.agent_number(); ++i)
                (*p_colgen.fixed_alt)[ins.alternative_index(j, i)] = 0;
            (*p_colgen.fixed_alt)[ins.alternative_index(j, i0)] = 1;
        }

        ItemIdx  j_best = -1;
        AgentIdx i_best = -1;
        double   x_best = -1;
        for (ItemIdx j = 0; j < ins.item_number(); ++j) {
            for (AgentIdx i = 0; i < ins.agent_number(); ++i) {
                AltIdx k = ins.alternative_index(j, i);
                if ((*p_colgen.fixed_alt)[k] >= 0)
                    continue;
                if (sol_curr.remaining_capacity(i) < ins.alternative(k).w)
                    continue;
                double x = - output_colgen.x[k];
                //double x = std::abs(output_colgen.x[k] - 0.5);
                if (j_best == -1 || x_best > x) {
                    j_best = j;
                    i_best = i;
                    x_best = x;
                }
            }
        }
        if (j_best == -1)
            return -1;
        //std::cout << "j " << j_best << " i " << i_best << " k " << ins.alternative_index(j_best, i_best) << " x " << x_best << std::endl;
        return ins.alternative_index(j_best, i_best);
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
        output_colgen(lb_columngeneration(ins, p_colgen)),
        k(init_k(ins)) { }

    Solution init_solution(const Instance& ins, const BranchAndPriceNode& node, int z)
    {
        (void)ins;
        Solution sol(node.sol_curr);
        if (z == 1)
            sol.set(node.k);
        return sol;
    }

    ColGenOutput init_output_colgen(const Instance& ins, const BranchAndPriceNode& node, int z)
    {
        //p_colgen.info      = Info().set_verbose(true);
        p_colgen.solver    = node.p_colgen.solver;
        p_colgen.columns   = node.p_colgen.columns;
        p_colgen.fixed_alt = new std::vector<int>(*(node.p_colgen.fixed_alt));
        (*p_colgen.fixed_alt)[node.k] = z;
        if (z == 1) {
            ItemIdx j = ins.alternative(node.k).j;
            AgentIdx i0 = ins.alternative(node.k).i;
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
        return lb_columngeneration(ins, p_colgen);
    }

    BranchAndPriceNode(const Instance& ins, const BranchAndPriceNode& node, int z):
        sol_curr(init_solution(ins, node, z)),
        output_colgen(init_output_colgen(ins, node, z)),
        k(init_k(ins))
    {
    }

    BranchAndPriceNode(const BranchAndPriceNode& node):
        sol_curr(node.sol_curr), p_colgen(node.p_colgen), output_colgen(node.output_colgen), k(node.k)
    {
        p_colgen.fixed_alt = new std::vector<int>(*(node.p_colgen.fixed_alt));
    }
    BranchAndPriceNode& operator=(const BranchAndPriceNode& node)
    {
        if (this != &node) {
            sol_curr = node.sol_curr;
            p_colgen = node.p_colgen;
            output_colgen = node.output_colgen;
            k = node.k;
            p_colgen.fixed_alt = new std::vector<int>(*(node.p_colgen.fixed_alt));
        }
        return *this;
    }
    ~BranchAndPriceNode() { delete p_colgen.fixed_alt; }

    Solution sol_curr;
    ColGenOptionalParameters p_colgen;
    ColGenOutput output_colgen;
    AltIdx k = -1;
};

BranchAndPriceOutput gap::sopt_branchandprice_dfs(const Instance& ins, BranchAndPriceOptionalParameters p)
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

        if (node.k != -1) {
            q.push_back(BranchAndPriceNode(ins, node, 0));
            q.push_back(BranchAndPriceNode(ins, node, 1));
        }

    }
    return output.algorithm_end(p.info);
}

BranchAndPriceOutput gap::sopt_branchandprice_astar(const Instance& ins, BranchAndPriceOptionalParameters p)
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

        if (node.k != -1) {
            q.push(BranchAndPriceNode(ins, node, 0));
            q.push(BranchAndPriceNode(ins, node, 1));
        }

    }
    return output.algorithm_end(p.info);
}

