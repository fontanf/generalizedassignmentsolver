#if COINOR_FOUND

#include "gap/opt_branchandprice/branchandprice.hpp"

#include "gap/lb_columngeneration/columngeneration.hpp"
#include "gap/ub_greedy/greedy.hpp"

using namespace gap;

BranchAndPriceOutput& BranchAndPriceOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    return *this;
}

struct BranchAndPriceRecData
{
    const Instance& ins;
    BranchAndPriceOutput& output;
    Info& info;

    Solution sol_curr;
    std::vector<int> fixed_alt;
    ColGenOptionalParameters p_colgen;

    std::unique_ptr<Desirability> f1;
    std::vector<std::vector<AgentIdx>> agents1;
    std::unique_ptr<Desirability> f2;
    std::vector<std::vector<AgentIdx>> agents2;
    std::unique_ptr<Desirability> f3;
    std::vector<std::vector<AgentIdx>> agents3;
};

void sopt_branchandprice_clp_rec(BranchAndPriceRecData& d);

BranchAndPriceOutput gap::sopt_branchandprice(const Instance& ins, BranchAndPriceOptionalParameters p)
{
    VER(p.info, "*** branchandprice " << p.solver << " ***" << std::endl);
    BranchAndPriceOutput output(ins, p.info);

    std::vector<std::vector<std::vector<ItemIdx>>> columns(ins.agent_number());
    std::vector<double> x(ins.alternative_number());

    BranchAndPriceRecData d_rec {
        .ins = ins,
        .output = output,
        .info = p.info,

        .sol_curr = Solution(ins),
        .fixed_alt = std::vector<int>(ins.alternative_number(), -1),
        .p_colgen = {
            .info = Info(),
            .solver = p.solver,
            .columns = &columns,
            .fixed_alt = &d_rec.fixed_alt,
            .fixed_agents = NULL,
        },

        .f1 = desirability("wij/ti", ins),
        .agents1 = sol_greedyregret_init(ins, *d_rec.f1),
        .f2 = desirability("-pij/wij", ins),
        .agents2 = sol_greedyregret_init(ins, *d_rec.f2),
        .f3 = desirability("cij", ins),
        .agents3 = sol_greedyregret_init(ins, *d_rec.f3),
    };

    auto output_colgen = lb_columngeneration(ins, d_rec.p_colgen);
    output.update_lower_bound(output_colgen.lower_bound, std::stringstream(""), p.info);

    sopt_branchandprice_clp_rec(d_rec);

    return output.algorithm_end(p.info);
}

/******************************************************************************/

void sopt_branchandprice_clp_rec(BranchAndPriceRecData& d)
{
    if (!d.info.check_time())
        return;

    ItemIdx  n = d.ins.item_number();
    AgentIdx m = d.ins.agent_number();

    //std::cout << d.sol_curr << std::endl;

    //for (ItemIdx j = 0; j < n; ++j) {
        //for (AgentIdx i = 0; i < m; ++i)
            //std::cout << d.fixed_alt[d.ins.alternative_index(j, i)] << " ";
        //std::cout << std::endl;
    //}

    if (d.output.optimal())
        return;
    if (d.sol_curr.feasible()) {
        if (compare(d.output.solution, d.sol_curr))
            d.output.update_solution(d.sol_curr, std::stringstream("current solution"), d.info);
        return;
    }

    // Lower bound
    auto output_colgen = lb_columngeneration(d.ins, d.p_colgen);
    if (d.output.solution.feasible() && output_colgen.lower_bound >= d.output.solution.cost())
        return;

    { // Primal heuristic 0
        Solution sol(d.ins);
        for (AltIdx k = 0; k < d.ins.alternative_number(); ++k)
            if (output_colgen.x[k] > 0.5)
                sol.set(k);
        if (compare(d.output.solution, sol))
            d.output.update_solution(sol, std::stringstream("primal heuristic 1"), d.info);
    }

    { // Primal heuristic 1
        Solution sol(d.sol_curr);
        sol_mthgregret(sol, *d.f1, d.agents1, d.fixed_alt);
        if (compare(d.output.solution, sol))
            d.output.update_solution(sol, std::stringstream("mthgregret wij/ti"), d.info);
    }

    { // Primal heuristic 2
        Solution sol(d.sol_curr);
        sol_mthgregret(sol, *d.f2, d.agents2, d.fixed_alt);
        if (compare(d.output.solution, sol))
            d.output.update_solution(sol, std::stringstream("mthgregret -pij/wij"), d.info);
    }

    { // Primal heuristic 3
        Solution sol(d.sol_curr);
        sol_mthgregret(sol, *d.f3, d.agents3, d.fixed_alt);
        if (compare(d.output.solution, sol))
            d.output.update_solution(sol, std::stringstream("mthgregret cij"), d.info);
    }


    ItemIdx  j_best = -1;
    AgentIdx i_best = -1;
    double   x_best = -1;
    for (ItemIdx j = 0; j < n; ++j) {
        if (d.sol_curr.agent(j) >= 0)
            continue;
        for (AgentIdx i = 0; i < m; ++i) {
            AltIdx k = d.ins.alternative_index(j, i);
            if (d.fixed_alt[k] >= 0)
                continue;
            if (d.sol_curr.remaining_capacity(i) < d.ins.alternative(k).w)
                continue;
            double x = - output_colgen.x[k];
            //double x = std::abs(d.colgen_data.x[k] - 0.5);
            if (j_best == -1 || x_best > x) {
                j_best = j;
                i_best = i;
                x_best = x;
            }
        }
    }
    if (j_best == -1)
        return;
    //std::cout << "j_best " << j_best << " i_best " << i_best << " x_best " << x_best << std::endl;
    AltIdx k_best = d.ins.alternative_index(j_best, i_best);

    std::vector<std::pair<AltIdx, int>> changes;
    changes.push_back({k_best, 1});
    for (AgentIdx i = 0; i < m; ++i) {
        AltIdx k = d.ins.alternative_index(j_best, i);
        if (i != i_best && d.fixed_alt[k] == -1)
            changes.push_back({k, 0});
    }

    d.sol_curr.set(j_best, i_best);
    for (auto p: changes)
        d.fixed_alt[p.first] = p.second;
    sopt_branchandprice_clp_rec(d);

    for (auto p: changes)
        d.fixed_alt[p.first] = -1;
    d.fixed_alt[k_best] = 0;
    d.sol_curr.set(j_best, -1);
    sopt_branchandprice_clp_rec(d);
    d.fixed_alt[k_best] = -1;
}

#endif

