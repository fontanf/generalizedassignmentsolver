#if COINOR_FOUND

#include "gap/opt_branchandprice_clp/branchandprice_clp.hpp"

#include "gap/lb_colgen_clp/colgen_clp.hpp"
#include "gap/ub_greedy/greedy.hpp"

using namespace gap;

struct BranchAndPriceClpRecData
{
    const Instance& ins;
    Solution& sol_best;
    Cost& lb;
    Info& info;

    Solution sol_curr;
    std::vector<int> fixed_alt;
    ColGenClpData colgen_data;

    std::unique_ptr<Desirability> f1;
    std::vector<std::vector<AgentIdx>> agents1;
    std::unique_ptr<Desirability> f2;
    std::vector<std::vector<AgentIdx>> agents2;
};

void sopt_branchandprice_clp_rec(BranchAndPriceClpRecData& d)
{
    ItemIdx  n = d.ins.item_number();
    AgentIdx m = d.ins.agent_number();

    //std::cout << sol_curr << std::endl;

    //for (ItemIdx j = 0; j < n; ++j) {
        //for (AgentIdx i = 0; i < m; ++i)
            //std::cout << d.fixed_alt[d.ins.alternative_index(j, i)] << " ";
        //std::cout << std::endl;
    //}

    if (d.sol_best.feasible() && d.sol_best.cost() == d.lb)
        return;
    if (d.sol_curr.feasible()) {
        if (compare(d.sol_best, d.sol_curr))
            d.sol_best.update(d.sol_curr, d.lb, std::stringstream("current solution"), d.info);
        return;
    }

    // Lower bound
    lb_colgen_clp(d.colgen_data);
    if (d.sol_best.feasible() && d.colgen_data.lb >= d.sol_best.cost())
        return;

    { // Primal heuristic 1
        Solution sol(d.ins);
        for (AltIdx k = 0; k < d.ins.alternative_number(); ++k)
            if (d.colgen_data.x[k] > 0.5)
                sol.set(k);
        if (compare(d.sol_best, sol))
            d.sol_best.update(sol, d.lb, std::stringstream("primal heuristic 1"), d.info);
    }

    { // Primal heuristic 2
        Solution sol(d.ins);
        sol_mthgregret(sol, *d.f1, d.agents1, d.fixed_alt);
        if (compare(d.sol_best, sol))
            d.sol_best.update(sol, d.lb, std::stringstream("mthgregret wij/ti"), d.info);
    }

    { // Primal heuristic 3
        Solution sol(d.ins);
        sol_mthgregret(sol, *d.f2, d.agents2, d.fixed_alt);
        if (compare(d.sol_best, sol))
            d.sol_best.update(sol, d.lb, std::stringstream("mthgregret -pij/wij"), d.info);
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
            if (j_best == -1 || x_best < d.colgen_data.x[k]) {
                j_best = j;
                i_best = i;
                x_best = d.colgen_data.x[k];
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

Solution gap::sopt_branchandprice_clp(BranchAndPriceClpData d)
{
    VER(d.info, "*** branchandprice_clp ***" << std::endl);
    init_display(d.info);

    Cost lb_colgen = 0;
    std::vector<std::vector<std::vector<ItemIdx>>> columns;
    std::vector<double> x(d.ins.alternative_number());

    BranchAndPriceClpRecData d_rec {
        .ins = d.ins,
        .sol_best = d.sol,
        .lb = d.lb,
        .info = d.info,

        .sol_curr = Solution(d.ins),
        .fixed_alt = std::vector<int>(d.ins.alternative_number(), -1),
        .colgen_data = {
            .ins = d.ins,
            .lb = lb_colgen,
            .gen = d.gen,
            .columns = columns,
            .fixed_alt = d_rec.fixed_alt,
            .x = x,
            //.info = Info().set_verbose(true),
        },

        .f1 = desirability("wij/ti", d.ins),
        .agents1 = sol_greedyregret_init(d.ins, *d_rec.f1),
        .f2 = desirability("-pij/wij", d.ins),
        .agents2 = sol_greedyregret_init(d.ins, *d_rec.f2),
    };

    lb_colgen_clp(d_rec.colgen_data);
    if (d.lb < d_rec.colgen_data.lb)
        update_lb(d.lb, d_rec.colgen_data.lb, d.sol, std::stringstream(""), d.info);

    sopt_branchandprice_clp_rec(d_rec);

    return algorithm_end(d.sol, d.info);
}

#endif

