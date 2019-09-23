#if COINOR_FOUND

#include "gap/opt_branchandprice_clp/branchandprice_clp.hpp"

#include "gap/lb_colgen_clp/colgen_clp.hpp"

using namespace gap;

void sopt_branchandprice_clp_rec(BranchAndPriceClpData& d0, Solution sol_curr, ColGenClpData& d)
{
    ItemIdx  n = d.ins.item_number();
    AgentIdx m = d.ins.agent_number();

    //std::cout << sol_curr.item_number() << std::endl;
    //std::cout << sol_curr << std::endl;

    //for (ItemIdx j = 0; j < n; ++j) {
        //for (AgentIdx i = 0; i < m; ++i)
            //std::cout << d.fixed_alt[d.ins.alternative_index(j, i)] << " ";
        //std::cout << std::endl;
    //}

    if (d0.sol.feasible() && d0.sol.cost() == d0.lb)
        return;
    if (sol_curr.feasible()) {
        if (compare(d0.sol, sol_curr)) {
            d0.sol.update(sol_curr, d0.lb, std::stringstream(""), d0.info);
        }
        return;
    }

    lb_colgen_clp(d);
    if (d0.sol.feasible() && d.lb >= d0.sol.cost())
        return;

    ItemIdx  j_best = -1;
    AgentIdx i_best = -1;
    double   x_best = -1;
    for (ItemIdx j = 0; j < n; ++j) {
        if (sol_curr.agent(j) >= 0)
            continue;
        for (AgentIdx i = 0; i < m; ++i) {
            AltIdx k = d.ins.alternative_index(j, i);
            if (d.fixed_alt[k] >= 0)
                continue;
            if (sol_curr.remaining_capacity(i) < d.ins.alternative(k).w)
                continue;
            if (j_best == -1 || x_best < d.x[k]) {
                j_best = j;
                i_best = i;
                x_best = d.x[k];
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

    sol_curr.set(j_best, i_best);
    for (auto p: changes)
        d.fixed_alt[p.first] = p.second;
    sopt_branchandprice_clp_rec(d0, sol_curr, d);

    for (auto p: changes)
        d.fixed_alt[p.first] = -1;
    d.fixed_alt[k_best] = 0;
    sol_curr.set(j_best, -1);
    sopt_branchandprice_clp_rec(d0, sol_curr, d);
    d.fixed_alt[k_best] = -1;
}

Solution gap::sopt_branchandprice_clp(BranchAndPriceClpData d)
{
    VER(d.info, "*** branchandprice_clp ***" << std::endl);
    init_display(d.info);

    Cost lb = 0;
    std::vector<std::vector<std::vector<ItemIdx>>> columns;
    std::vector<AltIdx> fixed_alt(d.ins.alternative_number(), -1);
    std::vector<double> x(d.ins.alternative_number());
    ColGenClpData d_rec {
        .ins = d.ins,
        .lb = lb,
        .gen = d.gen,
        .columns = columns,
        .fixed_alt = fixed_alt,
        .x = x,
        //.info = Info().set_verbose(true),
    };
    lb_colgen_clp(d_rec);
    if (d.lb < d_rec.lb)
        update_lb(d.lb, d_rec.lb, d.sol, std::stringstream(""), d.info);
    Solution sol_curr(d.ins);
    sopt_branchandprice_clp_rec(d, sol_curr, d_rec);

    return algorithm_end(d.sol, d.info);
}

#endif

