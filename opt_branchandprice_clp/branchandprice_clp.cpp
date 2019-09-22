#if COINOR_FOUND

#include "gap/opt_branchandprice_clp/branchandprice_clp.hpp"

#include "gap/lb_colgen_clp/colgen_clp.hpp"

using namespace gap;

void sopt_branchandprice_clp_rec(ColGenClpData& d)
{
    ItemIdx n = d.ins.item_number();
    AgentIdx m = d.ins.agent_number();

    (void)m;
    (void)n;
    (void)d;

}

Solution gap::sopt_branchandprice_clp(BranchAndPriceClpData d)
{
    VER(d.info, "*** branchandprice_clp ***" << std::endl);

    Cost lb = 0;
    std::vector<std::vector<std::vector<ItemIdx>>> columns;
    std::vector<AltIdx> fixed_alt(d.ins.alternative_number());
    ColGenClpData d_rec {
        .ins = d.ins,
        .lb = lb,
        .gen = d.gen,
        .columns = columns,
        .fixed_alt = fixed_alt,
    };
    sopt_branchandprice_clp_rec(d_rec);

    return algorithm_end(d.sol, d.info);
}

#endif

