#include "gap/ub_lagrangian/lagrangian.hpp"

#include "knapsack/opt_minknap/minknap.hpp"

#include <algorithm>

using namespace gap;

//#define DBG(x)
#define DBG(x) x

LagOut gap::ub_lagrangian(const SubInstance& sub, Info* info)
{
    DBG(std::cout << "LAGRANGIAN..." << std::endl;)
    (void)info;
    const Instance& ins = sub.instance();
    assert(ins.objective() == 1);
    std::vector<Profit> multipliers_curr(ins.item_number(), 0);
    Profit multipliers_sum = 0;
    LagOut out;
    out.bound = -1;
    out.multipliers.resize(ins.item_number());

    Profit gamma = 0;
    for (AltIdx k=0; k<ins.alternative_number(); ++k)
        if (ins.alternative(k).p > gamma)
            gamma = ins.alternative(k).p;

    for (size_t it=1; it<100; ++it) {
        DBG(std::cout << "ITERATION " << it << std::endl;)
        DBG(std::cout << "MULT" << std::flush;
        for (Profit m: multipliers_curr)
            std::cout << " " << m << std::flush;
        std::cout << " SUM " << multipliers_sum << std::endl;)

        std::vector<ItemIdx> x(ins.item_number(), 0);
        Profit ub = -multipliers_sum;
        for (AgentIdx i=0; i<ins.agent_number(); ++i) {
            DBG(std::cout << "AGENT " << i << std::flush;)
            knapsack::Instance ins_kp(sub.alternative_number(i), sub.capacity(i));
            for (ItemIdx j=0; j<ins.item_number(); ++j) {
                AltIdx k = ins.item(j).alt[i];
                if (sub.reduced(k) == 1)
                    continue;
                Profit p = ins.alternative(k).p + multipliers_curr[j];
                if (p < 0)
                    p = 0;
                ins_kp.add_item(ins.alternative(k).w, p, j);
            }
            knapsack::MinknapParams params;
            knapsack::Solution sol = knapsack::sopt_minknap_list_part(ins_kp, params);
            DBG(std::cout << " KPOPT " << sol.profit() << std::endl;)
            ub += sol.profit();
            for (knapsack::ItemPos j_kp=0; j_kp<ins_kp.item_number(); ++j_kp) {
                ItemIdx j_gap = ins_kp.item(j_kp).l;
                std::cout << "JKP " << j_kp << " JGAP " << j_gap << " " << sol.contains_idx(j_kp) << std::endl;
                if (sol.contains(j_kp))
                    x[j_gap]++;
            }
        }

        DBG(std::cout << "UB " << ub << std::flush;
        std::cout << " X" << std::flush;
        for (AgentIdx xj: x)
            std::cout << " " << xj << std::flush;
        std::cout << std::endl;)

        if (out.bound == -1 || ub < out.bound) {
            out.bound = ub;
            out.multipliers = multipliers_curr;
        }

        // Update multipliers
        multipliers_sum = 0;
        for (ItemIdx j=0; j<ins.item_number(); ++j) {
            std::cout << multipliers_curr[j] << " " << x[j];
            if (x[j] < 1) { // <=> x[j] == 0
                multipliers_curr[j] += gamma / it;
            } else if (x[j] > 1) {
                multipliers_curr[j] -= (gamma * (x[j]-1)) / it;
            }
            multipliers_sum += multipliers_curr[j];
            std::cout << " => " << multipliers_curr[j] << std::endl;
        }
    }

    DBG(std::cout << "LAGRANGIAN... END" << std::endl;)
    return out;
}

#undef DBG
