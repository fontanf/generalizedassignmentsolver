#include "gap/ub_greedy/greedy.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace gap;

std::unique_ptr<Desirability> gap::desirability(std::string str, const Instance& ins)
{
    if (str == "cij") {
        return std::unique_ptr<Desirability>(new DesirabilityCost(ins));
    } else if (str == "wij") {
        return std::unique_ptr<Desirability>(new DesirabilityWeight(ins));
    } else if (str == "cij*wij") {
        return std::unique_ptr<Desirability>(new DesirabilityCostWeight(ins));
    } else if (str == "pij/wij") {
        return std::unique_ptr<Desirability>(new DesirabilityEfficiency(ins));
    } else if (str == "wij/ti") {
        return std::unique_ptr<Desirability>(new DesirabilityWeightCapacity(ins));
    } else {
        return std::unique_ptr<Desirability>(new DesirabilityCost(ins));
    }
}

/******************************************************************************/

void gap::sol_greedy(Solution& sol, const Desirability& f)
{
    const Instance& ins = sol.instance();
    ItemIdx n = ins.item_number();
    AgentIdx m = ins.agent_number();
    std::vector<std::pair<ItemIdx, AgentIdx>> alt(ins.alternative_number());
    for (ItemIdx j=0; j<n; ++j) {
        for (AgentIdx i=0; i<m; ++i) {
            AltIdx k = ins.alternative_index(j, i);
            alt[k] = {j, i};
        }
    }
    sort(alt.begin(), alt.end(), [&f](
                const std::pair<ItemIdx, AgentIdx>& a,
                const std::pair<ItemIdx, AgentIdx>& b) -> bool {
            return f(a.first, a.second) < f(b.first, b.second); });

    for (auto p: alt) {
        ItemIdx j = p.first;
        if (sol.agent(j) != -1)
            continue;
        AgentIdx i = p.second;
        const Alternative& a = ins.alternative(j, i);
        if (sol.remaining_capacity(i) < a.w)
            continue;
        sol.set(j, i);
    }
}

Solution gap::sol_greedy(const Instance& ins, const Desirability& f, Info info)
{
    Solution sol(ins);
    sol_greedy(sol, f);
    return algorithm_end(sol, info);
}

/******************************************************************************/

void gap::sol_mthg(Solution& sol, const Desirability& f)
{
    (void)f;
    while (!sol.full()) {

    }
}

Solution gap::sol_mthg(const Instance& ins, const Desirability& f, Info info)
{
    Solution sol(ins);
    sol_mthg(sol, f);
    return algorithm_end(sol, info);
}

