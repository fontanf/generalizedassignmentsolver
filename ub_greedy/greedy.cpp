#include "gap/ub_greedy/greedy.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace gap;

std::vector<std::pair<ItemIdx, AgentIdx>> gap::sol_greedy_init(const Solution& sol, const Desirability& f)
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
    return alt;
}

void gap::sol_greedy(Solution& sol, const std::vector<std::pair<ItemIdx, AgentIdx>>& alt)
{
    const Instance& ins = sol.instance();
    for (auto p: alt) {
        ItemIdx j = p.first;
        if (sol.agent(j) != -1)
            continue;
        AgentIdx i = p.second;
        const Alternative& a = ins.alternative(j, i);
        if (sol.remaining_capacity(i) < a.w)
            continue;
        sol.set(j, i);
        if (sol.full())
            break;
    }
}

Solution gap::sol_greedy(const Instance& ins, const Desirability& f, Info info)
{
    VER(info, "*** greedy " << f.to_string() << " ***" << std::endl);
    Solution sol(ins);
    auto alt = sol_greedy_init(sol, f);
    sol_greedy(sol, alt);
    return algorithm_end(sol, info);
}

/******************************************************************************/

void gap::sol_greedyregret(Solution& sol, const Desirability& f)
{
    const Instance& ins = sol.instance();
    ItemIdx n = ins.item_number();
    AgentIdx m = ins.agent_number();
    std::vector<std::pair<AgentIdx, AgentIdx>> bests(n, {0, 1});

    std::vector<std::vector<AgentIdx>> agents(n, std::vector<AgentIdx>(m));
    for (ItemIdx j=0; j<n; ++j) {
        std::iota(agents[j].begin(), agents[j].end(), 0);
        sort(agents[j].begin(), agents[j].end(), [&f, &j](
                    AgentIdx i1, AgentIdx i2) -> bool {
                return f(j, i1) < f(j, i2); });
    }

    while (!sol.full()) {
        ItemIdx j_best = -1;
        double f_best = -1;
        for (ItemIdx j=0; j<n; ++j) {
            if (sol.agent(j) != -1)
                continue;

            AgentIdx& i_first = bests[j].first;
            AgentIdx& i_second = bests[j].second;

            while (i_first < m) {
                AgentIdx i = agents[j][i_first];
                if (ins.alternative(j, i).w > sol.remaining_capacity(i)) {
                    i_first++;
                    if (i_first == i_second)
                        i_second++;
                } else {
                    break;
                }
            }
            if (i_first == m)
                return;

            while (i_second < m) {
                AgentIdx i = agents[j][i_second];
                if (ins.alternative(j, i).w > sol.remaining_capacity(i)) {
                    i_second++;
                } else {
                    break;
                }
            }

            double f_curr = (i_second == m)?
                std::numeric_limits<double>::infinity():
                f(j, agents[j][i_second]) - f(j, agents[j][i_first]);
            if (j_best == -1 || f_best < f_curr) {
                f_best = f_curr;
                j_best = j;
            }
        }
        sol.set(j_best, agents[j_best][bests[j_best].first]);
    }
}

Solution gap::sol_greedyregret(const Instance& ins, const Desirability& f, Info info)
{
    VER(info, "*** greedyregret " << f.to_string() << " ***" << std::endl);
    Solution sol(ins);
    sol_greedyregret(sol, f);
    return algorithm_end(sol, info);
}

/******************************************************************************/

void nshift(Solution& sol)
{
    const Instance& ins = sol.instance();
    ItemIdx n = ins.item_number();
    AgentIdx m = ins.agent_number();
    for (ItemIdx j=0; j<n; ++j) {
        AgentIdx i_old = sol.agent(j);
        const Alternative& a_old = ins.alternative(j, i_old);
        Cost c_best = 0;
        AgentIdx i_best = -1;
        for (AgentIdx i=0; i<m; ++i) {
            if (i == i_old)
                continue;
            const Alternative& a = ins.alternative(j, i);
            if (sol.remaining_capacity(i) >= a.w && c_best > a.c - a_old.c) {
                i_best = i;
                c_best = a.c - a_old.c;
            }
        }
        if (i_best != -1)
            sol.set(j, i_best);
    }
}

void gap::sol_mthg(Solution& sol, const std::vector<std::pair<ItemIdx, AgentIdx>>& alt)
{
    sol_greedy(sol, alt);
    if (sol.feasible())
        nshift(sol);
}

Solution gap::sol_mthg(const Instance& ins, const Desirability& f, Info info)
{
    VER(info, "*** mthg " << f.to_string() << " ***" << std::endl);
    Solution sol(ins);
    auto alt = sol_greedy_init(sol, f);
    sol_mthg(sol, alt);
    return algorithm_end(sol, info);
}

void gap::sol_mthgregret(Solution& sol, const Desirability& f)
{
    sol_greedyregret(sol, f);
    if (sol.feasible())
        nshift(sol);
}

Solution gap::sol_mthgregret(const Instance& ins, const Desirability& f, Info info)
{
    VER(info, "*** mthgregret " << f.to_string() << " ***" << std::endl);
    Solution sol(ins);
    sol_mthgregret(sol, f);
    return algorithm_end(sol, info);
}

