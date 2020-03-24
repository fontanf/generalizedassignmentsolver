#include "generalizedassignmentsolver/algorithms/greedy.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace generalizedassignmentsolver;

std::vector<std::pair<ItemIdx, AgentIdx>> generalizedassignmentsolver::greedy_init(const Solution& solution, const Desirability& f)
{
    const Instance& instance = solution.instance();
    ItemIdx n = instance.item_number();
    AgentIdx m = instance.agent_number();
    std::vector<std::pair<ItemIdx, AgentIdx>> alt(instance.alternative_number());
    for (ItemIdx j = 0; j < n; ++j) {
        for (AgentIdx i = 0; i < m; ++i) {
            AltIdx k = instance.alternative_index(j, i);
            alt[k] = {j, i};
        }
    }
    sort(alt.begin(), alt.end(), [&f](
                const std::pair<ItemIdx, AgentIdx>& a,
                const std::pair<ItemIdx, AgentIdx>& b) -> bool {
            return f(a.first, a.second) < f(b.first, b.second); });
    return alt;
}

void generalizedassignmentsolver::greedy(Solution& solution, const std::vector<std::pair<ItemIdx, AgentIdx>>& alt)
{
    const Instance& instance = solution.instance();
    for (auto p: alt) {
        ItemIdx j = p.first;
        if (solution.agent(j) != -1)
            continue;
        AgentIdx i = p.second;
        const Alternative& a = instance.alternative(j, i);
        if (solution.remaining_capacity(i) < a.w)
            continue;
        solution.set(j, i);
        if (solution.full())
            break;
    }
}

Output generalizedassignmentsolver::greedy(const Instance& instance, const Desirability& f, Info info)
{
    VER(info, "*** greedy " << f.to_string() << " ***" << std::endl);
    Output output(instance, info);
    Solution solution(instance);
    auto alt = greedy_init(solution, f);
    greedy(solution, alt);
    output.update_solution(solution, std::stringstream(""), info);
    return output.algorithm_end(info);
}

/******************************************************************************/

std::vector<std::vector<AgentIdx>> generalizedassignmentsolver::greedyregret_init(const Instance& instance, const Desirability& f)
{
    ItemIdx n = instance.item_number();
    AgentIdx m = instance.agent_number();

    std::vector<std::vector<AgentIdx>> agents(n, std::vector<AgentIdx>(m));
    for (ItemIdx j = 0; j < n; ++j) {
        std::iota(agents[j].begin(), agents[j].end(), 0);
        sort(agents[j].begin(), agents[j].end(), [&f, &j](
                    AgentIdx i1, AgentIdx i2) -> bool {
                return f(j, i1) < f(j, i2); });
    }

    return agents;
}

void generalizedassignmentsolver::greedyregret(Solution& solution, const Desirability& f,
        const std::vector<std::vector<AgentIdx>>& agents,
        const std::vector<int>& fixed_alt)
{
    const Instance& instance = solution.instance();
    ItemIdx n = instance.item_number();
    AgentIdx m = instance.agent_number();
    std::vector<std::pair<AgentIdx, AgentIdx>> bests(n, {0, 1});

    while (!solution.full()) {
        ItemIdx j_best = -1;
        double f_best = -1;
        for (ItemIdx j = 0; j < n; ++j) {
            if (solution.agent(j) != -1)
                continue;

            AgentIdx& i_first = bests[j].first;
            AgentIdx& i_second = bests[j].second;

            while (i_first < m) {
                AgentIdx i = agents[j][i_first];
                if (instance.alternative(j, i).w > solution.remaining_capacity(i) ||
                        (!fixed_alt.empty() && fixed_alt[instance.alternative_index(j, i)] != -1)) {
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
                if (instance.alternative(j, i).w > solution.remaining_capacity(i) ||
                        (!fixed_alt.empty() && fixed_alt[instance.alternative_index(j, i)] != -1)) {
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
        solution.set(j_best, agents[j_best][bests[j_best].first]);
    }
}

Output generalizedassignmentsolver::greedyregret(const Instance& instance, const Desirability& f, Info info)
{
    VER(info, "*** greedyregret " << f.to_string() << " ***" << std::endl);
    Output output(instance, info);
    Solution solution(instance);
    auto agents = greedyregret_init(instance, f);
    greedyregret(solution, f, agents, {});
    output.update_solution(solution, std::stringstream(""), info);
    return output.algorithm_end(info);
}

/******************************************************************************/

void nshift(Solution& solution)
{
    const Instance& instance = solution.instance();
    ItemIdx n = instance.item_number();
    AgentIdx m = instance.agent_number();
    for (ItemIdx j = 0; j < n; ++j) {
        AgentIdx i_old = solution.agent(j);
        const Alternative& a_old = instance.alternative(j, i_old);
        Cost c_best = 0;
        AgentIdx i_best = -1;
        for (AgentIdx i = 0; i < m; ++i) {
            if (i == i_old)
                continue;
            const Alternative& a = instance.alternative(j, i);
            if (solution.remaining_capacity(i) >= a.w && c_best > a.c - a_old.c) {
                i_best = i;
                c_best = a.c - a_old.c;
            }
        }
        if (i_best != -1)
            solution.set(j, i_best);
    }
}

void generalizedassignmentsolver::mthg(Solution& solution, const std::vector<std::pair<ItemIdx, AgentIdx>>& alt)
{
    greedy(solution, alt);
    if (solution.feasible())
        nshift(solution);
}

Output generalizedassignmentsolver::mthg(const Instance& instance, const Desirability& f, Info info)
{
    VER(info, "*** mthg " << f.to_string() << " ***" << std::endl);
    Output output(instance, info);
    Solution solution(instance);
    auto alt = greedy_init(solution, f);
    mthg(solution, alt);
    output.update_solution(solution, std::stringstream(""), info);
    return output.algorithm_end(info);
}

void generalizedassignmentsolver::mthgregret(Solution& solution, const Desirability& f,
        const std::vector<std::vector<AgentIdx>>& agents,
        const std::vector<int>& fixed_alt)
{
    greedyregret(solution, f, agents, fixed_alt);
    if (solution.feasible())
        nshift(solution);
}

Output generalizedassignmentsolver::mthgregret(const Instance& instance, const Desirability& f, Info info)
{
    VER(info, "*** mthgregret " << f.to_string() << " ***" << std::endl);
    Output output(instance, info);
    Solution solution(instance);
    auto agents = greedyregret_init(instance, f);
    mthgregret(solution, f, agents, {});
    output.update_solution(solution, std::stringstream(""), info);
    return output.algorithm_end(info);
}

