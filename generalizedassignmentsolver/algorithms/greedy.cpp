#include "generalizedassignmentsolver/algorithms/greedy.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace generalizedassignmentsolver;

std::vector<std::pair<ItemIdx, AgentIdx>> generalizedassignmentsolver::greedy_init(
        const Instance& instance,
        const Desirability& f)
{
    ItemIdx n = instance.number_of_items();
    AgentIdx m = instance.number_of_agents();
    std::vector<std::pair<ItemIdx, AgentIdx>> alternatives;
    for (ItemIdx j = 0; j < n; ++j)
        for (AgentIdx i = 0; i < m; ++i)
            alternatives.push_back({j, i});
    sort(alternatives.begin(), alternatives.end(), [&f](
                const std::pair<ItemIdx, AgentIdx>& a,
                const std::pair<ItemIdx, AgentIdx>& b) -> bool
            {
                return f(a.first, a.second) < f(b.first, b.second);
            });
    return alternatives;
}

void generalizedassignmentsolver::greedy(
        Solution& solution,
        const std::vector<std::pair<ItemIdx, AgentIdx>>& alternatives)
{
    const Instance& instance = solution.instance();
    for (auto p: alternatives) {
        ItemIdx j = p.first;
        if (solution.agent(j) != -1)
            continue;
        AgentIdx i = p.second;
        if (solution.remaining_capacity(i) < instance.weight(j, i))
            continue;
        solution.set(j, i);
        if (solution.full())
            break;
    }
}

Output generalizedassignmentsolver::greedy(
        const Instance& instance,
        const Desirability& f, Info info)
{
    init_display(instance, info);
    FFOT_VER(info,
               "Algorithm" << std::endl
            << "---------" << std::endl
            << "Greedy" << std::endl
            << std::endl
            << "Parameters" << std::endl
            << "----------" << std::endl
            << "Desirability:  " << f.to_string() << std::endl
            << std::endl);

    Output output(instance, info);
    Solution solution(instance);
    auto alternatives = greedy_init(instance, f);
    greedy(solution, alternatives);
    output.update_solution(solution, std::stringstream(""), info);
    return output.algorithm_end(info);
}

/******************************************************************************/

std::vector<std::vector<AgentIdx>> generalizedassignmentsolver::greedyregret_init(
        const Instance& instance,
        const Desirability& f)
{
    ItemIdx n = instance.number_of_items();
    AgentIdx m = instance.number_of_agents();

    std::vector<std::vector<AgentIdx>> agents(n, std::vector<AgentIdx>(m));
    for (ItemIdx j = 0; j < n; ++j) {
        std::iota(agents[j].begin(), agents[j].end(), 0);
        sort(agents[j].begin(), agents[j].end(),
                [&f, &j](AgentIdx i1, AgentIdx i2) -> bool
                {
                    return f(j, i1) < f(j, i2);
                });
    }

    return agents;
}

void generalizedassignmentsolver::greedyregret(
        Solution& solution,
        const Desirability& f,
        const std::vector<std::vector<AgentIdx>>& agents,
        const std::vector<std::vector<int>>& fixed_alternatives)
{
    const Instance& instance = solution.instance();
    ItemIdx n = instance.number_of_items();
    AgentIdx m = instance.number_of_agents();
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
                if (instance.weight(j, i) > solution.remaining_capacity(i)
                        || (!fixed_alternatives.empty()
                            && fixed_alternatives[j][i] != -1)) {
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
                if (instance.weight(j, i) > solution.remaining_capacity(i)
                        || (!fixed_alternatives.empty()
                            && fixed_alternatives[j][i] != -1)) {
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

Output generalizedassignmentsolver::greedyregret(
        const Instance& instance,
        const Desirability& f,
        Info info)
{
    init_display(instance, info);
    FFOT_VER(info,
               "Algorithm" << std::endl
            << "---------" << std::endl
            << "Regret greedy" << std::endl
            << std::endl
            << "Parameters" << std::endl
            << "----------" << std::endl
            << "Desirability:  " << f.to_string() << std::endl
            << std::endl);

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
    ItemIdx n = instance.number_of_items();
    AgentIdx m = instance.number_of_agents();
    for (ItemIdx j = 0; j < n; ++j) {
        AgentIdx i_old = solution.agent(j);
        Cost c_best = 0;
        AgentIdx i_best = -1;
        for (AgentIdx i = 0; i < m; ++i) {
            if (i == i_old)
                continue;
            if (solution.remaining_capacity(i) >= instance.weight(j, i)
                    && c_best > instance.cost(j, i) - instance.cost(j, i_old)) {
                i_best = i;
                c_best = instance.cost(j, i) - instance.cost(j, i_old);
            }
        }
        if (i_best != -1)
            solution.set(j, i_best);
    }
}

void generalizedassignmentsolver::mthg(
        Solution& solution,
        const std::vector<std::pair<ItemIdx, AgentIdx>>& alternatives)
{
    greedy(solution, alternatives);
    if (solution.feasible())
        nshift(solution);
}

Output generalizedassignmentsolver::mthg(
        const Instance& instance,
        const Desirability& f,
        Info info)
{
    init_display(instance, info);
    FFOT_VER(info,
               "Algorithm" << std::endl
            << "---------" << std::endl
            << "MTHG" << std::endl
            << std::endl
            << "Parameters" << std::endl
            << "----------" << std::endl
            << "Desirability:  " << f.to_string() << std::endl
            << std::endl);

    Output output(instance, info);
    Solution solution(instance);
    auto alt = greedy_init(instance, f);
    mthg(solution, alt);
    output.update_solution(solution, std::stringstream(""), info);
    return output.algorithm_end(info);
}

void generalizedassignmentsolver::mthgregret(
        Solution& solution,
        const Desirability& f,
        const std::vector<std::vector<AgentIdx>>& agents,
        const std::vector<std::vector<int>>& fixed_alternatives)
{
    greedyregret(solution, f, agents, fixed_alternatives);
    if (solution.feasible())
        nshift(solution);
}

Output generalizedassignmentsolver::mthgregret(
        const Instance& instance,
        const Desirability& f,
        Info info)
{
    init_display(instance, info);
    FFOT_VER(info,
               "Algorithm" << std::endl
            << "---------" << std::endl
            << "Regret MTHG" << std::endl
            << std::endl
            << "Parameters" << std::endl
            << "----------" << std::endl
            << "Desirability:  " << f.to_string() << std::endl
            << std::endl);

    Output output(instance, info);
    Solution solution(instance);
    auto agents = greedyregret_init(instance, f);
    mthgregret(solution, f, agents, {});
    output.update_solution(solution, std::stringstream(""), info);
    return output.algorithm_end(info);
}

