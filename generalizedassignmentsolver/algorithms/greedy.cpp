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
    std::vector<std::pair<ItemIdx, AgentIdx>> alternatives;
    for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
        for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id)
            alternatives.push_back({item_id, agent_id});
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
        ItemIdx item_id = p.first;
        if (solution.agent(item_id) != -1)
            continue;
        AgentIdx agent_id = p.second;
        if (solution.remaining_capacity(agent_id) < instance.weight(item_id, agent_id))
            continue;
        solution.set(item_id, agent_id);
        if (solution.full())
            break;
    }
}

Output generalizedassignmentsolver::greedy(
        const Instance& instance,
        const Desirability& f,
        optimizationtools::Info info)
{
    init_display(instance, info);
    info.os()
            << "Algorithm" << std::endl
            << "---------" << std::endl
            << "Greedy" << std::endl
            << std::endl
            << "Parameters" << std::endl
            << "----------" << std::endl
            << "Desirability:  " << f.to_string() << std::endl
            << std::endl;

    Output output(instance, info);
    Solution solution(instance);
    auto alternatives = greedy_init(instance, f);
    greedy(solution, alternatives);
    output.update_solution(solution, std::stringstream(""), info);
    return output.algorithm_end(info);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

std::vector<std::vector<AgentIdx>> generalizedassignmentsolver::greedy_regret_init(
        const Instance& instance,
        const Desirability& f)
{
    ItemIdx n = instance.number_of_items();
    AgentIdx m = instance.number_of_agents();

    std::vector<std::vector<AgentIdx>> agents(n, std::vector<AgentIdx>(m));
    for (ItemIdx item_id = 0;
            item_id < instance.number_of_items();
            ++item_id) {
        std::iota(agents[item_id].begin(), agents[item_id].end(), 0);
        sort(agents[item_id].begin(), agents[item_id].end(),
                [&f, &item_id](AgentIdx agent_id_1, AgentIdx agent_id_2) -> bool
                {
                    return f(item_id, agent_id_1) < f(item_id, agent_id_2);
                });
    }

    return agents;
}

void generalizedassignmentsolver::greedy_regret(
        Solution& solution,
        const Desirability& f,
        const std::vector<std::vector<AgentIdx>>& agents,
        const std::vector<std::vector<int>>& fixed_alternatives)
{
    const Instance& instance = solution.instance();
    std::vector<std::pair<AgentIdx, AgentIdx>> bests(instance.number_of_items(), {0, 1});

    while (!solution.full()) {
        ItemIdx item_id_best = -1;
        double f_best = -1;
        for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id) {
            if (solution.agent(item_id) != -1)
                continue;

            AgentIdx& agent_id_first = bests[item_id].first;
            AgentIdx& agent_id_second = bests[item_id].second;

            while (agent_id_first < instance.number_of_agents()) {
                AgentIdx agent_id = agents[item_id][agent_id_first];
                if (instance.weight(item_id, agent_id) > solution.remaining_capacity(agent_id)
                        || (!fixed_alternatives.empty()
                            && fixed_alternatives[item_id][agent_id] != -1)) {
                    agent_id_first++;
                    if (agent_id_first == agent_id_second)
                        agent_id_second++;
                } else {
                    break;
                }
            }
            if (agent_id_first == instance.number_of_agents())
                return;

            while (agent_id_second < instance.number_of_agents()) {
                AgentIdx agent_id = agents[item_id][agent_id_second];
                if (instance.weight(item_id, agent_id) > solution.remaining_capacity(agent_id)
                        || (!fixed_alternatives.empty()
                            && fixed_alternatives[item_id][agent_id] != -1)) {
                    agent_id_second++;
                } else {
                    break;
                }
            }

            double f_curr = (agent_id_second == instance.number_of_agents())?
                std::numeric_limits<double>::infinity():
                f(item_id, agents[item_id][agent_id_second])
                - f(item_id, agents[item_id][agent_id_first]);
            if (item_id_best == -1 || f_best < f_curr) {
                f_best = f_curr;
                item_id_best = item_id;
            }
        }
        solution.set(item_id_best, agents[item_id_best][bests[item_id_best].first]);
    }
}

Output generalizedassignmentsolver::greedy_regret(
        const Instance& instance,
        const Desirability& f,
        optimizationtools::Info info)
{
    init_display(instance, info);
    info.os()
            << "Algorithm" << std::endl
            << "---------" << std::endl
            << "Regret greedy" << std::endl
            << std::endl
            << "Parameters" << std::endl
            << "----------" << std::endl
            << "Desirability:  " << f.to_string() << std::endl
            << std::endl;

    Output output(instance, info);
    Solution solution(instance);
    auto agents = greedy_regret_init(instance, f);
    greedy_regret(solution, f, agents, {});
    output.update_solution(solution, std::stringstream(""), info);
    return output.algorithm_end(info);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void nshift(Solution& solution)
{
    const Instance& instance = solution.instance();
    for (ItemIdx item_id = 0;
            item_id < instance.number_of_items();
            ++item_id) {
        AgentIdx agent_id_old = solution.agent(item_id);
        Cost c_best = 0;
        AgentIdx agent_id_best = -1;
        for (AgentIdx agent_id = 0;
                agent_id < instance.number_of_agents();
                ++agent_id) {
            if (agent_id == agent_id_old)
                continue;
            if (solution.remaining_capacity(agent_id) >= instance.weight(item_id, agent_id)
                    && c_best > instance.cost(item_id, agent_id)
                    - instance.cost(item_id, agent_id_old)) {
                agent_id_best = agent_id;
                c_best = instance.cost(item_id, agent_id)
                    - instance.cost(item_id, agent_id_old);
            }
        }
        if (agent_id_best != -1)
            solution.set(item_id, agent_id_best);
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
        optimizationtools::Info info)
{
    init_display(instance, info);
    info.os()
            << "Algorithm" << std::endl
            << "---------" << std::endl
            << "MTHG" << std::endl
            << std::endl
            << "Parameters" << std::endl
            << "----------" << std::endl
            << "Desirability:  " << f.to_string() << std::endl
            << std::endl;

    Output output(instance, info);
    Solution solution(instance);
    auto alt = greedy_init(instance, f);
    mthg(solution, alt);
    output.update_solution(solution, std::stringstream(""), info);
    return output.algorithm_end(info);
}

void generalizedassignmentsolver::mthg_regret(
        Solution& solution,
        const Desirability& f,
        const std::vector<std::vector<AgentIdx>>& agents,
        const std::vector<std::vector<int>>& fixed_alternatives)
{
    greedy_regret(solution, f, agents, fixed_alternatives);
    if (solution.feasible())
        nshift(solution);
}

Output generalizedassignmentsolver::mthg_regret(
        const Instance& instance,
        const Desirability& f,
        optimizationtools::Info info)
{
    init_display(instance, info);
    info.os()
            << "Algorithm" << std::endl
            << "---------" << std::endl
            << "Regret MTHG" << std::endl
            << std::endl
            << "Parameters" << std::endl
            << "----------" << std::endl
            << "Desirability:  " << f.to_string() << std::endl
            << std::endl;

    Output output(instance, info);
    Solution solution(instance);
    auto agents = greedy_regret_init(instance, f);
    mthg_regret(solution, f, agents, {});
    output.update_solution(solution, std::stringstream(""), info);
    return output.algorithm_end(info);
}

