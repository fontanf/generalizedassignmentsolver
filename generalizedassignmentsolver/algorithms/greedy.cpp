#include "generalizedassignmentsolver/algorithms/greedy.hpp"

#include "generalizedassignmentsolver/algorithm_formatter.hpp"

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

const Output generalizedassignmentsolver::greedy(
        const Instance& instance,
        const Desirability& f,
        const Parameters& parameters)
{
    Output output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Greedy");
    algorithm_formatter.print_header();

    Solution solution(instance);
    auto alternatives = greedy_init(instance, f);
    greedy(solution, alternatives);
    algorithm_formatter.update_solution(solution, "");

    algorithm_formatter.end();
    return output;
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

const Output generalizedassignmentsolver::greedy_regret(
        const Instance& instance,
        const Desirability& f,
        const Parameters& parameters)
{
    Output output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Regret greedy");
    algorithm_formatter.print_header();

    Solution solution(instance);
    auto agents = greedy_regret_init(instance, f);
    greedy_regret(solution, f, agents, {});
    algorithm_formatter.update_solution(solution, "");

    algorithm_formatter.end();
    return output;
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

const Output generalizedassignmentsolver::mthg(
        const Instance& instance,
        const Desirability& f,
        const Parameters& parameters)
{
    Output output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("MTHG");
    algorithm_formatter.print_header();

    Solution solution(instance);
    auto alt = greedy_init(instance, f);
    mthg(solution, alt);
    algorithm_formatter.update_solution(solution, "");

    algorithm_formatter.end();
    return output;
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

const Output generalizedassignmentsolver::mthg_regret(
        const Instance& instance,
        const Desirability& f,
        const Parameters& parameters)
{
    Output output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Regret MTHG");
    algorithm_formatter.print_header();

    Solution solution(instance);
    auto agents = greedy_regret_init(instance, f);
    mthg_regret(solution, f, agents, {});
    algorithm_formatter.update_solution(solution, "");

    algorithm_formatter.end();
    return output;
}
