#include "generalizedassignmentsolver/solution.hpp"

#include "optimizationtools/utils/utils.hpp"

#include <iomanip>
#include <iostream>

using namespace generalizedassignmentsolver;

Solution::Solution(const Instance& instance):
    instance_(&instance),
    x_(instance.number_of_items(), -1),
    agents_(instance.number_of_agents())
{ }

Solution::Solution(
        const Instance& instance,
        std::string certificate_path):
    Solution(instance)
{
    if (certificate_path.empty())
        return;
    std::ifstream file(certificate_path);
    if (!file.good()) {
        throw std::runtime_error(
                "Unable to open file \"" + certificate_path + "\".");
    }

    AgentIdx agent_id = -1;
    for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id) {
        file >> agent_id;
        set(item_id, agent_id);
    }
}

Solution::Solution(const Instance& instance, const std::vector<std::vector<ItemIdx>>& agents):
    Solution(instance)
{
    for (AgentIdx agent_id = 0; agent_id < (AgentIdx)agents.size(); agent_id++)
        for (ItemIdx item_id: agents[agent_id])
            set(item_id, agent_id);
}

void Solution::set(
        ItemIdx item_id,
        AgentIdx agent_id)
{
    assert(agent_id >= -1 || agent_id < instance().number_of_agents());
    assert(item_id >= 0 && item_id < instance().number_of_items());

    AgentIdx agent_id_old = agent(item_id);
    if (agent_id_old == agent_id)
        return;

    if (agent_id_old != -1) {
        Weight w_old = instance().weight(item_id, agent_id_old);
        Cost c_old = instance().cost(item_id, agent_id_old);
        if (agents_[agent_id_old].weight <= instance().capacity(agent_id_old)) {
        } else if (agents_[agent_id_old].weight - w_old >= instance().capacity(agent_id_old)) {
            agents_[agent_id_old].overcapacity -= w_old;
            total_overcapacity_ -= w_old;
        } else {
            agents_[agent_id_old].overcapacity -= agents_[agent_id_old].weight - instance().capacity(agent_id_old);
            total_overcapacity_ -= agents_[agent_id_old].weight - instance().capacity(agent_id_old);
        }
        agents_[agent_id_old].cost -= c_old;
        total_cost_ -= c_old;
        agents_[agent_id_old].weight -= w_old;
        total_weight_ -= w_old;
        number_of_items_--;
    }

    if (agent_id != -1) {
        Weight w = instance().weight(item_id, agent_id);
        Cost c = instance().cost(item_id, agent_id);
        if (agents_[agent_id].weight >= instance().capacity(agent_id)) {
            agents_[agent_id].overcapacity += w;
            total_overcapacity_ += w;
        } else if (agents_[agent_id].weight + w <= instance().capacity(agent_id)) {
        } else {
            Weight w_tmp = agents_[agent_id].weight + w - instance().capacity(agent_id);
            agents_[agent_id].overcapacity += w_tmp;
            total_overcapacity_ += w_tmp;
        }
        agents_[agent_id].cost += c;
        total_cost_ += c;
        agents_[agent_id].weight += w;
        total_weight_ += w;
        number_of_items_++;
    }

    x_[item_id] = agent_id;
}

std::ostream& Solution::format(
        std::ostream& os,
        int verbosity_level) const
{
    if (verbosity_level >= 1) {
        os
            << "Number of items:  " << optimizationtools::Ratio<ItemIdx>(number_of_items(), instance().number_of_items()) << std::endl
            << "Feasible:         " << feasible() << std::endl
            << "Cost:             " << cost() << std::endl
            ;
    }

    if (verbosity_level >= 2) {
        os << std::endl
            << std::setw(12) << "Item"
            << std::setw(12) << "Agent"
            << std::endl
            << std::setw(12) << "----"
            << std::setw(12) << "-----"
            << std::endl;
        for (ItemIdx item_id = 0;
                item_id < instance().number_of_items();
                ++item_id) {
            os
                << std::setw(12) << item_id
                << std::setw(12) << agent(item_id)
                << std::endl;
        }
    }

    return os;
}

nlohmann::json Solution::to_json() const
{
    return nlohmann::json {
        {"NumberOfItems", number_of_items()},
        {"NumberOfItems", number_of_items()},
        {"Feasible", feasible()},
        {"Cost", cost()},
    };
}

void Solution::write(
        const std::string& certificate_path) const
{
    if (certificate_path.empty())
        return;
    std::ofstream file(certificate_path);
    if (!file.good()) {
        throw std::runtime_error(
                "Unable to open file \"" + certificate_path + "\".");
    }

    std::copy(x_.begin(), x_.end(), std::ostream_iterator<AgentIdx>(file, " "));
    file << std::endl;
    file.close();
}
