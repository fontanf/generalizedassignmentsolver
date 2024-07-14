#include "generalizedassignmentsolver/instance_builder.hpp"

#include <fstream>

using namespace generalizedassignmentsolver;

void InstanceBuilder::add_agents(AgentIdx number_of_agents)
{
    instance_.capacities_.insert(instance_.capacities_.end(), number_of_agents, 0);
    for (ItemIdx item_id = 0;
            item_id < instance_.number_of_items();
            ++item_id) {
        instance_.items_[item_id].alternatives.insert(
                instance_.items_[item_id].alternatives.end(),
                number_of_agents,
                Alternative());
    }
}

void InstanceBuilder::set_capacity(
        AgentIdx agent_id,
        Weight capacity)
{
    instance_.capacities_[agent_id] = capacity;
}

void InstanceBuilder::add_items(ItemIdx number_of_items)
{
    Item item;
    item.alternatives.insert(
            item.alternatives.end(),
            instance_.number_of_agents(),
            Alternative());
    instance_.items_.insert(
            instance_.items_.end(),
            number_of_items,
            item);
}

void InstanceBuilder::set_weight(
        ItemIdx item_id,
        AgentIdx agent_id,
        Weight weight)
{
    instance_.items_[item_id].alternatives[agent_id].weight = weight;
}

void InstanceBuilder::set_cost(
        ItemIdx item_id,
        AgentIdx agent_id,
        Cost cost)
{
    instance_.items_[item_id].alternatives[agent_id].cost = cost;
}

void InstanceBuilder::read(
        const std::string& instance_path,
        const std::string& format)
{
    std::ifstream file(instance_path);
    if (!file.good()) {
        throw std::runtime_error(
                "Unable to open file \"" + instance_path + "\".");
    }

    if (format == "orlibrary" || format == "") {
        read_orlibrary(file);
    } else if (format == "standard") {
        read_standard(file);
    } else {
        throw std::invalid_argument(
                "Unknown instance format \"" + format + "\".");
    }

    file.close();
}

void InstanceBuilder::read_orlibrary(
        std::ifstream& file)
{
    ItemIdx number_of_items;
    AgentIdx number_of_agents;
    file >> number_of_agents >> number_of_items;

    add_agents(number_of_agents);
    add_items(number_of_items);

    Cost cost;
    for (AgentIdx agent_id = 0; agent_id < number_of_agents; ++agent_id) {
        for (ItemPos item_id = 0; item_id < number_of_items; ++item_id) {
            file >> cost;
            set_cost(
                    item_id,
                    agent_id,
                    cost);
        }
    }

    Cost weight;
    for (AgentIdx agent_id = 0; agent_id < number_of_agents; ++agent_id) {
        for (ItemPos item_id = 0; item_id < number_of_items; ++item_id) {
            file >> weight;
            set_weight(
                    item_id,
                    agent_id,
                    weight);
        }
    }

    Weight capacity = -1;
    for (AgentIdx agent_id = 0; agent_id < number_of_agents; ++agent_id) {
        file >> capacity;
        set_capacity(
                agent_id,
                capacity);
    }
}

void InstanceBuilder::read_standard(
        std::ifstream& file)
{
    ItemIdx number_of_items;
    AgentIdx number_of_agents;
    file >> number_of_agents >> number_of_items;

    add_agents(number_of_agents);
    add_items(number_of_items);

    Weight capacity = -1;
    for (AgentIdx agent_id = 0; agent_id < number_of_agents; ++agent_id) {
        file >> capacity;
        set_capacity(
                agent_id,
                capacity);
    }

    Weight weight;
    Cost cost;
    for (ItemPos item_id = 0; item_id < number_of_items; ++item_id) {
        for (AgentIdx agent_id = 0; agent_id < number_of_agents; ++agent_id) {
            file >> weight >> cost;
            set_weight(
                    item_id,
                    agent_id,
                    weight);
            set_cost(
                    item_id,
                    agent_id,
                    cost);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Build /////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

Instance InstanceBuilder::build()
{
    // Compute total and maximum weight and cost of the instance.
    for (ItemIdx item_id = 0;
            item_id < instance_.number_of_items();
            ++item_id) {

        for (AgentIdx agent_id = 0;
                agent_id < instance_.number_of_agents();
                ++agent_id) {
            Cost cost = instance_.cost(item_id, agent_id);
            Cost weight = instance_.weight(item_id, agent_id);

            instance_.total_cost_ += cost;
            instance_.maximum_cost_ = std::max(instance_.maximum_cost_, cost);
            instance_.maximum_weight_ = std::max(instance_.maximum_weight_, weight);
            instance_.items_[item_id].total_weight += weight;
            instance_.items_[item_id].total_cost += cost;

            if (instance_.items_[item_id].maximum_weight_agent_id == -1
                    || instance_.items_[item_id].maximum_weight < weight) {
                instance_.items_[item_id].maximum_weight = weight;
                instance_.items_[item_id].maximum_weight_agent_id = agent_id;
            }

            if (instance_.items_[item_id].minimum_weight_agent_id == -1
                    || instance_.items_[item_id].minimum_weight > weight) {
                instance_.items_[item_id].minimum_weight = weight;
                instance_.items_[item_id].minimum_weight_agent_id = agent_id;
            }

            if (instance_.items_[item_id].maximum_cost_agent_id == -1
                    || instance_.items_[item_id].maximum_cost < cost) {
                instance_.items_[item_id].maximum_cost = cost;
                instance_.items_[item_id].maximum_cost_agent_id = agent_id;
            }

            if (instance_.items_[item_id].minimum_cost_agent_id == -1
                    || instance_.items_[item_id].minimum_cost > cost) {
                instance_.items_[item_id].minimum_cost = cost;
                instance_.items_[item_id].minimum_cost_agent_id = agent_id;
            }
        }

        instance_.sum_of_minimum_costs_ += instance_.item(item_id).minimum_cost;
    }

    return std::move(instance_);
}
