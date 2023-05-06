#include "generalizedassignmentsolver/instance.hpp"

#include "generalizedassignmentsolver/solution.hpp"

using namespace generalizedassignmentsolver;

Instance::Instance(AgentIdx number_of_agents):
    capacities_(number_of_agents, 0)
{ }

void Instance::set_capacities(
        const std::vector<Weight>& capacities)
{
    for (AgentIdx agent_id = 0;
            agent_id < (AgentIdx)capacities.size();
            ++agent_id) {
        set_capacity(agent_id, capacities[agent_id]);
    }
}

void Instance::add_item()
{
    ItemIdx item_id = items_.size();
    items_.push_back({});
    items_[item_id].alternatives.resize(number_of_agents());
    for (AgentIdx agent_id = 0;
            agent_id < number_of_agents();
            ++agent_id) {
        items_[item_id].alternatives[agent_id].item_id = item_id;
        items_[item_id].alternatives[agent_id].agent_id = agent_id;
    }
}

void Instance::add_item(const std::vector<std::pair<Weight, Cost>>& a)
{
    ItemIdx item_id = number_of_items();
    add_item();
    for (AgentIdx agent_id = 0;
            agent_id < (AgentIdx)a.size();
            ++agent_id) {
        set_alternative(item_id, agent_id, a[agent_id].first, a[agent_id].second);
    }
}

void Instance::set_alternative(
        ItemIdx item_id,
        AgentIdx agent_id,
        Weight weight,
        Cost cost)
{
    // Update the weight and the cost of the alternative.
    items_[item_id].alternatives[agent_id].weight = weight;
    items_[item_id].alternatives[agent_id].cost = cost;

    // Update the total weight of the item.
    items_[item_id].total_weight += weight;
    // Update the total cost of the item.
    items_[item_id].total_cost += cost;

    // Update the minimum costl of the item.
    if (items_[item_id].i_minimum_cost != -1 && items_[item_id].minimum_cost > cost)
        sum_of_minimum_costs_ -= items_[item_id].minimum_cost;
    if (items_[item_id].i_minimum_cost == -1 || items_[item_id].minimum_cost > cost) {
        items_[item_id].i_minimum_cost = agent_id;
        items_[item_id].minimum_cost = cost;
        sum_of_minimum_costs_ += cost;
    }
    // Update the minimum weight of the item.
    if (items_[item_id].i_minimum_weight == -1 || items_[item_id].minimum_weight > weight) {
        items_[item_id].i_minimum_weight = agent_id;
        items_[item_id].minimum_weight = weight;
    }
    // Update the maximum cost of the item.
    if (items_[item_id].maximum_cost < cost) {
        items_[item_id].i_maximum_cost = agent_id;
        items_[item_id].maximum_cost = cost;
    }
    // Update to maximum weight of the item.
    if (items_[item_id].maximum_weight < weight) {
        items_[item_id].i_maximum_weight = agent_id;
        items_[item_id].maximum_weight = weight;
    }

    // Update the maximum cost of the instance.
    if (maximum_cost_ < cost)
        maximum_cost_ = cost;
    // Update the maximum weight of the instance.
    if (maximum_weight_ < weight)
        maximum_weight_ = weight;
    // Update the total cost of the instance.
    total_cost_ += cost;
}

Instance::Instance(std::string instance_path, std::string format):
    name_(instance_path)
{
    std::ifstream file(instance_path);
    if (!file.good()) {
        throw std::runtime_error(
                "Unable to open file \"" + instance_path + "\".");
    }

    if (format == "orlibrary") {
        read_orlibrary(file);
    } else if (format == "standard") {
        read_standard(file);
    } else {
        throw std::invalid_argument(
                "Unknown instance format \"" + format + "\".");
    }

    file.close();
}

void Instance::read_orlibrary(std::ifstream& file)
{
    ItemIdx number_of_items;
    AgentIdx number_of_agents;
    file >> number_of_agents >> number_of_items;

    capacities_.resize(number_of_agents);
    items_.reserve(number_of_items);
    for (ItemPos item_id = 0; item_id < number_of_items; ++item_id)
        add_item();
    for (AgentIdx agent_id = 0; agent_id < number_of_agents; ++agent_id)
        for (ItemPos item_id = 0; item_id < number_of_items; ++item_id)
            file >> items_[item_id].alternatives[agent_id].cost;
    for (AgentIdx agent_id = 0; agent_id < number_of_agents; ++agent_id)
        for (ItemPos item_id = 0; item_id < number_of_items; ++item_id)
            file >> items_[item_id].alternatives[agent_id].weight;
    for (AgentIdx agent_id = 0; agent_id < number_of_agents; ++agent_id)
        for (ItemPos item_id = 0; item_id < number_of_items; ++item_id)
            set_alternative(item_id, agent_id, items_[item_id].alternatives[agent_id].weight, items_[item_id].alternatives[agent_id].cost);

    Weight t = -1;
    for (AgentIdx agent_id = 0; agent_id < number_of_agents; ++agent_id) {
        file >> t;
        set_capacity(agent_id, t);
    }
}

void Instance::read_standard(std::ifstream& file)
{
    ItemIdx number_of_items;
    AgentIdx number_of_agents;
    file >> number_of_agents >> number_of_items;

    capacities_.resize(number_of_agents);
    Weight t = -1;
    for (AgentIdx agent_id = 0; agent_id < number_of_agents; ++agent_id) {
        file >> t;
        set_capacity(agent_id, t);
    }

    items_.reserve(number_of_items);
    Weight weight;
    Cost cost;
    for (ItemPos item_id = 0; item_id < number_of_items; ++item_id) {
        add_item();
        for (AgentIdx agent_id = 0; agent_id < number_of_agents; ++agent_id) {
            file >> weight >> cost;
            set_alternative(item_id, agent_id, weight, cost);
        }
    }
}

std::ostream& Instance::print(
        std::ostream& os,
        int verbose) const
{
    if (verbose >= 1) {
        os
            << "Number of agents:  " << number_of_agents() << std::endl
            << "Number of items:   " << number_of_items() << std::endl
            ;
    }

    if (verbose >= 2) {
        os
            << std::endl
            << std::setw(12) << "Agent"
            << std::setw(12) << "Capacity"
            << std::endl
            << std::setw(12) << "-----"
            << std::setw(12) << "--------"
            << std::endl;
        for (AgentIdx agent_id = 0;
                agent_id < number_of_agents();
                ++agent_id) {
            os
                << std::setw(12) << agent_id
                << std::setw(12) << capacity(agent_id)
                << std::endl;
        }

        os
            << std::endl
            << std::setw(12) << "Item"
            << std::setw(12) << "Agent"
            << std::setw(12) << "Weight"
            << std::setw(12) << "Cost"
            << std::endl
            << std::setw(12) << "----"
            << std::setw(12) << "-----"
            << std::setw(12) << "------"
            << std::setw(12) << "----"
            << std::endl;
        for (ItemIdx item_id = 0;
                item_id < number_of_items();
                ++item_id) {
            for (AgentIdx agent_id = 0;
                    agent_id < number_of_agents();
                    ++agent_id) {
                os
                    << std::setw(12) << item_id
                    << std::setw(12) << agent_id
                    << std::setw(12) << weight(item_id, agent_id)
                    << std::setw(12) << cost(item_id, agent_id)
                    << std::endl;
            }
        }
    }

    return os;
}

void Instance::write(std::string instance_path)
{
    std::ofstream file(instance_path);
    if (!file.good()) {
        throw std::runtime_error(
                "Unable to open file \"" + instance_path + "\".");
    }

    file << number_of_agents() << " " << number_of_items() << std::endl;
    for (AgentIdx agent_id = 0; agent_id < number_of_agents(); ++agent_id) {
        for (ItemIdx item_id = 0; item_id < number_of_items(); ++item_id)
            file << item(item_id).alternatives[agent_id].cost << " ";
        file << std::endl;
    }
    for (AgentIdx agent_id = 0; agent_id < number_of_agents(); ++agent_id) {
        for (ItemIdx item_id = 0; item_id < number_of_items(); ++item_id)
            file << item(item_id).alternatives[agent_id].weight << " ";
        file << std::endl;
    }
    for (AgentIdx agent_id = 0; agent_id < number_of_agents(); ++agent_id)
        file << capacity(agent_id) << " ";
    file << std::endl;
    file.close();
}

void generalizedassignmentsolver::init_display(
        const Instance& instance,
        optimizationtools::Info& info)
{
    info.os()
            << "=====================================" << std::endl
            << "    Generalized Assignment Solver    " << std::endl
            << "=====================================" << std::endl
            << std::endl
            << "Instance" << std::endl
            << "--------" << std::endl;
    instance.print(info.os(), info.verbosity_level());
    info.os() << std::endl;
}
