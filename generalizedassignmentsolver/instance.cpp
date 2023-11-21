#include "generalizedassignmentsolver/instance.hpp"

#include "generalizedassignmentsolver/instance_builder.hpp"
#include "generalizedassignmentsolver/solution.hpp"

using namespace generalizedassignmentsolver;

Instance::Instance(
        std::string instance_path,
        std::string format)
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

    InstanceBuilder instance_builder;
    instance_builder.add_agents(number_of_agents);
    instance_builder.add_items(number_of_items);

    Cost cost;
    for (AgentIdx agent_id = 0; agent_id < number_of_agents; ++agent_id) {
        for (ItemPos item_id = 0; item_id < number_of_items; ++item_id) {
            file >> cost;
            instance_builder.set_cost(
                    item_id,
                    agent_id,
                    cost);
        }
    }

    Cost weight;
    for (AgentIdx agent_id = 0; agent_id < number_of_agents; ++agent_id) {
        for (ItemPos item_id = 0; item_id < number_of_items; ++item_id) {
            file >> weight;
            instance_builder.set_weight(
                    item_id,
                    agent_id,
                    weight);
        }
    }

    Weight capacity = -1;
    for (AgentIdx agent_id = 0; agent_id < number_of_agents; ++agent_id) {
        file >> capacity;
        instance_builder.set_capacity(
                agent_id,
                capacity);
    }

    *this = instance_builder.build();
}

void Instance::read_standard(std::ifstream& file)
{
    ItemIdx number_of_items;
    AgentIdx number_of_agents;
    file >> number_of_agents >> number_of_items;

    InstanceBuilder instance_builder;
    instance_builder.add_agents(number_of_agents);
    instance_builder.add_items(number_of_items);

    Weight capacity = -1;
    for (AgentIdx agent_id = 0; agent_id < number_of_agents; ++agent_id) {
        file >> capacity;
        instance_builder.set_capacity(
                agent_id,
                capacity);
    }

    Weight weight;
    Cost cost;
    for (ItemPos item_id = 0; item_id < number_of_items; ++item_id) {
        for (AgentIdx agent_id = 0; agent_id < number_of_agents; ++agent_id) {
            file >> weight >> cost;
            instance_builder.set_weight(
                    item_id,
                    agent_id,
                    weight);
            instance_builder.set_cost(
                    item_id,
                    agent_id,
                    cost);
        }
    }

    *this = instance_builder.build();
}

std::ostream& Instance::print(
        std::ostream& os,
        int verbose) const
{
    if (verbose >= 1) {
        os
            << "Number of agents:  " << number_of_agents() << std::endl
            << "Number of items:   " << number_of_items() << std::endl
            << "Total cost:        " << total_cost() << std::endl
            << "Maximum cost:      " << maximum_cost() << std::endl
            << "Maximum weight:    " << maximum_weight() << std::endl
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
            << "     GeneralizedAssignmentSolver     " << std::endl
            << "=====================================" << std::endl
            << std::endl
            << "Instance" << std::endl
            << "--------" << std::endl;
    instance.print(info.os(), info.verbosity_level());
    info.os() << std::endl;
}
