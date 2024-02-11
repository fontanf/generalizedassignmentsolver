#include "generalizedassignmentsolver/instance.hpp"

#include <fstream>
#include <iomanip>

using namespace generalizedassignmentsolver;

std::ostream& Instance::format(
        std::ostream& os,
        int verbosity_level) const
{
    if (verbosity_level >= 1) {
        os
            << "Number of agents:  " << number_of_agents() << std::endl
            << "Number of items:   " << number_of_items() << std::endl
            << "Total cost:        " << total_cost() << std::endl
            << "Maximum cost:      " << maximum_cost() << std::endl
            << "Maximum weight:    " << maximum_weight() << std::endl
            ;
    }

    if (verbosity_level >= 2) {
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
