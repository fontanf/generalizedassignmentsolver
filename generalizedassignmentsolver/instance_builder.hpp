#pragma once

#include "generalizedassignmentsolver/instance.hpp"

namespace generalizedassignmentsolver
{

class InstanceBuilder
{

public:

    /** Constructor. */
    InstanceBuilder() { }

    /** Add agents. */
    void add_agents(AgentIdx number_of_agents);

    /** Set the capacity of an agent. */
    void set_capacity(
            AgentIdx agent_id,
            Weight capacity);

    /** Add items. */
    void add_items(ItemIdx number_of_items);

    /** Set the weight of assigning an item to an agent. */
    void set_weight(
            ItemIdx item_id,
            AgentIdx agent_id,
            Weight weight);

    /** Set the cost of assigning an item to an agent. */
    void set_cost(
            ItemIdx item_id,
            AgentIdx agent_id,
            Cost cost);

    /*
     * Build
     */

    /** Build. */
    Instance build();

private:

    /*
     * Private methods
     */

    /*
     * Private attributes
     */

    /** Instance. */
    Instance instance_;

};

}
