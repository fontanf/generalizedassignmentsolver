#pragma once

#include <cstdint>
#include <string>
#include <iostream>
#include <vector>

namespace generalizedassignmentsolver
{

using Cost = int64_t;
using Weight = int64_t;
using ItemIdx = int64_t;
using ItemPos = int64_t;
using AgentIdx = int64_t;
using AgentPos = int64_t;
using Counter = int64_t;
using Seed = int64_t;

/**
 * Structure for an alternative.
 */
struct Alternative
{
    /** Id of the item. */
    ItemIdx item_id;

    /** Id of the agent. */
    AgentIdx agent_id;

    /** Weight. */
    Weight weight;

    /** Cost. */
    Cost cost;

    /** Get the efficiency of the alternative. */
    double efficiency() const { return cost * weight; }
};

/**
 * Structure for an item.
 */
struct Item
{
    /** Alternatives of the item. */
    std::vector<Alternative> alternatives;

    /** Total cost of the item. */
    Cost total_cost;

    /** Minimum cost of the item. */
    Cost minimum_cost = -1;

    /** Id of an agent of minimum cost of the item. */
    AgentIdx minimum_cost_agent_id = -1;

    /** Maximum cost of the item. */
    Cost maximum_cost = -1;

    /** Id of an agent of maximum cost of the item. */
    AgentIdx maximum_cost_agent_id = -1;

    /** Total weight of the item. */
    Weight total_weight;

    /** Minimum weight of the item. */
    Weight minimum_weight = -1;

    /** Id of an agent of minimum weight of the item. */
    AgentIdx minimum_weight_agent_id = -1;

    /** Maximum weight of the item. */
    Weight maximum_weight = -1;

    /** Id of an agent of maximum weight of hte item. */
    AgentIdx maximum_weight_agent_id = -1;
};

/**
 * Instance class for a generalized assignment problem.
 */
class Instance
{

public:

    /*
     * Getters
     */

    /** Get the number of items. */
    ItemIdx number_of_items() const { return items_.size(); }

    /** Get an item. */
    const Item& item(ItemPos item_id) const { return items_[item_id]; }

    /** Get the number of agents. */
    AgentIdx number_of_agents() const { return capacities_.size(); }

    /** Get the capacity of an agent. */
    Weight capacity(AgentIdx agent_id) const { return capacities_[agent_id]; }

    /** Get the weight of an item when assigned to an agent. */
    inline Weight weight(ItemIdx item_id, AgentIdx agent_id) const { return items_[item_id].alternatives[agent_id].weight; }

    /** Get the cost of an item when assigned to an agent. */
    inline Cost cost(ItemIdx item_id, AgentIdx agent_id) const { return items_[item_id].alternatives[agent_id].cost; }

    /** Get the profit of an item when assigned to an agent. */
    inline Cost profit(ItemIdx item_id, AgentIdx agent_id) const { return items_[item_id].maximum_cost - items_[item_id].alternatives[agent_id].cost; }

    /** Get the total cost of the instance. */
    inline Cost total_cost() const { return total_cost_; }

    /** Get the maximum cost of the instance. */
    inline Cost maximum_cost() const { return maximum_cost_; }

    /** Get the maximum weight of the instance. */
    inline Cost maximum_weight() const { return maximum_weight_; }

    /** Get a trivial bound. */
    Cost bound() const { return total_cost_ + 1; }

    /** Get the bound of the combinatorial relaxation. */
    Cost combinatorial_relaxation() const { return sum_of_minimum_costs_ + 1; }

    /*
     * Export
     */

    /** Print the instance into a stream. */
    std::ostream& format(
            std::ostream& os,
            int verbosity_level = 1) const;

    /** Write the instance to a file. */
    void write(const std::string& instance_path);

private:

    /*
     * Private methods
     */

    /** Create an instance manually. */
    Instance() { }

    /*
     * Private attributes
     */

    /** Items. */
    std::vector<Item> items_;

    /** Capacities. */
    std::vector<Weight> capacities_;

    /** Maximum cost of the instance. */
    Cost maximum_cost_ = -1;

    /** Total cost of all alternatives. */
    Cost total_cost_ = 0;

    /** Maximum weight of the instance. */
    Cost maximum_weight_ = -1;

    /** Sum of the minimum cost of each item. */
    Cost sum_of_minimum_costs_ = 0;

    friend class InstanceBuilder;

};

}
