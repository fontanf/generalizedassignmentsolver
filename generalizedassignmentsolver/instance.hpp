#pragma once

#include "optimizationtools/utils/info.hpp"

#include <cstdint>
#include <random>
#include <string>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <memory>
#include <map>

namespace generalizedassignmentsolver
{

using Cost = int64_t;
using PCost = double;
using Weight = int64_t;
using ItemIdx = int64_t;
using ItemPos = int64_t;
using AgentIdx = int64_t;
using AgentPos = int64_t;
using Counter = int64_t;
using Seed = int64_t;

class Solution;

struct Alternative
{
    /** Id of the item. */
    ItemIdx j;
    /** Id of the agent. */
    AgentIdx i;
    /** Weight. */
    Weight weight;
    /** Cost. */
    Cost cost;

    /** Get the efficiency of the alternative. */
    double efficiency() const { return cost * weight; }
};

struct Item
{
    /** Unique id of the item. */
    ItemIdx j;
    /** Alternatives of the item. */
    std::vector<Alternative> alternatives;

    /** Total weight of the item. */
    Weight total_weight;
    /** Total cost of the item. */
    Cost total_cost;

    /** Minimum cost of the item. */
    Cost minimum_cost = -1;
    /** Maximum cost of the item. */
    Cost maximum_cost = -1;
    /** Id of an agent of minimum cost of the item. */
    AgentIdx i_minimum_cost = -1;
    /** Id of an agent of maximum cost of the item. */
    AgentIdx i_maximum_cost = -1;

    /** Minimum weight of the item. */
    Weight minimum_weight = -1;
    /** Maximum weight of the item. */
    Weight maximum_weight = -1;
    /** Id of an agent of minimum weight of the item. */
    AgentIdx i_minimum_weight = -1;
    /** Id of an agent of maximum weight of hte item. */
    AgentIdx i_maximum_weight = -1;
};

/**
 * Instance class for a Generalized Assignment Problem.
 */
class Instance
{

public:

    /*
     * Constructors and destructor.
     */

    /** Create an instance from a file. */
    Instance(std::string instance_path, std::string format = "orlibrary");

    /** Create an instance manually. */
    Instance(AgentIdx m);
    /** Set the name of the instance. */
    void set_name(std::string name) { name_ = name; }
    /** Set the capacity of agent 'i' to 't'. */
    void set_capacity(AgentIdx i, Weight t) { capacities_[i] = t; }
    /** Add an item. */
    void add_item();
    /** Set the weight and the profit of assigning item 'j' to agent 'i'. */
    void set_alternative(ItemIdx j, AgentIdx i, Weight weight, Cost cost);
    /** Clear the instance. */
    void clear();

    /** Add an item with its weights and costs. */
    void add_item(const std::vector<std::pair<Weight, Cost>>& a);
    /** Set the capacity of all agents. */
    void set_capacities(const std::vector<Weight>& t);
    /** Set the optimal solution. */
    void set_optimal_solution(Solution& solution);

    /*
     * Getters
     */

    /** Get the name of the instance. */
    std::string name() const { return name_; }
    /** Get item 'j'. */
    const Item& item(ItemPos j) const { return items_[j]; }

    /** Get the maximum cost of the instance. */
    inline Cost maximum_cost() const { return maximum_cost_; }
    /** Get the maximum weight of the instance. */
    inline Cost maximum_weight() const { return maximum_weight_; }

    /** Get the number of items. */
    ItemIdx number_of_items() const { return items_.size(); }
    /** Get the number of agents. */
    AgentIdx number_of_agents() const { return capacities_.size(); }
    /** Get the capacity of agent 'i'. */
    Weight capacity(AgentIdx i) const { return capacities_[i]; }

    /** Get the weight of item 'j' when assigned to agent 'i'. */
    inline Weight weight(ItemIdx j, AgentIdx i) const { return items_[j].alternatives[i].weight; }
    /** Get the cost of item 'j' when assigned to agent 'i'. */
    inline Cost cost(ItemIdx j, AgentIdx i) const { return items_[j].alternatives[i].cost; }
    /** Get the profit of item 'j' when assigned to agent 'i'. */
    inline Cost profit(ItemIdx j, AgentIdx i) const { return items_[j].maximum_cost - items_[j].alternatives[i].cost; }

    /** Get the optimal solution. */
    const Solution* optimal_solution() const { return optimal_solution_.get(); }
    /** Get the optimum value. */
    Cost optimum() const;
    /** Get a trivial bound. */
    Cost bound() const { return total_cost_ + 1; }
    /** Get the bound of the combinatorial relaxation. */
    Cost combinatorial_relaxation() const { return sum_of_minimum_costs_ + 1; }

    /** Write the instance to a file. */
    void write(std::string instance_path);

private:

    /*
     * Private methods.
     */

    /** Read an instance in 'orlibrary' format. */
    void read_orlibrary(std::ifstream& file);
    /** Read an instance in 'standard' format. */
    void read_standard(std::ifstream& file);

    /*
     * Private attributes.
     */

    /** Name of the instance. */
    std::string name_;
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
    /**
     * Optimal solution.
     *
     * 'nullptr' if not set.
     */
    std::shared_ptr<Solution> optimal_solution_;

};

/** Stream insertion operator. */
std::ostream& operator<<(std::ostream &os, const Alternative& alternative);
/** Stream insertion operator. */
std::ostream& operator<<(std::ostream &os, const Instance& instance);

void init_display(
        const Instance& instance,
        optimizationtools::Info& info);

}

