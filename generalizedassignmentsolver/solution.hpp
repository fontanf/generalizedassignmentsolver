#pragma once

#include "generalizedassignmentsolver/instance.hpp"

namespace generalizedassignmentsolver
{

/**
 * Solution class for a Generalized Assignment Problem.
 */
class Solution
{

public:

    /*
     * Structures.
     */

    struct SolutionAgent
    {
        Cost cost = 0;
        Weight weight = 0;
        Weight overcapacity = 0;
    };

    /*
     * Constructors and destructor.
     */

    /** Create an empty solution. */
    Solution(const Instance& instance);

    /** Create a solution from a file. */
    Solution(
            const Instance& instance,
            std::string certificate_path);

    Solution(
            const Instance& instance,
            const std::vector<std::vector<ItemIdx>>& agents);

    /*
     * Getters.
     */

    /** Get the instance. */
    inline const Instance& instance() const { return *instance_; }

    /** Get the total weight of the solution. */
    inline Weight weight() const { return total_weight_; }

    /** Get the weight of an agent. */
    inline Weight weight(AgentIdx agent_id) const { return agents_[agent_id].weight; }

    /** Get the remaining capacity of an agent. */
    inline Weight remaining_capacity(AgentIdx agent_id) const { return instance().capacity(agent_id) - weight(agent_id); }

    /** Get the total overcapacity of the solution. */
    inline Weight overcapacity() const { return total_overcapacity_; };

    /** Get the overcapacity of an agent. */
    inline Weight overcapacity(AgentIdx agent_id) const { return agents_[agent_id].overcapacity; };

    /** Get the total cost of the solution. */
    inline Cost cost() const { return total_cost_; }

    /** Get the cost of an agent. */
    inline Cost cost(AgentIdx agent_id) const { return agents_[agent_id].cost; }

    /** Return 'true' iff all items have been assigned. */
    inline bool full() const { return number_of_items_ == instance().number_of_items(); }

    /** Return 'true' iff the solution is feasible. */
    inline bool feasible() const { return full() && (overcapacity() == 0); }

    /** Get the number of items in the solution. */
    inline ItemIdx number_of_items() const { return number_of_items_; }

    /**
     * Get the agent to which an item is assigned.
     *
     * Return -1 if the item is not assigned to an agent.
     */
    inline AgentIdx agent(ItemIdx item_id) const { return x_[item_id]; }

    /*
     * Setters.
     */

    /**
     * Assign an item to an agent.
     *
     * If 'agent_id == -1', remove the affectation of the item.
     */
    void set(ItemIdx item_id, AgentIdx agent_id);

    /*
     * Export.
     */

    /** Print the instance. */
    std::ostream& print(
            std::ostream& os,
            int verbose = 1) const;

    /** Write the solution to a file. */
    void write(std::string filepath);

private:

    /** Instance. */
    const Instance* instance_;

    /**
     * For each item, the agent to which it is assigned.
     *
     * 'x_[item_id] == -1' if item 'j' has not been assigned.to any agent.
     */
    std::vector<AgentIdx> x_;

    /** Agents. */
    std::vector<SolutionAgent> agents_;

    /** Number of items assigned. */
    ItemIdx number_of_items_ = 0;

    /** Total cost of the solution. */
    Cost total_cost_ = 0;

    /** Total weight of assigned items. */
    Weight total_weight_ = 0;

    /** Total overcapacity of the solution. */
    Weight total_overcapacity_ = 0;

};

/**
 * Return 'true' iff 'current_solution' is strictly better than
 * 'best_solution'.
 */
bool compare(
        const Solution& best_solution,
        const Solution& current_solution);

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Output ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/**
 * Output structure for a generalized assignment problem.
 */
struct Output
{
    /** Constructor. */
    Output(
            const Instance& instance,
            optimizationtools::Info& info);

    /** Solution. */
    Solution solution;

    /** Bound. */
    Cost bound = 0;

    /** Elapsed time. */
    double time = -1;

    /** Return 'true' iff the solution is optimal. */
    bool optimal() const { return solution.feasible() && solution.cost() == bound; }

    /** Print current state. */
    void print(
            optimizationtools::Info& info,
            const std::stringstream& s) const;

    /** Update the solution. */
    void update_solution(
            const Solution& solution_new,
            const std::stringstream& s,
            optimizationtools::Info& info);

    /** Update the bound. */
    void update_bound(
            Cost bound_new,
            const std::stringstream& s,
            optimizationtools::Info& info);

    /** Print the algorithm statistics. */
    virtual void print_statistics(
            optimizationtools::Info& info) const { (void)info; }

    /** Method to call at the end of the algorithm. */
    Output& algorithm_end(optimizationtools::Info& info);
};

}

