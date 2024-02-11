#pragma once

#include "generalizedassignmentsolver/instance.hpp"

#include "optimizationtools/utils/output.hpp"
#include "optimizationtools/utils/utils.hpp"

#include "nlohmann//json.hpp"

#include <iomanip>

namespace generalizedassignmentsolver
{

/**
 * Solution class for a generalized assignment problem.
 */
class Solution
{

public:

    /*
     * Structures
     */

    struct SolutionAgent
    {
        Cost cost = 0;
        Weight weight = 0;
        Weight overcapacity = 0;
    };

    /*
     * Constructors and destructor
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
     * Getters
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

    /** Get the total cost of the solution. */
    inline Cost objective_value() const { return cost(); }

    /** Get the number of items in the solution. */
    inline ItemIdx number_of_items() const { return number_of_items_; }

    /**
     * Get the agent to which an item is assigned.
     *
     * Return -1 if the item is not assigned to an agent.
     */
    inline AgentIdx agent(ItemIdx item_id) const { return x_[item_id]; }

    /*
     * Setters
     */

    /**
     * Assign an item to an agent.
     *
     * If 'agent_id == -1', remove the affectation of the item.
     */
    void set(ItemIdx item_id, AgentIdx agent_id);

    /*
     * Export
     */

    /** Print the instance. */
    std::ostream& format(
            std::ostream& os,
            int verbosity_level = 1) const;

    /** Export solution characteristics to a JSON structure. */
    nlohmann::json to_json() const;

    /** Write the solution to a file. */
    void write(const std::string& certificate_path) const;

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

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Output ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

inline optimizationtools::ObjectiveDirection objective_direction()
{
    return optimizationtools::ObjectiveDirection::Minimize;
}

/**
 * Output structure for a generalized assignment problem.
 */
struct Output: optimizationtools::Output
{
    /** Constructor. */
    Output(const Instance& instance):
        solution(instance),
        bound(instance.combinatorial_relaxation()) { }


    /** Solution. */
    Solution solution;

    /** Bound. */
    Cost bound = 0;

    /** Elapsed time. */
    double time = 0.0;


    std::string solution_value() const
    {
        return optimizationtools::solution_value(
            objective_direction(),
            solution.feasible(),
            solution.objective_value());
    }

    double absolute_optimality_gap() const
    {
        return optimizationtools::absolute_optimality_gap(
                objective_direction(),
                solution.feasible(),
                solution.objective_value(),
                bound);
    }

    double relative_optimality_gap() const
    {
       return optimizationtools::relative_optimality_gap(
            objective_direction(),
            solution.feasible(),
            solution.objective_value(),
            bound);
    }

    virtual nlohmann::json to_json() const
    {
        return nlohmann::json {
            {"Solution", solution.to_json()},
            {"Value", solution_value()},
            {"Bound", bound},
            {"AbsoluteOptimalityGap", absolute_optimality_gap()},
            {"RelativeOptimalityGap", relative_optimality_gap()},
            {"Time", time}
        };
    }

    virtual int format_width() const { return 30; }

    virtual void format(std::ostream& os) const
    {
        int width = format_width();
        os
            << std::setw(width) << std::left << "Value: " << solution_value() << std::endl
            << std::setw(width) << std::left << "Bound: " << bound << std::endl
            << std::setw(width) << std::left << "Absolute optimality gap: " << absolute_optimality_gap() << std::endl
            << std::setw(width) << std::left << "Relative optimality gap (%): " << relative_optimality_gap() * 100 << std::endl
            << std::setw(width) << std::left << "Time (s): " << time << std::endl
            ;
    }
};

using NewSolutionCallback = std::function<void(const Output&, const std::string&)>;

struct Parameters: optimizationtools::Parameters
{
    /** Callback function called when a new best solution is found. */
    NewSolutionCallback new_solution_callback = [](const Output&, const std::string&) { };


    virtual nlohmann::json to_json() const override
    {
        nlohmann::json json = optimizationtools::Parameters::to_json();
        json.merge_patch({});
        return json;
    }

    virtual int format_width() const override { return 23; }

    virtual void format(std::ostream& os) const override
    {
        optimizationtools::Parameters::format(os);
    }
};

}
