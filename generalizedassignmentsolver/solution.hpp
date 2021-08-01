#pragma once

#include "generalizedassignmentsolver/instance.hpp"

namespace generalizedassignmentsolver
{

struct SolutionAgent
{
    Cost cost = 0;
    Weight weight = 0;
    Weight overcapacity = 0;
    PCost penalty = 0;
    PCost pcost = 0;
};

class Solution
{

public:

    /** Create an empty solution. */
    Solution(const Instance& instance);
    /** Create a solution from a file. */
    Solution(const Instance& instance, std::string certificate_path);
    Solution(const Instance& instance, const std::vector<std::vector<ItemIdx>>& agents);
    Solution(const Solution& solution);
    Solution& operator=(const Solution& solution);
    ~Solution() { }
    bool operator==(const Solution& solution);

    /**
     * Getters
     */

    /** Get the instance. */
    inline const Instance& instance() const { return instance_; }

    /** Get the total weight of the solution. */
    inline Weight weight() const { return total_weight_; }
    /** Get the weight of agent 'i'. */
    inline Weight weight(AgentIdx i) const { return agents_[i].weight; }
    /** Get the remaining capacity of agent 'i'. */
    inline Weight remaining_capacity(AgentIdx i) const { return instance().capacity(i) - weight(i); }
    /** Get the total overcapacity of the solution. */
    inline Weight overcapacity() const { return total_overcapacity_; };
    /** Get the overcapacity of agent 'i'. */
    inline Weight overcapacity(AgentIdx i) const { return agents_[i].overcapacity; };
    /** Get the total cost of the solution. */
    inline Cost cost() const { return total_cost_; }
    /** Get the cost of agent 'i'. */
    inline Cost cost(AgentIdx i) const { return agents_[i].cost; }

    inline PCost pcost()           const { return total_pcost_; }
    inline PCost pcost(AgentIdx i) const { return agents_[i].pcost; }

    /** Return 'true' iff all items have been assigned. */
    inline bool full() const { return n_ == instance().number_of_items(); }
    /** Return 'true' iff the solution is feasible. */
    inline bool feasible() const { return full() && (overcapacity() == 0); }
    /** Get the number of items in the solution. */
    inline ItemIdx number_of_items() const { return n_; }
    /**
     * Get the agent to which item 'j' has been assigned.
     *
     * Return -1 if item 'j' has not been assigned to an agent.
     */
    inline AgentIdx agent(ItemIdx j) const { return x_[j]; }

    /*
     * Setters
     */

    void set(ItemIdx j, AgentIdx i);

    void update_penalties(bool inc, PCost delta_inc, PCost delta_dec);
    void update_penalties(const std::vector<PCost>& penalty);
    void update_penalties(PCost delta_inc);

    void clear();

    void write(std::string filepath);
    std::string to_string(AgentIdx i);

private:

    const Instance& instance_;

    std::vector<AgentIdx> x_;
    std::vector<SolutionAgent> agents_;
    ItemIdx n_ = 0;
    Cost total_cost_ = 0;
    Weight total_weight_ = 0;
    Weight total_overcapacity_ = 0;
    PCost total_pcost_ = 0;

};

/**
 * Return the number of items assigned to different machines between sol1 and sol2.
 */
ItemIdx distance(const Solution& sol1, const Solution& sol2);

/**
 * Return true iff sol_best should be updated.
 */
bool compare(const Solution& sol_best, const Solution& sol_curr);

std::ostream& operator<<(std::ostream& os, const Solution& solution);

/*********************************** Output ***********************************/

struct Output
{
    Output(const Instance& instance, Info& info);

    Solution solution;
    Cost lower_bound = 0;
    double time = -1;

    bool optimal() const;
    bool feasible() const { return lower_bound < solution.instance().bound(); }

    std::string upper_bound_string() const;
    std::string lower_bound_string() const;
    std::string         gap_string() const;
    double gap() const;
    void print(Info& info, const std::stringstream& s) const;

    void update_solution(const Solution& solution_new, const std::stringstream& s, Info& info);
    void update_lower_bound(Cost lower_bound_new, const std::stringstream& s, Info& info);

    Output& algorithm_end(Info& info);
};

Cost algorithm_end(Cost lower_bound, Info& info);

}

