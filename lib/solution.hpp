#pragma once

#include "gap/lib/instance.hpp"

namespace gap
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

    Solution(const Instance& instance);
    Solution(const Solution& solution);
    Solution& operator=(const Solution& solution);
    ~Solution() { };
    bool operator==(const Solution& sol);

    /**
     * Getters
     */

    inline const Instance& instance() const { return instance_; }

    inline Weight weight() const { return total_weight_; }
    inline Weight weight(AgentIdx i) const { return agents_[i].weight; }

    inline Weight remaining_capacity(AgentIdx i) const { return instance().capacity(i) - weight(i); }

    inline Weight overcapacity()           const { return total_overcapacity_; };
    inline Weight overcapacity(AgentIdx i) const { return agents_[i].overcapacity; };

    inline Cost cost()           const { return total_cost_; }
    inline Cost cost(AgentIdx i) const { return agents_[i].cost; }

    inline PCost pcost()           const { return total_pcost_; }
    inline PCost pcost(AgentIdx i) const { return agents_[i].pcost; }

    inline bool full() const { return n_ == instance().item_number(); }
    inline bool feasible() const { return full() && (overcapacity() == 0); }

    inline ItemIdx item_number() const { return n_; }
    inline AgentIdx agent(ItemIdx j) const { return x_[j]; }

    /**
     * Setters
     */

    void set(ItemIdx j, AgentIdx i);
    void set(AltIdx k);

    void update_penalties(bool inc, PCost delta_inc, PCost delta_dec);
    void update_penalties(const std::vector<PCost>& penalty);
    void update_penalties(PCost delta_inc);

    void clear();

    void update(const Solution& sol, Cost lb, const std::stringstream& s, Info& info);

    void write_cert(std::string file);
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

void init_display(Info& info);

std::ostream& operator<<(std::ostream& os, const Solution& solution);

struct SolutionCompare
{
    SolutionCompare(int comparator_id): id(comparator_id) {  }
    double value(const Solution& s);
    bool operator()(const Solution& s1, const Solution& s2);
    int id;
};

}
