#pragma once

#include "gap/lib/instance.hpp"

namespace gap
{

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

    inline Weight weight() const { return w_tot_; }
    inline Weight weight(AgentIdx i) const { return w_[i]; }

    inline Weight remaining_capacity(AgentIdx i) const { return instance().capacity(i) - weight(i); }

    inline Value value() const { return v_tot_; }
    inline Value value(AgentIdx i) const { return v_[i]; }
    inline double value(double alpha) const { return v_tot_ + alpha * wf_tot_; }
    inline double value(AgentIdx i, double alpha) const { return v_[i] + alpha * wf_[i]; }
    double value(const std::vector<double>& alpha) const;

    inline bool full() const { return n_ == instance().item_number(); }
    inline bool feasible() const { return full() && (overcapacity() == 0); }

    Weight overcapacity() const { return wf_tot_; };
    Weight overcapacity(AgentIdx i) const { return wf_[i]; };

    inline ItemIdx item_number() const { return n_; }
    inline AgentIdx agent(ItemIdx j) const { return x_[j]; }

    /**
     * Setters
     */

    void set(ItemIdx j, AgentIdx i);
    void set(AltIdx k);

    void clear();

    void update(const Solution& sol, Value lb, const std::stringstream& s, Info& info);

    void write_cert(std::string file);
    std::string to_string(AgentIdx i);

private:

    const Instance& instance_;

    std::vector<AgentIdx> x_; // agent of each item (-1 if none)
    ItemIdx n_ = 0; // total assigned item number

    std::vector<Value> v_; // value of each agent
    Value v_tot_ = 0; // total value

    std::vector<Weight> w_; // weight of each agent
    Weight w_tot_ = 0; // total weight

    std::vector<Weight> wf_; // overcapacity of each agent
    Weight wf_tot_ = 0; // total overcapacity
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
