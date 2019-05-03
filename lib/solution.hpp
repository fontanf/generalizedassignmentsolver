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

    inline const Instance& instance() const { return instance_; }
    inline Weight weight() const { return w_tot_; }
    inline Weight weight(AgentIdx i) const { return w_[i]; }
    inline Weight remaining_capacity(AgentIdx i) const { return instance().capacity(i) - weight(i); }
    inline Value value() const { return v_; }
    inline double value(double alpha) const { return v_ + alpha * wf_; }
    inline ItemIdx item_number() const { return n_; }
    inline bool full() const { return n_ == instance().item_number(); }
    inline bool feasible() const { return full() && (overcapacity() == 0); }
    Weight overcapacity() const { return wf_; };

    const std::vector<AgentIdx>& data()  const { return x_; }
    const std::vector<Weight>& weights() const { return w_; }

    bool check_capacity() const;

    void set(ItemIdx j, AgentIdx i);
    void set(AltIdx k);
    AgentIdx agent(ItemIdx j) const;
    void clear();

    void update(const Solution& sol, Value lb, const std::stringstream& s, Info& info);
    void write_cert(std::string file);
    std::string to_string(AgentIdx i);

private:

    const Instance& instance_;
    ItemIdx n_ = 0;
    Value v_ = 0;
    Weight w_tot_ = 0;
    Weight wf_ = 0;
    std::vector<AgentIdx> x_;
    std::vector<Weight> w_;

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
