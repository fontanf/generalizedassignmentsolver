#pragma once

#include "instance.hpp"

class Solution
{

public:

    Solution(const Instance& instance);
    Solution(const Solution& solution);
    Solution& operator=(const Solution& solution);
    ~Solution() { };

    inline const Instance& instance()   const { return instance_; }
    inline Weight weight()              const { return w_tot_; }
    inline Weight weight(AgentIdx i)    const { return w_[i]; }
    inline Weight remaining_capacity(AgentIdx i) const { return instance().capacity(i) - weight(i); }
    inline Profit profit()              const { return p_; }
    inline ItemIdx item_number()        const { return k_; }
    bool feasible() const;

    const std::vector<AgentIdx>& data()  const { return x_; }
    const std::vector<Weight>& weights() const { return w_; }

    bool check_capacity() const;

    void set(const ItemIdx j, AgentIdx i);
    AgentIdx agent(ItemIdx j) const;
    void clear();

    bool update(const Solution& sol);

    void write_cert(std::string file);

    std::string print_bin() const;

private:

    const Instance& instance_;
    ItemIdx k_ = 0;
    Profit  p_ = 0;
    Weight  w_tot_ = 0;
    std::vector<AgentIdx> x_;
    std::vector<Weight> w_;

};

std::ostream& operator<<(std::ostream& os, const Solution& solution);

