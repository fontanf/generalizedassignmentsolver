#pragma once

#include "optimizationtools/info.hpp"

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

using optimizationtools::Info;

typedef int64_t Cost;
typedef double  PCost;
typedef int64_t Weight;
typedef int64_t ItemIdx;
typedef int64_t ItemPos;
typedef int64_t AgentIdx;
typedef int64_t AgentPos;
typedef int64_t StateIdx;
typedef int64_t Counter;
typedef int64_t Seed;

class Solution;

struct Alternative
{
    ItemIdx j;
    AgentIdx i;
    Weight w;
    Cost c;

    double efficiency() const { return c * w; }
};

struct Item
{
    ItemIdx j;
    std::vector<Alternative> alternatives;

    Weight w; // total weight
    Cost c; // total cost

    Cost c_min = -1; // minimum cost
    Cost c_max = -1; // maximum cost
    AgentIdx i_cmin = -1; // min cost agent
    AgentIdx i_cmax = -1; // max cost agent

    Weight w_min = -1; // minimum weight
    Weight w_max = -1; // maximum weight
    AgentIdx i_wmin = -1; // min weight agent
    AgentIdx i_wmax = -1; // max weight agent
};

class Instance
{

public:

    /**
     * Constructors and destructor
     */

    /** Create instance from file. */
    Instance(std::string filename, std::string format = "orlibrary");

    /** Manual constructor. */
    Instance(AgentIdx m);
    void set_name(std::string name) { name_ = name; }
    void set_capacity(AgentIdx i, Weight t) { t_[i] = t; }
    inline void add_item();
    inline void set_alternative(ItemIdx j, AgentIdx i, Weight w, Cost p);
    void clear();

    /** Constructor for test instances. */
    void add_item(const std::vector<std::pair<Weight, Cost>>& a);
    void set_capacities(const std::vector<Weight>& t);
    void set_optimal_solution(Solution& solution);

    /** Copy constructor. */
    Instance(const Instance& instance);
    /** Copy assignment operator. */
    Instance& operator=(const Instance& instance);
    /** Destructor. */
    ~Instance();

    /**
     * Getters
     */

    std::string name() const { return name_; }
    const Item& item(ItemPos j) const { return items_[j]; }

    inline Cost cost_max()   const { return c_max_; }
    inline Cost weight_max() const { return w_max_; }

    ItemIdx item_number()       const { return items_.size(); }
    AgentIdx agent_number()     const { return t_.size(); }
    Weight capacity(AgentIdx i) const { return t_[i]; }

    inline Weight weight(ItemIdx j, AgentIdx i) const { return items_[j].alternatives[i].w; }
    inline Cost cost(ItemIdx j, AgentIdx i) const { return items_[j].alternatives[i].c; }
    inline Cost profit(ItemIdx j, AgentIdx i) const { return items_[j].c_max - items_[j].alternatives[i].c; }

    const Solution* optimal_solution() const { return sol_opt_.get(); }
    Cost optimum() const;
    Cost bound() const { return c_tot_ + 1; }
    Cost combinatorial_relaxation() const { return c_min_sum_; }

    void write(std::string filename);

private:

    void read_orlibrary(std::ifstream& file);
    void read_standard(std::ifstream& file);

    std::string name_;
    std::vector<Item> items_;
    std::vector<Weight> t_;
    Cost c_max_ = -1;
    Cost c_tot_ = 0;
    Cost w_max_ = -1;
    Cost c_min_sum_ = 0;

    std::unique_ptr<Solution> sol_opt_;

};

void Instance::add_item()
{
    ItemIdx j = items_.size();
    items_.push_back({});
    items_[j].j = j;
    items_[j].alternatives.resize(agent_number());
    for (AgentIdx i = 0; i < agent_number(); ++i) {
        items_[j].alternatives[i].j = j;
        items_[j].alternatives[i].i = i;
    }
}

void Instance::set_alternative(ItemIdx j, AgentIdx i, Weight w, Cost v)
{
    items_[j].alternatives[i].w = w;
    items_[j].alternatives[i].c = v;
    items_[j].w += w;
    items_[j].c += v;
    if (items_[j].i_cmin != -1 && items_[j].c_min > v)
        c_min_sum_ -= items_[j].c_min;
    if (items_[j].i_cmin == -1 || items_[j].c_min > v) {
        items_[j].i_cmin = i;
        items_[j].c_min = v;
        c_min_sum_ += v;
    }
    if (items_[j].i_wmin == -1 || items_[j].w_min > w) {
        items_[j].i_wmin = i;
        items_[j].w_min = w;
    }
    if (items_[j].c_max < v) {
        items_[j].i_cmax = i;
        items_[j].c_max = v;
    }
    if (items_[j].w_max < w) {
        items_[j].i_wmax = i;
        items_[j].w_max = w;
    }
    if (c_max_ < v)
        c_max_ = v;
    if (w_max_ < w)
        w_max_ = w;
    c_tot_ += v;
}

std::ostream& operator<<(std::ostream &os, const Alternative& alternative);
std::ostream& operator<<(std::ostream &os, const Instance& instance);

}

