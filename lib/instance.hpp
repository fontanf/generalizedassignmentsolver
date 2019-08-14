#pragma once

#include "benchtools/info.hpp"
#include "benchtools/tools.hpp"

#include <cstdint>
#include <random>
#include <string>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <memory>
#include <map>

namespace gap
{

typedef int64_t Cost;
typedef double  PCost;
typedef int64_t Weight;
typedef int64_t ItemIdx;
typedef int64_t ItemPos;
typedef int64_t AgentIdx;
typedef int64_t AgentPos;
typedef int64_t AltIdx;
typedef int64_t AltPos;
typedef int64_t StateIdx;
typedef int64_t Cpt;
typedef int64_t Seed;

class Solution;

struct Alternative
{
    AltIdx k;
    ItemIdx j;
    AgentIdx i;
    Weight w;
    Cost c;

    double efficiency() const { return c * w; }
};

struct Item
{
    ItemIdx j;
    std::vector<AltIdx> alt;
    Weight w; // total weight
    Cost c; // total cost
    Cost c_min = -1; // minimum cost
    Cost c_max = -1; // maximum cost
    AgentIdx i_best = -1; // lowest cost agent
};

class Instance
{

public:

    /**
     * Constructors and destructor
     */

    Instance(std::string filename, std::string format = "gap_beasley");

    Instance(AgentIdx m, ItemIdx n=0);
    void set_name(std::string name) { name_ = name; }
    void set_capacity(AgentIdx i, Weight t) { t_[i] = t; }
    void set_capacity(const std::vector<Weight>& t);
    ItemIdx add_item();
    void set_alternative(ItemIdx j, AgentIdx i, Weight w, Cost p);
    void add_item(const std::vector<std::pair<Weight, Cost>>& a);
    void set_optimal_solution(Solution& sol);

    Instance(const Instance& ins);
    Instance& operator=(const Instance& ins);

    ~Instance();

    /**
     * Getters
     */

    std::string name() const { return name_; }
    const Item& item(ItemPos j) const { return items_[j]; }
    AltIdx alternative_index(ItemIdx j, AgentIdx i) const { return items_[j].alt[i]; } 
    const Alternative& alternative(AltPos k) const { return alternatives_[k]; }
    const Alternative& alternative(ItemIdx j, AgentIdx i) const { return alternatives_[items_[j].alt[i]]; } 

    inline Cost cost_max() const { return c_max_; }
    inline Cost profit(const Alternative& a)  const { return c_max_ - a.c; }
    inline Cost profit(AltIdx k)              const { return profit(alternative(k)); }
    inline Cost profit(ItemIdx j, AgentIdx i) const { return profit(alternative_index(i, j)); }

    ItemIdx item_number()       const { return items_.size(); }
    AgentIdx agent_number()     const { return t_.size(); }
    AltIdx alternative_number() const { return alternatives_.size(); }
    Weight capacity(AgentIdx i) const { return t_[i]; }

    Cost check(std::string filepath);
    const Solution* optimal_solution() const { return sol_opt_.get(); }
    Cost optimum() const;
    Cost bound() const { return c_tot_ + 1; }

    void plot(std::string filename);
    void write(std::string filename);

private:

    void read_beasley(std::ifstream& file);
    void read_standard(std::ifstream& file);

    std::string name_;
    std::vector<Item> items_;
    std::vector<Alternative> alternatives_;
    std::vector<Weight> t_;
    Cost c_max_ = -1;
    Cost c_tot_ = 0;

    std::unique_ptr<Solution> sol_opt_;

};

std::ostream& operator<<(std::ostream &os, const Alternative& alt);
std::ostream& operator<<(std::ostream &os, const Instance& instance);

}

