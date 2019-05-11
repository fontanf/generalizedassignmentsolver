#pragma once

#include "benchtools/info.hpp"

#include <cstdint>
#include <random>
#include <string>
#include <algorithm>
#include <iostream>
#include <fstream>
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
    AgentIdx i_best = -1; // lowest cost agent
};

class Instance
{

public:

    Instance(AgentIdx m, ItemIdx n=0);

    void add_items(ItemIdx n);
    ItemIdx add_item();
    void set_alternative(ItemIdx j, AgentIdx i, Weight w, Cost p);
    void set_capacity(AgentIdx i, Weight c) { c_[i] = c; }
    void set_optimal_solution(Solution& sol);

    Instance(std::string filename, std::string format);
    ~Instance();

    Instance(const Instance& ins);

    const Item& item(ItemPos j) const { return items_[j]; }
    const Alternative& alternative(AltPos k) const { return alternatives_[k]; }
    AltIdx alternative_index(ItemIdx j, AgentIdx i) const { return items_[j].alt[i]; } 
    const Alternative& alternative(ItemIdx j, AgentIdx i) const { return alternatives_[items_[j].alt[i]]; } 

    ItemIdx item_number()       const { return items_.size(); }
    AgentIdx agent_number()     const { return c_.size(); }
    AltIdx alternative_number() const { return alternatives_.size(); }
    Weight capacity(AgentIdx i) const { return c_[i]; }

    Cost check(std::string filepath);
    const Solution* optimal_solution() const { return sol_opt_.get(); }
    Cost optimum() const;

    void plot(std::string filename);
    void write(std::string filename);

private:

    void read_beasley(std::string filepath);
    void read_standard(std::string filepath);
    void read_standard_solution(std::string filepath);

    std::vector<Item> items_;
    std::vector<Alternative> alternatives_;
    std::vector<Weight> c_;

    std::unique_ptr<Solution> sol_opt_;

};

std::ostream& operator<<(std::ostream &os, const Alternative& alt);
std::ostream& operator<<(std::ostream &os, const Instance& instance);

Solution algorithm_end(const Solution& sol, Info& info);
void algorithm_end(Cost lb, Info& info);

}

