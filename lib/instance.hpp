#pragma once

#include <cstdint>
#include <random>
#include <string>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <chrono>
#include <memory>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/program_options.hpp>

#include "info.hpp"

namespace gap
{

typedef int_fast64_t Profit;
typedef int_fast64_t Weight;
typedef int_fast64_t ItemIdx;
typedef int_fast64_t ItemPos;
typedef int_fast64_t AgentIdx;
typedef int_fast64_t AgentPos;
typedef int_fast64_t AltIdx;
typedef int_fast64_t AltPos;
typedef int_fast64_t StateIdx;
typedef int_fast64_t Label;

class Solution;

struct Alternative
{
    Alternative(AltIdx idx, ItemIdx j, AgentIdx i): idx(idx), j(j), i(i) {  }
    double efficiency() const { return (double)p/(double)w; }
    AltIdx   idx;
    ItemIdx  j;
    AgentIdx i;
    Weight   w;
    Profit   p;
};

struct Item
{
    Item(ItemIdx j): j(j) {  }
    Item(ItemIdx j, Label l): j(j), l(l) {  }
    Item(const Item& item) {
        j   = item.j;
        alt = item.alt;
    }

    ItemIdx j;
    std::vector<AltIdx> alt;
    Label l = -1;
};

class Instance
{

public:

    Instance(ItemIdx n, AgentIdx m, int obj);

    void add_items(ItemIdx n);
    ItemIdx add_item(Label l);
    ItemIdx add_item() { return add_item(-1); }
    void set_alternative(ItemIdx j, AgentIdx i, Weight w, Profit p);
    void set_weight(ItemIdx j, AgentIdx i, Weight w);
    void set_profit(ItemIdx j, AgentIdx i, Profit p);
    void set_capacity(AgentIdx i, Weight c) { c_[i] = c; }

    Instance(boost::filesystem::path filename);
    ~Instance() { };

    int objective() const { return objective_; }
    void toggle_objective();

    const Item& item(ItemPos j) const { return items_[j]; }
    const Alternative& alternative(AltPos k) const { return alternatives_[k]; } 
    const Alternative& alternative(ItemIdx j, AgentIdx i) const { return alternatives_[items_[j].alt[i]]; } 

    ItemPos item_number()       const { return items_.size(); }
    AgentIdx agent_number()     const { return c_.size(); }
    Weight capacity(AgentIdx i) const { return c_[i]; }
    AltIdx alternative_number() const { return alternatives_.size(); }

    const Solution* optimal_solution() const { return sol_opt_; }
    Profit optimum() const;

    Profit t() const { return t_ * item_number(); }

    Profit check(boost::filesystem::path cert_file);

    std::string print_lb(Profit lb) const;
    std::string print_ub(Profit ub) const;
    std::string print_opt(Profit opt) const;

private:

    void read_beasley(boost::filesystem::path filename);
    void read_standard(boost::filesystem::path filename);
    void read_standard_solution(boost::filesystem::path filename);

    std::string name_;
    std::string format_;

    std::vector<Item> items_;
    std::vector<Alternative> alternatives_;
    std::vector<Weight> c_;

    int objective_ = -1;
    Profit t_ = 0;
    Solution* sol_opt_ = NULL; // Optimal solution

};

std::ostream& operator<<(std::ostream &os, const Alternative& alt);
std::ostream& operator<<(std::ostream &os, const Instance& instance);

}
