#pragma once

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

class Desirability
{
public:
    virtual double operator()(ItemIdx j, AgentIdx i) const = 0;
    virtual std::string to_string() const = 0;
};

/**
 * Examples of desirability functions.
 */

/**
 * fij = cij
 */
class DesirabilityCost: public Desirability
{
public:
    DesirabilityCost(const Instance& ins): instance(ins) { }
    double operator()(ItemIdx j, AgentIdx i) const
    {
        return instance.cost(j, i);
    }
    std::string to_string() const { return "cij"; }
private:
    const Instance& instance;
};

/**
 * fij = wij
 */
class DesirabilityWeight: public Desirability
{
public:
    DesirabilityWeight(const Instance& ins): instance(ins) { }
    double operator()(ItemIdx j, AgentIdx i) const
    {
        return instance.weight(j, i);
    }
    std::string to_string() const { return "wij"; }
private:
    const Instance& instance;
};

/**
 * fij = cij * wij
 */
class DesirabilityCostWeight: public Desirability
{
public:
    DesirabilityCostWeight(const Instance& ins): instance(ins) { }
    double operator()(ItemIdx j, AgentIdx i) const
    {
        return instance.cost(j, i) * instance.cost(j, i);
    }
    std::string to_string() const { return "cij*wij"; }
private:
    const Instance& instance;
};

/**
 * fij = (cij - cjmax) / wij
 * This function comes from the maximization formulation of GAP.
 */
class DesirabilityEfficiency: public Desirability
{
public:
    DesirabilityEfficiency(const Instance& ins): instance(ins) { }
    double operator()(ItemIdx j, AgentIdx i) const
    {
        return (double)(instance.cost(j, i) - instance.item(j).c_max) / instance.weight(j, i);
    }
    std::string to_string() const { return "-pij/wij"; }
private:
    const Instance& instance;
};

/**
 * fij = wij / ti
 */
class DesirabilityWeightCapacity: public Desirability
{
public:
    DesirabilityWeightCapacity(const Instance& ins): instance(ins) { }
    double operator()(ItemIdx j, AgentIdx i) const
    {
        return (double)instance.weight(j, i) / instance.capacity(i);
    }
    std::string to_string() const { return "wij/ti"; }
private:
    const Instance& instance;
};

/**
 * fij = cij - vj
 */
class DesirabilityRcost1: public Desirability
{
public:
    DesirabilityRcost1(const Instance& ins, const std::vector<double>& v):
        instance(ins), v_(v) { }
    DesirabilityRcost1(const Instance& ins, const double* v):
        instance(ins), v_(ins.item_number())
    {
        for (ItemIdx j=0; j<ins.item_number(); ++j)
            v_[j] = v[j];
    }
    double operator()(ItemIdx j, AgentIdx i) const
    {
        return instance.cost(j, i) - v_[j];
    }
    std::string to_string() const { return "cij-vj"; }
private:
    const Instance& instance;
    std::vector<double> v_;
};

/**
 * fij = cij - ui wij
 */
class DesirabilityRcost2: public Desirability
{
public:
    DesirabilityRcost2(const Instance& instance, const std::vector<double>& u):
        instance(instance), u_(u) { }
    DesirabilityRcost2(const Instance& instance, const double* u):
        instance(instance), u_(instance.agent_number())
    {
        for (AgentIdx i = 0; i < instance.agent_number(); ++i)
            u_[i] = u[i];
    }
    double operator()(ItemIdx j, AgentIdx i) const
    {
        return instance.cost(j, i) - u_[i] * instance.weight(j, i);
    }
    std::string to_string() const { return "cij-uj*wij"; }
private:
    const Instance& instance;
    std::vector<double> u_;
};

std::unique_ptr<Desirability> desirability(std::string str, const Instance& ins);

}

