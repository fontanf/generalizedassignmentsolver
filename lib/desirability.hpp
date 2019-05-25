#pragma once

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

class Desirability
{
public:
    Desirability() {  }
    virtual double operator()(ItemIdx j, AgentIdx i) const = 0;
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
    DesirabilityCost(const Instance& ins): ins_(ins) { }
    double operator()(ItemIdx j, AgentIdx i) const
    {
        return ins_.alternative(j, i).c;
    }
private:
    const Instance& ins_;
};

/**
 * fij = wij
 */
class DesirabilityWeight: public Desirability
{
public:
    DesirabilityWeight(const Instance& ins): ins_(ins) { }
    double operator()(ItemIdx j, AgentIdx i) const
    {
        return ins_.alternative(j, i).w;
    }
private:
    const Instance& ins_;
};

/**
 * fij = cij * wij
 */
class DesirabilityCostWeight: public Desirability
{
public:
    DesirabilityCostWeight(const Instance& ins): ins_(ins) { }
    double operator()(ItemIdx j, AgentIdx i) const
    {
        const Alternative& a = ins_.alternative(j, i);
        return a.c * a.w;
    }
private:
    const Instance& ins_;
};

/**
 * fij = (cij - cjmax) / wij
 * This function comes from the maximization formulation of GAP.
 */
class DesirabilityEfficiency: public Desirability
{
public:
    DesirabilityEfficiency(const Instance& ins): ins_(ins) { }
    double operator()(ItemIdx j, AgentIdx i) const
    {
        const Alternative& a = ins_.alternative(j, i);
        return (double)(a.c - ins_.item(j).c_max) / a.w;
    }
private:
    const Instance& ins_;
};

/**
 * fij = wij / ti
 */
class DesirabilityWeightCapacity: public Desirability
{
public:
    DesirabilityWeightCapacity(const Instance& ins): ins_(ins) { }
    double operator()(ItemIdx j, AgentIdx i) const
    {
        return (double)ins_.alternative(j, i).w / ins_.capacity(i);
    }
private:
    const Instance& ins_;
};

/**
 * fij = cij - vj
 */
class DesirabilityRcost1: public Desirability
{
public:
    DesirabilityRcost1(const Instance& ins, const std::vector<double>& v):
        ins_(ins), v_(v) { }
    DesirabilityRcost1(const Instance& ins, const double* v):
        ins_(ins), v_(ins.item_number())
    {
        for (ItemIdx j=0; j<ins.item_number(); ++j)
            v_[j] = v[j];
    }
    double operator()(ItemIdx j, AgentIdx i) const
    {
        return ins_.alternative(j, i).c - v_[j];
    }
private:
    const Instance& ins_;
    std::vector<double> v_;
};

/**
 * fij = cij - ui wij
 */
class DesirabilityRcost2: public Desirability
{
public:
    DesirabilityRcost2(const Instance& ins, const std::vector<double>& u):
        ins_(ins), u_(u) { }
    DesirabilityRcost2(const Instance& ins, const double* u):
        ins_(ins), u_(ins.agent_number())
    {
        for (AgentIdx i=0; i<ins.agent_number(); ++i)
            u_[i] = u[i];
    }
    double operator()(ItemIdx j, AgentIdx i) const
    {
        const Alternative& a = ins_.alternative(j, i);
        return a.c - u_[i] * a.w;
    }
private:
    const Instance& ins_;
    std::vector<double> u_;
};

std::unique_ptr<Desirability> desirability(std::string str, const Instance& ins);

}

