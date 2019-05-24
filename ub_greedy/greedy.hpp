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

std::unique_ptr<Desirability> desirability(std::string str, const Instance& ins);

/******************************************************************************/

void sol_greedy(Solution& sol, const Desirability& f);
Solution sol_greedy(const Instance& ins, const Desirability& f, Info info = Info());

void sol_mthg(Solution& sol, const Desirability& f);
Solution sol_mthg(const Instance& ins, const Desirability& f, Info info = Info());

}

