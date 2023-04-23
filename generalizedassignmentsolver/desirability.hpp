#pragma once

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

class Desirability
{

public:

    virtual double operator()(
            ItemIdx item_id,
            AgentIdx agent_id) const = 0;

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

    DesirabilityCost(const Instance& instance): instance_(instance) { }

    double operator()(
            ItemIdx item_id,
            AgentIdx agent_id) const
    {
        return instance_.cost(item_id, agent_id);
    }

    std::string to_string() const { return "cij"; }

private:

    const Instance& instance_;

};

/**
 * fij = wij
 */
class DesirabilityWeight: public Desirability
{

public:

    DesirabilityWeight(const Instance& instance): instance_(instance) { }

    double operator()(
            ItemIdx item_id,
            AgentIdx agent_id) const
    {
        return instance_.weight(item_id, agent_id);
    }

    std::string to_string() const { return "wij"; }

private:

    const Instance& instance_;

};

/**
 * fij = cij * wij
 */
class DesirabilityCostWeight: public Desirability
{

public:

    DesirabilityCostWeight(const Instance& instance): instance_(instance) { }

    double operator()(
            ItemIdx item_id,
            AgentIdx agent_id) const
    {
        return instance_.cost(item_id, agent_id) * instance_.cost(item_id, agent_id);
    }

    std::string to_string() const { return "cij*wij"; }

private:

    const Instance& instance_;

};

/**
 * fij = (cij - cjmax) / wij
 * This function comes from the maximization formulation of GAP.
 */
class DesirabilityEfficiency: public Desirability
{

public:

    DesirabilityEfficiency(const Instance& instance): instance_(instance) { }

    double operator()(
            ItemIdx item_id,
            AgentIdx agent_id) const
    {
        return -(double)instance_.profit(item_id, agent_id)
            / instance_.weight(item_id, agent_id);
    }

    std::string to_string() const { return "-pij/wij"; }

private:

    const Instance& instance_;

};

/**
 * fij = wij / ti
 */
class DesirabilityWeightCapacity: public Desirability
{

public:

    DesirabilityWeightCapacity(const Instance& instance): instance_(instance) { }

    double operator()(
            ItemIdx item_id,
            AgentIdx agent_id) const
    {
        return (double)instance_.weight(item_id, agent_id)
            / instance_.capacity(agent_id);
    }

    std::string to_string() const { return "wij/ti"; }

private:

    const Instance& instance_;

};

/**
 * fij = cij - vj
 */
class DesirabilityRcost1: public Desirability
{
public:

    DesirabilityRcost1(
            const Instance& instance,
            const std::vector<double>& v):
        instance_(instance),
        v_(v) { }

    DesirabilityRcost1(
            const Instance& instance,
            const double* v):
        instance_(instance),
        v_(instance.number_of_items())
    {
        for (ItemIdx item_id = 0;
                item_id < instance_.number_of_items();
                ++item_id) {
            v_[item_id] = v[item_id];
        }
    }

    double operator()(
            ItemIdx item_id,
            AgentIdx agent_id) const
    {
        return instance_.cost(item_id, agent_id) - v_[item_id];
    }

    std::string to_string() const { return "cij-vj"; }

private:

    const Instance& instance_;

    std::vector<double> v_;

};

/**
 * fij = cij - ui wij
 */
class DesirabilityRcost2: public Desirability
{

public:

    DesirabilityRcost2(
            const Instance& instance,
            const std::vector<double>& u):
        instance_(instance),
        u_(u) { }

    DesirabilityRcost2(
            const Instance& instance,
            const double* u):
        instance_(instance),
        u_(instance.number_of_agents())
    {
        for (AgentIdx agent_id = 0;
                agent_id < instance.number_of_agents();
                ++agent_id) {
            u_[agent_id] = u[agent_id];
        }
    }

    double operator()(
            ItemIdx item_id,
            AgentIdx agent_id) const
    {
        return instance_.cost(item_id, agent_id)
            - u_[agent_id] * instance_.weight(item_id, agent_id);
    }

    std::string to_string() const { return "cij-uj*wij"; }

private:

    const Instance& instance_;

    std::vector<double> u_;

};

std::unique_ptr<Desirability> desirability(
        std::string str,
        const Instance& instance);

}

