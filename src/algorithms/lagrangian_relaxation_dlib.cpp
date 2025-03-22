#include "generalizedassignmentsolver/algorithms/lagrangian_relaxation_dlib.hpp"

#include "generalizedassignmentsolver/algorithm_formatter.hpp"

#include "knapsacksolver/algorithms/dynamic_programming_primal_dual.hpp"
#include "knapsacksolver/algorithms/dynamic_programming_bellman.hpp"

#include <dlib/optimization.h>

#include <algorithm>
#include <iomanip>
#include <limits>

using namespace generalizedassignmentsolver;
using namespace dlib;

typedef matrix<double,0,1> column_vector;

////////////////////////////////////////////////////////////////////////////////
////////////////////// lagrangian_relaxation_assignment_dlib ///////////////////
////////////////////////////////////////////////////////////////////////////////

class LagrangianRelaxationAssignmentDlibFunction
{

public:

    LagrangianRelaxationAssignmentDlibFunction(
            const Instance& instance,
            LagrangianRelaxationAssignmentDlibParameters& p,
            ItemIdx number_of_unfixed_items,
            const std::vector<ItemIdx>& item_indices):
        instance_(instance), p_(p), item_indices_(item_indices), grad_(number_of_unfixed_items)
    {
        // Compute knapsack capacities
        kp_capacities_.resize(instance_.number_of_agents());
        for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id) {
            kp_capacities_[agent_id] = instance.capacity(agent_id);
            for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id) {
                if (p.fixed_alt != NULL && (*p.fixed_alt)[item_id][agent_id] == 1)
                    kp_capacities_[agent_id] -= instance.weight(item_id, agent_id);
            }
            if (kp_capacities_[agent_id] < 0)
                std::cout << "ERROR agent_id " << agent_id << " c " << kp_capacities_[agent_id] << std::endl;
        }

        // Initialize kp_indices_
        kp_indices_.resize(instance.number_of_items());
    }

    virtual ~LagrangianRelaxationAssignmentDlibFunction() { }

    double f(const column_vector& x);

    const column_vector der(const column_vector& x) const { (void)x; return grad_; }

private:

    const Instance& instance_;
    LagrangianRelaxationAssignmentDlibParameters& p_;
    /** item_indices_[item_id] is the index of item j in mu and grad_. */
    const std::vector<ItemIdx>& item_indices_;

    column_vector grad_;

    std::vector<knapsacksolver::Weight> kp_capacities_;
    /** kp_indices_[item_id] is the index of item j in the current KP. */
    std::vector<knapsacksolver::ItemIdx> kp_indices_;

};

double LagrangianRelaxationAssignmentDlibFunction::f(const column_vector& mu)
{
    // Initialize bound and gradient;
    double l = 0;
    for (ItemIdx item_id = 0; item_id < instance_.number_of_items(); ++item_id)
        if (item_indices_[item_id] >= 0)
            l += mu(item_indices_[item_id]);
    std::fill(grad_.begin(), grad_.end(), 1);

    Weight mult = 10000;
    for (AgentIdx agent_id = 0; agent_id < instance_.number_of_agents(); ++agent_id) {
        // Create knapsack instance
        knapsacksolver::Instance kp_instance;
        kp_instance.set_capacity(kp_capacities_[agent_id]);
        knapsacksolver::ItemIdx kp_item_id = 0;
        for (ItemIdx item_id = 0; item_id < instance_.number_of_items(); ++item_id) {
            if ((p_.fixed_alt != NULL && (*p_.fixed_alt)[item_id][agent_id] >= 0)
                    || instance_.weight(item_id, agent_id) > kp_capacities_[agent_id]) {
                kp_indices_[item_id] = -1;
                continue;
            }
            knapsacksolver::Profit profit = std::ceil(mult * mu(item_id) - mult * instance_.cost(item_id, agent_id));
            if (profit <= 0) {
                kp_indices_[item_id] = -1;
                continue;
            }
            kp_instance.add_item(instance_.weight(item_id, agent_id), profit);
            kp_indices_[item_id] = kp_item_id;
            kp_item_id++;
        }

        // Solve knapsack instance
        //auto kp_output = knapsacksolver::dynamic_programming_bellman_array_all(kp_instance, kp_parameters);
        auto kp_output = knapsacksolver::dynamic_programming_primal_dual(kp_instance);
        //std::cout << "i " << i << " opt " << kp_output.solution.profit() << std::endl;

        // Update bound and gradient
        for (ItemIdx item_id = 0; item_id < instance_.number_of_items(); ++item_id) {
            if (kp_indices_[item_id] >= 0 && kp_output.solution.contains_idx(kp_indices_[item_id])) {
                grad_(item_indices_[item_id])--;
                l += instance_.cost(item_id, agent_id) - mu(item_indices_[item_id]);
            }
        }
    }

    return l;
}

const LagrangianRelaxationAssignmentDlibOutput generalizedassignmentsolver::lagrangian_relaxation_assignment_dlib(
        const Instance& instance,
        const LagrangianRelaxationAssignmentDlibParameters& parameters)
{
    LagrangianRelaxationAssignmentDlibOutput o output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Lagrangian relaxation - assignment constraints (dlib)");
    algorithm_formatter.print_header();

    // Compute c0, item_indices and number_of_unfixed_items
    ItemIdx item_idx = 0;
    Cost c0 = 0;
    std::vector<ItemIdx> item_indices(instance.number_of_items(), -2);
    for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id) {
        for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id) {
            if (parameters.fixed_alt != NULL && (*parameters.fixed_alt)[item_id][agent_id] == 1) {
                c0 += instance.cost(item_id, agent_id);
                item_indices[item_id] = -1;
                break;
            }
        }
        if (item_indices[item_id] == -2) {
            item_indices[item_id] = item_idx;
            item_idx++;
        }
    }
    ItemIdx number_of_unfixed_items = item_idx;

    // Initialize multipliers
    column_vector mu(number_of_unfixed_items);
    if (parameters.initial_multipliers != NULL) {
        for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
            if (item_indices[item_id] >= 0)
                mu(item_indices[item_id]) = (*parameters.initial_multipliers)[item_id];
    } else {
        for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
            mu(item_id) = 0;
    }

    // Solve
    LagrangianRelaxationAssignmentDlibFunction func(instance, parameters, number_of_unfixed_items, item_indices);
    auto f   = [&func](const column_vector& x) { return func.f(x); };
    auto def = [&func](const column_vector& x) { return func.der(x); };
    auto stop_strategy = objective_delta_stop_strategy(0.0001);
    //auto stop_strategy = gradient_norm_stop_strategy().be_verbosity_level(),
    double res = find_max(
            dlib_search_strategy(256),
            stop_strategy,
            f,
            def,
            mu,
            std::numeric_limits<double>::max());

    // Compute output parameters
    Cost lb = c0 + std::ceil(res - FFOT_TOL);
    algorithm_formatter.update_bound(lb, "");
    output.multipliers.resize(instance.number_of_items());
    for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
        if (item_indices[item_id] >= 0)
            output.multipliers[item_id] = mu(item_indices[item_id]);

    algorithm_formatter.end();
    return output;
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////// lagrangian_relaxation_knapsack_dlib ////////////////////
////////////////////////////////////////////////////////////////////////////////

class LagrangianRelaxationKnapsackDlibFunction
{

public:

    LagrangianRelaxationKnapsackDlibFunction(const Instance& instance):
        instance_(instance),
        x_(instance.number_of_items()),
        grad_(instance.number_of_agents())
    {  }
    virtual ~LagrangianRelaxationKnapsackDlibFunction() { };

    double f(const column_vector& x);

    const column_vector der(const column_vector& x) const { (void)x; return grad_; }

    AgentIdx agent(ItemIdx item_id) const { return x_(item_id); }

private:

    const Instance& instance_;
    column_vector x_;
    column_vector grad_;

};

double LagrangianRelaxationKnapsackDlibFunction::f(const column_vector& mu)
{
    // Initialize bound and gradient
    double l = 0;
    for (AgentIdx agent_id = 0; agent_id < instance_.number_of_agents(); ++agent_id) {
        l += mu(agent_id) * instance_.capacity(agent_id);
        grad_(agent_id) = instance_.capacity(agent_id);
    }

    for (ItemIdx item_id = 0; item_id < instance_.number_of_items(); ++item_id) {
        // Solve the trivial Generalized Upper Bound Problem
        AgentIdx agent_id_best = -1;
        double rc_best = -1;
        for (AgentIdx agent_id = 0; agent_id < instance_.number_of_agents(); ++agent_id) {
            double rc = instance_.cost(item_id, agent_id) - mu(agent_id) * instance_.weight(item_id, agent_id);
            if (agent_id_best == -1
                    || rc_best > rc
                    // If the minimum reduced cost of a job is reached for
                    // several agents, schedule the job on the agent with the
                    // most available remaining capacity.
                    // Without this condition, the relaxation fails to get the
                    // optimal bound (the one from the linear relaxation) for
                    // some instances.
                    || (rc_best == rc && grad_(agent_id) > grad_(agent_id_best))) {
                agent_id_best = agent_id;
                rc_best = rc;
            }
        }

        // Update bound and gradient
        grad_(agent_id_best) -= instance_.weight(item_id, agent_id_best);
        x_(item_id) = agent_id_best;
        l += rc_best;
    }

    return l;
}

const LagrangianRelaxationKnapsackDlibOutput generalizedassignmentsolver::lagrangian_relaxation_knapsack_dlib(
        const Instance& instance,
        const Parameters& parameters)
{
    LagrangianRelaxationKnapsackDlibOutput output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Lagrangian relaxation - knapsack constraints (dlib)");
    algorithm_formatter.print_header();

    // Initialize multipliers
    column_vector mu(instance.number_of_agents());
    column_vector mu_lower(instance.number_of_agents());
    column_vector mu_upper(instance.number_of_agents());
    for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id) {
        //mu_lower(agent_id) = 0;
        //mu_upper(agent_id) = std::numeric_limits<double>::max();
        mu(agent_id) = 0;
        mu_lower(agent_id) = -std::numeric_limits<double>::max();
        mu_upper(agent_id) = 0;
    }

    // Solve
    LagrangianRelaxationKnapsackDlibFunction func(instance);
    auto f   = [&func](const column_vector& x) { return func.f(x); };
    auto def = [&func](const column_vector& x) { return func.der(x); };
    auto stop_strategy = objective_delta_stop_strategy();
    //auto stop_strategy = gradient_norm_stop_strategy();
    double res = find_max_box_constrained(
            dlib_search_strategy(256),
            stop_strategy,
            f,
            def,
            mu,
            mu_lower,
            mu_upper);

    // Compute output parameters
    Cost lb = std::ceil(res - FFOT_TOL);
    algorithm_formatter.update_bound(lb, "");
    output.multipliers.resize(instance.number_of_agents());
    for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id)
        output.multipliers[agent_id] = mu(agent_id);
    func.f(mu);
    for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id) {
        output.x.push_back(std::vector<double>(instance.number_of_agents(), 0));
        output.x[item_id][func.agent(item_id)] = 1;
    }

    algorithm_formatter.end();
    return output;
}
