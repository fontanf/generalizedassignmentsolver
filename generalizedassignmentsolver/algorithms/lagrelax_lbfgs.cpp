#include "generalizedassignmentsolver/algorithms/lagrelax_lbfgs.hpp"

#include "knapsacksolver/algorithms/minknap.hpp"
#include "knapsacksolver/algorithms/bellman.hpp"

#include <dlib/optimization.h>

#include <algorithm>
#include <iomanip>
#include <limits>

using namespace generalizedassignmentsolver;
using namespace dlib;

typedef matrix<double,0,1> column_vector;

/************************** lagrelax_assignment_lbfgs *************************/

LagRelaxAssignmentLbfgsOutput& LagRelaxAssignmentLbfgsOutput::algorithm_end(Info& info)
{
    //PUT(info, "Algorithm", "Iterations", it);
    Output::algorithm_end(info);
    //VER(info, "Iterations: " << it << std::endl);
    return *this;
}

class LagRelaxAssignmentLbfgsFunction
{

public:

    LagRelaxAssignmentLbfgsFunction(
            const Instance& instance,
            LagRelaxAssignmentLbfgsOptionalParameters& p,
            ItemIdx unfixed_item_number,
            const std::vector<ItemIdx>& item_indices):
        instance_(instance), p_(p), item_indices_(item_indices), grad_(unfixed_item_number)
    {
        ItemIdx n = instance_.item_number();
        AgentIdx m = instance_.agent_number();

        // Compute knapsack capacities
        kp_capacities_.resize(m);
        for (AgentIdx i = 0; i < m; ++i) {
            kp_capacities_[i] = instance.capacity(i);
            for (ItemIdx j = 0; j < n; ++j) {
                if (p.fixed_alt != NULL && (*p.fixed_alt)[j][i] == 1)
                    kp_capacities_[i] -= instance.weight(j, i);
            }
            if (kp_capacities_[i] < 0)
                std::cout << "ERROR i " << i << " c " << kp_capacities_[i] << std::endl;
        }

        // Initialize kp_indices_
        kp_indices_.resize(n);
    }

    virtual ~LagRelaxAssignmentLbfgsFunction() { }

    double f(const column_vector& x);

    const column_vector der(const column_vector& x) const { (void)x; return grad_; }

private:

    const Instance& instance_;
    LagRelaxAssignmentLbfgsOptionalParameters& p_;
    /** item_indices_[j] is the index of item j in mu and grad_. */
    const std::vector<ItemIdx>& item_indices_;

    column_vector grad_;

    std::vector<knapsacksolver::Weight> kp_capacities_;
    /** kp_indices_[j] is the index of item j in the current KP. */
    std::vector<knapsacksolver::ItemIdx> kp_indices_;

};

double LagRelaxAssignmentLbfgsFunction::f(const column_vector& mu)
{
    ItemIdx n = instance_.item_number();
    AgentIdx m = instance_.agent_number();

    // Initialize bound and gradient;
    double l = 0;
    for (ItemIdx j = 0; j < n; ++j)
        if (item_indices_[j] >= 0)
            l += mu(item_indices_[j]);
    std::fill(grad_.begin(), grad_.end(), 1);

    Weight mult = 1;
    for (AgentIdx i = 0; i < m; ++i) {
        // Create knapsack instance
        knapsacksolver::Instance kp_instance;
        kp_instance.set_capacity(kp_capacities_[i]);
        knapsacksolver::ItemIdx j_kp = 0;
        for (ItemIdx j = 0; j < n; ++j) {
            if ((p_.fixed_alt != NULL && (*p_.fixed_alt)[j][i] >= 0)
                    || instance_.weight(j, i) > kp_capacities_[i]) {
                kp_indices_[j] = -1;
                continue;
            }
            knapsacksolver::Profit profit = std::ceil(mult * mu(j) - mult * instance_.cost(j, i));
            if (profit <= 0) {
                kp_indices_[j] = -1;
                continue;
            }
            kp_instance.add_item(instance_.weight(j, i), profit);
            kp_indices_[j] = j_kp;
            j_kp++;
        }

        // Solve knapsack instance
        //auto kp_output = knapsacksolver::bellman_array_all(kp_instance, Info().set_verbose(false));
        auto kp_output = knapsacksolver::minknap(kp_instance);
        //std::cout << "i " << i << " opt " << sol.profit() << std::endl;

        // Update bound and gradient
        for (ItemIdx j = 0; j < n; ++j) {
            if (kp_indices_[j] >= 0 && kp_output.solution.contains_idx(kp_indices_[j])) {
                grad_(item_indices_[j])--;
                l += instance_.cost(j, i) - mu(item_indices_[j]);
            }
        }
    }

    return l;
}

LagRelaxAssignmentLbfgsOutput generalizedassignmentsolver::lagrelax_assignment_lbfgs(const Instance& instance, LagRelaxAssignmentLbfgsOptionalParameters p)
{
    VER(p.info, "*** lagrelax_assignment_lbfgs ***" << std::endl);
    LagRelaxAssignmentLbfgsOutput output(instance, p.info);

    ItemIdx n = instance.item_number();
    AgentIdx m = instance.agent_number();

    // Compute c0, item_indices and unfixed_item_number
    ItemIdx item_idx = 0;
    Cost c0 = 0;
    std::vector<ItemIdx> item_indices(n, -2);
    for (ItemIdx j = 0; j < n; ++j) {
        for (AgentIdx i = 0; i < m; ++i) {
            if (p.fixed_alt != NULL && (*p.fixed_alt)[j][i] == 1) {
                c0 += instance.cost(j, i);
                item_indices[j] = -1;
                break;
            }
        }
        if (item_indices[j] == -2) {
            item_indices[j] = item_idx;
            item_idx++;
        }
    }
    ItemIdx unfixed_item_number = item_idx;

    // Initialize multipliers
    column_vector mu(unfixed_item_number);
    if (p.initial_multipliers != NULL) {
        for (ItemIdx j = 0; j < n; ++j)
            if (item_indices[j] >= 0)
                mu(item_indices[j]) = (*p.initial_multipliers)[j];
    } else {
        for (ItemIdx j = 0; j < n; ++j)
            mu(j) = 0;
    }

    // Solve
    LagRelaxAssignmentLbfgsFunction func(instance, p, unfixed_item_number, item_indices);
    auto f   = [&func](const column_vector& x) { return func.f(x); };
    auto def = [&func](const column_vector& x) { return func.der(x); };
    auto stop_strategy = objective_delta_stop_strategy(0.0001);
    //auto stop_strategy = gradient_norm_stop_strategy().be_verbose(),
    double res = find_max(
            lbfgs_search_strategy(256),
            stop_strategy,
            f,
            def,
            mu,
            std::numeric_limits<double>::max());

    // Compute output parameters
    Cost lb = c0 + std::ceil(res - TOL);
    output.update_lower_bound(lb, std::stringstream(""), p.info);
    output.multipliers.resize(n);
    for (ItemIdx j = 0; j < n; ++j)
        if (item_indices[j] >= 0)
            output.multipliers[j] = mu(item_indices[j]);

    return output.algorithm_end(p.info);
}

/*************************** lagrelax_knapsack_lbfgs **************************/

LagRelaxKnapsackLbfgsOutput& LagRelaxKnapsackLbfgsOutput::algorithm_end(Info& info)
{
    //PUT(info, "Algorithm", "Iterations", it);
    Output::algorithm_end(info);
    //VER(info, "Iterations: " << it << std::endl);
    return *this;
}

class LagRelaxKnapsackLbfgsFunction
{

public:

    LagRelaxKnapsackLbfgsFunction(const Instance& instance): instance_(instance), grad_(instance.agent_number()) {  }
    virtual ~LagRelaxKnapsackLbfgsFunction() { };

    double f(const column_vector& x);

    const column_vector der(const column_vector& x) const { (void)x; return grad_; }

private:

    const Instance& instance_;
    column_vector grad_;

};

double LagRelaxKnapsackLbfgsFunction::f(const column_vector& mu)
{
    ItemIdx n = instance_.item_number();
    AgentIdx m = instance_.agent_number();

    // Initialize bound and gradient
    double l = 0;
    for (AgentIdx i = 0; i < m; ++i) {
        l += mu(i) * instance_.capacity(i);
        grad_(i) = instance_.capacity(i);
    }

    for (ItemIdx j = 0; j < n; ++j) {
        // Solve the trivial Generalized Upper Bound Problem
        AgentIdx i_best = -1;
        double rc_best = -1;
        for (AgentIdx i = 0; i < m; ++i) {
            double rc = instance_.cost(j, i) - mu(i) * instance_.weight(j, i);
            if (i_best == -1
                    || rc_best > rc
                    // If the minimum reduced cost of a job is reached for
                    // several agents, schedule the job on the agent with the
                    // most available remaining capacity.
                    // Without this condition, the relaxation fails to get the
                    // optimal bound (the one from the linear relaxation) for
                    // some instances.
                    || (rc_best == rc && grad_(i) > grad_(i_best))) {
                i_best = i;
                rc_best = rc;
            }
        }

        // Update bound and gradient
        grad_(i_best) -= instance_.weight(j, i_best);
        l += rc_best;
    }

    return l;
}

LagRelaxKnapsackLbfgsOutput generalizedassignmentsolver::lagrelax_knapsack_lbfgs(const Instance& instance, Info info)
{
    VER(info, "*** lagrelax_knapsack_lbfgs ***" << std::endl);
    LagRelaxKnapsackLbfgsOutput output(instance, info);

    AgentIdx m = instance.agent_number();

    // Initialize multipliers
    column_vector mu(m);
    column_vector mu_lower(m);
    column_vector mu_upper(m);
    for (AgentIdx i = 0; i < m; ++i) {
        //mu_lower(i) = 0;
        //mu_upper(i) = std::numeric_limits<double>::max();
        mu(i) = 0;
        mu_lower(i) = -std::numeric_limits<double>::max();
        mu_upper(i) = 0;
    }

    // Solve
    LagRelaxKnapsackLbfgsFunction func(instance);
    auto f   = [&func](const column_vector& x) { return func.f(x); };
    auto def = [&func](const column_vector& x) { return func.der(x); };
    auto stop_strategy = objective_delta_stop_strategy();
    //auto stop_strategy = gradient_norm_stop_strategy();
    double res = find_max_box_constrained(
            lbfgs_search_strategy(256),
            stop_strategy,
            f,
            def,
            mu,
            mu_lower,
            mu_upper);

    // Compute output parameters
    Cost lb = std::ceil(res - TOL);
    output.update_lower_bound(lb, std::stringstream(""), info);
    output.multipliers.resize(m);
    for (AgentIdx i = 0; i < m; ++i)
        output.multipliers[i] = mu(i);

    return output.algorithm_end(info);
}

