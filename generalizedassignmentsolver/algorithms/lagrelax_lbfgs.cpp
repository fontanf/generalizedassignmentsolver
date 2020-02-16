#if DLIB_FOUND

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
    Output::algorithm_end(info);
    //PUT(info, "Algorithm", "Iterations", it);
    //VER(info, "Iterations: " << it << std::endl);
    return *this;
}

class LagRelaxAssignmentLbfgsFunction
{

public:

    LagRelaxAssignmentLbfgsFunction(const Instance& ins,
            LagRelaxAssignmentLbfgsOptionalParameters& p,
            ItemIdx unfixed_item_number,
            const std::vector<ItemIdx>& item_indices):
        ins_(ins), p_(p), item_indices_(item_indices), grad_(unfixed_item_number)
    {
        ItemIdx n = ins_.item_number();
        AgentIdx m = ins_.agent_number();

        capacities_kp_.resize(m);
        for (AgentIdx i = 0; i < m; ++i) {
            capacities_kp_[i] = ins.capacity(i);
            for (ItemIdx j = 0; j < n; ++j) {
                AltIdx k = ins.alternative_index(j, i);
                if (p.fixed_alt != NULL && (*p.fixed_alt)[k] == 1)
                    capacities_kp_[i] -= ins.alternative(k).w;
            }
            if (capacities_kp_[i] < 0)
                std::cout << "ERROR i " << i << " c " << capacities_kp_[i] << std::endl;
        }

        indices_kp_.resize(n);
    }

    virtual ~LagRelaxAssignmentLbfgsFunction() { }

    double f(const column_vector& x);

    const column_vector der(const column_vector& x) const { (void)x; return grad_; }

private:

    const Instance& ins_;
    LagRelaxAssignmentLbfgsOptionalParameters& p_;
    const std::vector<ItemIdx>& item_indices_; // item_indices_[j] the index of item j in mu and grad

    column_vector grad_;

    std::vector<knapsacksolver::Weight> capacities_kp_;
    std::vector<knapsacksolver::ItemIdx> indices_kp_; // indices_kp_[j] is the index of item j in the current KP

};

double LagRelaxAssignmentLbfgsFunction::f(const column_vector& mu)
{
    ItemIdx n = ins_.item_number();
    AgentIdx m = ins_.agent_number();

    double l = 0;
    std::fill(grad_.begin(), grad_.end(), 1);

    for (ItemIdx j = 0; j < n; ++j)
        if (item_indices_[j] >= 0)
            l += mu(item_indices_[j]);

    Weight mult = 1000000;
    for (AgentIdx i = 0; i < m; ++i) {
        knapsacksolver::Instance ins_kp;
        ins_kp.set_capacity(capacities_kp_[i]);
        ItemIdx j_kp = 0;
        for (ItemIdx j = 0; j < n; ++j) {
            AltIdx k = ins_.alternative_index(j, i);
            const Alternative& a = ins_.alternative(k);
            if ((p_.fixed_alt != NULL && (*p_.fixed_alt)[k] >= 0)
                    || a.w > capacities_kp_[i]) {
                indices_kp_[j] = -1;
                continue;
            }
            knapsacksolver::Profit profit = std::ceil(mult * mu(j) - mult * a.c);
            if (profit <= 0) {
                indices_kp_[j] = -1;
                continue;
            }
            ins_kp.add_item(a.w, profit);
            indices_kp_[j] = j_kp;
            j_kp++;
        }
        //knapsacksolver::Solution sol = knapsacksolver::bellman_array_all(ins_kp, Info().set_verbose(false));
        auto output_kp = knapsacksolver::minknap(ins_kp);
        //std::cout << "i " << i << " opt " << sol.profit() << std::endl;
        for (ItemIdx j = 0; j < n; ++j) {
            if (indices_kp_[j] >= 0 && output_kp.solution.contains_idx(indices_kp_[j])) {
                AltIdx k = ins_.alternative_index(j, i);
                grad_(item_indices_[j])--;
                l += ins_.alternative(k).c - mu(item_indices_[j]);
            }
        }
    }

    return l;
}

LagRelaxAssignmentLbfgsOutput generalizedassignmentsolver::lagrelax_assignment_lbfgs(const Instance& ins, LagRelaxAssignmentLbfgsOptionalParameters p)
{
    VER(p.info, "*** lagrelax_assignment_lbfgs ***" << std::endl);
    LagRelaxAssignmentLbfgsOutput output(ins, p.info);

    ItemIdx n = ins.item_number();
    AgentIdx m = ins.agent_number();

    ItemIdx item_idx = 0;
    Cost c0 = 0;
    std::vector<ItemIdx> item_indices(n, -2);
    for (ItemIdx j = 0; j < n; ++j) {
        for (AgentIdx i = 0; i < m; ++i) {
            if (p.fixed_alt != NULL && (*p.fixed_alt)[ins.alternative_index(j, i)] == 1) {
                c0 += ins.alternative(j, i).c;
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

    LagRelaxAssignmentLbfgsFunction func(ins, p, unfixed_item_number, item_indices);

    column_vector mu(unfixed_item_number);
    if (p.initial_multipliers != NULL) {
        for (ItemIdx j = 0; j < n; ++j)
            if (item_indices[j] >= 0)
                mu(item_indices[j]) = (*p.initial_multipliers)[j];
    } else {
        for (ItemIdx j = 0; j < n; ++j)
            mu(j) = 0;
    }

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
    Output::algorithm_end(info);
    //PUT(info, "Algorithm", "Iterations", it);
    //VER(info, "Iterations: " << it << std::endl);
    return *this;
}

class LagRelaxKnapsackLbfgsFunction
{

public:

    LagRelaxKnapsackLbfgsFunction(const Instance& ins): ins_(ins), grad_(ins.agent_number()) {  }
    virtual ~LagRelaxKnapsackLbfgsFunction() { };

    double f(const column_vector& x);

    const column_vector der(const column_vector& x) const { (void)x; return grad_; }

private:

    const Instance& ins_;
    column_vector grad_;

};

double LagRelaxKnapsackLbfgsFunction::f(const column_vector& mu)
{
    ItemIdx n = ins_.item_number();
    AgentIdx m = ins_.agent_number();

    //std::cout << "mu";
    //for (AgentIdx i=0; i<m; ++i)
        //std::cout << " " << mu(i);
    //std::cout << std::endl;

    double l = 0;
    for (AgentIdx i=0; i<m; ++i) {
        l += mu(i) * ins_.capacity(i);
        grad_(i) = ins_.capacity(i);
    }
    //std::cout << "l0 " << l << std::endl;

    for (ItemIdx j=0; j<n; ++j) {
        AltIdx k_best = -1;
        AgentIdx i_best = -1;
        double rc_best = -1;
        for (AgentIdx i=0; i<m; ++i) {
            AltIdx k = ins_.alternative_index(j, i);
            double rc = ins_.alternative(k).c - mu(i) * ins_.alternative(k).w;
            if (k_best == -1
                    || rc_best > rc
                    // If the minimum reduced cost of a job is reached for
                    // several agents, schedule the job on the agent with the
                    // most available remaining capacity.
                    // Without this condition, the relaxation fails to get the
                    // optimal bound (the one from the linear relaxation) for
                    // some instances.
                    || (rc_best == rc && grad_(i) > grad_(i_best))) {
                k_best = k;
                i_best = i;
                rc_best = rc;
            }
        }
        grad_(i_best) -= ins_.alternative(k_best).w;
        l += rc_best;
        //std::cout << "l " << l << std::endl;
    }

    //std::cout << "grad";
    //for (AgentIdx i=0; i<m; ++i)
        //std::cout << " " << grad_(i);
    //std::cout << std::endl;
    //std::cout << "l " << l << std::endl;

    return l;
}

LagRelaxKnapsackLbfgsOutput generalizedassignmentsolver::lagrelax_knapsack_lbfgs(const Instance& ins, Info info)
{
    VER(info, "*** lagrelax_knapsack_lbfgs ***" << std::endl);
    LagRelaxKnapsackLbfgsOutput output(ins, info);

    AgentIdx m = ins.agent_number();
    LagRelaxKnapsackLbfgsFunction func(ins);
    column_vector mu(m);
    column_vector mu_lower(m);
    column_vector mu_upper(m);
    for (AgentIdx i=0; i<m; ++i) {
        //mu_lower(i) = 0;
        //mu_upper(i) = std::numeric_limits<double>::max();
        mu(i) = 0;
        mu_lower(i) = -std::numeric_limits<double>::max();
        mu_upper(i) = 0;
    }
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

    Cost lb = std::ceil(res - TOL);
    output.update_lower_bound(lb, std::stringstream(""), info);
    output.multipliers.resize(m);
    for (AgentIdx i=0; i<m; ++i)
        output.multipliers[i] = mu(i);

    //std::cout << "mu";
    //for (AgentIdx i=0; i<ins.agent_number(); ++i)
        //std::cout << " " << mu(i);
    //std::cout << std::endl;
    //std::cout << "lb " << out.lb << std::endl;

    return output.algorithm_end(info);
}

#endif

