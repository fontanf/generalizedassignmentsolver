#if DLIB_FOUND

#include "gap/lb_lagrelax_lbfgs/lagrelax_lbfgs.hpp"

#include "knapsack/opt_minknap/minknap.hpp"
#include "knapsack/opt_bellman/bellman.hpp"

#include <dlib/optimization.h>

#include <algorithm>
#include <iomanip>
#include <limits>

using namespace gap;
using namespace dlib;

typedef matrix<double,0,1> column_vector;

/************************ lb_lagrelax_assignment_lbfgs ************************/

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

    LagRelaxAssignmentLbfgsFunction(const Instance& ins): ins_(ins), grad_(ins.item_number()) {  }
    virtual ~LagRelaxAssignmentLbfgsFunction() { };

    double f(const column_vector& x);

    const column_vector der(const column_vector& x) const { (void)x; return grad_; }

private:

    const Instance& ins_;
    column_vector grad_;

};

double LagRelaxAssignmentLbfgsFunction::f(const column_vector& mu)
{
    ItemIdx n = ins_.item_number();
    AgentIdx m = ins_.agent_number();

    double l = 0;
    std::fill(grad_.begin(), grad_.end(), 1);

    for (ItemIdx j=0; j<n; ++j)
        l += mu(j);

    Weight mult = 1000000;
    std::vector<ItemIdx> indices(n);
    for (AgentIdx i=0; i<m; ++i) {
        knapsack::Instance ins_kp;
        ins_kp.set_capacity(ins_.capacity(i));
        for (ItemIdx j=0; j<n; ++j) {
            AltIdx k = ins_.alternative_index(j, i);
            const Alternative& a = ins_.alternative(k);
            knapsack::Profit p = std::ceil(mult * mu(j) - mult * a.c);
            if (p > 0) {
                ins_kp.add_item(a.w, p);
                knapsack::ItemIdx j_kp = ins_kp.item_number() - 1;
                indices[j_kp] = j;
            }
        }
        //knapsack::Solution sol = knapsack::sopt_bellman_array_all(ins_kp, Info().set_verbose(false));
        auto output_kp = knapsack::sopt_minknap(ins_kp);
        //std::cout << "i " << i << " opt " << sol.profit() << std::endl;
        for (knapsack::ItemIdx j_kp=0; j_kp<ins_kp.total_item_number(); ++j_kp) {
            if (output_kp.solution.contains_idx(j_kp)) {
                ItemIdx j = indices[j_kp];
                AltIdx k = ins_.alternative_index(j, i);
                grad_(j)--;
                l += ins_.alternative(k).c - mu(j);
            }
        }
    }

    return l;
}

LagRelaxAssignmentLbfgsOutput gap::lb_lagrelax_assignment_lbfgs(const Instance& ins, Info info)
{
    VER(info, "*** lagrelax_assignment_lbfgs ***" << std::endl);
    LagRelaxAssignmentLbfgsOutput output(ins, info);

    ItemIdx n = ins.item_number();
    LagRelaxAssignmentLbfgsFunction func(ins);
    column_vector mu(n);
    for (ItemIdx j=0; j<n; ++j)
        mu(j) = 0;
    auto f   = [&func](const column_vector& x) { return func.f(x); };
    auto def = [&func](const column_vector& x) { return func.der(x); };
    auto stop_strategy = objective_delta_stop_strategy(0.0001);
    //auto stop_strategy = gradient_norm_stop_strategy().be_verbose(),
    if (info.output->verbose)
        stop_strategy.be_verbose();
    double res = find_max(
            lbfgs_search_strategy(256),
            stop_strategy,
            f,
            def,
            mu,
            std::numeric_limits<double>::max());

    Cost lb = std::ceil(res - TOL);
    output.update_lower_bound(lb, std::stringstream(""), info);
    output.multipliers.resize(n);
    for (ItemIdx j=0; j<n; ++j)
        output.multipliers[j] = mu(j);

    return output.algorithm_end(info);
}

/************************* lb_lagrelax_knapsack_lbfgs *************************/

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

LagRelaxKnapsackLbfgsOutput gap::lb_lagrelax_knapsack_lbfgs(const Instance& ins, Info info)
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
    if (info.output->verbose)
        stop_strategy.be_verbose();
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

