#include "gap/lb_lagrelax_lbfgs/lagrelax_lbfgs.hpp"

#include "knapsack/opt_minknap/minknap.hpp"

#include <dlib/optimization.h>

#include <algorithm>
#include <iomanip>
#include <limits>

using namespace gap;
using namespace dlib;

typedef matrix<double,0,1> column_vector;

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
        knapsack::Instance ins_kp(n, ins_.capacity(i));
        for (ItemIdx j=0; j<n; ++j) {
            AltIdx k = ins_.alternative_index(j, i);
            const Alternative& a = ins_.alternative(k);
            knapsack::Profit p = std::ceil(mult * mu(j) - mult * a.c);
            if (p > 0) {
                knapsack::ItemIdx j_kp = ins_kp.add_item(a.w, p);
                indices[j_kp] = j;
            }
        }
        knapsack::Solution sol = knapsack::Minknap(ins_kp, knapsack::MinknapParams::combo()).run();
        //knapsack::Solution sol = knapsack::Minknap(ins_kp, knapsack::MinknapParams::pure()).run();
        //std::cout << "i " << i << " opt " << sol.profit() << std::endl;
        for (knapsack::ItemIdx j_kp=0; j_kp<ins_kp.total_item_number(); ++j_kp) {
            if (sol.contains_idx(j_kp)) {
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
    ItemIdx n = ins.item_number();
    LagRelaxAssignmentLbfgsOutput out;
    LagRelaxAssignmentLbfgsFunction func(ins);
    column_vector mu(n);
    auto f   = [&func](const column_vector& x) { return func.f(x); };
    auto def = [&func](const column_vector& x) { return func.der(x); };
    auto stop_strategy = objective_delta_stop_strategy(0.0001);
    //auto stop_strategy = gradient_norm_stop_strategy().be_verbose(),
    if (info.output->verbose)
        stop_strategy.be_verbose();
    out.lb = std::ceil(find_max(
            lbfgs_search_strategy(256),
            stop_strategy,
            f,
            def,
            mu,
            std::numeric_limits<double>::max()));
    algorithm_end(out.lb, info);
    return out;
}

LagRelaxKnapsackLbfgsOutput gap::lb_lagrelax_knapsack_lbfgs(const Instance& ins, Info info)
{
    LagRelaxKnapsackLbfgsOutput out;
    ItemIdx n = ins.item_number();
    (void)n;
    (void)info;
    return out;
}

