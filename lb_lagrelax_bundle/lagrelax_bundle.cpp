#if COINOR_FOUND

#include "gap/lb_lagrelax_bundle/lagrelax_bundle.hpp"

#include "knapsack/opt_minknap/minknap.hpp"

#include <coin/CglKnapsackCover.hpp>
#include <coin/CglClique.hpp>
#include <coin/ClpModel.hpp>
#include <coin/OsiClpSolverInterface.hpp>

#include <algorithm>
#include <iomanip>

using namespace gap;

/*********************** lb_lagrelax_assignment_bundle ************************/

LagRelaxAssignmentBundleOutput& LagRelaxAssignmentBundleOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    //PUT(info, "Algorithm", "Iterations", it);
    //VER(info, "Iterations: " << it << std::endl);
    return *this;
}

double lb_lagrelax_assignment_bundle_subproblem(
        const Instance& ins, const std::vector<double>& mu, std::vector<double>& grad)
{
    ItemIdx n = ins.item_number();
    AgentIdx m = ins.agent_number();

    double l = 0;
    std::fill(grad.begin(), grad.begin() + n, 1);

    for (ItemIdx j=0; j<n; ++j)
        l += mu[j];

    Weight mult = 10000;
    std::vector<ItemIdx> indices(n);
    for (AgentIdx i=0; i<m; ++i) {
        knapsack::Instance ins_kp;
        ins_kp.set_capacity(ins.capacity(i));
        for (ItemIdx j=0; j<n; ++j) {
            AltIdx k = ins.alternative_index(j, i);
            const Alternative& a = ins.alternative(k);
            knapsack::Profit p = std::ceil(mult * mu[j] - mult * a.c);
            if (p > 0) {
                ins_kp.add_item(a.w, p);
                knapsack::ItemIdx j_kp = ins_kp.item_number() - 1;
                indices[j_kp] = j;
            }
        }
        auto output_kp = knapsack::sopt_minknap(ins_kp);
        for (knapsack::ItemIdx j_kp=0; j_kp<ins_kp.item_number(); ++j_kp) {
            if (output_kp.solution.contains_idx(j_kp)) {
                ItemIdx j = indices[j_kp];
                AltIdx k = ins.alternative_index(j, i);
                grad[j]--;
                l += ins.alternative(k).c - mu[j];
            }
        }
    }

    return l;
}

LagRelaxAssignmentBundleOutput gap::lb_lagrelax_assignment_bundle(const Instance& ins, Info info)
{
    /*
     * L(μ^k) = min_x Σij cij xij + Σj μ^k_j (1 - Σi xij)
     * s.t.
     * Σj wij xij <= bj        for all i = 1..m-1
     *
     * μ^k: μ at iteration k
     * Note that ∇L(μ^k) = 1 - Σi xij
     *
     * At each iteration, we need to solve the following linear program:
     * Variables: μ_j, j=1..n, r
     * max r
     * r <= L(μ^k) + ∇L(μ^k) . (μ - μ^k)              for all μ^k considered.
     * i.e. (CLP format)
     * - L(μ^k) + ∇L(μ^k) . μ^k <= ∇L(μ^k) . μ - r    for all μ^k considered.
     *
     * For example, we first add μ^0 = (0, ..., 0).
     * In this case, no items are taken, i.e.:
     * xij*(μ^0) = 0 for all j = 0..n-1, i=0..m-1.
     * L(μ^0) = 0
     * ∇L(μ^0) = (1, ..., 1)
     * Therefore, the first added constraint is:
     * 0 <= μ_0 + μ_1 + ... + μ_n - r
     *
     */

    VER(info, "*** lagrelax_assignment_bundle ***" << std::endl);
    LagRelaxAssignmentBundleOutput output(ins, info);
    ItemIdx n = ins.item_number();

    // Initilze LP
    std::vector<int> vec_ind(n + 1);
    std::iota(vec_ind.begin(), vec_ind.end(), 0);
    std::vector<double> vec_elem(n + 1, 1);
    vec_elem[n] = -1;
    std::vector<int> rows(n + 1, 0);
    std::vector<int> start(n + 2, 0);
    std::iota(start.begin(), start.end(), 0);
    std::vector<int> length(n + 1, 1);
    CoinPackedMatrix mat(true, 1, n + 1, n + 1,
            vec_elem.data(), rows.data(), start.data(), length.data());
    std::vector<double> col_lower(n + 1, -DBL_MAX);
    std::vector<double> col_upper(n + 1, DBL_MAX);
    std::vector<double> objective(n + 1, 0);
    objective[n] = 1;
    std::vector<double> row_lower {0};
    std::vector<double> row_upper {DBL_MAX};
    ClpSimplex model;
    model.messageHandler()->setLogLevel(0); // Reduce printout
    model.loadProblem(mat,
            col_lower.data(), col_upper.data(), objective.data(),
            row_lower.data(), row_upper.data());
    model.setOptimizationDirection(-1); // Maximize

    std::vector<double> mu(n, 1); // Lagrangian multipliers
    std::mt19937_64 gen(0);
    double mu_max = 1;
    for (Cpt it=0; it<1000 ;++it) {
        // Solve LP
        model.primal();

        // Compute next mu
        if (model.status() == 2) {
            std::uniform_real_distribution<double> dis(-mu_max, mu_max);
            for (ItemIdx j=0; j<n; ++j)
                mu[j] = dis(gen);
            mu_max *= 1.01;
        } else {
            const double *solution = model.getColSolution();
            for (ItemIdx j=0; j<n; ++j)
                mu[j] = solution[j];
            std::cout << "toto" << std::endl;
        }

        // Solve subproblem
        double l = lb_lagrelax_assignment_bundle_subproblem(ins, mu, vec_elem);
        std::cout << l << std::endl;

        // Update lower bound
        Cost lb = std::ceil(l - TOL);
        if (output.lower_bound < lb) {
            std::stringstream ss;
            ss <<  "it " << it;
            output.update_lower_bound(lb, ss, info);
        }

        // Avoid adding useless rows
        bool pass = true;
        for (ItemIdx j=0; j<n; ++j) {
            if (vec_elem[j] != 1) {
                pass = false;
                break;
            }
        }
        if (pass)
            continue;

        // Add row
        double lower = -l;
        for (ItemIdx j=0; j<n; ++j)
            lower += (mu[j] * vec_elem[j]);
        model.addRow(n + 1, vec_ind.data(), vec_elem.data(), lower, DBL_MAX);
    }

    return output;
}

/************************ lb_lagrelax_knapsack_bundle *************************/

LagRelaxKnapsackBundleOutput& LagRelaxKnapsackBundleOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    //PUT(info, "Algorithm", "Iterations", it);
    //VER(info, "Iterations: " << it << std::endl);
    return *this;
}

LagRelaxKnapsackBundleOutput gap::lb_lagrelax_knapsack_bundle(const Instance& ins, Info info)
{
    VER(info, "*** lagrelax_knapsack_bundle ***" << std::endl);
    LagRelaxKnapsackBundleOutput output(ins, info);
    (void)ins;
    (void)info;
    return output.algorithm_end(info);
}

#endif

