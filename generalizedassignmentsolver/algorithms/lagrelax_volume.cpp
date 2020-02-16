#if COINOR_FOUND

#include "generalizedassignmentsolver/algorithms/lagrelax_volume.hpp"

#include "knapsacksolver/algorithms/minknap.hpp"

#include "coin/VolVolume.hpp"
#include "coin/CoinHelperFunctions.hpp"
#include "coin/CoinPackedMatrix.hpp"

using namespace generalizedassignmentsolver;

/**
 * Useful links to use Vol:
 * https://github.com/coin-or/Vol/blob/master/Vol/doc/volDoc.pdf
 * https://domino.watson.ibm.com/library/cyberdig.nsf/papers/7A1B25774639540A852565BE0070CFB5/$File/RC21103.pdf
 * https://github.com/coin-or/Vol/tree/master/Vol/examples/VolUfl
 * https://www.coin-or.org/Doxygen/Clp/classVOL__user__hooks.html
 * https://www.coin-or.org/Doxygen/Osi/classVOL__problem.html
 * https://www.coin-or.org/Doxygen/Osi/structVOL__parms.html
 */

/************************* lagrelax_assignment_volume *************************/

LagRelaxAssignmentVolumeOutput& LagRelaxAssignmentVolumeOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    //PUT(info, "Algorithm", "Iterations", it);
    //VER(info, "Iterations: " << it << std::endl);
    return *this;
}

class LagRelaxAssignmentHook: public VOL_user_hooks
{

public:

    LagRelaxAssignmentHook(const Instance& ins): ins_(ins) {  }
    virtual ~LagRelaxAssignmentHook() { }

    // for all hooks: return value of -1 means that volume should quit

    /**
     * compute reduced costs
     * @param u (IN) the dual variables
     * @param rc (OUT) the reduced cost with respect to the dual values
     */
    virtual int compute_rc(const VOL_dvector& u, VOL_dvector& rc);

    /**
     * Solve the subproblem for the subgradient step.
     * @param dual (IN) the dual variables
     * @param rc (IN) the reduced cost with respect to the dual values
     * @param lcost (OUT) the lagrangean cost with respect to the dual values
     * @param x (OUT) the primal result of solving the subproblem
     * @param v (OUT) b-Ax for the relaxed constraints
     * @param pcost (OUT) the primal objective value of <code>x</code>
     */
    virtual int solve_subproblem(const VOL_dvector& dual, const VOL_dvector& rc,
            double& lcost, VOL_dvector& x, VOL_dvector& v, double& pcost);

    /**
     * Starting from the primal vector x, run a heuristic to produce
     * an integer solution
     * @param x (IN) the primal vector
     * @param heur_val (OUT) the value of the integer solution (return
     * <code>DBL_MAX</code> here if no feas sol was found
     */
    virtual int heuristics(const VOL_problem& p,
            const VOL_dvector& x, double& heur_val)
    {
        (void)p;
        //(void)x;
        //(void)heur_val;
        //return 0;
        Solution sol(ins_);
        ItemIdx n = ins_.item_number();
        AgentIdx m = ins_.agent_number();
        for (ItemIdx j=0; j<n; ++j) {
            bool fixed = false;
            AgentIdx i_best = -1;
            Weight w_best = -1;
            for (AgentIdx i=0; i<m; ++i) {
                AltIdx k = ins_.alternative_index(j, i);
                if (x[k] == 1) {
                    sol.set(j, i);
                    fixed = true;
                    break;
                }
                Weight w = ins_.alternative(k).w;
                if (x[k] > 0 && (w_best < 0 || w_best > w)) {
                    w_best = w;
                    i_best = i;
                }
            }
            if (!fixed)
                sol.set(j, i_best);
        }
        if (sol.feasible())
            heur_val = sol.cost();
        return (sol.feasible())? 1: 0;
    }

private:

    const Instance& ins_;

};

int LagRelaxAssignmentHook::compute_rc(const VOL_dvector& u, VOL_dvector& rc)
{
    const Instance& ins = ins_;
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        for (AgentIdx i=0; i<ins.agent_number(); ++i) {
            AltIdx k = ins.alternative_index(j, i);
            rc[k] = ins.alternative(k).c - u[j];
        }
    }
    return 0;
}

int LagRelaxAssignmentHook::solve_subproblem(const VOL_dvector& dual, const VOL_dvector& rc,
        double& lcost, VOL_dvector& x, VOL_dvector& v, double& pcost)
{
    const Instance& ins = ins_;

    lcost = 0;
    pcost = 0;
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        lcost += dual[j];
        v[j] = 1;
    }

    // Solve independent knapsack problems
    //Weight mult = 10000;
    Weight mult = 1000000;
    std::vector<ItemIdx> indices(ins.item_number());
    for (AgentIdx i=0; i<ins.agent_number(); ++i) {
        knapsacksolver::Instance ins_kp;
        ins_kp.set_capacity(ins.capacity(i));
        for (ItemIdx j=0; j<ins.item_number(); ++j) {
            AltIdx k = ins.alternative_index(j, i);
            const Alternative& a = ins.alternative(k);
            x[k] = 0;
            knapsacksolver::Profit p = std::ceil(mult * dual[j] - mult * a.c);
            if (p > 0) {
                ins_kp.add_item(a.w, p);
                knapsacksolver::ItemIdx j_kp = ins_kp.item_number() - 1;
                indices[j_kp] = j;
            }
        }
        auto output_kp = knapsacksolver::minknap(ins_kp);
        for (knapsacksolver::ItemIdx j_kp=0; j_kp<ins_kp.item_number(); ++j_kp) {
            if (output_kp.solution.contains_idx(j_kp)) {
                ItemIdx j = indices[j_kp];
                AltIdx k = ins.alternative_index(j, i);
                x[k] = 1;
                v[j]--;
                pcost += ins.alternative(k).c;
                lcost += rc[k];
            }
        }
    }

    //std::cout << "mult";
    //for (ItemIdx j=0; j<ins.item_number(); ++j)
    //std::cout << " " << dual[j];
    //std::cout << std::endl;
    //std::cout << "pcost " << pcost << std::endl;
    //std::cout << "lcost " << lcost << std::endl;
    //std::cout << "v ";
    //for (ItemIdx j=0; j<ins.item_number(); ++j)
    //std::cout << " " << v[j];
    //std::cout << std::endl;

    //for (ItemIdx j=0; j<ins.item_number(); ++j) {
    //std::cout << "j " << j;
    //for (AgentIdx i=0; i<ins.agent_number(); ++i) {
    //AltIdx k = ins.alternative_index(j, i);
    //std::cout << " " << round(x[k] * 100) / 100;
    //}
    //std::cout << std::endl;
    //}

    return 0;
}

LagRelaxAssignmentVolumeOutput generalizedassignmentsolver::lagrelax_assignment_volume(const Instance& ins, Info info)
{
    VER(info, "*** lagrelax_assignment_volume ***" << std::endl);
    LagRelaxAssignmentVolumeOutput output(ins, info);

    VOL_problem volprob;
    volprob.parm.printflag = (info.output->verbose)? 1: 0;

    // These parameters don't seem too bad...
    volprob.parm.heurinvl = 10;
    volprob.parm.alphainit = 0.75;
    volprob.parm.alphafactor = 0.9;
    volprob.parm.maxsgriters = 10000;

    // Set the lb/ub on the duals
    volprob.psize = ins.alternative_number();
    volprob.dsize = ins.item_number();
    volprob.dual_lb.allocate(ins.item_number());
    volprob.dual_ub.allocate(ins.item_number());
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        volprob.dual_lb[j] = -1.0e31;
        volprob.dual_ub[j] =  1.0e31;
    }

    LagRelaxAssignmentHook hook(ins);
    volprob.solve(hook, false /* no warmstart */);

    // Extract solution

    Cost lb = std::ceil(volprob.value - TOL); // bound
    output.update_lower_bound(lb, std::stringstream(""), info);

    output.multipliers.resize(ins.item_number()); // multipliers
    for (ItemIdx j=0; j<ins.item_number(); ++j)
        output.multipliers[j] = volprob.dsol[j];

    output.x.resize(ins.alternative_number()); // x
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        for (AgentIdx i=0; i<ins.agent_number(); ++i) {
            AltIdx k = ins.alternative_index(j, i);
            output.x[k] = volprob.psol[k];
        }
    }

    return output.algorithm_end(info);
}

/************************** lagrelax_knapsack_volume **************************/

LagRelaxKnapsackVolumeOutput& LagRelaxKnapsackVolumeOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    //PUT(info, "Algorithm", "Iterations", it);
    //VER(info, "Iterations: " << it << std::endl);
    return *this;
}

class LagRelaxKnapsackHook: public VOL_user_hooks
{

public:

    LagRelaxKnapsackHook(const Instance& ins): ins_(ins) {  }
    virtual ~LagRelaxKnapsackHook() { }

    // for all hooks: return value of -1 means that volume should quit

    /**
     * compute reduced costs
     * @param u (IN) the dual variables
     * @param rc (OUT) the reduced cost with respect to the dual values
     */
    virtual int compute_rc(const VOL_dvector& u, VOL_dvector& rc);

    /**
     * Solve the subproblem for the subgradient step.
     * @param dual (IN) the dual variables
     * @param rc (IN) the reduced cost with respect to the dual values
     * @param lcost (OUT) the lagrangean cost with respect to the dual values
     * @param x (OUT) the primal result of solving the subproblem
     * @param v (OUT) b-Ax for the relaxed constraints
     * @param pcost (OUT) the primal objective value of <code>x</code>
     */
    virtual int solve_subproblem(const VOL_dvector& dual, const VOL_dvector& rc,
            double& lcost, VOL_dvector& x, VOL_dvector& v, double& pcost);

    /**
     * Starting from the primal vector x, run a heuristic to produce
     * an integer solution
     * @param x (IN) the primal vector
     * @param heur_val (OUT) the value of the integer solution (return
     * <code>DBL_MAX</code> here if no feas sol was found
     */
    virtual int heuristics(const VOL_problem& p,
            const VOL_dvector& x, double& heur_val)
    {
        (void)p;
        (void)x;
        (void)heur_val;
        return 0;
    }

private:

    const Instance& ins_;

};

int LagRelaxKnapsackHook::compute_rc(const VOL_dvector& u, VOL_dvector& rc)
{
    const Instance& ins = ins_;
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        for (AgentIdx i=0; i<ins.agent_number(); ++i) {
            AltIdx k = ins.alternative_index(j, i);
            rc[k] = ins.alternative(k).c - u[i] * ins.alternative(k).w;
        }
    }
    return 0;
}

int LagRelaxKnapsackHook::solve_subproblem(const VOL_dvector& dual, const VOL_dvector& rc,
        double& lcost, VOL_dvector& x, VOL_dvector& v, double& pcost)
{
    const Instance& ins = ins_;

    lcost = 0;
    pcost = 0;

    for (AgentIdx i=0; i<ins.agent_number(); ++i) {
        lcost += dual[i] * ins.capacity(i);
        v[i] = ins.capacity(i);
    }

    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        AltIdx k_best = -1;
        AgentIdx i_best = -1;
        double rc_best = -1;
        for (AgentIdx i=0; i<ins.agent_number(); ++i) {
            AltIdx k = ins.alternative_index(j, i);
            x[k] = 0;
            if (k_best == -1
                    || rc_best > rc[k]) {
                k_best = k;
                i_best = i;
                rc_best = rc[k];
            }
        }
        x[k_best] = 1;
        v[i_best] -= ins.alternative(k_best).w;
        pcost += ins.alternative(k_best).c;
        lcost += rc_best;
    }

    //std::cout << "mult";
    //for (AgentIdx i=0; i<ins.agent_number(); ++i)
    //std::cout << " " << dual[i];
    //std::cout << std::endl;
    //std::cout << "pcost " << pcost << std::endl;
    //std::cout << "lcost " << lcost << std::endl;
    //std::cout << "v ";
    //for (AgentIdx i=0; i<ins.agent_number(); ++i)
    //std::cout << " " << v[i];
    //std::cout << std::endl;

    return 0;
}

LagRelaxKnapsackVolumeOutput generalizedassignmentsolver::lagrelax_knapsack_volume(const Instance& ins, Info info)
{
    VER(info, "*** lagrelax_knapsack_volume ***" << std::endl);
    LagRelaxKnapsackVolumeOutput output(ins, info);

    VOL_problem volprob;
    volprob.parm.printflag = (info.output->verbose)? 1: 0;

    // Set the lb/ub on the duals
    volprob.psize = ins.alternative_number();
    volprob.dsize = ins.agent_number();
    volprob.dual_lb.allocate(ins.agent_number());
    volprob.dual_ub.allocate(ins.agent_number());
    for (AgentIdx i=0; i<ins.agent_number(); ++i) {
        volprob.dual_ub[i] = 0.0;
        volprob.dual_lb[i] = -1.0e31;
    }

    LagRelaxKnapsackHook hook(ins);
    volprob.solve(hook, false /* no warmstart */);

    // Extract solution

    Cost lb = std::ceil(volprob.value - TOL); // bound
    output.update_lower_bound(lb, std::stringstream(""), info);

    output.multipliers.resize(ins.agent_number()); // multipliers
    for (AgentIdx i=0; i<ins.agent_number(); ++i)
        output.multipliers[i] = volprob.dsol[i];

    output.x.resize(ins.alternative_number()); // x
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        for (AgentIdx i=0; i<ins.agent_number(); ++i) {
            AltIdx k = ins.alternative_index(j, i);
            output.x[k] = volprob.psol[k];
        }
    }

    return output.algorithm_end(info);
}

#endif

