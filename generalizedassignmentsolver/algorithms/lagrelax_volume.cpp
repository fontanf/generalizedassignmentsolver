#if COINOR_FOUND

#include "generalizedassignmentsolver/algorithms/lagrelax_volume.hpp"

#include "knapsacksolver/algorithms/dynamic_programming_primal_dual.hpp"

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

////////////////////////////////////////////////////////////////////////////////
////////////////////////// lagrelax_assignment_volume //////////////////////////
////////////////////////////////////////////////////////////////////////////////

LagRelaxAssignmentVolumeOutput& LagRelaxAssignmentVolumeOutput::algorithm_end(
        optimizationtools::Info& info)
{
    //info.add_to_json("Algorithm", "Iterations", it);
    Output::algorithm_end(info);
    //FFOT_VER(info, "Iterations: " << it << std::endl);
    return *this;
}

class LagRelaxAssignmentHook: public VOL_user_hooks
{

public:

    LagRelaxAssignmentHook(const Instance& instance):
        instance_(instance) {  }

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
        Solution sol(instance_);
        for (ItemIdx item_id = 0; item_id < instance_.number_of_items(); ++item_id) {
            bool fixed = false;
            AgentIdx agent_id_best = -1;
            Weight w_best = -1;
            for (AgentIdx agent_id = 0; agent_id < instance_.number_of_agents(); ++agent_id) {
                if (x[instance_.number_of_agents() * item_id + agent_id] == 1) {
                    sol.set(item_id, agent_id);
                    fixed = true;
                    break;
                }
                Weight w = instance_.weight(item_id, agent_id);
                if (x[instance_.number_of_agents() * item_id + agent_id] > 0
                        && (w_best < 0 || w_best > w)) {
                    w_best = w;
                    agent_id_best = agent_id;
                }
            }
            if (!fixed)
                sol.set(item_id, agent_id_best);
        }
        if (sol.feasible())
            heur_val = sol.cost();
        return (sol.feasible())? 1: 0;
    }

private:

    const Instance& instance_;

};

int LagRelaxAssignmentHook::compute_rc(const VOL_dvector& u, VOL_dvector& rc)
{
    for (ItemIdx item_id = 0; item_id < instance_.number_of_items(); ++item_id)
        for (AgentIdx agent_id = 0; agent_id < instance_.number_of_agents(); ++agent_id)
            rc[instance_.number_of_agents() * item_id + agent_id]
                = instance_.cost(item_id, agent_id) - u[item_id];
    return 0;
}

int LagRelaxAssignmentHook::solve_subproblem(const VOL_dvector& dual, const VOL_dvector& rc,
        double& lcost, VOL_dvector& x, VOL_dvector& v, double& pcost)
{
    lcost = 0;
    pcost = 0;
    for (ItemIdx item_id = 0; item_id < instance_.number_of_items(); ++item_id) {
        lcost += dual[item_id];
        v[item_id] = 1;
    }

    // Solve independent knapsack problems
    //Weight mult = 10000;
    Weight mult = 1000000;
    std::vector<ItemIdx> indices(instance_.number_of_items());
    for (AgentIdx agent_id = 0; agent_id < instance_.number_of_agents(); ++agent_id) {
        knapsacksolver::Instance kp_instance;
        kp_instance.set_capacity(instance_.capacity(agent_id));
        for (ItemIdx item_id = 0; item_id < instance_.number_of_items(); ++item_id) {
            x[instance_.number_of_agents() * item_id + agent_id] = 0;
            knapsacksolver::Profit p = std::ceil(mult * dual[item_id]
                    - mult * instance_.cost(item_id, agent_id));
            if (p > 0) {
                kp_instance.add_item(instance_.weight(item_id, agent_id), p);
                knapsacksolver::ItemIdx kp_item_id = kp_instance.number_of_items() - 1;
                indices[kp_item_id] = item_id;
            }
        }
        auto kp_output = knapsacksolver::dynamic_programming_primal_dual(kp_instance);
        for (knapsacksolver::ItemIdx kp_item_id = 0; kp_item_id < kp_instance.number_of_items(); ++kp_item_id) {
            if (kp_output.solution.contains_idx(kp_item_id)) {
                ItemIdx item_id = indices[kp_item_id];
                x[instance_.number_of_agents() * item_id + agent_id] = 1;
                v[item_id]--;
                pcost += instance_.cost(item_id, agent_id);
                lcost += rc[instance_.number_of_agents() * item_id + agent_id];
            }
        }
    }

    return 0;
}

LagRelaxAssignmentVolumeOutput generalizedassignmentsolver::lagrelax_assignment_volume(
        const Instance& instance,
        optimizationtools::Info info)
{
    init_display(instance, info);
    info.os()
            << "Algorithm" << std::endl
            << "---------" << std::endl
            << "Lagrangian Relaxation - Assignment Constraints (Volume)" << std::endl
            << std::endl;

    LagRelaxAssignmentVolumeOutput output(instance, info);

    VOL_problem volprob;
    volprob.parm.printflag = (info.output->verbosity_level)? 1: 0;

    // These parameters don't seem too bad...
    volprob.parm.heurinvl = 10;
    volprob.parm.alphainit = 0.75;
    volprob.parm.alphafactor = 0.9;
    volprob.parm.maxsgriters = 10000;

    // Set the lb/ub on the duals
    volprob.psize = instance.number_of_agents() * instance.number_of_items();
    volprob.dsize = instance.number_of_items();
    volprob.dual_lb.allocate(instance.number_of_items());
    volprob.dual_ub.allocate(instance.number_of_items());
    for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id) {
        volprob.dual_lb[item_id] = -1.0e31;
        volprob.dual_ub[item_id] =  1.0e31;
    }

    LagRelaxAssignmentHook hook(instance);
    volprob.solve(hook, false /* no warmstart */);

    // Extract solution

    Cost lb = std::ceil(volprob.value - FFOT_TOL); // bound
    output.update_lower_bound(lb, std::stringstream(""), info);

    output.multipliers.resize(instance.number_of_items()); // multipliers
    for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
        output.multipliers[item_id] = volprob.dsol[item_id];

    output.x.resize(
            instance.number_of_items(),
            std::vector<double>(instance.number_of_agents()));
    for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
        for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id)
            output.x[item_id][agent_id] = volprob.psol[instance.number_of_agents() * item_id + agent_id];

    return output.algorithm_end(info);
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////// lagrelax_knapsack_volume ///////////////////////////
////////////////////////////////////////////////////////////////////////////////

LagRelaxKnapsackVolumeOutput& LagRelaxKnapsackVolumeOutput::algorithm_end(optimizationtools::Info& info)
{
    //info.add_to_json("Algorithm", "Iterations", it);
    Output::algorithm_end(info);
    //FFOT_VER(info, "Iterations: " << it << std::endl);
    return *this;
}

class LagRelaxKnapsackHook: public VOL_user_hooks
{

public:

    LagRelaxKnapsackHook(const Instance& instance):
        instance_(instance) {  }

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

    const Instance& instance_;

};

int LagRelaxKnapsackHook::compute_rc(const VOL_dvector& u, VOL_dvector& rc)
{
    for (ItemIdx item_id = 0; item_id < instance_.number_of_items(); ++item_id)
        for (AgentIdx agent_id = 0; agent_id < instance_.number_of_agents(); ++agent_id)
            rc[instance_.number_of_agents() * item_id + agent_id]
                = instance_.cost(item_id, agent_id) - u[agent_id]
                * instance_.weight(item_id, agent_id);
    return 0;
}

int LagRelaxKnapsackHook::solve_subproblem(const VOL_dvector& dual, const VOL_dvector& rc,
        double& lcost, VOL_dvector& x, VOL_dvector& v, double& pcost)
{
    lcost = 0;
    pcost = 0;

    for (AgentIdx agent_id = 0; agent_id < instance_.number_of_agents(); ++agent_id) {
        lcost += dual[agent_id] * instance_.capacity(agent_id);
        v[agent_id] = instance_.capacity(agent_id);
    }

    for (ItemIdx item_id = 0; item_id < instance_.number_of_items(); ++item_id) {
        AgentIdx agent_id_best = -1;
        double rc_best = -1;
        for (AgentIdx agent_id = 0; agent_id < instance_.number_of_agents(); ++agent_id) {
            x[instance_.number_of_agents() * item_id + agent_id] = 0;
            if (agent_id_best == -1
                    || rc_best > rc[instance_.number_of_agents() * item_id + agent_id]) {
                agent_id_best = agent_id;
                rc_best = rc[instance_.number_of_agents() * item_id + agent_id];
            }
        }
        x[instance_.number_of_agents() * item_id + agent_id_best] = 1;
        v[agent_id_best] -= instance_.weight(item_id, agent_id_best);
        pcost += instance_.cost(item_id, agent_id_best);
        lcost += rc_best;
    }

    return 0;
}

LagRelaxKnapsackVolumeOutput generalizedassignmentsolver::lagrelax_knapsack_volume(
        const Instance& instance,
        optimizationtools::Info info)
{
    init_display(instance, info);
    info.os()
            << "Algorithm" << std::endl
            << "---------" << std::endl
            << "Lagrangian Relaxation - Knapsack Constraints (Volume)" << std::endl
            << std::endl;

    LagRelaxKnapsackVolumeOutput output(instance, info);

    VOL_problem volprob;
    volprob.parm.printflag = (info.output->verbosity_level)? 1: 0;

    // Set the lb/ub on the duals
    volprob.psize = instance.number_of_agents() * instance.number_of_items();
    volprob.dsize = instance.number_of_agents();
    volprob.dual_lb.allocate(instance.number_of_agents());
    volprob.dual_ub.allocate(instance.number_of_agents());
    for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id) {
        volprob.dual_ub[agent_id] = 0.0;
        volprob.dual_lb[agent_id] = -1.0e31;
    }

    LagRelaxKnapsackHook hook(instance);
    volprob.solve(hook, false /* no warmstart */);

    // Extract solution

    Cost lb = std::ceil(volprob.value - FFOT_TOL); // bound
    output.update_lower_bound(lb, std::stringstream(""), info);

    output.multipliers.resize(instance.number_of_agents()); // multipliers
    for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id)
        output.multipliers[agent_id] = volprob.dsol[agent_id];

    output.x.resize(
            instance.number_of_items(),
            std::vector<double>(instance.number_of_agents()));
    for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
        for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id)
            output.x[item_id][agent_id] = volprob.psol[instance.number_of_agents() * item_id + agent_id];

    return output.algorithm_end(info);
}

#endif

