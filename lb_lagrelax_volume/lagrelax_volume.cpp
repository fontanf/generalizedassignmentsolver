#include "gap/lb_lagrelax_volume/lagrelax_volume.hpp"

#include "knapsack/opt_minknap/minknap.hpp"

#include "coin/VolVolume.hpp"
#include "coin/CoinHelperFunctions.hpp"
#include "coin/CoinPackedMatrix.hpp"

using namespace gap;

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
       (void)x;
       (void)heur_val;
       return 0;
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

   for (AgentIdx j=0; j<ins.item_number(); ++j) {
       lcost += dual[j];
       v[j] = 1;
   }

   // Solve independent knapsack problems
   Weight mult = 10000;
   std::vector<ItemIdx> indices(ins.item_number());
   for (AgentIdx i=0; i<ins.agent_number(); ++i) {
       knapsack::Instance ins_kp(ins.item_number(), ins.capacity(i));
       for (ItemIdx j=0; j<ins.item_number(); ++j) {
           const Alternative& a = ins.alternative(j, i);
           knapsack::Profit p = mult * dual[j] - mult * a.c;
           if (p > 0) {
               knapsack::ItemIdx j_kp = ins_kp.add_item(a.w, p);
               indices[j_kp] = j;
           }
       }
       knapsack::Solution sol = knapsack::Minknap(ins_kp, knapsack::MinknapParams::combo()).run();
       for (knapsack::ItemIdx j_kp=0; j_kp<ins_kp.total_item_number(); ++j_kp) {
           if (sol.contains_idx(j_kp)) {
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

   return 0;
}

LagRelaxAssignmentVolumeOutput gap::lb_lagrelax_assignment_volume(const Instance& ins, Info info)
{
    VER(info, "*** lagrelax_assignment_volume ***" << std::endl);

    VOL_problem volprob;
    volprob.parm.printflag = (info.output->verbose)? 1: 0;

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
    LagRelaxAssignmentVolumeOutput output;

    output.lb = volprob.value; // bound

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

    algorithm_end(output.lb, info);
    return output;
}


/*****************************************************************************/

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
           if (k_best == -1 || rc_best > rc[k]) {
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

LagRelaxKnapsackVolumeOutput gap::lb_lagrelax_knapsack_volume(const Instance& ins, Info info)
{
    VER(info, "*** lagrelax_knapsack_volume ***" << std::endl);

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
    LagRelaxKnapsackVolumeOutput output;

    output.lb = volprob.value; // bound

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

    algorithm_end(output.lb, info);
    return output;
}

