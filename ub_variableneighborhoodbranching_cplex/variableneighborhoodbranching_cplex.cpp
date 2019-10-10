#if CPLEX_FOUND

#include "gap/ub_variableneighborhoodbranching_cplex/variableneighborhoodbranching_cplex.hpp"

#include <ilcplex/ilocplex.h>

using namespace gap;

ILOSTLBEGIN

typedef IloArray<IloNumVarArray> NumVarMatrix;
typedef IloArray<IloNumArray>    NumMatrix;

VariableNeighborhoodBranchingOutput& VariableNeighborhoodBranchingOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    return *this;
}

VariableNeighborhoodBranchingOutput gap::sol_variableneighborhoodbranching_cplex(
        const Instance& ins, std::mt19937_64& gen,
        VariableNeighborhoodBranchingOptionalParameters p)
{
    (void)gen;
    VER(p.info, "*** variableneighborhoodbranching_cplex ***" << std::endl);
    VariableNeighborhoodBranchingOutput output(ins, p.info);

    ItemIdx n = ins.item_number();
    AgentIdx m = ins.agent_number();
    AltIdx o = ins.alternative_number();


    // Initialize MIP

    IloEnv env;
    IloModel model(env);

    // Variables
    IloNumVarArray x(env, o, 0, 1, ILOBOOL);

    // Objective
    IloExpr expr(env);
    for (ItemIdx j=0; j<n; j++)
        for (AgentIdx i=0; i<m; i++)
            expr += x[ins.alternative_index(j, i)] * ins.alternative(j, i).c;
    IloObjective obj = IloMinimize(env, expr);
    model.add(obj);

    // Capacity constraints
    for (AgentIdx i=0; i<m; i++) {
        IloExpr lhs(env);
        for (ItemIdx j=0; j<n; j++)
            lhs += x[ins.alternative_index(j, i)] * ins.alternative(j, i).w;
        model.add(IloRange(env, 0, lhs, ins.capacity(i)));
    }

    // One alternative per item constraint
    for (ItemIdx j=0; j<n; j++) {
        IloExpr lhs(env);
        for (AgentIdx i=0; i<m; i++)
            lhs += x[ins.alternative_index(j, i)];
        model.add(IloRange(env, 1, lhs, 1));
    }


    // Initial solution

    IloCplex cplex(model);

    // Time limit
    if (p.info.timelimit != std::numeric_limits<double>::infinity())
        cplex.setParam(IloCplex::TiLim, p.info.remaining_time());

    // Stop after first solution
    cplex.setParam(IloCplex::IntSolLim, 1);

    // Optimize
    cplex.solve();

    if (cplex.getStatus() == IloAlgorithm::Infeasible) {
        output.update_lower_bound(ins.bound(), std::stringstream(""), p.info);
    } else if (cplex.getStatus() == IloAlgorithm::Optimal) {
        if (!output.solution.feasible() || output.solution.cost() > cplex.getObjValue() + 0.5) {
            Solution sol_curr(ins);
            for (AltIdx k=0; k<o; ++k)
                if (cplex.getValue(x[k]) > 0.5)
                    sol_curr.set(k);
            output.update_solution(sol_curr, std::stringstream(""), p.info);
        }
        output.update_lower_bound(output.solution.cost(), std::stringstream(""), p.info);
    } else if (cplex.isPrimalFeasible()) {
        if (!output.solution.feasible() || output.solution.cost() > cplex.getObjValue() + 0.5) {
            Solution sol_curr(ins);
            for (AltIdx k=0; k<o; ++k)
                if (cplex.getValue(x[k]) > 0.5)
                    sol_curr.set(k);
            output.update_solution(sol_curr, std::stringstream(""), p.info);
        }
        Cost lb = std::ceil(cplex.getBestObjValue() - TOL);
        output.update_lower_bound(lb, std::stringstream(""), p.info);
    } else {
        Cost lb = std::ceil(cplex.getBestObjValue() - TOL);
        output.update_lower_bound(lb, std::stringstream(""), p.info);
    }

    if (output.lower_bound >= ins.bound() || !output.solution.feasible())
        return output.algorithm_end(p.info);


    // Local search

    while (p.info.check_time()) {

            //IloExpr lhs(env);
            //for (ItemIdx j=0; j<n; ++j) {
                //for (AgentIdx i=0; i<m; ++i) {
                    //if (i == sol_best.agent(j)) {
                        //lhs -= x[j][i];
                    //} else {
                        //lhs += x[j][i];
                    //}
                //}
            //}
            //model.add(IloRange(env, - n, lhs, k_max - n));

    }

    return output.algorithm_end(p.info);
}

#endif

