#if GUROBI_FOUND

#include "gap/opt_branchandcut_gurobi/branchandcut_gurobi.hpp"

#include "gurobi_c++.h"

/*
 * https://stackoverflow.com/questions/46779850/cannot-compile-gurobi-examples-in-version-7-5-1
 */

using namespace gap;

class mycallback: public GRBCallback
{

public:

    mycallback(Solution& sol, Info& info, GRBVar* x):
        sol_(sol), info_(info), x_(x) { }

protected:

    void callback()
    {
        if (where != GRB_CB_MIPSOL)
            return;

        if(!sol_.feasible() || sol_.cost() > getDoubleInfo(GRB_CB_MIPSOL_OBJ) + 0.5) {
            Solution sol_curr(sol_.instance());
            AltIdx   o = sol_.instance().alternative_number();
            double* x = getSolution(x_, o);
            for (AltIdx k=0; k<o; ++k)
                if (x[k] > 0.5)
                    sol_curr.set(k);
            std::stringstream ss;
            sol_.update(sol_curr, 0, ss, info_);
        }
    }

private:

    Solution& sol_;
    Info& info_;
    GRBVar* x_;

};

Solution gap::sopt_branchandcut_gurobi(BranchAndCutGurobiData d)
{
    GRBEnv env;
    VER(d.info, "*** branchandcut_gurobi ***" << std::endl);

    init_display(d.info);

    ItemIdx n = d.ins.item_number();
    AgentIdx m = d.ins.agent_number();
    AltIdx o = d.ins.alternative_number();

    if (n == 0)
        return d.sol;

    GRBModel model(env);

    // Variables
    GRBVar* x = model.addVars(o, GRB_BINARY);

    // Objective
    for (AltIdx k=0; k<o; ++k)
        x[k].set(GRB_DoubleAttr_Obj, d.ins.alternative(k).c);
    model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

    // Capacity constraints
    for (AgentIdx i=0; i<m; i++) {
        GRBLinExpr expr;
        for (ItemIdx j=0; j<n; j++)
            expr += x[d.ins.alternative_index(j, i)] * d.ins.alternative(j, i).w;
        model.addConstr(expr <= d.ins.capacity(i));
    }

    // One alternative per item constraint
    for (ItemIdx j=0; j<n; j++) {
        GRBLinExpr expr;
        for (AgentIdx i=0; i<m; i++)
            expr += x[d.ins.alternative_index(j, i)];
        model.addConstr(expr == 1);
    }

    // Initial solution
    if (d.sol.feasible())
        for (ItemIdx j=0; j<n; ++j)
            for (AgentIdx i=0; i<m; ++i)
                x[d.ins.alternative_index(j, i)].set(GRB_DoubleAttr_Start, (d.sol.agent(j) == i)? 1: 0);

    // Display
    model.getEnv().set(GRB_IntParam_OutputFlag, 0);

    // Time limit
    if (d.info.timelimit != std::numeric_limits<double>::infinity())
        model.set(GRB_DoubleParam_TimeLimit, d.info.timelimit);

    // Callback
    mycallback cb = mycallback(d.sol, d.info, x);
    model.setCallback(&cb);

    // Optimize
    model.optimize();

    for (AltIdx k=0; k<o; ++k)
        if (x[k].get(GRB_DoubleAttr_X) > 0.5)
            d.sol.set(k);

    return algorithm_end(d.sol, d.info);
}

#endif

