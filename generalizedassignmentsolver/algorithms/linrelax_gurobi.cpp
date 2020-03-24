#if GUROBI_FOUND

#include "generalizedassignmentsolver/algorithms/linrelax_gurobi.hpp"

#include "gurobi_c++.h"

using namespace generalizedassignmentsolver;

LinRelaxGurobiOutput& LinRelaxGurobiOutput::algorithm_end(Info& info)
{
    //PUT(info, "Algorithm", "Iterations", it);
    Output::algorithm_end(info);
    //VER(info, "Iterations: " << it << std::endl);
    return *this;
}

LinRelaxGurobiOutput generalizedassignmentsolver::linrelax_gurobi(const Instance& instance, Info info)
{
    GRBEnv env;
    VER(info, "*** linrelax_gurobi ***" << std::endl);

    LinRelaxGurobiOutput output(instance, info);

    ItemIdx n = instance.item_number();
    AgentIdx m = instance.agent_number();
    AltIdx o = instance.alternative_number();

    GRBModel model(env);

    // Variables and objective
    GRBVar* x = model.addVars(o);
    for (AltIdx k=0; k<o; ++k) {
        x[k].set(GRB_CharAttr_VType, GRB_CONTINUOUS);
        x[k].set(GRB_DoubleAttr_LB, 0);
        x[k].set(GRB_DoubleAttr_UB, 1);
        x[k].set(GRB_DoubleAttr_Obj, instance.alternative(k).c);
    }
    model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

    // Capacity constraints
    for (AgentIdx i = 0; i < m; i++) {
        GRBLinExpr expr;
        for (ItemIdx j = 0; j < n; j++)
            expr += x[instance.alternative_index(j, i)] * instance.alternative(j, i).w;
        model.addConstr(expr <= instance.capacity(i));
    }

    // One alternative per item constraint
    for (ItemIdx j = 0; j < n; j++) {
        GRBLinExpr expr;
        for (AgentIdx i = 0; i < m; i++)
            expr += x[instance.alternative_index(j, i)];
        model.addConstr(expr == 1);
    }

    // Redirect standard output to log file
    model.set(GRB_StringParam_LogFile, "gurobi.log");
    model.set(GRB_IntParam_LogToConsole, 0);

    // Optimize
    model.optimize();

    // Get solution
    Cost lb = std::ceil(model.get(GRB_DoubleAttr_ObjVal) - TOL);
    output.update_lower_bound(lb, std::stringstream(""), info);
    output.x = std::vector<double>(o, 0);
    for (AltIdx k = 0; k < instance.alternative_number(); ++k)
        output.x[k] = x[k].get(GRB_DoubleAttr_X);

    return output.algorithm_end(info);
}

#endif

