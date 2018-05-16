#include "gap/opt_cplex/cplex.hpp"

#include <ilcplex/ilocplex.h>

ILOSTLBEGIN

typedef IloArray<IloNumVarArray> NumVarMatrix;
typedef IloArray<IloNumArray>    NumMatrix;

using namespace gap;

Solution gap::sopt_cplex(const Instance& ins, Info* info)
{
    (void)info;

    IloEnv env;
    Solution solution(ins);
    try {
        IloModel model(env);

        // Variables
        NumVarMatrix x(env, ins.item_number());
        for (ItemIdx j=0; j<ins.item_number(); ++j)
            x[j] = IloNumVarArray(env, ins.alternative_number(), 0, 1, ILOBOOL);

        // Objective
        IloExpr expr(env);
        for (ItemIdx j=0; j<ins.item_number(); j++)
            for (AgentIdx i=0; i<ins.agent_number(); i++)
                expr += x[j][i] * ins.alternative(j, i).p;
        IloObjective obj = (ins.objective() == 1)?
            IloMaximize(env, expr):
            IloMinimize(env, expr);
        model.add(obj);

        // Capacity constraints
        for (AgentIdx i=0; i<ins.agent_number(); i++) {
            IloExpr lhs(env);
            for (ItemIdx j=0; j<ins.item_number(); j++)
                lhs += x[j][i] * ins.alternative(j, i).w;
            model.add(IloRange(env, 0, lhs, ins.capacity(i)));
        }

        // One alternative per item constraint
        for (ItemIdx j=0; j<ins.item_number(); j++) {
            IloExpr lhs(env);
            for (AgentIdx i=0; i<ins.agent_number(); i++)
                lhs += x[j][i];
            model.add(IloRange(env, 1, lhs, 1));
        }

        // Optimize
        IloCplex cplex(model);
        cplex.setOut(env.getNullStream());
        if (cplex.solve()) {
            // Get solution
            for (AgentIdx i=0; i<ins.agent_number(); i++)
                for (ItemIdx j=0; j<ins.item_number(); j++)
                    if (cplex.getValue(x[j][i]) > 0.5)
                        solution.set(j, i);
        } else {
        }
    } catch (IloException& ex) {
        std::cerr << "Error: " << ex.getMessage() << std::endl;
    } catch (...) {
        std::cerr << "Error" << std::endl;
    }
    env.end();

    assert(solution.profit() == ins.optimum());
    return solution;
}
