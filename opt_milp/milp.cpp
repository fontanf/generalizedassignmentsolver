#include "gap/opt_milp/milp.hpp"

#include <coin/CbcModel.hpp>
#include <coin/OsiCbcSolverInterface.hpp>

using namespace gap;

Solution gap::sopt_milp(const Instance& ins, Info info)
{
    VER(info, "*** milp ***" << std::endl);

    int numberRows = ins.agent_number() + ins.item_number();
    int numberColumns = ins.alternative_number();
    int numberElements = 2 * ins.alternative_number();

    // matrix data - column ordered
    std::vector<CoinBigIndex> start(numberColumns + 1);
    for (AltIdx k=0; k<=numberColumns; ++k)
        start[k] = 2 * k;
    std::vector<int> length(numberColumns, 2);

    std::vector<int> rows(numberElements);
    std::vector<double> elements(numberElements);
    for (AltIdx k=0; k<ins.alternative_number(); ++k) {
            rows[2 * k] = ins.alternative(k).i;
        elements[2 * k] = ins.alternative(k).w;
            rows[2 * k + 1] = ins.agent_number() + ins.alternative(k).j;
        elements[2 * k + 1] = 1;
    }

    CoinPackedMatrix matrix(true, numberRows, numberColumns, numberElements,
            elements.data(), rows.data(), start.data(), length.data());

    // rim data
    std::vector<double> objective(numberColumns);
    for (AltIdx k=0; k<numberColumns; ++k)
        objective[k] = ins.alternative(k).v;

    std::vector<double> rowLower(numberRows);
    std::vector<double> rowUpper(numberRows);
    for (AgentIdx i=0; i<ins.agent_number(); ++i) {
        rowLower[i] = 0;
        rowUpper[i] = ins.capacity(i);
    }
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        rowLower[ins.agent_number() + j] = 1;
        rowUpper[ins.agent_number() + j] = 1;
    }
    std::vector<double> colLower(numberColumns, 0);
    std::vector<double> colUpper(numberColumns, 1);
    OsiCbcSolverInterface solver1;
    solver1.getModelPtr()->setLogLevel(0);
    solver1.messageHandler()->setLogLevel(0);
    // load problem
    solver1.loadProblem(matrix, colLower.data(), colUpper.data(),
              objective.data(), rowLower.data(), rowUpper.data());
    // mark integer
    for (AltIdx k=0; k<ins.alternative_number(); ++k)
        solver1.setInteger(k);

    // Solve
    solver1.initialSolve();

    // Pass data and solver to CbcModel
    CbcModel model(solver1);

    // reduce printout
    model.setLogLevel(0);
    model.solver()->setHintParam(OsiDoReducePrint, true, OsiHintTry);
    // Do complete search
    model.branchAndBound();
    /* Print solution.  CbcModel clones solver so we
       need to get current copy */

    const double *solution = model.solver()->getColSolution();
    Solution sol(ins);
    for (AltIdx k=0; k<ins.alternative_number(); ++k)
        if (solution[k] > 0.5)
            sol.set(k);
    return algorithm_end(sol, info);
}
