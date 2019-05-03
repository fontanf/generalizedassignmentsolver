#include "gap/opt_milp/milp.hpp"

#include <coin/CbcModel.hpp>
#include <coin/OsiCbcSolverInterface.hpp>

using namespace gap;

Solution gap::sopt_milp(MilpData d)
{
    VER(d.info, "*** milp ***" << std::endl);

    int loglevel = (d.info.output->verbose)? 1: 0;
    int numberRows = d.ins.agent_number() + d.ins.item_number();
    int numberColumns = d.ins.alternative_number();
    int numberElements = 2 * d.ins.alternative_number();

    // Matrix data - column ordered
    std::vector<CoinBigIndex> start(numberColumns + 1);
    for (AltIdx k=0; k<=numberColumns; ++k)
        start[k] = 2 * k;
    std::vector<int> length(numberColumns, 2);

    std::vector<int> rows(numberElements);
    std::vector<double> elements(numberElements);
    for (AltIdx k=0; k<d.ins.alternative_number(); ++k) {
            rows[2 * k] = d.ins.alternative(k).i;
        elements[2 * k] = d.ins.alternative(k).w;
            rows[2 * k + 1] = d.ins.agent_number() + d.ins.alternative(k).j;
        elements[2 * k + 1] = 1;
    }

    CoinPackedMatrix matrix(true, numberRows, numberColumns, numberElements,
            elements.data(), rows.data(), start.data(), length.data());

    // Rim data
    std::vector<double> objective(numberColumns);
    for (AltIdx k=0; k<numberColumns; ++k)
        objective[k] = d.ins.alternative(k).v;

    std::vector<double> rowLower(numberRows);
    std::vector<double> rowUpper(numberRows);
    for (AgentIdx i=0; i<d.ins.agent_number(); ++i) {
        rowLower[i] = 0;
        rowUpper[i] = d.ins.capacity(i);
    }
    for (ItemIdx j=0; j<d.ins.item_number(); ++j) {
        rowLower[d.ins.agent_number() + j] = 1;
        rowUpper[d.ins.agent_number() + j] = 1;
    }
    std::vector<double> colLower(numberColumns, 0);
    std::vector<double> colUpper(numberColumns, 1);
    OsiCbcSolverInterface solver1;

    // Reduce printout
    solver1.getModelPtr()->setLogLevel(loglevel);
    solver1.messageHandler()->setLogLevel(loglevel);

    // Load problem
    solver1.loadProblem(matrix, colLower.data(), colUpper.data(),
              objective.data(), rowLower.data(), rowUpper.data());

    // Mark integer
    for (AltIdx k=0; k<d.ins.alternative_number(); ++k)
        solver1.setInteger(k);

    // Solve
    solver1.initialSolve();

    // Pass data and solver to CbcModel
    CbcModel model(solver1);
    model.setMaximumSeconds(d.info.timelimit);

    // Add initial solution
    std::vector<double> sol_init(d.ins.alternative_number(), 0);
    if (d.sol.feasible()) {
        for (AltIdx k=0; k<d.ins.alternative_number(); ++k)
            if (d.sol.agent(d.ins.alternative(k).j) == d.ins.alternative(k).i)
                sol_init[k] = 1;
        model.setBestSolution(sol_init.data(), d.ins.alternative_number(), d.sol.value());
    }
    if (d.stop_at_first_improvment)
        model.setMaximumSolutions(1);

    // Reduce printout
    model.setLogLevel(loglevel);
    model.solver()->setHintParam(OsiDoReducePrint, true, OsiHintTry);

    // Do complete search
    model.branchAndBound();

    if (d.sol.feasible() && d.sol.value() <= model.getObjValue() + 0.5)
        return algorithm_end(d.sol, d.info);

    // Get solution
    const double *solution = model.solver()->getColSolution();
    for (AltIdx k=0; k<d.ins.alternative_number(); ++k)
        if (solution[k] > 0.5)
            d.sol.set(k);
    if (!d.sol.feasible())
        d.sol = Solution(d.ins);
    return algorithm_end(d.sol, d.info);
}

