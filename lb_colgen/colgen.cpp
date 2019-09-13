#include "gap/lb_colgen/colgen.hpp"

#if CPLEX_FOUND
#include <ilcplex/ilocplex.h>
ILOSTLBEGIN
#endif

#if GUROBI_FOUND
#include <gurobi_c++.h>
#endif

#if COINOR_FOUND
#include <coin/CbcModel.hpp>
#include <coin/OsiCbcSolverInterface.hpp>
#include <coin/CglKnapsackCover.hpp>
#include <coin/CglClique.hpp>
#endif

using namespace gap;

void gap::lb_colgen(ColGenData d)
{
    VER(d.info, "*** colgen s " << d.solver << " ***" << std::endl);

    if (d.solver == "clp") {
#ifndef COINOR_FOUND
        std::cerr << "\033[31m" << "ERROR, unable to use solver \"clp\"" << "\033[0m" << std::endl;
        return;
#endif
    } else if (d.solver == "cplex") {
#ifndef CPLEX_FOUND
        std::cerr << "\033[31m" << "ERROR, unable to use solver \"cplex\"" << "\033[0m" << std::endl;
        return;
#endif
    } else if (d.solver == "gurobi") {
#ifndef GUROBI_FOUND
        std::cerr << "\033[31m" << "ERROR, unable to use solver \"gurobi\"" << "\033[0m" << std::endl;
        return;
#endif
    }

    // Add initial columns
    if (d.columns.empty()) {

    } else {

    }

    for (;;) {
        // Solve LP

        // Find and add new columns

    }

}

