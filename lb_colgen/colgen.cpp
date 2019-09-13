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

}

