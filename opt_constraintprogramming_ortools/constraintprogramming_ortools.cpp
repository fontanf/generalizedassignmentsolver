#if ORTOOLS_FOUND

#include "gap/opt_constraintprogramming_ortools/constraintprogramming_ortools.hpp"

#include "ortools/base/commandlineflags.h"
#include "ortools/base/logging.h"
#include "ortools/sat/cp_model.h"
#include "ortools/sat/sat_parameters.pb.h"

/**
 * Useful links:
 * https://github.com/google/or-tools/blob/stable/examples/cpp/multi_knapsack_sat.cc
 */

using namespace gap;

Solution gap::sopt_constraintprogramming_ortools(ConstraintProgrammingOrtoolsData d)
{
    VER(d.info, "*** constraintprogramming_ortools ***" << std::endl);

    init_display(d.info);

    if (d.ins.item_number() == 0)
        return d.sol;


    return algorithm_end(d.sol, d.info);
}

#endif

