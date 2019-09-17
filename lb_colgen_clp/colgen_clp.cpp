#if COINOR_FOUND

#include "gap/lb_colgen_clp/colgen_clp.hpp"

#include <coin/ClpModel.hpp>
#include <coin/OsiClpSolverInterface.hpp>

using namespace gap;

void gap::lb_colgen(ColGenClpData d)
{
    VER(d.info, "*** colgen_clp ***" << std::endl);

    // Initialize solver

    // Add initial columns
    if (d.column_elts.empty()) {

    } else {

    }

    for (;;) {
        // Solve LP

        // Find and add new columns

    }

}

#endif

