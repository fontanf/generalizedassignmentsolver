#include "gap/opt_constraintprogramming_gecode/constraintprogramming_gecode.hpp"

using namespace gap;

Solution gap::sopt_constraintprogramming_gecode(ConstraintProgrammingGecodeData d)
{
    VER(d.info, "*** constraintprogramming_gecode ***" << std::endl);

    return algorithm_end(d.sol, d.info);
}

