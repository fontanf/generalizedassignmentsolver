#if GECODE_FOUND

#include "generalizedassignment/lib/tester.hpp"
#include "generalizedassignment/opt_constraintprogramming_gecode/constraintprogramming_gecode.hpp"

using namespace generalizedassignment;

Output sopt_constraintprogramming_gecode_test(Instance& ins)
{
    ConstraintProgrammingGecodeOptionalParameters p;
    return sopt_constraintprogramming_gecode(ins, p);
}

std::vector<Output (*)(Instance&)> f = {
        sopt_constraintprogramming_gecode_test,
};

TEST(BranchAndCutCbc, TEST) { test(TEST, f, SOPT); }

#endif

