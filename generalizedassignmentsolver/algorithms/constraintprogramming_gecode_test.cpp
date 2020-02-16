#if GECODE_FOUND

#include "generalizedassignmentsolver/tester.hpp"
#include "generalizedassignmentsolver/algorithms/constraintprogramming_gecode.hpp"

using namespace generalizedassignmentsolver;

Output constraintprogramming_gecode_test(Instance& ins)
{
    ConstraintProgrammingGecodeOptionalParameters p;
    return constraintprogramming_gecode(ins, p);
}

std::vector<Output (*)(Instance&)> f = {
        constraintprogramming_gecode_test,
};

TEST(BranchAndCutCbc, TEST) { test(TEST, f, SOPT); }

#endif

