#if GECODE_FOUND

#include "generalizedassignmentsolver/tester.hpp"
#include "generalizedassignmentsolver/algorithms/constraintprogramming_gecode.hpp"

using namespace generalizedassignmentsolver;

Output constraintprogramming_gecode_test(Instance& instance)
{
    ConstraintProgrammingGecodeOptionalParameters parameters;
    return constraintprogramming_gecode(instance, parameters);
}

std::vector<Output (*)(Instance&)> f = {
        constraintprogramming_gecode_test,
};

TEST(ConstraintProgrammingGecode, TEST) { test(TEST, f, SOPT); }

#endif

