#pragma once

#include "generalizedassignmentsolver/solution.hpp"

#include <gtest/gtest.h>

namespace generalizedassignmentsolver
{

enum TestType { SOPT, LB, UB };
enum InstacesType { TEST };

void test(InstacesType instances_type, std::vector<Output (*)(Instance&)> algorithms, TestType test_type = SOPT);

}

