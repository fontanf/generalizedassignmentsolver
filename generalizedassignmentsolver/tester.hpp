#pragma once

#include "generalizedassignmentsolver/solution.hpp"

#include <gtest/gtest.h>

namespace generalizedassignmentsolver
{

Instance test_instance(Counter i);
enum TestType { SOPT, LB, UB };
enum InstacesType { TEST };

void test(InstacesType it, std::vector<Output (*)(Instance&)> fs, TestType tt = SOPT);

}

