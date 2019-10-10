#pragma once

#include "gap/lib/solution.hpp"

#include <gtest/gtest.h>

namespace gap
{

Instance test_instance(Cpt i);
enum TestType { SOPT, LB, UB };
enum InstacesType { TEST };

void test(InstacesType it, std::vector<Output (*)(Instance&)> fs, TestType tt = SOPT);

}

