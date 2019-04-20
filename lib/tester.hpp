#pragma once

#include "gap/lib/instance.hpp"

#include <gtest/gtest.h>

namespace gap
{

Instance test_instance(Cpt i);
enum TestType { OPT, LB, UB };
enum InstacesType { TEST };

void test(InstacesType it, std::vector<Value (*)(Instance&)> fs, TestType tt = OPT);

}

