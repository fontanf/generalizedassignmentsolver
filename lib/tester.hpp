#pragma once

#include <gtest/gtest.h>

#include "gap/lib/instance.hpp"

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

namespace gap
{

void test(std::string exec, std::string test);

void test_gen(
        std::vector<std::string> types,
        std::vector<AgentIdx> ms,
        std::vector<ItemIdx> ns,
        std::vector<int> seeds,
        int obj,
        std::vector<Value (*)(Instance&)> fs,
        int test = 0);

}

