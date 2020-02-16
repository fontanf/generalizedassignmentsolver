#pragma once

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

std::function<Output (Instance&, std::mt19937_64&, Info)> get_algorithm(std::string str);

}

