#pragma once

#include "generalizedassignment/lib/solution.hpp"

namespace generalizedassignment
{

std::function<Output (Instance&, std::mt19937_64&, Info)> get_algorithm(std::string str);

}

