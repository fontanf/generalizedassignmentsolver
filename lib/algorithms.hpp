#pragma once

#include "gap/lib/solution.hpp"

namespace gap
{

std::function<Output (Instance&, std::mt19937_64&, Info)> get_algorithm(std::string str);

}

