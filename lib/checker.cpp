#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

#include <iomanip>

using namespace gap;

int main(int argc, char *argv[])
{
    if (argc <= 1)
        return 1;

    std::string instancefile = argv[1];
    std::string certfile = (argc <= 2)? instancefile + ".sol": argv[2];

    Instance ins(instancefile);

    Solution sol(ins, certfile);
    Cost lb = 0;
    std::ifstream f_bound(instancefile + ".bound");
    if (f_bound.good())
        f_bound >> lb;
    f_bound.close();

    double ub = (!sol.feasible())? std::numeric_limits<double>::infinity(): sol.cost();
    double gap = (lb == 0 || !sol.feasible())? std::numeric_limits<double>::infinity():
        (double)(10000 * (sol.cost() - lb) / lb) / 100;

    if (ub == lb)
        std::cout << "\033[32m";
    if (lb == ins.bound())
        std::cout << "\033[31m";

    std::cout << std::left << std::setw(3) << "UB";
    std::cout << std::right << std::setw(8) << ub;
    std::cout << std::left << std::setw(6) << " | LB";
    std::cout << std::right << std::setw(8) << lb;
    std::cout << std::left << std::setw(7) << " | GAP";
    std::cout << std::right << std::setw(6) << ub - lb;
    std::cout << std::left << std::setw(11) << " | GAP (%)";
    std::cout << std::right << std::setw(6) << gap;
    std::cout << "\033[0m" << std::endl;
    return 0;
}

