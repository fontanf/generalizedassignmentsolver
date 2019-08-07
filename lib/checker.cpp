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

    if (!sol.feasible()) {
        std::cout << "INFEASIBLE" << std::endl;
    } else {
        std::cout << std::left << std::setw(12) << "UB";
        std::cout << std::left << std::setw(12) << "LB";
        std::cout << std::left << std::setw(10) << "GAP";
        std::cout << std::left << std::setw(10) << "GAP (%)";
        std::cout << std::endl;

        double gap = (lb == 0)? std::numeric_limits<double>::infinity():
            (double)(10000 * (sol.cost() - lb) / lb) / 100;
        std::cout << std::left << std::setw(12) << sol.cost();
        std::cout << std::left << std::setw(12) << lb;
        std::cout << std::left << std::setw(10) << sol.cost() - lb;
        std::cout << std::left << std::setw(10) << gap;
        std::cout << std::endl;
    }
    return 0;
}

