#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

using namespace gap;

int main(int argc, char *argv[])
{
    if (argc <= 1)
        return 1;

    std::string instancefile = argv[1];
    std::string certfile = (argc <= 2)? instancefile + ".sol": argv[2];

    Instance ins(instancefile);

    Solution sol(ins);
    sol.read(certfile);

    if (!sol.feasible()) {
        std::cout << "INFEASIBLE" << std::endl;
    } else {
        std::cout << sol.cost() << std::endl;
    }
    return 0;
}

