#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

using namespace gap;

int main(int argc, char *argv[])
{
    if (argc <= 2)
        return 1;
    Instance ins(argv[1]);
    Solution sol(ins);
    sol.read(argv[2]);
    std::cout << "Feasible: " << ((sol.feasible())? "OK": "NO") << std::endl;
    std::cout << "Cost: " << sol.cost() << std::endl;
    return 0;
}

