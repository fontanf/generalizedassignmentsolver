#include "gap/lib/solution.hpp"
#include "gap/lib/datasets.hpp"

#include <iomanip>

using namespace gap;

void checker(const Instance& ins, std::string solfile, std::string bndfile)
{
    Solution sol(ins, solfile);
    Cost lb = 0;
    std::ifstream f_bound(bndfile);
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

    std::cout << std::left << std::setw(16) << ins.name();
    std::cout << std::left << std::setw(3) << "UB";
    std::cout << std::right << std::setw(8) << ub;
    std::cout << std::left << std::setw(6) << " | LB";
    std::cout << std::right << std::setw(8) << lb;
    std::cout << std::left << std::setw(7) << " | GAP";
    std::cout << std::right << std::setw(6) << ub - lb;
    std::cout << std::left << std::setw(11) << " | GAP (%)";
    std::cout << std::right << std::setw(6) << gap;
    std::cout << "\033[0m" << std::endl;
}

int main(int argc, char *argv[])
{
    if (argc <= 1) {
        for (auto it = datasets.begin(); it != datasets.end(); ++it) {
            // Print dataset name centered
            std::string s = "--- " + it->first + " --- ";
            int pos = (int)((80-s.length())/2);
            for(int i=0; i<pos; i++)
                std::cout << " ";
            std::cout << s << std::endl;

            Dataset<Instance>* d = it->second.get();
            for (InsId i=0; i<d->size(); ++i) {
                Instance ins = d->instance(i);
                std::string solfile = ins.name() + ".sol";
                std::string bndfile = ins.name() + ".bound";
                checker(ins, solfile, bndfile);
            }
        }
    } else {
        std::string insfile = argv[1];
        Instance ins(insfile);
        std::string solfile = (argc <= 2)? insfile + ".sol": argv[2];
        std::string bndfile = insfile + ".bound";
        checker(ins, solfile, bndfile);
    }

    return 0;
}

