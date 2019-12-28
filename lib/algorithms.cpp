#include "generalizedassignment/lib/algorithms.hpp"

#include "generalizedassignment/lb_linrelax_clp/linrelax_clp.hpp"
#include "generalizedassignment/lb_linrelax_gurobi/linrelax_gurobi.hpp"
#include "generalizedassignment/lb_lagrelax_volume/lagrelax_volume.hpp"
#include "generalizedassignment/lb_lagrelax_bundle/lagrelax_bundle.hpp"
#include "generalizedassignment/lb_lagrelax_lbfgs/lagrelax_lbfgs.hpp"
#include "generalizedassignment/lb_columngeneration/columngeneration.hpp"
#include "generalizedassignment/opt_branchandcut_cbc/branchandcut_cbc.hpp"
#include "generalizedassignment/opt_branchandcut_cplex/branchandcut_cplex.hpp"
#include "generalizedassignment/opt_branchandcut_gurobi/branchandcut_gurobi.hpp"
#include "generalizedassignment/opt_branchandprice/branchandprice.hpp"
#include "generalizedassignment/opt_constraintprogramming_gecode/constraintprogramming_gecode.hpp"
#include "generalizedassignment/opt_constraintprogramming_cplex/constraintprogramming_cplex.hpp"
#include "generalizedassignment/ub_random/random.hpp"
#include "generalizedassignment/ub_greedy/greedy.hpp"
#include "generalizedassignment/ub_repair/repair.hpp"
#include "generalizedassignment/ub_ls_shiftswap/ls_shiftswap.hpp"
#include "generalizedassignment/ub_localsolver/localsolver.hpp"
#include "generalizedassignment/ub_colgenheuristics/colgenheuristics.hpp"

#include <map>

using namespace generalizedassignment;

std::function<Output (Instance&, std::mt19937_64&, Info)> generalizedassignment::get_algorithm(std::string algorithm)
{
    std::stringstream ss(algorithm);
    std::istream_iterator<std::string> begin(ss);
    std::istream_iterator<std::string> end;
    std::vector<std::string> argv(begin, end);

    if (argv[0] == "") {
        std::cerr << "\033[32m" << "ERROR, missing algorithm." << "\033[0m" << std::endl;
        return [](Instance& ins, std::mt19937_64&, Info info) { return Output(ins, info); };

    /*
     * Lower bounds
     */
#if COINOR_FOUND
    } else if (argv[0] == "linrelax_clp") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            return lb_linrelax_clp(ins, info);
        };
#endif
#if GUROBI_FOUND
    } else if (argv[0] == "linrelax_gurobi") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            return lb_linrelax_gurobi(ins, info);
        };
#endif
#if COINOR_FOUND
    } else if (argv[0] == "lagrelax_knapsack_volume") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            return lb_lagrelax_knapsack_volume(ins, info);
        };
#endif
#if COINOR_FOUND
    } else if (argv[0] == "lagrelax_knapsack_bundle") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            return lb_lagrelax_knapsack_bundle(ins, info);
        };
#endif
#if DLIB_FOUND
    } else if (argv[0] == "lagrelax_knapsack_lbfgs") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            return lb_lagrelax_knapsack_lbfgs(ins, info);
        };
#endif
#if COINOR_FOUND
    } else if (argv[0] == "lagrelax_assignment_volume") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            return lb_lagrelax_assignment_volume(ins, info);
        };
#endif
#if COINOR_FOUND
    } else if (argv[0] == "lagrelax_assignment_bundle") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            return lb_lagrelax_assignment_bundle(ins, info);
        };
#endif
#if DLIB_FOUND
    } else if (argv[0] == "lagrelax_assignment_lbfgs") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            LagRelaxAssignmentLbfgsOptionalParameters p;
            p.info = info;
            return lb_lagrelax_assignment_lbfgs(ins, p);
        };
#endif
    } else if (argv[0] == "columngeneration") {
        return [argv](Instance& ins, std::mt19937_64&, Info info) {
            ColGenOptionalParameters p;
            p.info = info;
            for (auto it = argv.begin() + 1; it != argv.end(); ++it)
                if (*it == "solver") { p.solver = *(++it); }
            return lb_columngeneration(ins, p);
        };

    /*
     * Exact algorithms
     */
#if COINOR_FOUND
    } else if (argv[0] == "branchandcut_cbc") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            BranchAndCutCbcOptionalParameters p;
            p.info = info;
            return sopt_branchandcut_cbc(ins, p);
        };
#endif
#if CPLEX_FOUND
    } else if (argv[0] == "branchandcut_cplex") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            BranchAndCutCplexOptionalParameters p;
            p.info = info;
            return sopt_branchandcut_cplex(ins, p);
        };
#endif
#if GUROBI_FOUND
    } else if (argv[0] == "branchandcut_gurobi") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            BranchAndCutGurobiOptionalParameters p;
            p.info = info;
            return sopt_branchandcut_gurobi(ins, p);
        };
#endif
    } else if (argv[0] == "branchandprice_dfs") {
        return [argv](Instance& ins, std::mt19937_64&, Info info) {
            BranchAndPriceOptionalParameters p;
            p.info = info;
            p.set_params(argv);
            return sopt_branchandprice_dfs(ins, p);
        };
    } else if (argv[0] == "branchandprice_astar") {
        return [argv](Instance& ins, std::mt19937_64&, Info info) {
            BranchAndPriceOptionalParameters p;
            p.info = info;
            p.set_params(argv);
            return sopt_branchandprice_astar(ins, p);
        };
#if GECODE_FOUND
    } else if (argv[0] == "constraintprogramming_gecode") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            ConstraintProgrammingGecodeOptionalParameters p;
            p.info = info;
            return sopt_constraintprogramming_gecode(ins, p);
        };
#endif
#if CPLEX_FOUND
    } else if (argv[0] == "constraintprogramming_cplex") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            ConstraintProgrammingCplexOptionalParameters p;
            p.info = info;
            return sopt_constraintprogramming_cplex(ins, p);
        };
#endif

    /*
     * Upper bounds
     */
    } else if (argv[0] == "random") {
        return [](Instance& ins, std::mt19937_64& gen, Info info) {
            return sol_random(ins, gen, info);
        };
    } else if (argv[0] == "greedy") {
        return [argv](Instance& ins, std::mt19937_64&, Info info) {
            auto it = std::find(argv.begin(), argv.end(), "f");
            std::string des_str = (it == argv.end())? "cij": *(++it);
            std::unique_ptr<Desirability> f = desirability(des_str, ins);
            return sol_greedy(ins, *f, info);
        };
    } else if (argv[0] == "greedyregret") {
        return [argv](Instance& ins, std::mt19937_64&, Info info) {
            auto it = std::find(argv.begin(), argv.end(), "f");
            std::string des_str = (it == argv.end())? "cij": *(++it);
            std::unique_ptr<Desirability> f = desirability(des_str, ins);
            return sol_greedyregret(ins, *f, info);
        };
    } else if (argv[0] == "mthg") {
        return [argv](Instance& ins, std::mt19937_64&, Info info) {
            auto it = std::find(argv.begin(), argv.end(), "f");
            std::string des_str = (it == argv.end())? "cij": *(++it);
            std::unique_ptr<Desirability> f = desirability(des_str, ins);
            return sol_mthg(ins, *f, info);
        };
    } else if (argv[0] == "mthgregret") {
        return [argv](Instance& ins, std::mt19937_64&, Info info) {
            auto it = std::find(argv.begin(), argv.end(), "f");
            std::string des_str = (it == argv.end())? "cij": *(++it);
            std::unique_ptr<Desirability> f = desirability(des_str, ins);
            return sol_mthgregret(ins, *f, info);
        };
#if COINOR_FOUND
    } else if (argv[0] == "repaircombrelax") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            return sol_repaircombrelax(ins, info);
        };
#endif
#if COINOR_FOUND
    } else if (argv[0] == "repairgreedy") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            return sol_repairgreedy(ins, info);
        };
#endif
#if COINOR_FOUND
    } else if (argv[0] == "repairlinrelax_clp") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            LinRelaxClpOutput linrelax_output = lb_linrelax_clp(ins);
            return sol_repairlinrelax_clp(ins, linrelax_output, info);
        };
#endif
    } else if (argv[0] == "ls_shiftswap") {
        return [argv](Instance& ins, std::mt19937_64& gen, Info info) {
            LSShiftSwapOptionalParameters p;
            p.info = info;
            return sol_ls_shiftswap(ins, gen, p);
        };
    } else if (argv[0] == "ts_shiftswap") {
        return [argv](Instance& ins, std::mt19937_64& gen, Info info) {
            TSShiftSwapOptionalParameters p;
            p.info = info;
            p.set_params(argv);
            return sol_ts_shiftswap(ins, gen, p);
        };
    } else if (argv[0] == "sa_shiftswap") {
        return [argv](Instance& ins, std::mt19937_64& gen, Info info) {
            SAShiftSwapOptionalParameters p;
            p.info = info;
            p.set_params(argv);
            return sol_sa_shiftswap(ins, gen, p);
        };
#if LOCALSOLVER_FOUND
    } else if (argv[0] == "localsolver") {
        return [](Instance& ins, std::mt19937_64&, Info info) {
            LocalSolverOptionalParameters p;
            p.info = info;
            return sol_localsolver(ins, p);
        };
#endif
    } else if (argv[0] == "cgh_restrictedmaster") {
        return [argv](Instance& ins, std::mt19937_64&, Info info) {
            CghRestrictedMasterOptionalParameters p;
            p.info = info;
            p.set_params(argv);
            return sol_cgh_restrictedmaster(ins, p);
        };
    } else if (argv[0] == "cgh_purediving") {
        return [argv](Instance& ins, std::mt19937_64&, Info info) {
            CghPureDivingOptionalParameters p;
            p.info = info;
            p.set_params(argv);
            return sol_cgh_purediving(ins, p);
        };
    } else if (argv[0] == "cgh_divingwithlds") {
        return [argv](Instance& ins, std::mt19937_64&, Info info) {
            CghDivingWithLdsOptionalParameters p;
            p.info = info;
            p.set_params(argv);
            return sol_cgh_divingwithlds(ins, p);
        };


    } else {
        std::cerr << "\033[31m" << "ERROR, unknown algorithm: " << argv[0] << "\033[0m" << std::endl;
        assert(false);
        return [](Instance& ins, std::mt19937_64&, Info info) { return Output(ins, info); };
    }
}

