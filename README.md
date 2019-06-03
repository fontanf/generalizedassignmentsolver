WORK IN PROGRESS

# Generalized Assignment Problem

Algorithm implementations for the Generalized Assignment Problem.

## Description

The Generalized Assignment Problem (GAP) is the problem of assigning n jobs to m agent at minimum cost:
- for each agent i, each job j has associated cost cij and weight wij
- each agent i has a capacity ti
- each job must be assigned to exactly one agent
- the sum of the weights of jobs assigned to an agent must not overcome its capacity
- the total cost of the assignment must be minimized

GAP is interesting for several reasons:
- it is simple (to state, to understand...)
- it is difficult to solve, i.e. it is NP-complete, and there are several instances of reasonable size that are not solved exactly (unlike for example the Knapsack Problem, which is also NP-complete, but all instances of the literature are solved exactly by state-of-the-art algorithms)
- the size of input data is not very large: O(nm) (unlike for example TSP, which input size is O(n^2))
- it appears as subproblem of many practical applications
- it is structured for simple local search algorithms, i.e. the basic shift-swap neighbourhood makes it possible to implement simulated annealing, tabu search or other classical meta-heuristics (unlike for example the roadef2018 problem, which has no simple neighbourhoods)
- it is also structured for decomposition techniques (Lagrangian relaxations, Column generation...)
- it is well-studied, i.e. researchers have proposed many and various algorithms to solve it (branch-and-bound, cutting plane, large neighbourhood search, path relinking...)

## Technical informations

Dependencies:
- boost `sudo apt-get install libboost-all-dev`
- CLP, CBC, VOL `sudo apt-get install coinor-lib*-dev`
- DIP: https://bintray.com/coin-or/download/Dip you may need to link DIP libs somewhere Bazel will find them:
```
DIP_HOME="/opt/Dip-master-linux-x86_64-gcc4.8"
LIB_DIR="/usr/lib/x86_64-linux-gnu/"
sudo ln -s "${DIP_HOME}/lib/libDecomp.so" "${LIB_DIR}"
```
- CPLEX (optional): you may need to link CPLEX libs somewhere Bazel will find them:
```
CPLEX_HOME="/opt/ibm/ILOG/CPLEX_Studio126"
LIB_DIR="/usr/lib/x86_64-linux-gnu/"
sudo ln -s "${CPLEX_HOME}/cplex/lib/x86-64_linux/static_pic/libcplex.a" "${LIB_DIR}"
sudo ln -s "${CPLEX_HOME}/cplex/lib/x86-64_linux/static_pic/libcp.a" "${LIB_DIR}"
sudo ln -s "${CPLEX_HOME}/cplex/lib/x86-64_linux/static_pic/libcplexdistmip.a" "${LIB_DIR}"
sudo ln -s "${CPLEX_HOME}/concert/lib/x86-64_linux/static_pic/libconcert.a" "${LIB_DIR}"
sudo ln -s "${CPLEX_HOME}/cpoptimizer/lib/x86-64_linux/static_pic/libcp.a" "${LIB_DIR}"
```

Compile:
```
bazel build --cxxopt='-std=c++14' --compilation_mode=opt -- //...
```

Solve:
```
./bazel-bin/lib/main -v -a branchandcut_cbc -i data/a05100 -o out.ini -c sol.txt
```

Unit tests:
```
bazel test --cxxopt='-std=c++14' --compilation_mode=opt -- //...
```

## Lower bounds

- Linear relaxation solved with CLP `-a linrelax_clp` :heavy_check_mark:
- Lagrangian relaxation of knapsack constraints
  - solved with volume method `-a lagrelax_knapsack_volume` :heavy_check_mark:
  - solved with bundle method `-a lagrelax_knapsack_bundle` :x:
- Lagrangian relaxation of assignment constraints
  - solved with volume method `-a lagrelax_assignment_volume` :heavy_check_mark:
  - solved with bundle method `-a lagrelax_assignment_bundle` :x:

## Upper bounds

Polynomial algorithms:
- Basic greedy:
  - `-a "greedy f cij"` :heavy_check_mark: `-a "greedy f wij"` :heavy_check_mark:
  - `-a "greedy f cij*wij"` :heavy_check_mark: `-a "greedy f -pij/wij"` :heavy_check_mark: `-a "greedy f wij/tij"` :heavy_check_mark:
- Greedy with regret measure:
  - `-a "greedyregret f cij"` :heavy_check_mark: `-a "greedyregret f wij"` :heavy_check_mark:
  - `-a "greedyregret f cij*wij"` :heavy_check_mark: `-a "greedyregret f -pij/wij"` :heavy_check_mark: `-a "greedyregret f wij/tij"` :heavy_check_mark:
- MTHG, basic greedy (+ n shifts):
  - `-a "mthg f cij"` :heavy_check_mark: `-a "mthg f wij"` :heavy_check_mark:
  - `-a "mthg f cij*wij"` :heavy_check_mark: `-a "mthg f -pij/wij"` :heavy_check_mark: `-a "mthg f wij/tij"` :heavy_check_mark:
- MTHG, greedy with regret measure (+ n shifts):
  - `-a "mthgregret f cij"` :heavy_check_mark: `-a "mthgregret f wij"` :heavy_check_mark:
  - `-a "mthgregret f cij*wij"` :heavy_check_mark: `-a "mthgregret f -pij/wij"` :heavy_check_mark: `-a "mthgregret f wij/tij"` :heavy_check_mark:

Classical meta-heuristics based on shift-swap neighborhood and fixed penalty of capacity constraint violation:
- Hill climbing, first improvment `-a lsfirst_shiftswap` :heavy_check_mark:
- Hill climbing, best improvment `-a lsbest_shiftswap` :heavy_check_mark:
- Tabu search `-a ts_shiftswap` :heavy_check_mark:
- Simulated annealing `-a sa_shiftswap` :heavy_check_mark:
- Path relinking `-a pr_shiftswap` :heavy_check_mark:

Others:
- Random initial solution `-a random` :heavy_check_mark:
- Repair linear relaxation solution `-a repairlinrelax` :heavy_check_mark:
- Variable Neighborhood Branching (see "Handbook of Metaheuristics", 3.6.1 Variable Neighborhood Branching)
  - with CBC `-a vnsbranching_cbc` :heavy_check_mark:
  - with CPLEX `-a vnsbranching_cplex` :heavy_check_mark:
- Variable-Depth Neighborhood Search (see "Handbook of Metaheuristics", 4.5.1 Variable-Depth Methods)
  - k-agents reallocation neighborhood `-a vdns_simple` :heavy_check_mark:
- Very Large Scale Neighborhood Search using Monotone Binary Program Neighborhood, based on "Variable-fixing then subgradient optimization guided very large scale neighborhood search for the generalized assignment problem" (Haddadi, 2018) `-a vlsn_mbp` :x:

## Exact algorithms

- Branch-and-cut
  - with CBC `-a branchandcut_cbc` :heavy_check_mark:
  - with CPLEX `-a branchandcut_cplex` :heavy_check_mark:
- Branch-and-price :x:

## Results

The largest gap between the lower bound from the linear relaxation and the best known upper bound is 1.93%.

The bound from the lagrangian relaxation of knapsack constraints is theoritically equal to the bound from the linear relaxation. However, here, the bound obtain by solving the lagrangian relaxation by the Volume method is rather far from it.

The bound obtained by solving the lagrangian relaxation of assignment constraints by the Volume method is not as good as the value obtained in the literature.

`vdns_simple` does not compete with state of the art meta-heuristics like the ones presented in "A path relinking approach with ejection chains for the generalized assignment problem" (Yagiura, 2006) or "Variable-fixing then subgradient optimization guided very large scale neighborhood search for the generalized assignment problem" (Haddadi, 2018) in terms of solution quality on long runs. However:
- on short runs (2 minutes, Processor Intel® Core™ i5-8500 CPU @ 3.00GHz × 6), it provides solutions of good quality (less than 1% gap from optimal for all instances of the literature, and less than 0.5% for instances with more than 900 items)
- it is very simple and the implementation is very short
- it is available and free (MIT License)

