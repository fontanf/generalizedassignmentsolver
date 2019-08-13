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

The first goal of this repository is for me to have reference implementations for classical algorithms and solvers. I'll also try to implement state of the art algorithms and provide free implementations.

## Lower bounds

- Linear relaxation
  - solved with CLP `-a linrelax_clp` :heavy_check_mark:
  - solved with Gurobi `-a linrelax_gurobi` :heavy_check_mark:
- Lagrangian relaxation of knapsack constraints
  - solved with volume method `-a lagrelax_knapsack_volume` :heavy_check_mark:
  - solved with L-BFGS method `-a lagrelax_knapsack_lbfgs` :heavy_check_mark:
- Lagrangian relaxation of assignment constraints
  - solved with volume method `-a lagrelax_assignment_volume` :heavy_check_mark:
  - solved with L-BFGS method `-a lagrelax_assignment_lbfgs` :heavy_check_mark:

## Upper bounds

Polynomial algorithms (options `f cij` `f wij` `f cij*wij` `f -pij/wij` `f wij/ti`):
- Basic greedy `-a "greedy f wij` :heavy_check_mark:
- Greedy with regret measure `-a "greedyregret f wij"` :heavy_check_mark:
- MTHG, basic greedy (+ n shifts) `-a "mthg f wij"` :heavy_check_mark:
- MTHG, greedy with regret measure (+ n shifts) `-a "mthgregret f wij"` :heavy_check_mark:

Classical meta-heuristics based on shift-swap neighborhood and fixed penalty of capacity constraint violation:
- Hill climbing, first improvment `-a lsfirst_shiftswap` :heavy_check_mark:
- Hill climbing, best improvment `-a lsbest_shiftswap` :heavy_check_mark:
- Tabu search `-a ts_shiftswap` :heavy_check_mark:
- Simulated annealing `-a sa_shiftswap` :heavy_check_mark:
- Path relinking `-a pr_shiftswap` :heavy_check_mark:

Others heuristics and meta-heuristics:
- Random feasible solution `-a random` :heavy_check_mark:
- Repair linear relaxation solution `-a repairlinrelax_clp` :heavy_check_mark:
- Variable neighborhood branching (see "Handbook of Metaheuristics", 3.6.1 Variable Neighborhood Branching)
  - with CBC `-a vnsbranching_cbc` :heavy_check_mark:
  - with CPLEX `-a vnsbranching_cplex` :heavy_check_mark:
- Variable-depth neighborhood search (see "Handbook of Metaheuristics", 4.5.1 Variable-Depth Methods)
  - k-agents reallocation neighborhood `-a vdns_simple` :heavy_check_mark:
- Local search with LocalSolver `-a localsolver` :heavy_check_mark:

## Exact algorithms

- Branch-and-cut
  - with CBC `-a branchandcut_cbc` :heavy_check_mark:
  - with CPLEX `-a branchandcut_cplex` :heavy_check_mark:
  - with Gurobi `-a branchandcut_gurobi` :heavy_check_mark:
- Constraint programming
  - with Gecode `-a constraintprogramming_gecode` :heavy_check_mark:
  - with CPLEX `-a constraintprogramming_cplex` :heavy_check_mark:

## Results

### Linear relaxation gap

The largest gap between the lower bound from the linear relaxation and the best known upper bound is 1.93%.

### Lagrangian relaxation implementation

The bound from the lagrangian relaxation of knapsack constraints is theoritically equal to the bound from the linear relaxation. The optimal bounds are found by the L-BGFS algorithm from DLib; however, the volume method stays rather far from it. 

The same happens for the bound obtained by solving the lagrangian relaxation of assignment constraints; the Volume method give poor results while the L-BFGS algorithm returns the best known ones found in "An exact method with variable fixing for solving the generalized assignment problem" (Posta et al., 2011).

### Others

`vdns_simple` does not compete with state of the art meta-heuristics like the ones presented in "A path relinking approach with ejection chains for the generalized assignment problem" (Yagiura et al., 2006) or "Variable-fixing then subgradient optimization guided very large scale neighborhood search for the generalized assignment problem" (Haddadi, 2018) in terms of solution quality on long runs. However:
- on short runs (2 minutes, Processor Intel® Core™ i5-8500 CPU @ 3.00GHz × 6), it provides solutions of good quality (less than 1% gap from optimal for all instances of the literature, and less than 0.5% for instances with more than 900 items)
- it is very simple and the implementation is very short
- it is available and free (MIT License)

## Usage (command line)

The only required dependency is Boost:
```shell
sudo apt-get install libboost-all-dev
```

Compile:
```
bazel build -- //...
```

However, most algorithms require additional libraries (see below).
Compile with additional libraries (or just uncomment the corresponding lines in `.bazelrc`):
```
bazel build \
    --define coinor=true \
    --define cplex=true \
    --define gurobi=true \
    --define gecode=true \
    --define dlib=true \
    --define localsolver=true \
    -- //...
```

Solve:
```
./bazel-bin/lib/main -v -a 'mthg f -pij/wij' -i "data/a05100" -o "out.ini" -c "sol.txt"
```

Unit tests:
```
bazel test --compilation_mode=dbg -- //...
```

Checker:
```
bazel run -- //lib:checker                # show best bounds for all instances
bazel run -- //lib:checker "data/a05100"  # show best bounds for one instance
bazel build -- //lib:checker              # check another solution file
./bazel-bin/lib/checker "instancefile" "solutionfile"
```

Run benchmarks (results stored in `out/algorithm/`)
```
bazel run -- //lib:bench "mthg f wij"             # no time limit
bazel run -- //lib:bench "branchandcut_cbc" 7200  # 2h time limit
```
Output files can then be retrieved from `bazel-out/k8-opt/bin/lib/bench.runfiles/__main__/`.

## Optional dependencies

### COIN-OR (CLP, CBC, VOL, DIP)

Install (https://coin-or.github.io/coinbrew/):
```shell
git clone https://www.github.com/coin-or/coinbrew
cd coinbrew
./coinbrew fetch build Dip --no-prompt
./coinbrew fetch build Vol --no-prompt
```

Update `.bashrc`:
```shell
# COIN-OR
export COINOR_HOME="/home/florian/Programmes/coinbrew"
export PATH="${PATH}:${COINOR_HOME}/build/bin"
export CPLUS_INCLUDE_PATH="${CPLUS_INCLUDE_PATH}:${COINOR_HOME}/build/include"
```

Create symlinks for libraries:
```shell
LIB_DIR="/usr/lib/x86_64-linux-gnu/"
for i in "${COINOR_HOME}"/build/lib/*.so*; do sudo ln -f -s "$i" "${LIB_DIR}" done
```

### Gecode

Download latest version: download https://www.gecode.org/download.html

Compile (more info https://www.gecode.org/doc/2.2.0/reference/PageComp.html):
```shell
./configure
make
```

Update `.bashrc`:
```shell
# Gecode
export GECODE_HOME="/opt/gecode-release-6.2.0"
export CPLUS_INCLUDE_PATH="${CPLUS_INCLUDE_PATH}:${GECODE_HOME}"
```

Create symlinks for libraries:
```shell
LIB_DIR="/usr/lib/x86_64-linux-gnu/"
for i in "${GECODE_HOME}"/*.so*; do sudo ln -f -s "$i" "${LIB_DIR}" done
```

### DLib

Download and compile
```shell
git clone https://github.com/davisking/dlib.git
cd dlib
mkdir build; cd build; cmake .. ; cmake --build .
```

Update `.bashrc`:
```shell
# DLib
export DLIB_HOME="/home/florian/Programmes/dlib"
export CPLUS_INCLUDE_PATH="${CPLUS_INCLUDE_PATH}:${DLIB_HOME}"
```

Create symlinks for libraries:
```shell
LIB_DIR="/usr/lib/x86_64-linux-gnu/"
sudo ln -f -s "${DLIB_HOME}/build/dlib/libdlib.a" "${LIB_DIR}"
```

### CPLEX

Update `.bashrc`:
```shell
# CPlex
export CPLEX_VERSION="126"
export CPLEX_HOME="/opt/ibm/ILOG/CPLEX_Studio${CPLEX_VERSION}"
export PATH="$PATH:${CPLEX_HOME}/cplex/bin/x86-64_linux"
export CLASSPATH="$CLASSPATH:${CPLEX_HOME}/cplex/lib/cplex.jar"
export CPLUS_INCLUDE_PATH="$CPLUS_INCLUDE_PATH:${CPLEX_HOME}/cplex/include"
export CPLUS_INCLUDE_PATH="$CPLUS_INCLUDE_PATH:${CPLEX_HOME}/concert/include"
export CPLUS_INCLUDE_PATH="$CPLUS_INCLUDE_PATH:${CPLEX_HOME}/cpoptimizer/include"
```

Create symlinks for libraries:
```shell
LIB_DIR="/usr/lib/x86_64-linux-gnu/"
sudo ln -f -s "${CPLEX_HOME}/cplex/lib/x86-64_linux/static_pic/libcplex.a"        "${LIB_DIR}"
sudo ln -f -s "${CPLEX_HOME}/cplex/lib/x86-64_linux/static_pic/libilocplex.a"     "${LIB_DIR}"
sudo ln -f -s "${CPLEX_HOME}/cplex/lib/x86-64_linux/static_pic/libcplexdistmip.a" "${LIB_DIR}"
sudo ln -f -s "${CPLEX_HOME}/concert/lib/x86-64_linux/static_pic/libconcert.a"    "${LIB_DIR}"
sudo ln -f -s "${CPLEX_HOME}/cpoptimizer/lib/x86-64_linux/static_pic/libcp.a"     "${LIB_DIR}"
```

### Gurobi

Update `.bashrc`:
```shell
# Gurobi
export GUROBI_HOME="/home/florian/Programmes/gurobi811"
export GRB_LICENSE_FILE=/home/florian/gurobi.lic
export PATH="${PATH}:${GUROBI_HOME}/linux64/bin"
export CPLUS_INCLUDE_PATH="${CPLUS_INCLUDE_PATH}:${GUROBI_HOME}/linux64/include"
```

Execute the following commands:
```shell
cd ${GUROBI_HOME}/linux64/src/build/
make
cp libgurobi_c++.a ../../lib/
```

Create symlinks for libraries:
```shell
LIB_DIR="/usr/lib/x86_64-linux-gnu/"
sudo ln -f -s "${GUROBI_HOME}/linux64/lib/libgurobi_c++.a" "${LIB_DIR}"
sudo ln -f -s "${GUROBI_HOME}/linux64/lib/libgurobi81.so"  "${LIB_DIR}"
```

### LocalSolver

Update `.bashrc`:
```shell
# LocalSolver
export LOCALSOLVER_HOME="/opt/localsolver_8_5"
export CPLUS_INCLUDE_PATH="${CPLUS_INCLUDE_PATH}:${LOCALSOLVER_HOME}/include"
```

Create symlinks for libraries:
```shell
LIB_DIR="/usr/lib/x86_64-linux-gnu/"
for i in "${LOCALSOLVER_HOME}"/bin/*.so*; do sudo ln -f -s "$i" "${LIB_DIR}"; done
```

