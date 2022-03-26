# Generalized Assignment Solver

A solver for the Generalized Assignment Problem.

This problem is interesting because many different optimization methods can and have been applied to solve it (Branch-and-cut, Branch-and-price, Branch-and-relax, Local search, Constraint programming, Column generation heuristics...). Thus, the main goal of this repository is for me to have reference implementations for classical algorithms and optimization solvers.

The variant handled here is the variant:
* with a minimization objective (items have costs)
* where all items have to be assigned

It is possible to solve the variant with a maximization objective (items have profits) by setting the cost of assigning item an `j` to agent an `i` to `maximum profit of item j - profit of assigning item j to agent i`.

It is possible to solve the variant where not all items have to be assigned by adding an additional dummy agent `i` such that the weight of assigning an item `j` to this agent `i` is equal to `0` and the cost of assigning an item `j` to this agent `i` is equal to `maximum profit of item j`

## Implemented algorithms

### Lower bounds

- Linear relaxation
  - solved with CLP `-a linrelax_clp`
  - solved with Gurobi `-a "milp_gurobi --only-linear-relaxation"`
  - solved with Cplex `-a "milp_cplex --only-linear-relaxation"`

- Lagrangian relaxation of knapsack constraints. The value of this relaxation is the same as the value of the linear relaxation. However, it might be cheaper to compute, especially on large instances.
  - solved with volume method `-a lagrelax_knapsack_volume`
  - solved with L-BFGS method `-a lagrelax_knapsack_lbfgs`

- Lagrangian relaxation of assignment constraints
  - solved with volume method `-a lagrelax_assignment_volume`
  - solved with L-BFGS method `-a lagrelax_assignment_lbfgs`

- Column generation `-a "columngeneration --linear-programming-solver clp"` `-a "columngeneration --linear-programming-solver cplex"`

### Upper bounds

- Polynomial algorithms from "Generalized Assignment Problems" (Martello et al., 1992), options `-f cij` `-f wij` `-f cij*wij` `-f -pij/wij` `-f wij/ti`:
  - Basic greedy `-a "greedy -f wij"`
  - Greedy with regret measure `-a "greedyregret -f wij"`
  - MTHG, basic greedy (+ n shifts) `-a "mthg -f wij"`
  - MTHG, greedy with regret measure (+ n shifts) `-a "mthgregret -f wij"`

- Local search algorithm implemented with [fontanf/localsearchsolver](https://github.com/fontanf/localsearchsolver) `-a "localsearch --threads 3"`

- Tree search algorithms based on the Dantzig-Wolfe reformulation branching scheme (i.e. column generation heuristics) implemented with [fontanf/columngenerationsolver](https://github.com/fontanf/columngenerationsolver):
  - Greedy `-a "columngenerationheuristic_greedy --linear-programming-solver cplex"`
  - Limited discrepency search `-a "columngenerationheuristic_limiteddiscrepancysearch --linear-programming-solver cplex"`

- Others heuristics:
  - Random feasible solution found with a Local search `-a random`
  - Local search with LocalSolver `-a localsolver`

### Exact algorithms

- Mixed-Integer Linear Programs
  - with CBC `-a milp_cbc`
  - with CPLEX `-a milp_cplex`
  - with Gurobi `-a milp_gurobi`

- Constraint programming
  - with Gecode `-a constraintprogramming_gecode`
  - with CPLEX `-a constraintprogramming_cplex`

## Usage (command line)

The only required dependency is Boost:
```shell
sudo apt-get install libboost-all-dev
```

Compile:
```shell
bazel build -- //...
```

However, most algorithms require additional libraries (see below).
Compile with additional libraries (or just uncomment the corresponding lines in `.bazelrc`):
```shell
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
```shell
./bazel-bin/generalizedassignmentsolver/main -v -a 'mthg -f -pij/wij' -i "data/chu1997/a05100" -o "a05100_output.json" -c "a05100_solution.txt"
```
```
=====================================
    Generalized Assignment Solver    
=====================================

Instance
--------
Number of items:    100
Number of agents:   5

Algorithm
---------
MTHG

Parameters
----------
Desirability:  -pij/wij

     T (s)            UB            LB           GAP   GAP (%)                 Comment
     -----            --            --           ---   -------                 -------
         0           inf          1694           inf       inf                        
    0.0001          1713          1694            19      1.12                        

Final statistics
----------------
Value:                    1713
Bound:                    1694
Gap:                      19
Gap (%):                  1.12
Time (s):                 0.0001
```

Unit tests:
```shell
bazel test --compilation_mode=dbg -- //...
```

Checker:
```shell
./bazel-bin/generalizedassignmentsolver/checker data/a05100 output/best/a05100_solution.txt
```

Run benchmarks:
```shell
python3 ../optimizationtools/optimizationtools/bench_run.py --algorithms "mthg -f cij" "mthg -f wij" "mthg -f cij*wij" "mthg -f -pij/wij" "mthg -f wij/ti" "mthgregret -f cij" "mthgregret -f wij" "mthgregret -f cij*wij" "mthgregret -f -pij/wij" "mthgregret -f wij/ti" "random"
python3 ../optimizationtools/optimizationtools/bench_process.py --benchmark heuristicshort --labels "mthg -f cij" "mthg -f wij" "mthg -f cij*wij" "mthg -f -pij/wij" "mthg -f wij/ti" "mthgregret -f cij" "mthgregret -f wij" "mthgregret -f cij*wij" "mthgregret -f -pij/wij" "mthgregret -f wij/ti" "random" --timelimit 0.1
```

![heuristicshort](img/heuristicshort.png?raw=true "heuristicshort")

## Optional dependencies

To enable a dependency, uncomment the corresponding line in the `.bazelrc` file and adapt its path in the `WORKSPACEi` file.

Here are some notes for their installations:

### COIN-OR (CLP, CBC, VOL, DIP)

Install (https://coin-or.github.io/coinbrew/):
```shell
git clone https://www.github.com/coin-or/coinbrew
cd coinbrew
./coinbrew fetch build Vol --no-prompt
```

### Gecode

Download latest version: download https://www.gecode.org/download.html

Compile (more info https://www.gecode.org/doc/2.2.0/reference/PageComp.html):
```shell
./configure
make
```

### Gurobi

After installing, execute the following commands:
```shell
cd ${GUROBI_HOME}/linux64/src/build/
make
cp libgurobi_c++.a ../../lib/
```

