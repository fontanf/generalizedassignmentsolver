# GeneralizedAssignmentSolver

A solver for the generalized assignment problem.

This problem is interesting because many different optimization methods can and have been applied to solve it (Branch-and-cut, Branch-and-price, Branch-and-relax, Local search, Constraint programming, Column generation heuristics...). Thus, the main goal of this repository is for me to have reference implementations for classical algorithms and optimization solvers.

The variant handled here is the variant:
* with a minimization objective (items have costs)
* where all items have to be assigned

It is possible to solve the variant with a maximization objective (items have profits) by setting the cost of assigning item an `j` to agent an `i` to `maximum profit of item j - profit of assigning item j to agent i`.

It is possible to solve the variant where not all items have to be assigned by adding an additional dummy agent `i` such that the weight of assigning an item `j` to this agent `i` is equal to `0` and the cost of assigning an item `j` to this agent `i` is equal to `maximum profit of item j`

## Implemented algorithms

### Lower bounds

- Linear relaxation
  - solved with CLP `-a linrelax-clp`
  - solved with Gurobi `-a "milp-gurobi --only-linear-relaxation"`
  - solved with Cplex `-a "milp-cplex --only-linear-relaxation"`

- Lagrangian relaxation of knapsack constraints. The value of this relaxation is the same as the value of the linear relaxation. However, it might be cheaper to compute, especially on large instances.
  - solved with volume method `-a lagrelax-knapsack-volume`
  - solved with L-BFGS method `-a lagrelax-knapsack-lbfgs`

- Lagrangian relaxation of assignment constraints
  - solved with volume method `-a lagrelax-assignment-volume`
  - solved with L-BFGS method `-a lagrelax-assignment-lbfgs`

- Column generation `-a "column-generation --linear-programming-solver clp"` `-a "column-generation --linear-programming-solver cplex"`

### Upper bounds

- Polynomial algorithms from "Generalized Assignment Problems" (Martello et al., 1992), options `--desirability cij` `--desirability wij` `--desirability cij*wij` `--desirability -pij/wij` `--desirability wij/ti`:
  - Basic greedy `-a "greedy --desirability wij"`
  - Greedy with regret measure `-a "greedy-regret --desirability wij"`
  - MTHG, basic greedy (+ n shifts) `-a "mthg --desirability wij"`
  - MTHG, greedy with regret measure (+ n shifts) `-a "mthg-regret --desirability wij"`

- Local search algorithm implemented with [fontanf/localsearchsolver](https://github.com/fontanf/localsearchsolver) `-a "local-search --threads 3"`

- Tree search algorithms based on the Dantzig-Wolfe reformulation branching scheme (i.e. column generation heuristics) implemented with [fontanf/columngenerationsolver](https://github.com/fontanf/columngenerationsolver):
  - Greedy `-a "column-generation-heuristic-greedy --linear-programming-solver cplex"`
  - Limited discrepency search `-a "column-generation-heuristic-limited-discrepancy-search --linear-programming-solver cplex"`

- Others heuristics:
  - Random feasible solution found with a Local search `-a random`
  - Local search with LocalSolver `-a localsolver`

### Exact algorithms

- Mixed-Integer Linear Programs
  - with CBC `-a milp-cbc`
  - with CPLEX `-a milp-cplex`
  - with Gurobi `-a milp-gurobi`
  - with Knitro `-a milp-knitro`

- Constraint programming
  - with Gecode `-a constraint-programming-gecode`
  - with CPLEX `-a constraint-programming-cplex`

## Usage (command line)

Compile:
```shell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel
cmake --install build --config Release --prefix install
```

Solve:
```shell
./install/bin/generalizedassignmentsolver --verbosity-level 1 --algorithm mthg --desirability "-pij/wij" --input "data/chu1997/a05100" --output "a05100_output.json" --certificate "a05100_solution.txt"
```
```
=====================================
     GeneralizedAssignmentSolver     
=====================================

Instance
--------
Number of agents:  5
Number of items:   100
Total cost:        15634
Maximum cost:      50
Maximum weight:    25

Algorithm
---------
MTHG

Parameters
----------
Time limit:            inf
Messages
    Verbosity level:   1
    Standard output:   1
    File path:         
    # streams:         0
Logger
    Has logger:        0
    Standard error:    0
    File path:         

    Time (s)       Value       Bound         Gap     Gap (%)                 Comment
    --------       -----       -----         ---     -------                 -------
       0.000         inf        1694         inf         inf                        
       0.001        1713        1694          19        1.12                        

Final statistics
----------------
Value:                        1713
Bound:                        1694
Absolute optimality gap:      19
Relative optimality gap (%):  1.12161
Time (s):                     0.0082436

Solution
--------
Number of items:  100 / 100 (100%)
Feasible:         1
Cost:             1713
```

Checker:
```shell
./install/bin/generalizedassignmentsolver_checker data/chu1997/a05100 a05100_solution.txt
```
