# GeneralizedAssignmentSolver

A solver for the generalized assignment problem.

The variant handled here is the variant:
* with a minimization objective (items have costs)
* where all items have to be assigned

It is possible to solve the variant with a maximization objective (items have profits) by setting the cost of assigning item an `j` to agent an `i` to `maximum profit of item j - profit of assigning item j to agent i`.

It is possible to solve the variant where not all items have to be assigned by adding an additional dummy agent `i` such that the weight of assigning an item `j` to this agent `i` is equal to `0` and the cost of assigning an item `j` to this agent `i` is equal to `maximum profit of item j`

## Implemented algorithms

- Polynomial algorithms from "Generalized Assignment Problems" (Martello et al., 1992), options `--desirability cij` `--desirability wij` `--desirability cij*wij` `--desirability -pij/wij` `--desirability wij/ti`:
  - Basic greedy `--algorithm "greedy --desirability wij"`
  - Greedy with regret measure `--algorithm "greedy-regret --desirability wij"`
  - MTHG, basic greedy (+ n shifts) `--algorithm "mthg --desirability wij"`
  - MTHG, greedy with regret measure (+ n shifts) `--algorithm "mthg-regret --desirability wij"`

- Mixed-Integer Linear Program `--algorithm milp --solver highs`

<!--- Constraint programming-->
<!--  - with Gecode `--algorithm constraint-programming-gecode`-->

- Lagrangian relaxation
  - of knapsack constraints. The value of this relaxation is the same as the value of the linear relaxation. However, it might be cheaper to compute, especially on large instances.
    - solved with dlib `--algorithm lagrangian-relaxation-knapsack-dlib`
  - of assignment constraints
    - solved with dlib `--algorithm lagrangian-relaxation-assignment-dlib`

- Local search algorithm implemented with [fontanf/localsearchsolver](https://github.com/fontanf/localsearchsolver) `--algorithm "local-search --threads 3"`

- Tree search algorithms based on the Dantzig-Wolfe reformulation branching scheme (i.e. column generation heuristics) implemented with [fontanf/columngenerationsolver](https://github.com/fontanf/columngenerationsolver):
  - Column generation `--algorithm column-generation --linear-programming-solver highs`
  - Greedy `--algorithm column-generation-heuristic-greedy --linear-programming-solver highs`
  - Limited discrepency search `--algorithm column-generation-heuristic-limited-discrepancy-search --linear-programming-solver highs`

## Usage (command line)

Compile:
```shell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DGENERALIZEDASSIGNMENTSOLVER_USE_HIGHS=ON
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
