# GeneralizedAssignmentSolver

A solver for the Generalized Assignment Problem.

This problem is interesting because many different optimization methods can and have been applied to solve it (Branch-and-cut, Branch-and-price, Branch-and-relax, Local search, Constraint programming, Column generation heuristics...). Thus, the main goal of this repository is for me to have reference implementations for classical algorithms and optimization solvers.

## Implemented algorithms

### Lower bounds

- Linear relaxation
  - solved with CLP `-a linrelax_clp` :heavy_check_mark:
  - solved with Gurobi `-a linrelax_gurobi` :heavy_check_mark:

- Lagrangian relaxation of knapsack constraints
  - solved with volume method `-a lagrelax_knapsack_volume` :heavy_check_mark:
  - solved with L-BFGS method `-a lagrelax_knapsack_lbfgs` :heavy_check_mark:

- Lagrangian relaxation of assignment constraints
  - solved with volume method `-a lagrelax_assignment_volume` :heavy_check_mark:
  - solved with L-BFGS method `-a lagrelax_assignment_lbfgs` :heavy_check_mark:

- Column generation `-a columngeneration --solver clp` :heavy_check_mark: `-a columngeneration --solver cplex` :heavy_check_mark:

### Upper bounds

Polynomial algorithms from "Generalized Assignment Problems" (Martello et al., 1992), options `f cij` `f wij` `f cij*wij` `f -pij/wij` `f wij/ti`:
- Basic greedy `-a "greedy -f wij"` :heavy_check_mark:
- Greedy with regret measure `-a "greedyregret -f wij"` :heavy_check_mark:
- MTHG, basic greedy (+ n shifts) `-a "mthg -f wij"` :heavy_check_mark:
- MTHG, greedy with regret measure (+ n shifts) `-a "mthgregret -f wij"` :heavy_check_mark:

Classical meta-heuristics based on shift-swap neighborhood optimized for large instances:
- Local search `-a localsearch` :heavy_check_mark:
- Tabu search `-a tabusearch` :heavy_check_mark:
- Simulated annealing `-a simulatedannealing` :heavy_check_mark:

Others heuristics and meta-heuristics:
- Random feasible solution found with a Local search `-a random` :heavy_check_mark:
- Repair linear relaxation solution `-a repairlinrelax_clp` :heavy_check_mark:
- Local search with LocalSolver `-a localsolver` :heavy_check_mark:
- Tree search algorithms based on Branch-and-price branching scheme :x:
- Tree search algorithms based on Dantzig-Wolfe reformulation branching scheme :x:

### Exact algorithms

- Branch-and-cut
  - with CBC `-a branchandcut_cbc` :heavy_check_mark:
  - with CPLEX `-a branchandcut_cplex` :heavy_check_mark:
  - with Gurobi `-a branchandcut_gurobi` :heavy_check_mark:

- Branch-and-price
  - Depth First Search `-a branchandprice_dfs --solver clp` :heavy_check_mark: `-a branchandprice_dfs --solver cplex` :heavy_check_mark:
  - A* (Best First Search) `-a branchandprice_astar --solver clp` :heavy_check_mark: `-a branchandprice_astar --solver cplex` :heavy_check_mark:

- Constraint programming
  - with Gecode `-a constraintprogramming_gecode` :heavy_check_mark:
  - with CPLEX `-a constraintprogramming_cplex` :heavy_check_mark:

## Notes

### Linear relaxation gap

The largest gap between the lower bound from the linear relaxation and the best known upper bound is 1.93%.

### Lagrangian relaxation implementation

The bound from the lagrangian relaxation of knapsack constraints is theoritically equal to the bound from the linear relaxation. The optimal bounds are found by the L-BGFS algorithm from DLib; however, the volume method stays rather far from it. 

The same happens for the bound obtained by solving the lagrangian relaxation of assignment constraints; the Volume method give poor results while the L-BFGS algorithm returns the best known ones found in "An exact method with variable fixing for solving the generalized assignment problem" (Posta et al., 2011).

### Finding a feasible solution

* [mthgregret -f wij/ti](https://librallu.gitlab.io/splitted-cell-viz/?u=https://raw.githubusercontent.com/fontanf/generalizedassignment/master/bench/mthgregret_f_wij_ti.json)
* [random](https://librallu.gitlab.io/splitted-cell-viz/?u=https://raw.githubusercontent.com/fontanf/generalizedassignment/master/bench/random.json)

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
./bazel-bin/generalizedassignmentsolver/main -v \
    -a 'mthg -f -pij/wij' \
    -i "data/a05100" \
    -o "a05100_output.json" \
    -c "a05100_solution.txt"
```

Unit tests:
```shell
bazel test --compilation_mode=dbg -- //...
```

Checker:
```
bazel run -- //generalizedassignmentsolver:checker                # show best bounds for all instances
bazel run -- //generalizedassignmentsolver:checker "data/a05100"  # show best bounds for one instance
bazel build -- //generalizedassignmentsolver:checker              # check another solution file
./bazel-bin/generalizedassignmentsolver/checker "instancefile" "solutionfile"
```

Run benchmarks (results stored in `out/algorithm/`)
```
bazel run -- //generalizedassignmentsolver:bench "mthg -f wij"             # no time limit
bazel run -- //generalizedassignmentsolver:bench "branchandcut_cbc" 7200  # 2h time limit
```
Output files can then be retrieved from `bazel-out/k8-opt/bin/generalizedassignmentsolver/bench.runfiles/__main__/`.

## Optional dependencies

To enable a dependency, uncomment the corresponding line in the `.bazelrc` file and adapt its path in the `WORKSPACEi` file.

Here are some notes for their installations:

### COIN-OR (CLP, CBC, VOL, DIP)

Install (https://coin-or.github.io/coinbrew/):
```shell
git clone https://www.github.com/coin-or/coinbrew
cd coinbrew
./coinbrew fetch build Dip --no-prompt
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

