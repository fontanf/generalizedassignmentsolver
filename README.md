WORK IN PROGRESS

# Generalized Assignment Problem

Algorithm implementations for the Generalized Assignment Problem.

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
- Lagrangian relaxation of knapsack constraints solved with volume method `-a lagrelax_knapsack_volume` :heavy_check_mark:
- Lagrangian relaxation of assignment constraints solved with volume method `-a lagrelax_assignment_volume` :x:
- Lagrangian relaxation of knapsack constraints solved with bundle method `-a lagrelax_knapsack_bundle` :x:
- Lagrangian relaxation of assignment constraints solved with bundle method `-a lagrelax_assignment_bundle` :x:
- Column generation :x:

## Upper bounds

Classical meta-heuristics based on shift-swap neighborhood and fixed penalty of capacity constraint violation:
- Hill climbing, first improvment `-a lsfirst_shiftswap` :heavy_check_mark:
- Hill climbing, best improvment `-a lsbest_shiftswap` :heavy_check_mark:
- Tabu search `-a ts_shiftswap` :heavy_check_mark:
- Simulated annealing `-a sa_shiftswap` :heavy_check_mark:
- Path relinking `-a pr_shiftswap` :heavy_check_mark:

Others:
- Random initial solution `-a random` :heavy_check_mark:
- Variable Depth Neighborhood Search `-a vdns_simple` :heavy_check_mark:

## Exact algorithms

- Branch-and-cut with CBC `-a branchandcut_cbc` :heavy_check_mark:
- Branch-and-cut with CPLEX `-a branchandcut_cplex` :heavy_check_mark:
- Branch-and-price :x:

## Results

`vdns_simple` does not compete with state of the art meta-heuristics like the ones presented in "A path relinking approach with ejection chains for the generalized assignment problem" (Yagiura, 2006) or "Variable-fixing then subgradient optimization guided very large scale neighborhood search for the generalized assignment problem" (Haddadi, 2018) in terms of solution quality on long runs. However:
- on short runs (2 minutes, Processor Intel® Core™ i5-8500 CPU @ 3.00GHz × 6), it provides solutions of good quality (less than 1% gap from optimal for all instances of the literature, and less than 0.5% for instances with more than 900 items)
- it is very simple and the implementation is very short
- it is available and free (MIT License)

