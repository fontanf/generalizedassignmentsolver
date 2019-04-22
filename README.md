WORK IN PROGRESS

# Generalized Assignment Problem

Algorithm implementations for the Generalized Assignment Problem.

Compile:
```
bazel build --cxxopt='-std=c++14' --compilation_mode=opt -- //...
```

Solve:
```
./bazel-bin/lib/main -v -a milp -i data/a05100 -o out.ini -c sol.txt
```

Unit tests:
```
bazel test --cxxopt='-std=c++14' --compilation_mode=opt -- //...
```

## Lower bounds

- Lagrangian relaxation solved with volume method :x:
- Lagrangian relaxation solved with bundle method :x:
- Column generation :x:

## Upper bounds

- Random initial solution `-a random` :heavy_check_mark:
- Simple hill climbing `-a lssimple` :heavy_check_mark:

## Exact algorithms

- MILP solved with CBC `-a milp` :heavy_check_mark:
- Branch-and-price :x:

