import argparse
import sys
import os

parser = argparse.ArgumentParser(description='')
parser.add_argument('directory')
parser.add_argument(
        "-t", "--tests",
        type=str,
        nargs='*',
        help='')

args = parser.parse_args()


generalizedassignmentsolver_main = os.path.join(
        "install",
        "bin",
        "generalizedassignmentsolver")
data_dir = os.environ['GENERALIZED_ASSIGNMENT_DATA']


greedy_data = [
        (os.path.join("chu1997", "a05100"), "orlibrary"),
        (os.path.join("chu1997", "a10100"), "orlibrary"),
        (os.path.join("chu1997", "b05100"), "orlibrary"),
        (os.path.join("chu1997", "b10100"), "orlibrary"),
        (os.path.join("chu1997", "c05100"), "orlibrary"),
        (os.path.join("chu1997", "c10100"), "orlibrary"),
        (os.path.join("chu1997", "d05100"), "orlibrary"),
        (os.path.join("chu1997", "d10100"), "orlibrary"),
        ]

if args.tests is None or "greedy" in args.tests:
    print("Greedy")
    print("------")
    print()

    for instance, instance_format in greedy_data:
        instance_path = os.path.join(
                data_dir,
                instance)

        for algorithm in ["greedy", "greedy-regret", "mthg", "mthg-regret"]:
            for desirability in ["cij", "wij", "cij*wij", "-pij/wij", "wij/ti"]:

                json_output_path = os.path.join(
                        args.directory,
                        algorithm.replace("-", "_"),
                        desirability.replace("/", "_"),
                        instance + ".json")
                if not os.path.exists(os.path.dirname(json_output_path)):
                    os.makedirs(os.path.dirname(json_output_path))
                command = (
                        generalizedassignmentsolver_main
                        + "  --verbosity-level 1"
                        + "  --input \"" + instance_path + "\""
                        + "  --format \"" + instance_format + "\""
                        + "  --algorithm \"" + algorithm + "\""
                        + "  --desirability \"" + desirability + "\""
                        + "  --output \"" + json_output_path + "\"")
                print(command)
                status = os.system(command)
                if status != 0:
                    sys.exit(1)
                print()
            print()
            print()


column_generation_data = [
        (os.path.join("chu1997", "a05100"), "orlibrary"),
        (os.path.join("chu1997", "a10100"), "orlibrary"),
        (os.path.join("chu1997", "b05100"), "orlibrary"),
        (os.path.join("chu1997", "b10100"), "orlibrary"),
        (os.path.join("chu1997", "c05100"), "orlibrary"),
        (os.path.join("chu1997", "c10100"), "orlibrary"),
        (os.path.join("chu1997", "d05100"), "orlibrary"),
        (os.path.join("chu1997", "d10100"), "orlibrary"),
        ]

# if args.tests is None or "greedy" in args.tests:
#     print("Column generation heuristic greedy")
#     print("----------------------------------")
#     print()

#     for instance, instance_format in column_generation_data:
#         instance_path = os.path.join(
#                 data_dir,
#                 instance)
#         json_output_path = os.path.join(
#                 args.directory,
#                 algorithm.replace("-", "_"),
#                 instance + ".json")
#         if not os.path.exists(os.path.dirname(json_output_path)):
#             os.makedirs(os.path.dirname(json_output_path))
#         command = (
#                 generalizedassignmentsolver_main
#                 + "  --verbosity-level 1"
#                 + "  --input \"" + instance_path + "\""
#                 + "  --format \"" + instance_format + "\""
#                 + "  --algorithm \"column-generation-heuristic-greedy\""
#                 + "  --output \"" + json_output_path + "\"")
#         print(command)
#         status = os.system(command)
#         if status != 0:
#             sys.exit(1)
#         print()
#     print()
#     print()
