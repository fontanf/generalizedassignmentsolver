import os
import json
import sys

project = "generalizedassignmentsolver"
data       = sys.argv[1]
algorithm  = sys.argv[2]
timelimit  = sys.argv[3] if len(sys.argv) > 3 else None

datas = {}
datas["cattrysse1994"] = [("cattrysse1994/gap" + str(i) + "-" + str(j), "-f orlibrary") for i in range(1, 13) for j in range(0, 5)]
datas["chu1997"] = [("chu1997/" + c + m + n, "-f orlibrary") for c in ['a', 'b', 'c', 'd'] for n in ["100", "200"] for m in ["05", "10", "20"]]
datas["yagiura2004"] = [("yagiura2004/" + c + mn, "-f orlibrary") for c in ['c', 'd', 'e'] for mn in ["10400", "20400", "40400", "15900", "30900", "60900", "201600", "401600", "801600"]]

directory_in = "data"
directory_out = os.path.join("output", algorithm + ("_" + str(timelimit) if timelimit != None else ""), data)
if not os.path.exists(directory_out):
    os.makedirs(directory_out)

results_path = os.path.join(directory_out, "results.csv")
with open(results_path, "w") as f:
    f.write("Instance,Value,,Time to best,Time to end\n")

    main_exec = os.path.join(".", "bazel-bin", project, "main")
    for instance_name, args in datas[data]:
        s = "_unicost" if "--unicost" in args else ""
        instance_path = os.path.join(directory_in, instance_name)
        output_path   = os.path.join(directory_out, instance_name + s + ".json")
        cert_path     = os.path.join(directory_out, instance_name + s + "_solution.txt")
        if not os.path.exists(os.path.dirname(output_path)):
            os.makedirs(os.path.dirname(output_path))

        command = main_exec \
                + " -v" \
                + " -i \"" + instance_path + "\"" \
                + " " + args \
                + (" -t " + str(timelimit) if timelimit != None else "") \
                + " -a \"" + algorithm + "\"" \
                + " -c \"" + cert_path + "\"" \
                + " -o \"" + output_path + "\""
        print(command)
        os.system(command)
        print()

        with open(output_path, "r") as read_file:
            d = json.load(read_file)

            k = 0
            while "Solution" + str(k + 1) in d.keys():
                k += 1

            f.write(instance_name \
                    + "," + str(d["Solution"]["Value"]) \
                    + "," \
                    + "," + str(d["Solution" + str(k)]["Time"]) if "Solution" + str(k) in d.keys() else "" \
                    + "," + str(d["Solution"]["Time"]) \
                    + "\n")

