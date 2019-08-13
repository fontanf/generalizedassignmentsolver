#pragma once

#include <gap/lib/instance.hpp>

#include <benchtools/dataset.hpp>

#include <map>

namespace gap
{

const std::map<std::string, std::shared_ptr<Dataset<Instance>>> datasets = {
    {"A", std::shared_ptr<Dataset<Instance>>(new DatasetFromFiles<Instance>({
            "data/a05100",
            "data/a10100",
            "data/a20100",
            "data/a05200",
            "data/a10200",
            "data/a20200",
        }, "gap_beasley"))},
    {"B", std::shared_ptr<Dataset<Instance>>(new DatasetFromFiles<Instance>({
            "data/b05100",
            "data/b10100",
            "data/b20100",
            "data/b05200",
            "data/b10200",
            "data/b20200",
        }, "gap_beasley"))},
    {"C", std::shared_ptr<Dataset<Instance>>(new DatasetFromFiles<Instance>({
            "data/c05100",
            "data/c10100",
            "data/c20100",
            "data/c05200",
            "data/c10200",
            "data/c20200",
            "data/c10400",
            "data/c20400",
            "data/c40400",
            "data/c15900",
            "data/c30900",
            "data/c60900",
            "data/c201600",
            "data/c401600",
            "data/c801600",
        }, "gap_beasley"))},
    {"D", std::shared_ptr<Dataset<Instance>>(new DatasetFromFiles<Instance>({
            "data/d05100",
            "data/d10100",
            "data/d20100",
            "data/d05200",
            "data/d10200",
            "data/d20200",
            "data/d10400",
            "data/d20400",
            "data/d40400",
            "data/d15900",
            "data/d30900",
            "data/d60900",
            "data/d201600",
            "data/d401600",
            "data/d801600",
        }, "gap_beasley"))},
    {"D", std::shared_ptr<Dataset<Instance>>(new DatasetFromFiles<Instance>({
            "data/e05100",
            "data/e10100",
            "data/e20100",
            "data/e05200",
            "data/e10200",
            "data/e20200",
            "data/e10400",
            "data/e20400",
            "data/e40400",
            "data/e15900",
            "data/e30900",
            "data/e60900",
            "data/e201600",
            "data/e401600",
            "data/e801600",
        }, "gap_beasley"))},
};

}

