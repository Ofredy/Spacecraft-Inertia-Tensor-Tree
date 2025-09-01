#include <iostream>
#include <fstream>

#include "nlohmann/json.hpp"

#include "station_module_inertia.h"
#include "tree_loader.h"

using json = nlohmann::json;


auto printI = [](const std::array<std::array<double,3>,3>& I) 
{
    std::cout << std::fixed << std::setprecision(3);
    for (int i = 0; i < 3; ++i) {
        std::cout << "[ ";
        for (int j = 0; j < 3; ++j) std::cout << std::setw(10) << I[i][j] << " ";
        std::cout << "]\n";
    }
};

int main() 
{
    // 1) Load JSON from file
    std::ifstream f("station.json");
    if (!f) {
        std::cerr << "Could not open station.json\n";
        return 1;
    }
    json j;
    f >> j;

    // 2) Build tree depth-first (arena owns memory, class holds raw pointers)
    std::vector<std::unique_ptr<SpaceStationModuleInertia>> arena;
    SpaceStationModuleInertia* root_module = build_tree_from_json(j, arena, nullptr);

    // 3) Compute total inertia of the station
    compute_total_inertia(root_module);

    return 0;
}