#include <iostream>
#include "station_module_inertia.h"


static void print_mat3(const SpaceStationModuleInertia::Mat3& I) 
{
    std::cout.setf(std::ios::fixed);
    std::cout.precision(6);
    std::cout << "I(COM) =\n";
    for (int r = 0; r < 3; ++r) {
        std::cout << "  [ "
                  << I[r][0] << "  "
                  << I[r][1] << "  "
                  << I[r][2] << " ]\n";
    }
}


int main() 
{
    SpaceStationModuleInertia module;

    // Example test masses (kg) at positions (m)
    module.add_mass(500.0,  -1.0,  1.0,  0.0);
    module.add_mass(500.0,   1.0,  1.0,  0.0);
    module.add_mass(500.0,  -1.0, -1.0,  0.0);
    module.add_mass(500.0,   1.0, -1.0,  0.0);

    const auto n   = module.size();
    const auto M   = module.total_mass();
    const auto com = module.center_of_mass();
    const auto I   = module.inertia_about_com();

    std::cout << "N points: " << n << "\n";
    std::cout << "Total mass: " << M << " kg\n";
    std::cout << "COM: [ " << com[0] << ", " << com[1] << ", " << com[2] << " ] m\n";

    print_mat3(I);

    return 0;
}