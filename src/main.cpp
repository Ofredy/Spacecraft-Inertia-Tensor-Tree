#include <iostream>
#include "station_module_inertia.h"


int main() {
    SpaceStationModuleInertia mod;

    std::vector<double> masses = {500, 500, 500, 500};
    std::vector<std::array<double,3>> pos = {
        {-1.0,  1.0, 0.0},
        { 1.0,  1.0, 0.0},
        {-1.0, -1.0, 0.0},
        { 1.0, -1.0, 0.0}
    };

    auto I = mod.calculate_module_inertia(masses, pos);

    std::cout << "I(COM):\n";
    std::cout << "  [ " << I[0][0] << "  " << I[0][1] << "  " << I[0][2] << " ]\n";
    std::cout << "  [ " << I[1][0] << "  " << I[1][1] << "  " << I[1][2] << " ]\n";
    std::cout << "  [ " << I[2][0] << "  " << I[2][1] << "  " << I[2][2] << " ]\n";
    return 0;
}