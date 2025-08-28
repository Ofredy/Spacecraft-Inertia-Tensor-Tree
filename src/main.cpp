#include <iostream>
#include "station_module_inertia.h"


int main() 
{

    // module f is a child of module d
    SpaceStationModuleInertia mod_f;

    std::vector<double> masses = {500, 500, 500, 500};
    std::vector<std::array<double,3>> pos = {
        {-1.0,  1.0, 0.0},
        { 1.0,  1.0, 0.0},
        {-1.0, -1.0, 0.0},
        { 1.0, -1.0, 0.0}
    };

    mod_f.calculate_module_inertia(masses, pos);
    mod_f.calculate_module_inertia_wrt_parent();

    // module d
    SpaceStationModuleInertia mod_d;
    mod_d.add_module_child(&mod_f);

    mod_d.calculate_module_inertia(masses, pos);

    return 0;
}