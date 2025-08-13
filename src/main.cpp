#include <iostream>
#include <memory>
#include "../lib/ModuleInertiaTensor.h"

int main() {
    using astro::ModuleInertiaTensor;

    // TODO: build the ISS tree here (root = whole station at its COM),
    // add N modules, each with N components, set masses/poses/inertias,
    // then call your composite mass/COM/inertia routines.

    std::unique_ptr<ModuleInertiaTensor> iss = std::make_unique<ModuleInertiaTensor>("ISS");

    // example placeholder structure (you’ll flesh this out):
    // auto us_lab = std::make_unique<ModuleInertiaTensor>("US_Lab");
    // iss->AddChild(std::move(us_lab));
    // ...

    std::cout << "ISS inertia builder: skeleton ready.\n";
    return 0;
}
