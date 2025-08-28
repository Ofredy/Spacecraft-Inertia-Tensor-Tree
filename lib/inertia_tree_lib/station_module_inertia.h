#pragma once
#include <vector>
#include <array>


class SpaceStationModuleInertia 
{
public:
    // The ONLY public function:
    // Compute inertia about the center of mass (COM) from masses & positions.
    // Throws std::invalid_argument on bad input.
    std::array<std::array<double,3>,3>
    calculate_module_inertia(const std::vector<double>& masses,
                             const std::vector<std::array<double,3>>& positions);

private:

    struct MassPoint { double m, x, y, z; };

    void load_points_(const std::vector<double>& m,
                      const std::vector<std::array<double,3>>& p);
    void compute_total_mass_and_com_();
    void compute_inertia_about_com_();

    // internal storage
    std::vector<MassPoint> points_;

    // stored results (kept private)
    double total_mass_{0.0};
    std::array<double,3> com_{};                 
    std::array<std::array<double,3>,3> inertia_com_{}; 
};
