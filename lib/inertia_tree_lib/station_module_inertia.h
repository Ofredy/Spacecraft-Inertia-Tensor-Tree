#pragma once
#include <iostream>
#include <vector>
#include <array>
#include <stdexcept>


class SpaceStationModuleInertia 
{
public:
    
    std::array<std::array<double,3>,3> inertia_wrt_parent{}; 
    SpaceStationModuleInertia* parent_{nullptr};                 // single parent

    void calculate_module_inertia(const std::vector<double>& masses,
                                  const std::vector<std::array<double,3>>& positions);
    void calculate_module_inertia_wrt_parent();
    void add_module_child(SpaceStationModuleInertia* child_module);

private:

    std::vector<SpaceStationModuleInertia*> children_{};         // N children

    std::array<double,3> parent_distance = { 0.0, 10.0, 0.0 };
    std::array<double,3> q_module{};
    std::array<double,3> q_module_wrt_parent{};

    struct MassPoint { double m, x, y, z; };
    std::vector<MassPoint> points_;

    // stored results (kept private)
    double total_mass_{0.0};
    std::array<double,3> com_{};                 
    std::array<std::array<double,3>,3> inertia_com_{}; 

    void load_points_(const std::vector<double>& m,
                      const std::vector<std::array<double,3>>& p);
    void compute_total_mass_and_com_();
    void compute_inertia_about_com_();
};
