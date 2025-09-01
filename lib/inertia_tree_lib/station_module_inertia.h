#pragma once
#include <iostream>
#include <vector>
#include <array>
#include <stdexcept>

#include "quat_math.h"


class SpaceStationModuleInertia 
{
public:
    
    std::array<std::array<double,3>,3> inertia_wrt_parent{}; 
    std::string name_;
    SpaceStationModuleInertia* parent_{nullptr};                 // single parent

    void load_points_(const std::vector<double>& m,
                      const std::vector<std::array<double,3>>& p);

    void calculate_module_inertia();
    void calculate_module_inertia_wrt_parent();
    void add_module_child(SpaceStationModuleInertia* child_module);

    void set_parent_offset(const std::array<double,3>& r) { parent_distance_parent = r; }
    void set_q_parent_from_child(const std::array<double,4>& q) { q_parent_from_child = q; }

    bool has_children() const { return !children_.empty(); }
    const std::vector<SpaceStationModuleInertia*>& children() const { return children_; }

private:

    std::vector<SpaceStationModuleInertia*> children_{};         // N children

    std::array<double,3> parent_distance_parent = {};
    std::array<double, 4> q_parent_from_child = {};

    struct MassPoint { double m, x, y, z; };
    std::vector<MassPoint> points_;

    // stored results (kept private)
    double total_mass_{0.0};
    std::array<double,3> com_{};                 
    std::array<std::array<double,3>,3> inertia_com_{}; 

    double calculate_subtree_mass_();
    void compute_total_mass_and_com_();
    void compute_inertia_about_com_();
};
