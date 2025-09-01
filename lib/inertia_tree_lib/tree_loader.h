// inertia_tree_loader.hpp
#pragma once
#include <memory>
#include <vector>
#include <array>
#include <string>
#include <stdexcept>
#include <iostream>

#include "station_module_inertia.h"
#include "nlohmann/json.hpp"
using json = nlohmann::json;

/*
JSON schema (example):
{
  "name": "Root",
  "points": [ {"m": 500, "pos": [0,0,0]}, {"m": 300, "pos": [2,0,0]} ],
  "parent_offset": [0, 0, 0],                 // vector from this module's COM to parent COM, in parent axes (root can be [0,0,0])
  "q_parent_from_child": [1, 0, 0, 0],        // [w,x,y,z] rotation child->parent
  "children": [
    {
      "name": "Lab-A",
      "points": [ {"m": 200, "pos":[0.5,0.0,0.0]} ],
      "parent_offset": [0.0, 10.0, 0.0],
      "q_parent_from_child": [0.9659, 0.2588, 0.0, 0.0],
      "children": [ ... ]
    }
  ]
}
*/

// Internal: load one module’s mass points and transforms from JSON into an existing node
inline void load_module_payload_from_json(const json& j, SpaceStationModuleInertia* node) 
{
    if (!node) throw std::runtime_error("Null node in load_module_payload_from_json");

    // points -> (masses, positions)
    std::vector<double> masses;
    std::vector<std::array<double,3>> positions;
    if (j.contains("points")) 
    {
        for (const auto& p : j.at("points")) 
        {
            double m = p.at("m").get<double>();
            auto pos = p.at("pos").get<std::array<double,3>>();
            masses.push_back(m);
            positions.push_back(pos);
        }
    }

    node->load_points_(masses, positions);

    // transforms (optional, default identity/no-offset)
    if (j.contains("parent_offset")) 
    {
        node->set_parent_offset(j.at("parent_offset").get<std::array<double,3>>());
    }
    if (j.contains("q_parent_from_child")) 
    {
        node->set_q_parent_from_child(j.at("q_parent_from_child").get<std::array<double,4>>());
    }
}

inline SpaceStationModuleInertia* build_tree_from_json(
    const json& j,
    std::vector<std::unique_ptr<SpaceStationModuleInertia>>& arena,
    SpaceStationModuleInertia* parent = nullptr)
{
    arena.emplace_back(std::make_unique<SpaceStationModuleInertia>());
    SpaceStationModuleInertia* node = arena.back().get();

    // Name
    if (j.contains("name"))
        node->name_ = j.at("name").get<std::string>();
    else
        node->name_ = "unnamed";

    // Payload (points, transforms)
    load_module_payload_from_json(j, node);

    if (parent) parent->add_module_child(node);

    if (j.contains("children")) {
        for (const auto& cj : j.at("children")) {
            build_tree_from_json(cj, arena, node);
        }
    }
    return node;
}

// Compute inertia_wrt_parent for every node (except root which has no parent)
void compute_total_inertia(SpaceStationModuleInertia* root_module)
{
    if( root_module->has_children() )
    {
        for( auto child_module : root_module->children() )
        {
            compute_total_inertia(child_module);
        }
    }

    root_module->calculate_module_inertia();
    if( root_module->parent_ != nullptr )
    {
        root_module->calculate_module_inertia_wrt_parent();
    }
    std::cout << root_module->calculate_subtree_mass_() << std::endl;
}
