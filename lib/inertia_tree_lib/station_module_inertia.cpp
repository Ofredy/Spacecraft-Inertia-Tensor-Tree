#include "station_module_inertia.h"


void SpaceStationModuleInertia::calculate_module_inertia(
    const std::vector<double>& masses,
    const std::vector<std::array<double,3>>& positions)
{
    load_points_(masses, positions);
    compute_total_mass_and_com_();
    compute_inertia_about_com_();
}

void SpaceStationModuleInertia::calculate_module_inertia_wrt_parent()
{
    // Assumes child and parent frames are aligned (no rotation).
    // Requires: total_mass_ and inertia_com_ already computed (call calculate_module_inertia first).
    if (total_mass_ <= 0.0) {
        throw std::runtime_error("module_inertia_wrt_parent: total mass not set. Call calculate_module_inertia first.");
    }

    const double M  = total_mass_;
    const double dx = parent_distance[0];
    const double dy = parent_distance[1];
    const double dz = parent_distance[2];
    const double d2 = dx*dx + dy*dy + dz*dz;

    // Parallel-axis term: M * (||d||^2 I - d d^T)
    std::array<std::array<double,3>,3> PA = {{
        {{  M*(d2 - dx*dx), -M*(dx*dy),     -M*(dx*dz) }},
        {{ -M*(dy*dx),      M*(d2 - dy*dy), -M*(dy*dz) }},
        {{ -M*(dz*dx),     -M*(dz*dy),      M*(d2 - dz*dz) }}
    }};

    // Inertia wrt parent origin, expressed in parent frame (aligned frames)
    inertia_wrt_parent = {{
        {{ inertia_com_[0][0] + PA[0][0], inertia_com_[0][1] + PA[0][1], inertia_com_[0][2] + PA[0][2] }},
        {{ inertia_com_[1][0] + PA[1][0], inertia_com_[1][1] + PA[1][1], inertia_com_[1][2] + PA[1][2] }},
        {{ inertia_com_[2][0] + PA[2][0], inertia_com_[2][1] + PA[2][1], inertia_com_[2][2] + PA[2][2] }}
    }};

    std::cout << "I(COM):\n";
    std::cout << "  [ " << inertia_wrt_parent[0][0] << "  " << inertia_wrt_parent[0][1] << "  " << inertia_wrt_parent[0][2] << " ]\n";
    std::cout << "  [ " << inertia_wrt_parent[1][0] << "  " << inertia_wrt_parent[1][1] << "  " << inertia_wrt_parent[1][2] << " ]\n";
    std::cout << "  [ " << inertia_wrt_parent[2][0] << "  " << inertia_wrt_parent[2][1] << "  " << inertia_wrt_parent[2][2] << " ]\n";
}

void SpaceStationModuleInertia::add_module_child(SpaceStationModuleInertia* child_module)
{
    child_module->parent_ = this;
    children_.push_back(child_module);
}

void SpaceStationModuleInertia::load_points_(
    const std::vector<double>& m,
    const std::vector<std::array<double,3>>& p)
{
    if (m.empty() || p.empty())
        throw std::invalid_argument("Inputs must be non-empty.");
    if (m.size() != p.size())
        throw std::invalid_argument("masses and positions must have the same length.");

    points_.clear();
    points_.reserve(m.size());
    for (std::size_t i = 0; i < m.size(); ++i) 
    {
        if (m[i] <= 0.0) throw std::invalid_argument("Each mass must be positive.");
        points_.push_back(MassPoint{ m[i], p[i][0], p[i][1], p[i][2] });
    }
}

void SpaceStationModuleInertia::compute_total_mass_and_com_() 
{
    double M  = 0.0;
    double cx = 0.0, cy = 0.0, cz = 0.0;

    for (const auto& p : points_) 
    {
        M  += p.m;
        cx += p.m * p.x;
        cy += p.m * p.y;
        cz += p.m * p.z;
    }
    if (M == 0.0) throw std::runtime_error("Total mass is zero.");

    total_mass_ = M;
    com_ = { cx / M, cy / M, cz / M };
}

void SpaceStationModuleInertia::compute_inertia_about_com_() 
{
    inertia_com_ = {};

    const auto& c = com_;
    for (const auto& p : points_) 
    {
        const double rx = p.x - c[0];
        const double ry = p.y - c[1];
        const double rz = p.z - c[2];
        const double r2 = rx*rx + ry*ry + rz*rz;

        inertia_com_[0][0] += p.m * (r2 - rx*rx);
        inertia_com_[0][1] -= p.m * (rx*ry);
        inertia_com_[0][2] -= p.m * (rx*rz);

        inertia_com_[1][0] -= p.m * (ry*rx);
        inertia_com_[1][1] += p.m * (r2 - ry*ry);
        inertia_com_[1][2] -= p.m * (ry*rz);

        inertia_com_[2][0] -= p.m * (rz*rx);
        inertia_com_[2][1] -= p.m * (rz*ry);
        inertia_com_[2][2] += p.m * (r2 - rz*rz);
    }

    // Add children (already in parent frame about parent origin)
    for (auto* child : children_) {
        const auto& C = child->inertia_wrt_parent;
        inertia_com_[0][0] += C[0][0]; inertia_com_[0][1] += C[0][1]; inertia_com_[0][2] += C[0][2];
        inertia_com_[1][0] += C[1][0]; inertia_com_[1][1] += C[1][1]; inertia_com_[1][2] += C[1][2];
        inertia_com_[2][0] += C[2][0]; inertia_com_[2][1] += C[2][1]; inertia_com_[2][2] += C[2][2];
    }

    std::cout << "I(COM):\n";
    std::cout << "  [ " << inertia_com_[0][0] << "  " << inertia_com_[0][1] << "  " << inertia_com_[0][2] << " ]\n";
    std::cout << "  [ " << inertia_com_[1][0] << "  " << inertia_com_[1][1] << "  " << inertia_com_[1][2] << " ]\n";
    std::cout << "  [ " << inertia_com_[2][0] << "  " << inertia_com_[2][1] << "  " << inertia_com_[2][2] << " ]\n";
}
