#include <stdexcept>
#include <cmath>

#include "station_module_inertia.h"


void SpaceStationModuleInertia::add_mass(double m, double x, double y, double z) 
{
    if (m <= 0.0) throw std::invalid_argument("Mass must be positive.");
    points_.push_back({m, x, y, z});
}

void SpaceStationModuleInertia::set_points(const std::vector<MassPoint>& pts) 
{
    for (const auto& p : pts) 
    {
        if (p.m <= 0.0) throw std::invalid_argument("Mass must be positive.");
    }
    points_ = pts;
}

std::size_t SpaceStationModuleInertia::size() const 
{
    return points_.size();
}

double SpaceStationModuleInertia::total_mass() const 
{
    double M = 0.0;
    for (const auto& p : points_) M += p.m;
    return M;
}

SpaceStationModuleInertia::Vec3 SpaceStationModuleInertia::center_of_mass() const 
{
    require_nonempty();
    double M = 0.0, cx = 0.0, cy = 0.0, cz = 0.0;
    for (const auto& p : points_) {
        M  += p.m;
        cx += p.m * p.x;
        cy += p.m * p.y;
        cz += p.m * p.z;
    }
    return { cx / M, cy / M, cz / M };
}

SpaceStationModuleInertia::Mat3 SpaceStationModuleInertia::inertia_about_com() const 
{
    require_nonempty();
    const Vec3 com = center_of_mass();

    Mat3 I = {{ {0,0,0}, {0,0,0}, {0,0,0} }};
    for (const auto& p : points_) 
    {
        const double rx = p.x - com[0];
        const double ry = p.y - com[1];
        const double rz = p.z - com[2];
        const double r2 = rx*rx + ry*ry + rz*rz;

        // m * (r^2 * I3 - r r^T)
        I[0][0] += p.m * (r2 - rx*rx);
        I[0][1] -= p.m * (rx*ry);
        I[0][2] -= p.m * (rx*rz);

        I[1][0] -= p.m * (ry*rx);
        I[1][1] += p.m * (r2 - ry*ry);
        I[1][2] -= p.m * (ry*rz);

        I[2][0] -= p.m * (rz*rx);
        I[2][1] -= p.m * (rz*ry);
        I[2][2] += p.m * (r2 - rz*rz);
    }
    
    return I;
}

void SpaceStationModuleInertia::require_nonempty() const 
{
    if (points_.empty())
        throw std::runtime_error("No masses defined. Add masses before computing inertia.");
}
