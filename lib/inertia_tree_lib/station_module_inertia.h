#pragma once
#include <vector>
#include <array>
#include <cstddef> // for std::size_t

class SpaceStationModuleInertia 
{
public:

    struct MassPoint {
        double m;          // mass (kg)
        double x, y, z;    // position in module frame (meters)
    };

    using Mat3 = std::array<std::array<double,3>,3>;
    using Vec3 = std::array<double,3>;

    SpaceStationModuleInertia() = default;

    // Add a single point mass
    void add_mass(double m, double x, double y, double z);

    // Bulk set (replaces any existing points)
    void set_points(const std::vector<MassPoint>& pts);

    // Number of points
    std::size_t size() const;

    // Total mass
    double total_mass() const;

    // Center of mass (computed from current points)
    Vec3 center_of_mass() const;

    // Inertia tensor about the center of mass (3x3, in the input coordinate frame)
    Mat3 inertia_about_com() const;

private:

    void require_nonempty() const;

    std::vector<MassPoint> points_;

};
