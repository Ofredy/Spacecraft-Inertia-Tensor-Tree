#pragma once
#include <array>
#include <cmath>
#include <stdexcept>

// Quaternion type: [w, x, y, z]
// Inertia tensor: 3x3 matrix

using mat3 = std::array<std::array<double,3>,3>;

// --- Quaternion basics ---
std::array<double,4> quat_normalize(const std::array<double,4>& q);
std::array<double,4> quat_conjugate(const std::array<double,4>& q);
std::array<double,4> quat_inverse(const std::array<double,4>& q);

// Hamilton product: q = q2 ⊗ q1
std::array<double,4> quat_multiply(const std::array<double,4>& q2,
                                   const std::array<double,4>& q1);

// Relative rotation between two quaternions
std::array<double,4> quat_between(const std::array<double,4>& q_from,
                                  const std::array<double,4>& q_to);

// Convert quaternion to DCM
mat3 quat_to_dcm(const std::array<double,4>& q_unit);

// Rotate inertia tensor by quaternion rotation
mat3 rotate_inertia(const mat3& I_old, const std::array<double,4>& q_rot);
