#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <functional>


namespace astro {

using Scalar = double;
using Vec3   = std::array<Scalar, 3>;   // [x,y,z]
using Mat3   = std::array<Scalar, 9>;   // row-major

class ModuleInertiaTensor {
    
public:
    // ---- ctors/dtor ----
    ModuleInertiaTensor();
    explicit ModuleInertiaTensor(std::string name);

    // No copying (children_ holds unique_ptr)
    ModuleInertiaTensor(const ModuleInertiaTensor&) = delete;
    ModuleInertiaTensor& operator=(const ModuleInertiaTensor&) = delete;

    // Moves OK
    ModuleInertiaTensor(ModuleInertiaTensor&& other) noexcept;
    ModuleInertiaTensor& operator=(ModuleInertiaTensor&& other) noexcept;

    ~ModuleInertiaTensor();

    // ---- identity / helpers (declare only) ----
    static Mat3 Identity3();
    static Mat3 Zero3();
    static Vec3 Zero3v();

    // ---- configuration setters (declare only) ----
    void SetName(const std::string& name);
    void SetMass(Scalar mass);
    void SetLocalCOM(const Vec3& r_b);                    // COM in this node's local frame
    void SetLocalInertiaCOM(const Mat3& I_b_com);         // inertia about this node's COM in local frame
    void SetOrientationBodyToParent(const Mat3& R_bp);    // rotation body->parent
    void SetPositionBodyOriginInParent(const Vec3& r_p);  // position of body origin in parent frame

    // ---- hierarchy ----
    void AddChild(std::unique_ptr<ModuleInertiaTensor> child);
    ModuleInertiaTensor*       Parent();
    const ModuleInertiaTensor* Parent() const;
    const std::vector<std::unique_ptr<ModuleInertiaTensor>>& Children() const;
    void DetachFromParent();

    // ---- queries ----
    const std::string& Name() const;
    Scalar             Mass() const;
    const Vec3&        LocalCOM() const;
    const Mat3&        LocalInertiaCOM() const;
    const Mat3&        R_body_to_parent() const;
    const Vec3&        r_body_origin_in_parent() const;

    // ---- traversal ----
    void ForEach(const std::function<void(const ModuleInertiaTensor&)>& visitor) const;
    void ForEach(const std::function<void(ModuleInertiaTensor&)>& visitor);

    // ---- math to implement later ----
    Scalar ComputeTotalMass() const;
    Vec3   ComputeSubtreeCOM_Local() const;
    Mat3   ComputeCompositeInertiaAboutSubtreeCOM_Local() const;
    Mat3   ComputeCompositeInertiaAboutOrigin_Local() const;

private:
    std::string name_;

    Scalar mass_{0.0};
    Vec3   r_com_local_{ {0.0, 0.0, 0.0} };
    Mat3   I_com_local_{ {0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0} };

    Mat3   R_body_to_parent_{ {1.0,0.0,0.0,  0.0,1.0,0.0,  0.0,0.0,1.0} };
    Vec3   r_body_origin_in_parent_{ {0.0, 0.0, 0.0} };

    ModuleInertiaTensor* parent_{nullptr};  // non-owning
    std::vector<std::unique_ptr<ModuleInertiaTensor>> children_;
};

} // namespace astro
