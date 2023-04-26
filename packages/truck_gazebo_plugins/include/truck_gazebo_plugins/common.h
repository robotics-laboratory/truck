#include <gazebo/physics/physics.hh>

#include <sdf/sdf.hh>

#include <string>

namespace gazebo {

template <class T>
T GetParam(sdf::ElementPtr sdf, const std::string& name) {
    const auto [value, has_value] = sdf->Get<T>(name, T());
    if (not has_value) {
        gzerr << "No param '" << name << "'!" << std::endl;
        throw std::runtime_error("Bad urdf file!");
    }

    return value;
}

sdf::ElementPtr GetElement(sdf::ElementPtr sdf, const std::string& name);

physics::JointPtr GetJoint(gazebo::physics::ModelPtr model, const std::string& name);

}  // namespace gazebo