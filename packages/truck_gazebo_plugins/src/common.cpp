#include "truck_gazebo_plugins/common.h"

namespace gazebo {

sdf::ElementPtr GetElement(sdf::ElementPtr sdf, const std::string& name) {
    const auto element = sdf->GetElement(name);
    if (not element) {
        gzerr << "No element '" << name << "'!" << std::endl;
        throw std::runtime_error("Bad urdf file!");
    }

    return element;
}

physics::JointPtr GetJoint(gazebo::physics::ModelPtr model, const std::string& name) {
    auto joint = model->GetJoint(name);
    if (not joint) {
        gzerr << "No joint '" << name << "'!" << std::endl;
        throw std::runtime_error("Bad urdf file!");
    }

    return joint;
}

}  // namespace gazebo
