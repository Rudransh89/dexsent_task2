#include "dexsent_control/cartesian_controller.hpp"

namespace dexsent_control {

CartesianController::CartesianController() : target_twist_(6) { target_twist_.setZero(); }

controller_interface::CallbackReturn CartesianController::on_init() {
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianController::on_configure(const rclcpp_lifecycle::State &) {
    auto node = get_node();
    joint_names_ = node->get_parameter("joints").as_string_array();
    
    // Hardcoded Kinematics for CRX-10iA (Approximate for assignment visualization)
    // In production, use kdl_parser to read from URDF
    kdl_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
    kdl_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY)));
    kdl_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY)));
    kdl_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX)));
    kdl_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY)));
    kdl_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX)));

    twist_sub_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
        "~/cmd_vel", 10, std::bind(&CartesianController::twistCallback, this, std::placeholders::_1));
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration CartesianController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joint_names_) config.names.push_back(joint + "/velocity");
    return config;
}

controller_interface::InterfaceConfiguration CartesianController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joint_names_) config.names.push_back(joint + "/position");
    return config;
}

controller_interface::CallbackReturn CartesianController::on_activate(const rclcpp_lifecycle::State &) {
    jacobian_.resize(joint_names_.size());
    current_joint_positions_.resize(joint_names_.size());
    jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(kdl_chain_);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianController::on_deactivate(const rclcpp_lifecycle::State &) {
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianController::update(const rclcpp::Time &, const rclcpp::Duration &) {
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        current_joint_positions_(i) = state_interfaces_[i].get_value();
    }
    
    jac_solver_->JntToJac(current_joint_positions_, jacobian_);
    Eigen::MatrixXd J = jacobian_.data;
    // Damped Least Squares Inverse: J^T * (J * J^T + lambda^2 * I)^-1
    double lambda = 0.5; // Damping factor
    Eigen::MatrixXd J_pinv = J.transpose() * (J * J.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(6, 6)).inverse();
    Eigen::VectorXd joint_velocities = J_pinv * target_twist_;

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        command_interfaces_[i].set_value(joint_velocities(i));
    }
    return controller_interface::return_type::OK;
}

void CartesianController::twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    target_twist_ << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z, 
                     msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z;
}

} 
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dexsent_control::CartesianController, controller_interface::ControllerInterface)
