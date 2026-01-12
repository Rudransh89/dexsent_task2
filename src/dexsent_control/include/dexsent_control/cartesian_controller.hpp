#ifndef DEXSENT_CONTROL__CARTESIAN_CONTROLLER_HPP_
#define DEXSENT_CONTROL__CARTESIAN_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/Dense>

namespace dexsent_control {
class CartesianController : public controller_interface::ControllerInterface {
public:
  CartesianController();
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  std::vector<std::string> joint_names_;
  KDL::Chain kdl_chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  KDL::Jacobian jacobian_;
  KDL::JntArray current_joint_positions_;
  Eigen::VectorXd target_twist_;
};
} 
#endif
