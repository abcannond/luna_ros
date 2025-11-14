#include "quad_swerve_controller/quad_swerve_controller.hpp"

namespace quad_swerve_controller
{

QuadSwerveController::QuadSwerveController() : ControllerInterface()
{
}

controller_interface::return_type QuadSwerveController::init(const std::string &controller_name)
{
  logger_->info("Initializing QuadSwerveController: {}", controller_name);
  return controller_interface::return_type::OK;
}

controller_interface::return_type QuadSwerveController::update()
{
  // This is where your control loop will go
  RCLCPP_INFO(logger_, "Running update loop");
  return controller_interface::return_type::OK;
}

} // namespace my_luna_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_luna_controller::QuadSwerveController, controller_interface::ControllerInterface)
