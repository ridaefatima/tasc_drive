#include "phidgets_hardware/phidgets_bldc_hardware.hpp"
#include <cmath>
#include <iomanip>
#include <sstream>
#include <pluginlib/class_list_macros.hpp>

namespace phidgets_hardware
{

hardware_interface::CallbackReturn
PhidgetsBldcHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto get_param = [&](const std::string & key, const std::string & def) -> std::string {
    auto it = info_.hardware_parameters.find(key);
    return (it == info_.hardware_parameters.end()) ? def : it->second;
  };

  gear_ratio_ = std::stod(get_param("gear_ratio", "106.0"));
  commutations_per_motor_rev_ = std::stoi(get_param("commutations_per_motor_rev", "24"));
  acceleration_ = std::stod(get_param("acceleration", "5.0"));
  stall_velocity_ = std::stod(get_param("stall_velocity", "0.0"));
  command_limit_ = std::stod(get_param("command_limit", "1.0"));

  rescale_factor_rot_ = 1.0 / (gear_ratio_ * static_cast<double>(commutations_per_motor_rev_));

  // Auto-detect motor configuration from URDF joint names
  int32_t default_serial = 765818;
  
  for (const auto& joint : info_.joints) {
    const std::string& name = joint.name;
    
    // Skip if not a wheel joint
    if (name.find("wheel") == std::string::npos) continue;
    
    joint_names_.push_back(name);
    is_left_wheel_.push_back(is_left_wheel_joint(name));
    
    // Map based on naming convention
    if (name.find("front_left") != std::string::npos) {
      device_serial_.push_back(default_serial);
      hub_port_.push_back(0);
      channel_.push_back(0);
      direction_sign_.push_back(-1.0);
    } else if (name.find("middle_left") != std::string::npos) {
      device_serial_.push_back(default_serial);
      hub_port_.push_back(1);
      channel_.push_back(0);
      direction_sign_.push_back(-1.0);
    } else if (name.find("rear_left") != std::string::npos) {
      device_serial_.push_back(default_serial);
      hub_port_.push_back(2);
      channel_.push_back(0);
      direction_sign_.push_back(-1.0);
    } else if (name.find("front_right") != std::string::npos) {
      device_serial_.push_back(default_serial);
      hub_port_.push_back(5);
      channel_.push_back(0);
      direction_sign_.push_back(1.0);
    } else if (name.find("middle_right") != std::string::npos) {
      device_serial_.push_back(default_serial);
      hub_port_.push_back(4);
      channel_.push_back(0);
      direction_sign_.push_back(1.0);
    } else if (name.find("rear_right") != std::string::npos) {
      device_serial_.push_back(default_serial);
      hub_port_.push_back(3);
      channel_.push_back(0);
      direction_sign_.push_back(1.0);
    } else {
      // Generic assignment
      static int generic_counter = 0;
      device_serial_.push_back(default_serial);
      hub_port_.push_back(generic_counter++ % 6);
      channel_.push_back(0);
      direction_sign_.push_back(is_left_wheel_.back() ? -1.0 : 1.0);
    }
  }
  
  const size_t num_motors = joint_names_.size();
  
  if (num_motors == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("PhidgetsBldcHardware"), "No wheel joints found!");
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // Initialize all vectors
  motors_.resize(num_motors, nullptr);
  temperature_sensors_.resize(num_motors, nullptr);
  motor_enabled_.resize(num_motors, false);
  connection_attempted_.resize(num_motors, false);
  attached_.resize(num_motors, false);
  temperature_attached_.resize(num_motors, false);
  pos_rad_.resize(num_motors, 0.0);
  vel_state_.resize(num_motors, 0.0);
  temperature_c_.resize(num_motors, 0.0);
  cmd_.resize(num_motors, 0.0);
  
  setup_ros_communication();
  
  RCLCPP_INFO(rclcpp::get_logger("PhidgetsBldcHardware"),
              "Auto-detected %zu motors. Connecting all motors automatically...", num_motors);
  
  // CONNECT ALL MOTORS IMMEDIATELY - NO WAITING
  connect_all_motors();
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool PhidgetsBldcHardware::is_left_wheel_joint(const std::string& joint_name)
{
  return joint_name.find("left") != std::string::npos;
}

void PhidgetsBldcHardware::setup_ros_communication()
{
  telemetry_node_ = rclcpp::Node::make_shared("phidgets_bldc_hardware");
  
  // Publisher that matches your dashboard's expected topic
  motor_telemetry_pub_ = telemetry_node_->create_publisher<std_msgs::msg::String>(
    "/rover/drive/motor_telemetry", 10);
}

void PhidgetsBldcHardware::connect_all_motors()
{
  RCLCPP_INFO(rclcpp::get_logger("PhidgetsBldcHardware"),
              "Connecting to %zu motors...", joint_names_.size());
  
  for (size_t i = 0; i < joint_names_.size(); i++) {
    connect_motor(i);
  }
  
  // Check results
  int connected = std::count(attached_.begin(), attached_.end(), true);
  int enabled = std::count(motor_enabled_.begin(), motor_enabled_.end(), true);
  
  RCLCPP_INFO(rclcpp::get_logger("PhidgetsBldcHardware"),
              "Connection complete: %d/%zu motors attached, %d/%zu enabled",
              connected, joint_names_.size(), enabled, joint_names_.size());
  
  if (connected < (int)joint_names_.size()) {
    RCLCPP_WARN(rclcpp::get_logger("PhidgetsBldcHardware"),
                "Some motors failed to connect.");
  }
}

void PhidgetsBldcHardware::connect_motor(int i)
{
  if (connection_attempted_[i]) return;
  
  connection_attempted_[i] = true;
  
  RCLCPP_INFO(rclcpp::get_logger("PhidgetsBldcHardware"),
              "Connecting to motor %d: %s...", i, joint_names_[i].c_str());
  
  try_attach_motor(i);
  
  if (attached_[i]) {
    motor_enabled_[i] = true;
    RCLCPP_INFO(rclcpp::get_logger("PhidgetsBldcHardware"),
                "Motor %d (%s) CONNECTED", i, joint_names_[i].c_str());
    try_attach_temperature_sensor(i);
  } else {
    motor_enabled_[i] = false;
    RCLCPP_ERROR(rclcpp::get_logger("PhidgetsBldcHardware"),
                "Motor %d (%s) FAILED to connect", i, joint_names_[i].c_str());
  }
}

void PhidgetsBldcHardware::disable_motor(int i, const std::string& reason)
{
  if (!motor_enabled_[i]) return;
  
  motor_enabled_[i] = false;
  
  RCLCPP_ERROR(rclcpp::get_logger("PhidgetsBldcHardware"),
               "Motor %d (%s) DISABLED - %s", i, joint_names_[i].c_str(), reason.c_str());
  
  if (motors_[i]) close_phidget(i);
  pos_rad_[i] = 0.0;
  vel_state_[i] = 0.0;
  cmd_[i] = 0.0;
  attached_[i] = false;
}

void PhidgetsBldcHardware::publish_motor_telemetry()
{
  if (!motor_telemetry_pub_) return;
  
  rclcpp::Time now = telemetry_node_->now();
  if (last_telemetry_publish_time_.nanoseconds() != 0 &&
      (now - last_telemetry_publish_time_).seconds() < telemetry_publish_period_sec_) {
    return;
  }
  last_telemetry_publish_time_ = now;
  
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(4);
  stream << "{\"motors\":[";
  
  for (size_t i = 0; i < joint_names_.size(); i++) {
    if (i > 0) stream << ",";
    
    // Determine health status for dashboard
    std::string health;
    std::string fault;
    
    if (!motor_enabled_[i]) {
      health = "offline";
      fault = "Motor disabled";
    } else if (!attached_[i]) {
      health = "offline";
      fault = "Motor not attached";
    } else {
      health = (temperature_c_[i] < 70.0) ? "healthy" : "warning";
      fault = (temperature_c_[i] < 70.0) ? "None" : "High temperature";
    }
    
    stream << "{"
           << "\"name\":\"" << joint_names_[i] << "\","
           << "\"attached\":" << (attached_[i] ? "true" : "false") << ","
           << "\"position\":" << pos_rad_[i] << ","
           << "\"velocity\":" << vel_state_[i] << ","
           << "\"temperature\":" << temperature_c_[i] << ","
           << "\"health\":\"" << health << "\","
           << "\"fault\":\"" << fault << "\","
           << "\"enabled\":" << (motor_enabled_[i] ? "true" : "false") << ","
           << "\"last_update\":\"" << std::to_string(now.seconds()) << "\""
           << "}";
  }
  stream << "]}";
  
  std_msgs::msg::String msg;
  msg.data = stream.str();
  motor_telemetry_pub_->publish(msg);
}

// Hardware interface methods
std::vector<hardware_interface::StateInterface>
PhidgetsBldcHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); i++) {
    state_interfaces.emplace_back(joint_names_[i], "position", &pos_rad_[i]);
    state_interfaces.emplace_back(joint_names_[i], "velocity", &vel_state_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
PhidgetsBldcHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joint_names_.size(); i++) {
    command_interfaces.emplace_back(joint_names_[i], "velocity", &cmd_[i]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn
PhidgetsBldcHardware::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PhidgetsBldcHardware"), "Activating...");
  
  RCLCPP_INFO(rclcpp::get_logger("PhidgetsBldcHardware"),
              "Activated. Motors ready.");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
PhidgetsBldcHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PhidgetsBldcHardware"), "Deactivating...");
  
  for (size_t i = 0; i < joint_names_.size(); i++) {
    if (motors_[i]) PhidgetBLDCMotor_setTargetVelocity(motors_[i], 0.0);
    close_phidget(i);
    close_temperature_sensor(i);
    attached_[i] = false;
    temperature_attached_[i] = false;
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
PhidgetsBldcHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Read from all motors
  for (size_t i = 0; i < joint_names_.size(); i++) {
    if (!motor_enabled_[i] || !attached_[i] || !motors_[i]) {
      vel_state_[i] = 0.0;
      pos_rad_[i] = 0.0;
      continue;
    }
    
    double vel_duty = 0.0, pos_val = 0.0;
    
    if (PhidgetBLDCMotor_getVelocity(motors_[i], &vel_duty) != EPHIDGET_OK) {
      disable_motor(i, "Velocity read failed during operation");
      continue;
    }
    
    if (PhidgetBLDCMotor_getPosition(motors_[i], &pos_val) != EPHIDGET_OK) {
      disable_motor(i, "Position read failed during operation");
      continue;
    }
    
    vel_state_[i] = vel_duty * direction_sign_[i];
    pos_rad_[i] = pos_val * 2.0 * M_PI * direction_sign_[i];
    
    if (temperature_sensors_[i]) {
      double temp = 0.0;
      if (PhidgetTemperatureSensor_getTemperature(temperature_sensors_[i], &temp) == EPHIDGET_OK) {
        temperature_c_[i] = temp;
      }
    }
  }
  
  publish_motor_telemetry();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
PhidgetsBldcHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Calculate average command per side based on ENABLED motors only
  double left_sum = 0.0, right_sum = 0.0;
  int left_count = 0, right_count = 0;
  
  for (size_t i = 0; i < joint_names_.size(); i++) {
    if (!motor_enabled_[i] || !attached_[i]) continue;
    
    if (is_left_wheel_[i]) {
      left_sum += cmd_[i];
      left_count++;
    } else {
      right_sum += cmd_[i];
      right_count++;
    }
  }
  
  double left_target = (left_count > 0) ? left_sum / left_count : 0.0;
  double right_target = (right_count > 0) ? right_sum / right_count : 0.0;
  
  // Send commands to enabled motors
  for (size_t i = 0; i < joint_names_.size(); i++) {
    if (!motor_enabled_[i] || !attached_[i] || !motors_[i]) continue;
    
    double target = is_left_wheel_[i] ? left_target : right_target;
    const double duty = clamp(target * direction_sign_[i], -command_limit_, command_limit_);
    
    if (PhidgetBLDCMotor_setTargetVelocity(motors_[i], duty) != EPHIDGET_OK) {
      disable_motor(i, "Command send failed during operation");
    }
  }
  
  return hardware_interface::return_type::OK;
}

void PhidgetsBldcHardware::close_phidget(int i)
{
  if (motors_[i]) {
    Phidget_close((PhidgetHandle)motors_[i]);
    PhidgetBLDCMotor_delete(&motors_[i]);
    motors_[i] = nullptr;
  }
}

void PhidgetsBldcHardware::close_temperature_sensor(int i)
{
  if (temperature_sensors_[i]) {
    Phidget_close((PhidgetHandle)temperature_sensors_[i]);
    PhidgetTemperatureSensor_delete(&temperature_sensors_[i]);
    temperature_sensors_[i] = nullptr;
  }
}

void PhidgetsBldcHardware::try_attach_motor(int i)
{
  close_phidget(i);
  attached_[i] = false;
  
  PhidgetReturnCode rc = PhidgetBLDCMotor_create(&motors_[i]);
  if (rc != EPHIDGET_OK || motors_[i] == nullptr) return;
  
  Phidget_setDeviceSerialNumber((PhidgetHandle)motors_[i], device_serial_[i]);
  Phidget_setHubPort((PhidgetHandle)motors_[i], hub_port_[i]);
  Phidget_setChannel((PhidgetHandle)motors_[i], channel_[i]);
  
  rc = Phidget_openWaitForAttachment((PhidgetHandle)motors_[i], 3000);
  if (rc != EPHIDGET_OK) {
    close_phidget(i);
    return;
  }
  
  attached_[i] = true;
  PhidgetBLDCMotor_setRescaleFactor(motors_[i], rescale_factor_rot_);
  PhidgetBLDCMotor_setAcceleration(motors_[i], acceleration_);
  PhidgetBLDCMotor_setStallVelocity(motors_[i], stall_velocity_);
  PhidgetBLDCMotor_setTargetVelocity(motors_[i], 0.0);
}

void PhidgetsBldcHardware::try_attach_temperature_sensor(int i)
{
  close_temperature_sensor(i);
  temperature_attached_[i] = false;
  
  PhidgetReturnCode rc = PhidgetTemperatureSensor_create(&temperature_sensors_[i]);
  if (rc != EPHIDGET_OK || temperature_sensors_[i] == nullptr) return;
  
  Phidget_setDeviceSerialNumber((PhidgetHandle)temperature_sensors_[i], device_serial_[i]);
  Phidget_setHubPort((PhidgetHandle)temperature_sensors_[i], hub_port_[i]);
  Phidget_setChannel((PhidgetHandle)temperature_sensors_[i], channel_[i]);
  
  rc = Phidget_openWaitForAttachment((PhidgetHandle)temperature_sensors_[i], 500);
  if (rc != EPHIDGET_OK) {
    close_temperature_sensor(i);
    return;
  }
  
  temperature_attached_[i] = true;
}

}  // namespace phidgets_hardware

PLUGINLIB_EXPORT_CLASS(phidgets_hardware::PhidgetsBldcHardware, hardware_interface::SystemInterface)