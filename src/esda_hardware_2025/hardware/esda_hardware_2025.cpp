#include "esda_hardware_2025/esda_hardware_2025.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <filesystem>

static std::string read_file(const std::string &path)
{
    std::ifstream file(path);
    if (!file.is_open()) return "unknown";

    std::string value;
    std::getline(file, value);
    return value;
}

static std::string get_usb_description(const std::string &tty)
{
    // tty = "/dev/ttyACM0" → "ttyACM0"
    std::string name = std::filesystem::path(tty).filename();

    std::string base = "/sys/class/tty/" + name + "/device/../";

    std::string vendor  = read_file(base + "idVendor");
    std::string product = read_file(base + "idProduct");
    std::string manuf   = read_file(base + "manufacturer");
    std::string prod    = read_file(base + "product");

    std::stringstream ss;
    ss << manuf << " " << prod << " (" << vendor << ":" << product << ")";
    return ss.str();
}

namespace esda_hardware_2025 {

    hardware_interface::CallbackReturn EsdaHardware2025::on_init(const hardware_interface::HardwareInfo & info) {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
        cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
        cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
        cfg_.device = info_.hardware_parameters["device"];
        cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
        cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
        cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
        if (info_.hardware_parameters.count("pid_p") > 0) {
            cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
            cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
            cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
            cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("EsdaHardware2025"), "PID values not supplied, using defaults.");
        }

        wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
        wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

        for (const hardware_interface::ComponentInfo & joint : info_.joints)
        {
          // DiffBotSystem has exactly two states and one command interface on each joint
          if (joint.command_interfaces.size() != 1)
          {
            RCLCPP_FATAL(
              rclcpp::get_logger("EsdaHardware2025"),
              "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
              joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
          }
      
          if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
          {
            RCLCPP_FATAL(
              rclcpp::get_logger("EsdaHardware2025"),
              "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
              joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
          }
      
          if (joint.state_interfaces.size() != 2)
          {
            RCLCPP_FATAL(
              rclcpp::get_logger("EsdaHardware2025"),
              "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
              joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
          }
      
          if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
          {
            RCLCPP_FATAL(
              rclcpp::get_logger("EsdaHardware2025"),
              "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
              joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
          }
      
          if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
          {
            RCLCPP_FATAL(
              rclcpp::get_logger("EsdaHardware2025"),
              "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
              joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
          }
        }
      
        return hardware_interface::CallbackReturn::SUCCESS;

    }

    std::vector<hardware_interface::StateInterface> EsdaHardware2025::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> EsdaHardware2025::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

        return command_interfaces;
    }

    hardware_interface::CallbackReturn EsdaHardware2025::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("EsdaHardware2025"), "Configuring ...please wait...");

        // RCLCPP_INFO(rclcpp::get_logger("EsdaHardware2025"))
      
        if (comms_.connected()) {
          comms_.disconnect();
        }
        comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);

        // 🔹 Query USB identity    
        std::string usb_desc = get_usb_description(cfg_.device);

        RCLCPP_INFO(rclcpp::get_logger("EsdaHardware2025"),
                    "Connected to: %s",
                    usb_desc.c_str());

        RCLCPP_INFO(rclcpp::get_logger("EsdaHardware2025"), "Successfully configured!");
      
        return hardware_interface::CallbackReturn::SUCCESS;
      }
      
    hardware_interface::CallbackReturn EsdaHardware2025::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("EsdaHardware2025"), "Cleaning up ...please wait...");
        if (comms_.connected()) {
            comms_.disconnect();
        }
        RCLCPP_INFO(rclcpp::get_logger("EsdaHardware2025"), "Successfully cleaned up!");
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    
    
    hardware_interface::CallbackReturn EsdaHardware2025::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("EsdaHardware2025"), "Activating ...please wait...");
        if (!comms_.connected()) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (cfg_.pid_p > 0) {
            comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
        }
        RCLCPP_INFO(rclcpp::get_logger("EsdaHardware2025"), "Successfully activated!");
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn EsdaHardware2025::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_INFO(rclcpp::get_logger("EsdaHardware2025"), "Deactivating ...please wait...");
        RCLCPP_INFO(rclcpp::get_logger("EsdaHardware2025"), "Successfully deactivated!");
      
        return hardware_interface::CallbackReturn::SUCCESS;
    }
      
    hardware_interface::return_type EsdaHardware2025::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period) {
        if (!comms_.connected()) {
            return hardware_interface::return_type::ERROR;
        }
        
        comms_.read_encoder_values(wheel_l_.enc, wheel_r_.enc);
        
        double delta_seconds = period.seconds();
        
        double pos_prev = wheel_l_.pos;
        wheel_l_.pos = wheel_l_.calc_enc_angle();
        wheel_l_.vel = (wheel_l_.pos - pos_prev) / delta_seconds;
        
        pos_prev = wheel_r_.pos;
        wheel_r_.pos = wheel_r_.calc_enc_angle();
        wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;

        
        
        return hardware_interface::return_type::OK;
    }
    
    hardware_interface::return_type esda_hardware_2025 ::EsdaHardware2025::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        if (!comms_.connected()) {
            return hardware_interface::return_type::ERROR;
        }
        
        int motor_l_counts_per_loop = wheel_l_.cmd / wheel_l_.rads_per_count / cfg_.loop_rate;
        int motor_r_counts_per_loop = wheel_r_.cmd / wheel_r_.rads_per_count / cfg_.loop_rate;
        comms_.set_motor_values(motor_l_counts_per_loop, motor_r_counts_per_loop);
        return hardware_interface::return_type::OK;
    }
      
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  esda_hardware_2025::EsdaHardware2025, hardware_interface::SystemInterface)