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
        cfg_.can_interface = info_.hardware_parameters["can_interface"];
        if (info_.hardware_parameters.count("feedback_timeout_ms") > 0) {
            cfg_.feedback_timeout_ms = std::stoi(info_.hardware_parameters["feedback_timeout_ms"]);
        }
        cfg_.gear_ratio = std::stod(info_.hardware_parameters["gear_ratio"]);
        cfg_.heartbeat_timeout_ms = std::stoi(info_.hardware_parameters["heartbeat_timeout_ms"]);
        cfg_.left_node_id = std::stoi(info_.hardware_parameters["left_node_id"]);
        cfg_.right_node_id = std::stoi(info_.hardware_parameters["right_node_id"]);
        cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

        auto to_bool = [](const std::string &s) {
            return s == "true" || s == "1" || s == "True" || s == "TRUE";
            };

        if (info_.hardware_parameters.count("fake_odom") > 0) {
            cfg_.fake_odom = to_bool(info_.hardware_parameters["fake_odom"]);

            RCLCPP_INFO(rclcpp::get_logger("EsdaHardware2025"), "fake_odom parameter supplied, using value: %s", cfg_.fake_odom ? "true" : "false");

        }

        RCLCPP_WARN(rclcpp::get_logger("EsdaHardware2025"), "fake_odom = %s", cfg_.fake_odom ? "true" : "false");

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
        //Check for fake_odom before checking whether can0 port is open
        if (!cfg_.fake_odom && !comms_.connected()) {
            return hardware_interface::return_type::ERROR;
        }
        
        //Clear can0 socket queue of CAN frames, take the most recent frame for both wheels and store
        comms_.poll();
        //Check whether can frames are fresh
        if (!comms_.encoder_fresh(cfg_.left_node_id, cfg_.feedback_timeout_ms) ||
            !comms_.encoder_fresh(cfg_.right_node_id, cfg_.feedback_timeout_ms) ||
            !comms_.heartbeat_fresh(cfg_.left_node_id, cfg_.heartbeat_timeout_ms) ||
            !comms_.heartbeat_fresh(cfg_.right_node_id, cfg_.heartbeat_timeout_ms)) {
            return hardware_interface::return_type::ERROR;
        }

        // Retrieve the latest data for the left and right wheel from the object table, copy them into wheel variables
        const auto * fb_l = comms_.feedback(cfg_.left_node_id);
        const auto * fb_r = comms_.feedback(cfg_.right_node_id);

        //Some error checks
        if (fb_l == nullptr || fb_r == nullptr) {
            return hardware_interface::return_type::ERROR;
        }

        if (fb_l->axis_error != 0 || fb_r->axis_error != 0) {
            return hardware_interface::return_type::ERROR;
        }

        // Convert from turns (as used in the Odrive), to rads (as used in controller)
        wheel_l_.pos = odrive::turnsToRad(fb_l->pos_turns) / cfg_.gear_ratio;
        wheel_l_.vel = odrive::turnsToRad(fb_l->vel_turns_per_s) / cfg_.gear_ratio;




        //Fake odometry mode stuff (for testing software purely)
        double delta_seconds = period.seconds();
        if (cfg_.fake_odom) {
            // Fake: measured vel = commanded vel, integrate to position
            wheel_l_.vel = wheel_l_.cmd;
            wheel_r_.vel = wheel_r_.cmd;

            wheel_l_.pos += wheel_l_.vel * delta_seconds;
            wheel_r_.pos += wheel_r_.vel * delta_seconds;

            return hardware_interface::return_type::OK;
        }   
        
        
        
        wheel_l_.vel = (wheel_l_.pos - pos_prev) / delta_seconds;
        
        wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;

        
        
        return hardware_interface::return_type::OK;
    }
    
    hardware_interface::return_type esda_hardware_2025 ::EsdaHardware2025::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        if (cfg_.fake_odom) {
            return hardware_interface::return_type::OK;  // don't touch serial at all
        }
        
        if (!comms_.connected()) {
            return hardware_interface::return_type::ERROR;
        }

        // Converting rads to motor turns for each wheel
        double motor_turns_per_s_l = odrive::radToTurns(wheel_l_.cmd) * cfg_.gear_ratio;
        double motor_turns_per_s_r = odrive::radToTurns(wheel_r_.cmd) * cfg_.gear_ratio;

        //Checking whether velocity commands sent to O-drive
        bool left_sent = comms_.set_velocity(cfg_.left_node_id, motor_turns_per_s_l);
        bool right_sent = comms_.set_velocity(cfg_.right_node_id, motor_turns_per_s_r);

        if (!left_ok || !right_ok) {
            return hardware_interface::return_type::ERROR;
        }
        
        return hardware_interface::return_type::OK;
    }
      
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  esda_hardware_2025::EsdaHardware2025, hardware_interface::SystemInterface)