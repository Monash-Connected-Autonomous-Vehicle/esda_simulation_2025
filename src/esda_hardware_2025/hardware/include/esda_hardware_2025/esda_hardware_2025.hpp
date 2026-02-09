#ifndef ESDA_HARDWARE_REAL_HPP_
#define ESDA_HARDWARE_REAL_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "esda_hardware_2025/visibility_control.h"

#include "esda_hardware_2025/stm_comms.hpp"
#include "esda_hardware_2025/wheels.hpp"


namespace esda_hardware_2025 {
    class EsdaHardware2025 : public hardware_interface::SystemInterface {

        struct Config {
            std::string left_wheel_name = "";
            std::string right_wheel_name = "";
            float loop_rate = 0.0;
            std::string device = "";
            int baud_rate = 0;
            int timeout_ms = 0;
            int enc_counts_per_rev = 0;
            int pid_p = 0;
            int pid_d = 0;
            int pid_i = 0;
            int pid_o = 0;
        };

        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(EsdaHardware2025);

            ESDA_HARDWARE_2025_PUBLIC
            hardware_interface::CallbackReturn on_init(
                const hardware_interface::HardwareInfo & info
            ) override;
            
            ESDA_HARDWARE_2025_PUBLIC
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            ESDA_HARDWARE_2025_PUBLIC
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            ESDA_HARDWARE_2025_PUBLIC
            hardware_interface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State & previous_state) override;

            ESDA_HARDWARE_2025_PUBLIC
            hardware_interface::CallbackReturn on_cleanup(
                const rclcpp_lifecycle::State & previous_state) override;


            ESDA_HARDWARE_2025_PUBLIC
            hardware_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State & previous_state) override;

            ESDA_HARDWARE_2025_PUBLIC
            hardware_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State & previous_state) override;

            ESDA_HARDWARE_2025_PUBLIC
            hardware_interface::return_type read(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;

            ESDA_HARDWARE_2025_PUBLIC
            hardware_interface::return_type write(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private: 
            STMComms comms_;
            Config cfg_;
            Wheel wheel_l_;
            Wheel wheel_r_;
    };
}

#endif