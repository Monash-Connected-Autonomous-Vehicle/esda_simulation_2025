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

namespace esda_hardware_2025 {
    class EsdaHardware2025 : public hardware_interface::SystemInterface {


        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(EsdaHardware2025);

            
    }
}