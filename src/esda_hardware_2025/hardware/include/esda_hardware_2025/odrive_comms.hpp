#ifndef ESDA_HARDWARE_2025_ODRIVE_COMMS_HPP
#define ESDA_HARDWARE_2025_ODRIVE_COMMS_HPP

// ODrive v3.x (firmware 0.5.x) CAN Simple client over Linux SocketCAN.
//
// Layer 1: protocol (IDs, frame builders, frame parsers) - no OS dependencies,
//          unit-testable on a laptop.
// Layer 2: ODriveCanComms - SocketCAN transport + latest-feedback table.
//
// The CAN interface must already be up on the host, e.g.
//   sudo ip link set can0 up type can bitrate 250000
// and the ODrive must use standard 11-bit IDs (can.config.is_extended = false).
//
// UNVERIFIED against firmware source (check can_simple.cpp at your firmware tag):
//   - payload layouts of the callbacks (set input vel, controller modes, etc.)
//   - whether encoder estimates are broadcast periodically (encoder_rate_ms)
//   - whether any CAN message feeds the ODrive watchdog

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace odrive
{

// ===========================================================================
// Layer 1: protocol
// ===========================================================================

/** CAN Simple command IDs (lower 5 bits of the 11-bit CAN ID). */
enum CommandId : uint32_t
{
  CMD_ODRIVE_HEARTBEAT = 0x001,
  CMD_ESTOP = 0x002,
  CMD_GET_MOTOR_ERROR = 0x003,
  CMD_GET_ENCODER_ERROR = 0x004,
  CMD_SENSORLESS_ERROR = 0x005,
  CMD_SET_AXIS_NODE_ID = 0x006,
  CMD_SET_AXIS_REQUESTED_STATE = 0x007,
  CMD_SET_AXIS_STARTUP_CONFIG = 0x008,
  CMD_GET_ENCODER_ESTIMATES = 0x009,
  CMD_GET_ENCODER_COUNT = 0x00A,
  CMD_SET_CONTROLLER_MODES = 0x00B,
  CMD_SET_INPUT_POS = 0x00C,
  CMD_SET_INPUT_VEL = 0x00D,
  CMD_SET_INPUT_TORQUE = 0x00E,
  CMD_SET_LIMITS = 0x00F,
  CMD_START_ANTICOGGING = 0x010,
  CMD_GET_IQ = 0x014,
  CMD_GET_SENSORLESS_ESTIMATES = 0x015,
  CMD_REBOOT_ODRIVE = 0x016,
  CMD_GET_BUS_VOLTAGE_CURRENT = 0x017,
  CMD_CLEAR_ERRORS = 0x018,
  CMD_SET_LINEAR_COUNT = 0x019,
  CMD_SET_POS_GAIN = 0x01A,
  CMD_SET_VEL_GAINS = 0x01B
};

enum AxisState : uint32_t
{
  AXIS_STATE_UNDEFINED = 0,
  AXIS_STATE_IDLE = 1,
  AXIS_STATE_STARTUP_SEQUENCE = 2,
  AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,
  AXIS_STATE_MOTOR_CALIBRATION = 4,
  AXIS_STATE_ENCODER_INDEX_SEARCH = 6,
  AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7,
  AXIS_STATE_CLOSED_LOOP_CONTROL = 8,
  AXIS_STATE_LOCKIN_SPIN = 9,
  AXIS_STATE_ENCODER_DIR_FIND = 10,
  AXIS_STATE_HOMING = 11,
  AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION = 12,
  AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION = 13
};

enum ControlMode : int32_t
{
  CONTROL_MODE_VOLTAGE = 0,
  CONTROL_MODE_TORQUE = 1,
  CONTROL_MODE_VELOCITY = 2,
  CONTROL_MODE_POSITION = 3
};

enum InputMode : int32_t
{
  INPUT_MODE_INACTIVE = 0,
  INPUT_MODE_PASSTHROUGH = 1,
  INPUT_MODE_VEL_RAMP = 2
};

/** Plain CAN frame (standard 11-bit ID), independent of the OS. */
struct CanFrame
{
  uint32_t id = 0;
  uint8_t dlc = 0;
  std::array<uint8_t, 8> data{};
};

namespace detail
{
template<typename T>
inline void writeType(std::array<uint8_t, 8> & buf, T value, size_t start)
{
  std::memcpy(&buf[start], &value, sizeof(T));
}

template<typename T>
inline T readType(const std::array<uint8_t, 8> & buf, size_t start)
{
  T value;
  std::memcpy(&value, &buf[start], sizeof(T));
  return value;
}
}  // namespace detail

inline uint32_t makeMsgId(uint8_t node_id, CommandId cmd)
{
  return (static_cast<uint32_t>(node_id) << 5) | (static_cast<uint32_t>(cmd) & 0x1F);
}

inline CanFrame makeFrame(uint8_t node_id, CommandId cmd, uint8_t dlc)
{
  CanFrame f;
  f.id = makeMsgId(node_id, cmd);
  f.dlc = dlc;
  return f;
}

// --- Outbound frame builders (Tx) -----------------------------------------

inline CanFrame createSetAxisStateMsg(uint8_t node_id, AxisState state)
{
  CanFrame f = makeFrame(node_id, CMD_SET_AXIS_REQUESTED_STATE, 4);
  detail::writeType<uint32_t>(f.data, static_cast<uint32_t>(state), 0);
  return f;
}

inline CanFrame createSetControllerModesMsg(
  uint8_t node_id, ControlMode control_mode, InputMode input_mode)
{
  CanFrame f = makeFrame(node_id, CMD_SET_CONTROLLER_MODES, 8);
  detail::writeType<int32_t>(f.data, static_cast<int32_t>(control_mode), 0);
  detail::writeType<int32_t>(f.data, static_cast<int32_t>(input_mode), 4);
  return f;
}

/** velocity in turns/s, torque feedforward in Nm. */
inline CanFrame createSetInputVelMsg(uint8_t node_id, float velocity, float torque_feedforward = 0.0f)
{
  CanFrame f = makeFrame(node_id, CMD_SET_INPUT_VEL, 8);
  detail::writeType<float>(f.data, velocity, 0);
  detail::writeType<float>(f.data, torque_feedforward, 4);
  return f;
}

inline CanFrame createClearErrorsMsg(uint8_t node_id)
{
  return makeFrame(node_id, CMD_CLEAR_ERRORS, 0);
}

inline CanFrame createEstopMsg(uint8_t node_id)
{
  return makeFrame(node_id, CMD_ESTOP, 0);
}

// --- Inbound frame parsers (Rx): return false if the frame is too short ----

inline bool parseHeartbeat(const CanFrame & f, uint32_t & axis_error, uint8_t & axis_state)
{
  if (f.dlc < 5) {
    return false;
  }
  axis_error = detail::readType<uint32_t>(f.data, 0);
  axis_state = f.data[4];
  return true;
}

/** pos in turns, vel in turns/s. */
inline bool parseEncoderEstimates(const CanFrame & f, float & pos_estimate, float & vel_estimate)
{
  if (f.dlc < 8) {
    return false;
  }
  pos_estimate = detail::readType<float>(f.data, 0);
  vel_estimate = detail::readType<float>(f.data, 4);
  return true;
}

// --- Unit conversion (ODrive uses turns, ros2_control uses radians) --------
constexpr double kTwoPi = 6.283185307179586;
inline double turnsToRad(double turns) {return turns * kTwoPi;}
inline double radToTurns(double rad) {return rad / kTwoPi;}

// ===========================================================================
// Layer 2: SocketCAN transport + feedback table
// ===========================================================================

struct AxisFeedback
{
  double pos_turns = 0.0;
  double vel_turns_per_s = 0.0;
  uint32_t axis_error = 0;
  uint8_t axis_state = 0;
  bool have_encoder = false;
  bool have_heartbeat = false;
  std::chrono::steady_clock::time_point last_encoder_time{};
  std::chrono::steady_clock::time_point last_heartbeat_time{};
};


class ODriveCanComms
{
public:
  ODriveCanComms() = default;
  ~ODriveCanComms() {disconnect();}

  ODriveCanComms(const ODriveCanComms &) = delete;
  ODriveCanComms & operator=(const ODriveCanComms &) = delete;

  // Register each node ID you expect (e.g. 0 and 1) before polling
  void add_axis(uint8_t node_id) {feedback_[node_id];}

  bool connect(const std::string & interface_name)
  {
    disconnect();

    sock_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
      RCLCPP_ERROR(logger(), "socket(PF_CAN) failed: %s", std::strerror(errno));
      return false;
    }

    ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
    if (::ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(
        logger(), "CAN interface '%s' not found (is it up? container on host network?): %s",
        interface_name.c_str(), std::strerror(errno));
      disconnect();
      return false;
    }

    sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(sock_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(logger(), "bind() to '%s' failed: %s", interface_name.c_str(), std::strerror(errno));
      disconnect();
      return false;
    }

    // Non-blocking so read()/write() in the control loop never stall.
    const int flags = ::fcntl(sock_, F_GETFL, 0);
    if (flags < 0 || ::fcntl(sock_, F_SETFL, flags | O_NONBLOCK) < 0) {
      RCLCPP_ERROR(logger(), "Failed to set O_NONBLOCK: %s", std::strerror(errno));
      disconnect();
      return false;
    }

    RCLCPP_INFO(logger(), "Connected to CAN interface '%s'", interface_name.c_str());
    return true;
  }

  void disconnect()
  {
    if (sock_ >= 0) {
      ::close(sock_);
      sock_ = -1;
    }
  }

  bool connected() const {return sock_ >= 0;}

  // ---- commands ----------------------------------------------------------
  bool set_axis_state(uint8_t node_id, AxisState state)
  {
    return send(createSetAxisStateMsg(node_id, state));
  }

  bool set_velocity_control_mode(uint8_t node_id, InputMode input_mode = INPUT_MODE_PASSTHROUGH)
  {
    return send(createSetControllerModesMsg(node_id, CONTROL_MODE_VELOCITY, input_mode));
  }

  /** Velocity in turns/s (use radToTurns() on a rad/s command). */
  bool set_velocity(uint8_t node_id, double vel_turns_per_s, double torque_ff_nm = 0.0)
  {
    return send(createSetInputVelMsg(
        node_id, static_cast<float>(vel_turns_per_s), static_cast<float>(torque_ff_nm)));
  }

  bool clear_errors(uint8_t node_id) {return send(createClearErrorsMsg(node_id));}
  bool estop(uint8_t node_id) {return send(createEstopMsg(node_id));}

  // ---- receiving ---------------------------------------------------------
  // Call at the start of read, poll() clears all encoder/heartbeat frames from queue and takes in the newest one to the object table
  int poll()
  {
    if (!connected()) {
      return 0;
    }
    int count = 0;
    can_frame raw;
    while (true) {
      const ssize_t n = ::read(sock_, &raw, sizeof(raw));
      if (n == static_cast<ssize_t>(sizeof(raw))) {
        // Ignore error, remote and extended frames.
        if (!(raw.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG | CAN_EFF_FLAG))) {
          CanFrame f;
          f.id = raw.can_id & CAN_SFF_MASK;
          f.dlc = raw.can_dlc > 8 ? 8 : raw.can_dlc;
          std::memcpy(f.data.data(), raw.data, f.dlc);
          process_frame(f, std::chrono::steady_clock::now());
          ++count;
        }
      } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        break;  // queue empty
      } else if (n < 0 && errno == EINTR) {
        continue;
      } else {
        RCLCPP_ERROR_THROTTLE(
          logger(), *clock_, 2000, "CAN read error: %s",
          n < 0 ? std::strerror(errno) : "short frame");
        break;
      }
    }
    return count;
  }

  // Called by poll(), writes the newest frame into the object table
  void process_frame(const CanFrame & f, std::chrono::steady_clock::time_point now)
  {
    const uint8_t node_id = static_cast<uint8_t>(f.id >> 5);
    const uint32_t cmd_id = f.id & 0x1F;

    auto it = feedback_.find(node_id);
    if (it == feedback_.end()) {
      return;  // not one of our axes
    }
    AxisFeedback & fb = it->second;

    if (cmd_id == CMD_GET_ENCODER_ESTIMATES) {
      float pos, vel;
      if (parseEncoderEstimates(f, pos, vel)) {
        fb.pos_turns = pos;
        fb.vel_turns_per_s = vel;
        fb.have_encoder = true;
        fb.last_encoder_time = now;
      }
    } else if (cmd_id == CMD_ODRIVE_HEARTBEAT) {
      uint32_t err;
      uint8_t state;
      if (parseHeartbeat(f, err, state)) {
        fb.axis_error = err;
        fb.axis_state = state;
        fb.have_heartbeat = true;
        fb.last_heartbeat_time = now;
      }
    }
  }

  // ---- queries -----------------------------------------------------------
  //Feedback, returns the most recent data for the specified wheel from the object table
  //Most recent data was recieved when poll() is called
  const AxisFeedback * feedback(uint8_t node_id) const
  {
    auto it = feedback_.find(node_id);
    return it == feedback_.end() ? nullptr : &it->second;
  }

  // Called for each wheel in read function, checks whether encoder data has arrived recently
  bool encoder_fresh(uint8_t node_id, int timeout_ms) const
  {
    const AxisFeedback * fb = feedback(node_id);
    if (fb == nullptr || !fb->have_encoder) {
      return false;
    }
    return (std::chrono::steady_clock::now() - fb->last_encoder_time) <
           std::chrono::milliseconds(timeout_ms);
  }

  bool heartbeat_fresh(uint8_t node_id, int timeout_ms) const
  {
    const AxisFeedback * fb = feedback(node_id);
    if (fb == nullptr || !fb->have_heartbeat) {
      return false;
    }
    return (std::chrono::steady_clock::now() - fb->last_heartbeat_time) <
           std::chrono::milliseconds(timeout_ms);
  }

private:
  static rclcpp::Logger logger() {return rclcpp::get_logger("ODriveCanComms");}

  bool send(const CanFrame & f)
  {
    if (!connected()) {
      return false;
    }
    can_frame raw;
    std::memset(&raw, 0, sizeof(raw));
    raw.can_id = f.id;  // standard frame
    raw.can_dlc = f.dlc;
    std::memcpy(raw.data, f.data.data(), f.dlc);

    const ssize_t n = ::write(sock_, &raw, sizeof(raw));
    if (n == static_cast<ssize_t>(sizeof(raw))) {
      return true;
    }
    RCLCPP_ERROR_THROTTLE(
      logger(), *clock_, 2000, "CAN write failed (id 0x%03X): %s",
      static_cast<unsigned>(f.id), n < 0 ? std::strerror(errno) : "short write");
    return false;
  }

  int sock_ = -1;
  std::map<uint8_t, AxisFeedback> feedback_;
  std::shared_ptr<rclcpp::Clock> clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
};

}  // namespace odrive

#endif  // ESDA_HARDWARE_2025_ODRIVE_COMMS_HPP