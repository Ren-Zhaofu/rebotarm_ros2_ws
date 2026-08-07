#include "rs_motor_sdk/rs_motor.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
constexpr std::size_t kCount = 6;
constexpr auto kPeriod = std::chrono::milliseconds(20);
constexpr double kArmDuration = 1.0;
constexpr double kDefaultDuration = 5.0;
constexpr double kMinDuration = 1.0;
constexpr double kMaxDuration = 120.0;
constexpr double kPositionTolerance = 0.05;
constexpr double kVelocityTolerance = 0.15;
constexpr double kSettleTimeout = 5.0;
constexpr std::size_t kSettleStableCycles = 10;
using Clock = std::chrono::steady_clock;
std::atomic<bool> stop_requested{false};

void request_stop(int) { stop_requested.store(true); }

struct Options {
  double duration{kDefaultDuration};
  bool verbose{false};
  std::vector<std::size_t> joints;
};

const char *source_name(rs_motor_sdk::CommunicationType source) {
  return source == rs_motor_sdk::CommunicationType::kActiveReport
             ? "active-report"
             : "command-feedback";
}

void print_state(const char *event, std::size_t index,
                 const rs_motor_sdk::MotorState &state, double elapsed,
                 double target_position, double target_velocity) {
  std::cout << std::fixed << std::setprecision(6) << "[home-debug t=" << elapsed
            << "s] " << event << " joint" << index + 1
            << " frame=" << source_name(state.source) << " can_id=0x"
            << std::hex << state.raw_can_id << std::dec
            << " mode=" << unsigned(state.mode) << " fault=0x" << std::hex
            << unsigned(state.fault) << std::dec
            << " target_pos=" << target_position
            << " actual_pos=" << state.position
            << " error=" << target_position - state.position
            << " target_vel=" << target_velocity
            << " actual_vel=" << state.velocity << " effort=" << state.effort
            << " temp=" << state.temperature << '\n'
            << std::flush;
}

Options parse_options(int argc, char **argv) {
  Options options;
  for (int i = 3; i < argc; ++i) {
    if (std::string(argv[i]) == "--verbose") {
      options.verbose = true;
      continue;
    }
    if (std::string(argv[i]) == "--duration") {
      if (++i >= argc)
        throw std::invalid_argument("--duration requires a value");
      std::size_t used = 0;
      options.duration = std::stod(argv[i], &used);
      if (used != std::string(argv[i]).size() ||
          !std::isfinite(options.duration) || options.duration < kMinDuration ||
          options.duration > kMaxDuration) {
        throw std::invalid_argument("duration must be from 1 to 120 seconds");
      }
      continue;
    }
    std::size_t used = 0;
    const auto n = std::stoul(argv[i], &used);
    if (used != std::string(argv[i]).size() || n < 1 || n > kCount)
      throw std::invalid_argument("joint ID must be from 1 to 6");
    if (std::find(options.joints.begin(), options.joints.end(), n - 1) ==
        options.joints.end())
      options.joints.push_back(n - 1);
  }
  if (options.joints.empty())
    throw std::invalid_argument("at least one joint ID is required");
  return options;
}

bool feedback(rs_motor_sdk::MotorBus &bus, const std::vector<std::size_t> &sel,
              std::array<rs_motor_sdk::MotorState, kCount> &states,
              std::chrono::milliseconds timeout) {
  std::array<bool, kCount> seen{};
  const auto end = Clock::now() + timeout;
  while (Clock::now() < end) {
    rs_motor_sdk::MotorState s;
    std::size_t i = 0;
    if (bus.receive_state(s, i, std::chrono::milliseconds(10))) {
      states[i] = s;
      seen[i] = true;
    }
    if (std::all_of(sel.begin(), sel.end(), [&](auto i) { return seen[i]; }))
      return true;
  }
  return false;
}
} // namespace

int main(int argc, char **argv) {
  if (argc < 4 || std::string(argv[1]) != "--execute") {
    std::cerr << "usage: " << argv[0]
              << " --execute <can-interface> [--duration seconds] <joint-id> "
                 "[joint-id ...] [--verbose]\n";
    return 2;
  }
  Options options;
  try {
    options = parse_options(argc, argv);
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 2;
  }
  const auto &sel = options.joints;
  const std::array<rs_motor_sdk::MotorModel, kCount> models{
      rs_motor_sdk::MotorModel::kRs06, rs_motor_sdk::MotorModel::kRs06,
      rs_motor_sdk::MotorModel::kRs06, rs_motor_sdk::MotorModel::kRs00,
      rs_motor_sdk::MotorModel::kRs00, rs_motor_sdk::MotorModel::kRs00};
  rs_motor_sdk::MotorBus bus;
  for (std::size_t i = 0; i < kCount; ++i)
    bus.add_motor({"joint" + std::to_string(i + 1),
                   static_cast<std::uint8_t>(i + 1), 0xFD, models[i]});
  if (!bus.connect(argv[2])) {
    std::cerr << bus.last_error() << '\n';
    return 1;
  }
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  std::array<bool, kCount> reporting{};
  const auto stop = [&]() {
    for (auto i : sel)
      bus.disable(i);
    for (auto i : sel) {
      if (reporting[i])
        bus.set_active_report(i, false);
    }
    bus.disconnect();
  };
  for (auto i : sel) {
    if (!bus.set_active_report(i, true)) {
      std::cerr << bus.last_error() << '\n';
      stop();
      return 1;
    }
    reporting[i] = true;
  }
  std::array<rs_motor_sdk::MotorState, kCount> state{};
  if (!feedback(bus, sel, state, std::chrono::milliseconds(800))) {
    std::cerr << "feedback timeout; homing aborted\n";
    stop();
    return 1;
  }
  for (auto i : sel)
    if (state[i].fault != 0) {
      std::cerr << "joint" << i + 1 << " fault=0x" << std::hex
                << unsigned(state[i].fault) << std::dec << '\n';
      stop();
      return 1;
    }
  std::array<double, kCount> start{};
  for (auto i : sel) {
    start[i] = state[i].position;
    if (options.verbose)
      print_state("initial", i, state[i], 0.0, start[i], 0.0);
  }
  // Conservative homing gains. High derivative gains amplify the measurable
  // standstill velocity noise and can make multiple enabled joints oscillate.
  const std::array<double, kCount> kp{20.0, 40.0, 40.0, 15.0, 12.0, 10.0};
  const std::array<double, kCount> kd{0.5, 0.8, 0.8, 0.4, 0.3, 0.3};

  // Preload a torque-free hold command so enable cannot apply a stale target.
  for (auto i : sel) {
    if (!bus.send_mit(i, {start[i], 0.0, 0.0, 0.0, 0.0})) {
      std::cerr << "joint" << i + 1
                << " pre-enable command: " << bus.last_error() << '\n';
      stop();
      return 1;
    }
  }
  std::vector<std::size_t> armed_joints;
  bool armed = true;
  for (auto i : sel) {
    if (!bus.enable(i)) {
      std::cerr << bus.last_error() << '\n';
      stop();
      return 1;
    }
    if (!bus.send_mit(i, {start[i], 0.0, 0.0, 0.0, 0.0})) {
      std::cerr << "joint" << i + 1
                << " post-enable command: " << bus.last_error() << '\n';
      stop();
      return 1;
    }
    armed_joints.push_back(i);
    if (options.verbose) {
      std::cout << "[home-debug] enable sent to joint" << i + 1 << '\n';
    }
    std::cout << "Soft-starting joint" << i + 1 << " hold: " << kArmDuration
              << " s\n";
    const auto arm_begin = Clock::now();
    while (!stop_requested.load()) {
      const double elapsed =
          std::chrono::duration<double>(Clock::now() - arm_begin).count();
      const double gain_scale = std::clamp(elapsed / kArmDuration, 0.0, 1.0);
      for (auto enabled : armed_joints) {
        const double scale = enabled == i ? gain_scale : 1.0;
        if (!bus.send_mit(enabled, {start[enabled], 0.0, kp[enabled] * scale,
                                    kd[enabled] * scale, 0.0})) {
          std::cerr << "joint" << enabled + 1
                    << " soft-start command: " << bus.last_error() << '\n';
          armed = false;
          break;
        }
      }
      for (std::size_t n = 0; armed && n < sel.size() * 2; ++n) {
        rs_motor_sdk::MotorState current;
        std::size_t index = 0;
        if (!bus.receive_state(current, index, std::chrono::milliseconds(1)))
          break;
        if (std::find(armed_joints.begin(), armed_joints.end(), index) ==
            armed_joints.end())
          continue;
        state[index] = current;
        if (options.verbose && index == i && elapsed <= 0.05)
          print_state("after-enable", index, current, elapsed, start[index],
                      0.0);
        if (current.fault != 0) {
          print_state("soft-start-fault", index, current, elapsed, start[index],
                      0.0);
          std::cerr << "joint" << index + 1 << " fault=0x" << std::hex
                    << unsigned(current.fault) << std::dec
                    << " while enabling joint" << i + 1 << "; homing stopped\n";
          armed = false;
          break;
        }
      }
      if (!armed || elapsed >= kArmDuration)
        break;
      std::this_thread::sleep_for(kPeriod);
    }
    if (!armed || stop_requested.load())
      break;
  }
  if (!armed || stop_requested.load()) {
    stop();
    return 1;
  }
  if (!feedback(bus, sel, state, std::chrono::milliseconds(500))) {
    std::cerr << "feedback timeout after soft start; homing aborted\n";
    stop();
    return 1;
  }
  for (auto i : sel) {
    if (state[i].fault != 0) {
      std::cerr << "joint" << i + 1 << " fault=0x" << std::hex
                << unsigned(state[i].fault) << std::dec << '\n';
      stop();
      return 1;
    }
    start[i] = state[i].position;
  }
  const double duration = options.duration;
  std::cout << "Minimum-jerk homing duration: " << duration << " s\n";
  const auto begin = Clock::now();
  std::array<Clock::time_point, kCount> last_feedback{};
  std::array<Clock::time_point, kCount> last_log{};
  std::array<double, kCount> target_position = start;
  std::array<double, kCount> target_velocity{};
  for (auto i : sel)
    last_feedback[i] = last_log[i] = begin;
  bool ok = true;
  while (!stop_requested.load()) {
    const double elapsed =
        std::chrono::duration<double>(Clock::now() - begin).count();
    for (auto i : sel) {
      const auto target =
          rs_motor_sdk::minimum_jerk(start[i], 0.0, duration, elapsed);
      target_position[i] = target.position;
      target_velocity[i] = target.velocity;
      if (!bus.send_mit(
              i, {target.position, target.velocity, kp[i], kd[i], 0.0})) {
        std::cerr << "joint" << i + 1 << " command: " << bus.last_error()
                  << '\n';
        ok = false;
        break;
      }
    }
    for (std::size_t n = 0; n < sel.size() * 2; ++n) {
      rs_motor_sdk::MotorState current;
      std::size_t index = 0;
      if (!bus.receive_state(current, index, std::chrono::milliseconds(1)))
        break;
      if (std::find(sel.begin(), sel.end(), index) == sel.end())
        continue;
      state[index] = current;
      last_feedback[index] = Clock::now();
      if (options.verbose &&
          (elapsed <= 0.3 ||
           Clock::now() - last_log[index] >= std::chrono::milliseconds(200))) {
        print_state("control", index, current, elapsed, target_position[index],
                    target_velocity[index]);
        last_log[index] = Clock::now();
      }
      if (current.fault != 0) {
        print_state("fault", index, current, elapsed, target_position[index],
                    target_velocity[index]);
        std::cerr << "joint" << index + 1 << " fault=0x" << std::hex
                  << unsigned(current.fault) << std::dec << " from "
                  << source_name(current.source) << "; homing stopped\n";
        ok = false;
        break;
      }
    }
    for (auto i : sel) {
      if (Clock::now() - last_feedback[i] > std::chrono::milliseconds(500)) {
        std::cerr << "joint" << i + 1 << " feedback timeout; homing stopped\n";
        ok = false;
        break;
      }
    }
    if (!ok || elapsed >= duration)
      break;
    std::this_thread::sleep_for(kPeriod);
  }
  if (stop_requested.load()) {
    std::cerr << "Homing interrupted; disabling selected motors\n";
    ok = false;
  }
  if (!ok) {
    stop();
    return 1;
  }
  std::cout << "Settling at zero for up to " << kSettleTimeout << " s\n";
  const auto settle_begin = Clock::now();
  bool settled = false;
  std::size_t stable_cycles = 0;
  while (!stop_requested.load()) {
    for (auto i : sel) {
      if (!bus.send_mit(i, {0.0, 0.0, kp[i], kd[i], 0.0})) {
        std::cerr << "joint" << i + 1 << " settle command: " << bus.last_error()
                  << '\n';
        ok = false;
        break;
      }
    }
    if (!ok)
      break;
    if (!feedback(bus, sel, state, std::chrono::milliseconds(100))) {
      std::cerr << "settling feedback timeout\n";
      ok = false;
      break;
    }
    const bool within_tolerance =
        std::all_of(sel.begin(), sel.end(), [&](auto i) {
          return state[i].fault == 0 &&
                 std::abs(state[i].position) <= kPositionTolerance &&
                 std::abs(state[i].velocity) <= kVelocityTolerance;
        });
    stable_cycles = within_tolerance ? stable_cycles + 1 : 0;
    settled = stable_cycles >= kSettleStableCycles;
    if (settled)
      break;
    const double settle_elapsed =
        std::chrono::duration<double>(Clock::now() - settle_begin).count();
    if (settle_elapsed >= kSettleTimeout)
      break;
    std::this_thread::sleep_for(kPeriod);
  }
  if (!settled) {
    std::cerr << (stop_requested.load() ? "Homing interrupted during settling\n"
                                        : "Zero settling timeout\n");
    ok = false;
  }
  for (auto i : sel)
    std::cout << "joint" << i + 1 << " -> " << state[i].position << " rad"
              << ", " << state[i].velocity << " rad/s"
              << (std::abs(state[i].position) <= kPositionTolerance &&
                          std::abs(state[i].velocity) <= kVelocityTolerance
                      ? " OK"
                      : " VERIFY FAILED")
              << '\n';
  stop();
  return ok ? 0 : 1;
}
