#include "rs_motor_sdk/rs_motor.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
constexpr std::size_t kCount = 6;
constexpr auto kPeriod = std::chrono::milliseconds(20);
constexpr double kDefaultDuration = 5.0;
constexpr double kMinDuration = 1.0;
constexpr double kMaxDuration = 120.0;
constexpr double kPositionTolerance = 0.05;
using Clock = std::chrono::steady_clock;
std::atomic<bool> stop_requested{false};

void request_stop(int) { stop_requested.store(true); }

struct Options {
  double duration{kDefaultDuration};
  std::vector<std::size_t> joints;
};

Options parse_options(int argc, char **argv) {
  Options options;
  for (int i = 3; i < argc; ++i) {
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
                 "[joint-id ...]\n";
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
  for (auto i : sel)
    if (!bus.enable(i)) {
      std::cerr << bus.last_error() << '\n';
      stop();
      return 1;
    }
  const std::array<double, kCount> kp{50.0, 150.0, 150.0, 50.0, 50.0, 50.0};
  const std::array<double, kCount> kd{3.0, 5.0, 5.0, 5.0, 4.0, 4.0};
  std::array<double, kCount> start{};
  for (auto i : sel) {
    start[i] = state[i].position;
  }
  const double duration = options.duration;
  std::cout << "Minimum-jerk homing duration: " << duration << " s\n";
  const auto begin = Clock::now();
  std::array<Clock::time_point, kCount> last_feedback{};
  for (auto i : sel)
    last_feedback[i] = begin;
  bool ok = true;
  while (!stop_requested.load()) {
    const double elapsed =
        std::chrono::duration<double>(Clock::now() - begin).count();
    for (auto i : sel) {
      const auto target =
          rs_motor_sdk::minimum_jerk(start[i], 0.0, duration, elapsed);
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
      if (current.fault != 0) {
        std::cerr << "joint" << index + 1 << " fault=0x" << std::hex
                  << unsigned(current.fault) << std::dec
                  << "; homing stopped\n";
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
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  if (!feedback(bus, sel, state, std::chrono::milliseconds(500))) {
    std::cerr << "verification feedback timeout\n";
    stop();
    return 1;
  }
  for (auto i : sel)
    if (state[i].fault != 0 || std::abs(state[i].position) > kPositionTolerance)
      ok = false;
  for (auto i : sel)
    std::cout << "joint" << i + 1 << " -> " << state[i].position << " rad"
              << (std::abs(state[i].position) <= kPositionTolerance
                      ? " OK"
                      : " VERIFY FAILED")
              << '\n';
  stop();
  return ok ? 0 : 1;
}
