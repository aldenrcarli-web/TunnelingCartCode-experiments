// Combined controller for the supplied Unitree cart and GPIO actuator programs.
// See README.md. Simulation is the default, including in a hardware-enabled build.
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

#ifdef ROBOT_HARDWARE
#include <gpiod.h>
#include "unitreeMotor/unitreeMotor.h"
#include "serialPort/SerialPort.h"
#endif

using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;
using namespace std::chrono_literals;

namespace config {
constexpr double wheelSlowRotor = 6.28 * 3.0;  // Retains the supplied program.
constexpr double wheelFastRotor = 6.28 * 6.33;
constexpr double fullStrokeInches = 4.0;
constexpr double fullStrokeSeconds = 10.0;
constexpr auto relayDelay = 100ms;
constexpr auto motorPeriod = 20ms;
constexpr auto feedbackLimit = 250ms;
constexpr auto startupLimit = 2s;
constexpr double maxMoveSeconds = 30.0;
constexpr double rotorAcceleration = 60.0; // rad/s^2; tune to your mechanism.
constexpr int temperatureLimit = 70;       // Application limit, not a motor rating.
}

volatile std::sig_atomic_t receivedSignal = 0;
void handleSignal(int signal) { receivedSignal = signal; }
double seconds(Time end, Time begin) {
    return std::chrono::duration<double>(end - begin).count();
}
Time after(Time now, double duration) {
    return now + std::chrono::duration_cast<Clock::duration>(
                     std::chrono::duration<double>(duration));
}

struct Options {
    bool hardware = false;
    bool plain = false;
    std::string wheelPort = "/dev/ttyUSB0";
    int wheelId = 0;
    std::string rotationPort;
    int rotationId = -1; // No guessed rotation ID or automatic second-port opening.
    double rotationOutputSpeed = 0.5; // rad/s; provisional jog setting, not 90-degree positioning.
    std::string chip = "gpiochip4";
    bool rotationEnabled() const { return !rotationPort.empty(); }
};

double parseNumber(const std::string& value) {
    std::size_t used = 0;
    double result;
    try { result = std::stod(value, &used); }
    catch (...) { throw std::runtime_error("Invalid number: " + value); }
    if (used != value.size() || !std::isfinite(result))
        throw std::runtime_error("Number must be finite: " + value);
    return result;
}
int parseId(const std::string& value) {
    const double n = parseNumber(value);
    if (n < 0 || n > 14 || std::floor(n) != n)
        throw std::runtime_error("Motor ID must be an integer from 0 to 14");
    return static_cast<int>(n);
}
Options parseOptions(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto value = [&]() -> std::string {
            if (i + 1 == argc) throw std::runtime_error("Missing value for " + arg);
            return argv[++i];
        };
        if (arg == "--hardware") o.hardware = true;
        else if (arg == "--plain") o.plain = true;
        else if (arg == "--wheel-port") o.wheelPort = value();
        else if (arg == "--wheel-id") o.wheelId = parseId(value());
        else if (arg == "--rotation-port") o.rotationPort = value();
        else if (arg == "--rotation-id") o.rotationId = parseId(value());
        else if (arg == "--rotation-speed") o.rotationOutputSpeed = parseNumber(value());
        else if (arg == "--chip") o.chip = value();
        else if (arg == "--help") {
            std::cout << "Usage: combined_controller [--hardware] [--plain]\n"
                         "  [--wheel-port /dev/ttyUSB0] [--wheel-id 0]\n"
                         "  [--rotation-port PORT --rotation-id ID --rotation-speed OUTPUT_RAD_S]\n"
                         "  [--chip gpiochip4]\nSimulation is the default. See README.md.\n";
            std::exit(0);
        } else throw std::runtime_error("Unknown option: " + arg);
    }
    if (o.rotationEnabled() != (o.rotationId >= 0))
        throw std::runtime_error("Supply both --rotation-port and --rotation-id, or neither");
    if (o.rotationOutputSpeed <= 0 || o.rotationOutputSpeed > 2.0)
        throw std::runtime_error("Rotation jog speed must be > 0 and <= 2 output rad/s");
    if (o.rotationEnabled()) {
        struct stat a{}, b{};
        const bool sameDevice = stat(o.wheelPort.c_str(), &a) == 0 &&
            stat(o.rotationPort.c_str(), &b) == 0 && S_ISCHR(a.st_mode) &&
            S_ISCHR(b.st_mode) && a.st_rdev == b.st_rdev;
        if (o.wheelPort == o.rotationPort || sameDevice)
            throw std::runtime_error("Wheel and rotation require separate serial adapters in this version");
    }
#ifndef ROBOT_HARDWARE
    if (o.hardware) throw std::runtime_error("Rebuild with -DROBOT_HARDWARE=ON to use real hardware");
#endif
    return o;
}

class Safety {
    std::mutex mutex_;
    std::string fault_;
public:
    std::atomic<bool> stopped{true};
    void trip(const std::string& reason) {
        std::lock_guard<std::mutex> lock(mutex_);
        stopped.store(true);
        if (fault_.empty()) fault_ = reason;
    }
    bool arm() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!fault_.empty()) return false;
        stopped.store(false);
        return true;
    }
    std::string fault() {
        std::lock_guard<std::mutex> lock(mutex_);
        return fault_;
    }
};

struct MotorStatus {
    bool ready = false;
    bool failed = false;
    Time feedback{};
    double command = 0.0;
    double measured = 0.0;
    double position = 0.0;
    int temperature = 0;
    std::string error;
};

// Exactly one worker owns each SerialPort, MotorCmd and MotorData. A slow motor
// exchange cannot block the UI or the GPIO actuator timer.
class MotorAxis {
    std::string name_, port_;
    int id_;
    bool hardware_;
    Safety& safety_;
    mutable std::mutex mutex_;
    double requested_ = 0.0;
    Time deadline_{};
    MotorStatus status_;
    std::atomic<bool> exiting_{false};
    std::thread worker_;

    void run() noexcept {
#ifdef ROBOT_HARDWARE
        std::unique_ptr<SerialPort> serial;
        MotorCmd cmd;
        MotorData data;
        // MotorCmd's published constructor does not initialize these fields.
        cmd.motorType = MotorType::GO_M8010_6;
        cmd.id = static_cast<unsigned short>(id_);
        cmd.mode = queryMotorMode(cmd.motorType, MotorMode::BRAKE);
        cmd.q = cmd.dq = cmd.kp = cmd.kd = cmd.tau = 0.0f;
        data.motorType = MotorType::GO_M8010_6;
#endif
        try {
#ifdef ROBOT_HARDWARE
            if (hardware_) serial = std::make_unique<SerialPort>(port_);
#endif
            auto previous = Clock::now();
            auto tick = previous;
            double applied = 0.0, simulatedPosition = 0.0;
            while (!exiting_.load()) {
                const auto now = Clock::now();
                double desired;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (now >= deadline_) requested_ = 0.0;
                    desired = requested_;
                }
                if (safety_.stopped.load()) desired = 0.0;
                const double dt = std::clamp(seconds(now, previous), 0.0, 0.1);
                previous = now;
                if (desired == 0.0) applied = 0.0; // Stop requests bypass the jog ramp.
                else {
                    const double target = applied * desired < 0.0 ? 0.0 : desired;
                    const double step = config::rotorAcceleration * dt;
                    applied += std::clamp(target - applied, -step, step);
                }
                MotorStatus fresh;
                fresh.ready = true;
                fresh.command = applied;
#ifdef ROBOT_HARDWARE
                if (hardware_) {
                    cmd.mode = queryMotorMode(cmd.motorType,
                        applied == 0.0 ? MotorMode::BRAKE : MotorMode::FOC);
                    cmd.q = 0.0f;
                    cmd.dq = static_cast<float>(applied);
                    cmd.kp = 0.0f;
                    cmd.kd = applied == 0.0 ? 0.0f : 0.01f;
                    cmd.tau = 0.0f;
                    data.correct = false; // Never treat an earlier packet as new feedback.
                    serial->sendRecv(&cmd, &data);
                    if (!data.correct || data.motor_id != id_)
                        throw std::runtime_error("No valid matching feedback");
                    if (data.merror != 0)
                        throw std::runtime_error("Motor error " + std::to_string(data.merror));
                    if (!std::isfinite(data.q) || !std::isfinite(data.dq))
                        throw std::runtime_error("Non-finite motor feedback");
                    if (data.temp >= config::temperatureLimit)
                        throw std::runtime_error("Application temperature limit reached");
                    fresh.position = data.q;
                    fresh.measured = data.dq;
                    fresh.temperature = data.temp;
                } else
#endif
                {
                    simulatedPosition += applied * dt;
                    fresh.position = simulatedPosition;
                    fresh.measured = applied;
                    fresh.temperature = 25;
                }
                fresh.feedback = Clock::now();
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    status_ = fresh;
                }
                tick += config::motorPeriod;
                if (tick < Clock::now()) tick = Clock::now();
                std::this_thread::sleep_until(tick);
            }
        } catch (const std::exception& e) {
            safety_.trip(name_ + ": " + e.what());
            std::lock_guard<std::mutex> lock(mutex_);
            status_.failed = true;
            status_.ready = false;
            status_.error = e.what();
        } catch (...) {
            safety_.trip(name_ + ": unknown motor exception");
            std::lock_guard<std::mutex> lock(mutex_);
            status_.failed = true;
            status_.ready = false;
            status_.error = "unknown exception";
        }
#ifdef ROBOT_HARDWARE
        if (serial) {
            // Best effort: a failed link cannot guarantee that a stop arrives.
            try {
                cmd.mode = queryMotorMode(cmd.motorType, MotorMode::BRAKE);
                cmd.q = cmd.dq = cmd.kp = cmd.kd = cmd.tau = 0.0f;
                serial->sendRecv(&cmd, &data);
            } catch (...) {}
        }
#endif
    }
public:
    MotorAxis(std::string name, std::string port, int id, bool hardware, Safety& safety)
        : name_(std::move(name)), port_(std::move(port)), id_(id), hardware_(hardware),
          safety_(safety), worker_([this] { run(); }) {}
    ~MotorAxis() { shutdown(); }
    MotorAxis(const MotorAxis&) = delete;
    MotorAxis& operator=(const MotorAxis&) = delete;
    void shutdown() {
        stop();
        exiting_.store(true);
        if (worker_.joinable()) worker_.join();
    }
    void move(double rotorSpeed, double duration) {
        std::lock_guard<std::mutex> lock(mutex_);
        requested_ = rotorSpeed;
        deadline_ = after(Clock::now(), duration);
    }
    void stop() {
        std::lock_guard<std::mutex> lock(mutex_);
        requested_ = 0.0;
        deadline_ = Time{};
    }
    MotorStatus status() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return status_;
    }
};

class HardwareError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

class Actuator {
    bool hardware_;
    bool powered_ = false;
    bool retract_ = false;
    enum class Phase { Idle, PowerOffWait, DirectionWait, Moving } phase_ = Phase::Idle;
    Time due_{};
    double duration_ = 0.0;
#ifdef ROBOT_HARDWARE
    gpiod_chip* chip_ = nullptr;
    gpiod_line* direction_ = nullptr;
    gpiod_line* power_ = nullptr;
    bool directionOwned_ = false, powerOwned_ = false;
#endif
    void power(bool on) {
#ifdef ROBOT_HARDWARE
        if (hardware_ && gpiod_line_set_value(power_, on ? 1 : 0) < 0)
            throw HardwareError("Failed to set actuator power GPIO");
#endif
        powered_ = on;
    }
    void direction(bool retract) {
#ifdef ROBOT_HARDWARE
        if (hardware_ && gpiod_line_set_value(direction_, retract ? 1 : 0) < 0)
            throw HardwareError("Failed to set actuator direction GPIO");
#else
        (void)retract;
#endif
    }
    void release() noexcept {
#ifdef ROBOT_HARDWARE
        // Keep ownership until process cleanup; output state after release is not guaranteed.
        if (powerOwned_) gpiod_line_release(power_);
        if (directionOwned_) gpiod_line_release(direction_);
        if (chip_) gpiod_chip_close(chip_);
        powerOwned_ = directionOwned_ = false;
        chip_ = nullptr;
#endif
    }
public:
    Actuator(bool hardware, const std::string& chip) : hardware_(hardware) {
#ifdef ROBOT_HARDWARE
        if (hardware_) {
            try {
                chip_ = gpiod_chip_open_by_name(chip.c_str());
                if (!chip_) throw std::runtime_error("Cannot open " + chip);
                power_ = gpiod_chip_get_line(chip_, 10);
                direction_ = gpiod_chip_get_line(chip_, 17);
                if (!power_ || !direction_) throw std::runtime_error("Cannot get GPIO lines 10/17");
                if (gpiod_line_request_output(power_, "combined_robot", 0) < 0)
                    throw std::runtime_error("Cannot request GPIO 10 as power-off output");
                powerOwned_ = true;
                if (gpiod_line_request_output(direction_, "combined_robot", 0) < 0)
                    throw std::runtime_error("Cannot request GPIO 17 as direction output");
                directionOwned_ = true;
            } catch (...) {
                if (powerOwned_) gpiod_line_set_value(power_, 0);
                release();
                throw;
            }
        }
#else
        (void)chip;
#endif
    }
    ~Actuator() { stopNoThrow(); release(); }
    bool busy() const { return phase_ != Phase::Idle; }
    bool powered() const { return powered_; }
    void start(bool retract, double inches) {
        if (busy()) throw std::runtime_error("Actuators busy; use t before another actuator move");
        if (!std::isfinite(inches) || inches <= 0.0 || inches > config::fullStrokeInches)
            throw std::runtime_error("Distance must be > 0 and <= 4 inches");
        power(false);
        retract_ = retract;
        duration_ = inches / config::fullStrokeInches * config::fullStrokeSeconds;
        phase_ = Phase::PowerOffWait;
        due_ = Clock::now() + config::relayDelay;
    }
    // These waits advance by deadlines; they never sleep or block keyboard processing.
    void update(Time now) {
        if (phase_ == Phase::Idle || now < due_) return;
        if (phase_ == Phase::PowerOffWait) {
            direction(retract_);
            phase_ = Phase::DirectionWait;
            due_ = now + config::relayDelay; // Also let the direction relay settle.
        } else if (phase_ == Phase::DirectionWait) {
            power(true);
            phase_ = Phase::Moving;
            due_ = after(Clock::now(), duration_);
        } else stop();
    }
    void stop() {
        phase_ = Phase::Idle; // Cancel a pending power-on even if the GPIO write fails.
        power(false);
    }
    void stopNoThrow() noexcept {
        try { stop(); } catch (...) {}
    }
    std::string state() const {
        if (phase_ == Phase::Idle) return "STOP";
        if (phase_ != Phase::Moving) return "WAIT";
        return retract_ ? "RETRACT" : "EXTEND";
    }
};

class Terminal {
    bool active_ = false;
    termios original_{};
    int originalFlags_ = -1;
public:
    Terminal() {
        if (!isatty(STDIN_FILENO)) return; // --plain can also consume a pipe for tests.
        if (tcgetattr(STDIN_FILENO, &original_) < 0)
            throw std::runtime_error("Cannot read terminal settings");
        originalFlags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (originalFlags_ < 0) throw std::runtime_error("Cannot read stdin flags");
        termios raw = original_;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) < 0)
            throw std::runtime_error("Cannot set terminal mode");
        active_ = true;
        if (fcntl(STDIN_FILENO, F_SETFL, originalFlags_ | O_NONBLOCK) < 0) {
            tcsetattr(STDIN_FILENO, TCSANOW, &original_);
            active_ = false;
            throw std::runtime_error("Cannot make stdin nonblocking");
        }
    }
    ~Terminal() {
        if (active_) {
            tcsetattr(STDIN_FILENO, TCSANOW, &original_);
            fcntl(STDIN_FILENO, F_SETFL, originalFlags_);
        }
    }
};

std::string motorLabel(const MotorStatus& s) {
    if (s.failed) return "FAULT";
    if (!s.ready) return "STARTING";
    return s.command == 0.0 ? "STOP" : "RUN";
}

int control(const Options& o, Safety& safety, MotorAxis& wheel,
            MotorAxis* rotation, Actuator& actuator) {
    Terminal terminal;
    const bool screen = isatty(STDOUT_FILENO) && !o.plain;
    bool quit = false, fast = false;
    std::string input;
    std::deque<std::string> events;
    const auto launched = Clock::now();
    auto lastRender = Time{};
    bool handledFault = false;
    int escapeState = 0;
    auto escapeAt = Time{};
    auto log = [&](const std::string& text) {
        events.push_back(text);
        if (events.size() > 5) events.pop_front();
        if (!screen) std::cout << text << '\n' << std::flush;
    };
    auto stopAll = [&]() {
        safety.stopped.store(true);
        wheel.stop();
        if (rotation) rotation->stop();
        actuator.stop();
    };
    auto healthy = [&](const MotorStatus& s) {
        return s.ready && !s.failed && Clock::now() - s.feedback <= config::feedbackLimit;
    };
    auto execute = [&](const std::string& text) {
        std::istringstream stream(text);
        std::vector<std::string> words;
        for (std::string word; stream >> word;) words.push_back(word);
        if (words.empty()) return;
        const std::string& command = words[0];
        auto exact = [&]() {
            if (words.size() != 1) throw std::runtime_error("This command takes no arguments");
        };
        try {
            if (command == "0" || command == "stop" || command == "q" || command == "quit") {
                exact(); stopAll(); log("STOP ALL (latched)");
                quit = command == "q" || command == "quit";
            } else if (command == "u" || command == "reset") {
                exact();
                if (!safety.fault().empty()) throw std::runtime_error("Fix fault and restart the program");
                if (!healthy(wheel.status()) || (rotation && !healthy(rotation->status())))
                    throw std::runtime_error("Wait for fresh motor feedback before reset");
                stopAll(); // No old movement or relay timer may survive reset.
                if (!safety.arm()) throw std::runtime_error("Fix fault and restart the program");
                log("ARMED; all axes remain stopped until commanded");
            } else if (command == "x" || command == "f" || command == "t") {
                exact();
                if (command == "x") wheel.stop();
                else if (command == "f") { if (rotation) rotation->stop(); }
                else actuator.stop();
                log("Individual stop: " + command);
            } else if (command == "1" || command == "2") {
                exact(); fast = command == "2";
                log(fast ? "Next wheel move: FAST" : "Next wheel move: SLOW");
            } else if (command == "h" || command == "help") {
                exact(); log("u reset; w/s [seconds]; a/d [seconds]; e/r inches; x/f/t stop axes; 1/2 speed; 0 stop all; q quit");
            } else if (command == "w" || command == "s" || command == "a" || command == "d" ||
                       command == "e" || command == "r") {
                if (safety.stopped.load()) throw std::runtime_error("STOPPED; enter u to reset first");
                if (words.size() > 2) throw std::runtime_error("Too many arguments");
                if (command == "e" || command == "r") {
                    if (words.size() != 2) throw std::runtime_error("Use e INCHES or r INCHES");
                    actuator.start(command == "r", parseNumber(words[1]));
                    log("Actuator move scheduled: " + text);
                } else {
                    const bool wheelMove = command == "w" || command == "s";
                    MotorAxis* axis = wheelMove ? &wheel : rotation;
                    if (!axis) throw std::runtime_error("Rotation disabled; configure its port and ID");
                    if (!healthy(axis->status())) throw std::runtime_error("No fresh feedback for this motor");
                    const double duration = words.size() == 2 ? parseNumber(words[1]) : (wheelMove ? 5.0 : 1.0);
                    if (duration <= 0.0 || duration > config::maxMoveSeconds)
                        throw std::runtime_error("Duration must be > 0 and <= 30 seconds");
                    const int sign = command == "w" || command == "a" ? 1 : -1;
                    const double speed = wheelMove ? (fast ? config::wheelFastRotor : config::wheelSlowRotor) :
                        o.rotationOutputSpeed * 6.33;
                    axis->move(sign * speed, duration);
                    log("Motor move scheduled: " + text);
                }
            } else throw std::runtime_error("Unknown command; enter h for help");
        } catch (const HardwareError& e) {
            safety.trip(e.what());
            wheel.stop(); if (rotation) rotation->stop(); actuator.stopNoThrow();
            log(std::string("FAULT: ") + e.what());
        } catch (const std::exception& e) { log(std::string("ERROR: ") + e.what()); }
    };
    log(o.hardware ? "REAL HARDWARE; enter u to reset, then a movement command" :
                        "SIMULATION ONLY; enter u to reset, then a movement command");
    log("Commands require Enter. Escape or ! stops all immediately; Ctrl+C stops and exits.");
    if (!rotation) log("Rotation disabled until --rotation-port and --rotation-id are supplied.");
    try {
        while (!quit && !receivedSignal) {
            const auto now = Clock::now();
            if (escapeState && now - escapeAt > 100ms) escapeState = 0;
            // Hardware workers report exceptions. Independently detect a stuck serial call.
            for (MotorAxis* axis : {&wheel, rotation}) {
                if (!axis) continue;
                const auto s = axis->status();
                if (s.failed || (s.ready && now - s.feedback > config::feedbackLimit) ||
                    (!s.ready && now - launched > config::startupLimit))
                    safety.trip("Motor feedback fault or timeout");
            }
            if (!safety.fault().empty() && !handledFault) {
                stopAll(); handledFault = true;
                log("FAULT: " + safety.fault());
            }
            if (safety.stopped.load()) {
                if (actuator.busy() || actuator.powered()) actuator.stop();
            }
            else actuator.update(now);

            pollfd fd{STDIN_FILENO, POLLIN, 0};
            const int available = poll(&fd, 1, 5);
            if (available < 0 && errno != EINTR) throw std::runtime_error("Keyboard poll failed");
            if (available > 0 && (fd.revents & POLLIN)) {
                char buffer[256];
                const auto count = read(STDIN_FILENO, buffer, sizeof(buffer));
                if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
                    throw std::runtime_error("Keyboard read failed");
                if (count == 0 && !isatty(STDIN_FILENO)) quit = true;
                // Stop has priority over every other command received in the same read.
                bool immediateStop = false;
                for (ssize_t i = 0; i < count; ++i)
                    if (buffer[i] == '\x1b' || buffer[i] == '!') immediateStop = true;
                if (immediateStop) {
                    stopAll(); input.clear(); log("STOP ALL (latched, immediate key)");
                    // Discard arrow/function-key escape sequences; trailing A/D are never commands.
                    escapeState = 0; escapeAt = Clock::now();
                    for (ssize_t i = 0; i < count; ++i) {
                        const unsigned char c = static_cast<unsigned char>(buffer[i]);
                        if (c == 27) escapeState = 1;
                        else if (escapeState == 1) escapeState = (c == '[' || c == 'O') ? 2 : 0;
                        else if (escapeState == 2 && c >= 0x40 && c <= 0x7e) escapeState = 0;
                    }
                } else {
                    for (ssize_t i = 0; i < count; ++i) {
                        const unsigned char c = static_cast<unsigned char>(buffer[i]);
                        if (escapeState) {
                            if (escapeState == 1 && (c == '[' || c == 'O')) escapeState = 2;
                            else if (escapeState == 2 && c >= 0x40 && c <= 0x7e) escapeState = 0;
                            else if (escapeState == 1) escapeState = 0;
                            continue;
                        }
                        if (c == '\n' || c == '\r') {
                            const std::string command = input;
                            input.clear(); execute(command);
                            if (quit || safety.stopped.load()) break; // Do not execute pasted commands after stop.
                        } else if (c == 127 || c == '\b') {
                            if (!input.empty()) input.pop_back();
                        } else if (c == 4) { stopAll(); quit = true; break; }
                        else if (c >= 32 && c < 127 && input.size() < 100) input.push_back(static_cast<char>(c));
                    }
                }
            }
            if (available > 0 && (fd.revents & (POLLHUP | POLLERR | POLLNVAL))) quit = true;
            if (now - lastRender >= 100ms) {
                const auto w = wheel.status();
                const auto r = rotation ? rotation->status() : MotorStatus{};
                if (screen) {
                    std::cout << "\033[H\033[2JCombined Tunnel Lining Robot Controller - "
                        << (o.hardware ? "REAL HARDWARE" : "SIMULATION") << '\n'
                        << "State: " << (safety.stopped.load() ? "STOPPED / LATCHED" : "ARMED") << '\n'
                        << "Wheel: " << motorLabel(w) << " | rotor target " << std::fixed << std::setprecision(2)
                        << w.command << " rad/s | feedback " << w.measured << " rad/s | " << w.temperature << " C\n"
                        << "Rotation: " << (rotation ? motorLabel(r) : "DISABLED") << "\n"
                        << "Actuators: " << actuator.state() << " | 12V output " << actuator.powered() << "\n\n"
                        << "Enter commands (motion continues while you type):\n"
                        << "  u reset     w/s [seconds] wheel forward/back (default 5s)\n"
                        << "  1 slow      2 fast (for next wheel move)\n"
                        << "  a/d [seconds] rotation + / - (default 1s; verify physical direction)\n"
                        << "  e INCHES extend     r INCHES retract (0 < inches <= 4)\n"
                        << "  x wheel stop   f rotation stop   t actuator stop\n"
                        << "  0 stop all   q stop and quit   Escape/! IMMEDIATE STOP\n\n";
                    for (const auto& event : events) std::cout << event << '\n';
                    std::cout << "\n> " << input << std::flush;
                } else {
                    std::cout << "STATE safety=" << (safety.stopped.load() ? "STOPPED" : "ARMED")
                        << " wheel=" << motorLabel(w) << " rotation=" << (rotation ? motorLabel(r) : "DISABLED")
                        << " LA=" << actuator.state() << " power=" << actuator.powered() << '\n' << std::flush;
                }
                lastRender = now;
            }
        }
        stopAll();
        log("Shutdown: stop commands requested; actuator power output is OFF.");
    } catch (...) {
        safety.trip("Control loop failed");
        wheel.stop(); if (rotation) rotation->stop(); actuator.stopNoThrow();
        throw;
    }
    return safety.fault().empty() ? 0 : 1;
}

int main(int argc, char** argv) {
    try {
        const auto options = parseOptions(argc, argv);
        std::signal(SIGINT, handleSignal);
        std::signal(SIGTERM, handleSignal);
        std::signal(SIGHUP, handleSignal);
        Safety safety;
        Actuator actuator(options.hardware, options.chip); // Power is requested OFF before motor workers start.
        MotorAxis wheel("Wheel", options.wheelPort, options.wheelId, options.hardware, safety);
        std::unique_ptr<MotorAxis> rotation;
        if (options.rotationEnabled()) rotation = std::make_unique<MotorAxis>(
            "Rotation", options.rotationPort, options.rotationId, options.hardware, safety);
        // Terminal cleanup happens before joining SDK workers, even if a serial call is slow.
        const int result = control(options, safety, wheel, rotation.get(), actuator);
        safety.stopped.store(true);
        actuator.stopNoThrow();
        return result;
    } catch (const std::exception& e) {
        std::cerr << "Controller error: " << e.what() << '\n';
        return 1;
    }
}
