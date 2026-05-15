/*
 * Swerve motor tuning/debug tool
 *
 * Commands:
 *   duty <id> <val>             open-loop duty cycle (-1.0 to 1.0)
 *   goto <motor_id> <radians>   steer one motor (1-4) using software PID
 *   goto all <radians>          steer all motors
 *   zero                        set current encoder positions as 0
 *   n                           print current encoder positions
 *   kp <val>                    set P gain at runtime
 *   kd <val>                    set D gain at runtime
 *   limit <val>                 set output clamp (0.0 to 1.0)
 *   faults                      clear faults on all motors
 *   reset <id>                  factory reset a motor (power cycle after)
 *   q                           quit (returns wheels to 0 first)
 *
 * Created for 2025-26 WPI Lunabotics MQP
 */

#include <iostream>
#include <sstream>
#include <thread>
#include <atomic>
#include <array>
#include <chrono>
#include <algorithm>
#include <csignal>
#include <memory>
#include "luna_control/SparkMax.hpp"

using namespace std::chrono_literals;

// ---------------- CONFIG ----------------
const char* CAN_IFACE = "can1"; // bitrate: 1000000

// ---------------- STATE ----------------
std::unique_ptr<SparkMax> swerve[5];
std::array<std::atomic<float>, 5> swerve_pos_targets;
std::array<std::atomic<float>, 5> steer_cmds;
std::array<std::atomic<float>, 5> encoder_offsets;
std::array<std::atomic<bool>, 5>  motor_in_pid; // per-motor: true = PID, false = duty cycle
std::atomic<bool> running(true);
std::atomic<bool> shutdown_requested(false); // set by 'q' or SIGINT; main runs cleanup, then clears running

// PID gains (tunable at runtime via kp/ki/kd commands).
std::atomic<float> KP{1.0f};
std::atomic<float> KI{0.5f};
std::atomic<float> KD{0.0f};
std::atomic<float> OUTPUT_LIMIT{1.0f};
const float INTEGRAL_LIMIT = 0.5f; // anti-windup clamp on the integral term

// ---------------- CONTROL LOOP ----------------
// Software PD position control: error = target - (encoder - offset),
// output = kp*error + kd*(error - last_error)/dt, clamped, sent as duty cycle.
void control_thread() {
    std::array<float, 5> last_error{};
    std::array<float, 5> integral{};
    auto last_time = std::chrono::steady_clock::now();

    while (running) {
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last_time).count();
        last_time = now;
        if (dt <= 0.0f) dt = 0.002f; // safety, avoid divide-by-zero on first tick

        const float kp = KP.load();
        const float ki = KI.load();
        const float kd = KD.load();
        const float lim = OUTPUT_LIMIT.load();

        for (int i = 1; i <= 4; i++) {
            swerve[i]->Heartbeat();

            if (motor_in_pid[i].load()) {
                float current = swerve[i]->GetAltEncoderPosition() - encoder_offsets[i].load();
                float error = swerve_pos_targets[i].load() - current;
                std::cout << "Error" << i << ": " << error << " ";
                integral[i] = std::clamp(integral[i] + error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
                float derror = (error - last_error[i]) / dt;
                float output = std::clamp(kp * error + ki * integral[i] + kd * derror, -lim, lim);
                swerve[i]->SetDutyCycle(output);
                last_error[i] = error;
                if((error <= 0.05) and (error >= -0.05)) {
                    swerve[i]->SetDutyCycle(0);
                    last_error[i] = 0;
                    integral[i] = 0;
                }
            } else {
                float duty = std::clamp(steer_cmds[i].load(), -1.0f, 1.0f);
                swerve[i]->SetDutyCycle(duty);
                last_error[i] = 0.0f;
                integral[i]   = 0.0f; // reset integrator when leaving position mode
            }
        }
        std::this_thread::sleep_for(2ms);
    }
}

// ---------------- INPUT ----------------
void command_thread() {
    std::string cmd;
    std::cout << "Commands: duty <id> <val> | goto <id> <rad> | goto all <rad> | zero | n | "
                 "kp <val> | ki <val> | kd <val> | limit <val> | faults | reset <id> | q\n";

    while (running && !shutdown_requested) {
        std::getline(std::cin, cmd);
        if (shutdown_requested) break;

        if (cmd == "q") {
            shutdown_requested = true;

        } else if (cmd == "n") {
            for (int i = 1; i <= 4; i++) {
                float pos = swerve[i]->GetAltEncoderPosition() - encoder_offsets[i].load();
                std::cout << "Pos" << i << ": " << pos << "  ";
            }
            std::cout << "\n";

        } else if (cmd == "zero") {
            for (int i = 1; i <= 4; i++) {
                encoder_offsets[i].store(swerve[i]->GetAltEncoderPosition());
                swerve_pos_targets[i].store(0.0f);
            }
            std::cout << "Zeroed all motors at current position.\n";

        } else if (cmd == "faults") {
            for (int i = 1; i <= 4; i++) {
                swerve[i]->ResetFaults();
                std::cout << "Faults cleared on motor " << i << "\n";
            }

        } else if (cmd.rfind("kp ", 0) == 0) {
            try {
                float v = std::stof(cmd.substr(3));
                KP.store(v);
                std::cout << "KP = " << v << "\n";
            } catch (...) { std::cout << "Usage: kp <val>\n"; }

        } else if (cmd.rfind("ki ", 0) == 0) {
            try {
                float v = std::stof(cmd.substr(3));
                KI.store(v);
                std::cout << "KI = " << v << "\n";
            } catch (...) { std::cout << "Usage: ki <val>\n"; }

        } else if (cmd.rfind("kd ", 0) == 0) {
            try {
                float v = std::stof(cmd.substr(3));
                KD.store(v);
                std::cout << "KD = " << v << "\n";
            } catch (...) { std::cout << "Usage: kd <val>\n"; }

        } else if (cmd.rfind("limit ", 0) == 0) {
            try {
                float v = std::clamp(std::stof(cmd.substr(6)), 0.0f, 1.0f);
                OUTPUT_LIMIT.store(v);
                std::cout << "OUTPUT_LIMIT = " << v << "\n";
            } catch (...) { std::cout << "Usage: limit <0.0 to 1.0>\n"; }

        } else if (cmd == "checktype") {
            for (int i = 1; i <= 4; i++) {
                uint8_t t = swerve[i]->GetMotorType();
                std::cout << "Motor " << i << " type: " << (int)t << " (" << (t == 1 ? "Brushless" : "Brushed") << ")\n";
            }

        } else if (cmd == "resetall") {
            for (int i = 1; i <= 4; i++) {
                swerve[i]->FactoryDefaults();
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                std::cout << "Factory defaults sent to motor " << i << "\n";
            }
            std::cout << "Power cycle the rover now — do NOT run swerve_debug again before power cycling\n";

        } else if (cmd.rfind("reset ", 0) == 0) {
            try {
                int id = std::stoi(cmd.substr(6));
                if (id >= 1 && id <= 4) {
                    swerve[id]->FactoryDefaults();
                    std::cout << "Factory defaults sent to motor " << id << " — power cycle it now\n";
                } else {
                    std::cout << "Motor ID must be 1-4\n";
                }
            } catch (...) {
                std::cout << "Usage: reset <motor_id>\n";
            }

        } else if (cmd.rfind("duty ", 0) == 0) {
            try {
                std::istringstream ss(cmd.substr(5));
                int id; float val;
                ss >> id >> val;
                val = std::clamp(val, -1.0f, 1.0f);
                if (id >= 1 && id <= 4) {
                    motor_in_pid[id].store(false);
                    steer_cmds[id].store(val);
                    std::cout << "Motor " << id << " duty = " << val << "\n";
                } else {
                    std::cout << "Motor ID must be 1-4\n";
                }
            } catch (...) {
                std::cout << "Usage: duty <motor_id> <-1.0 to 1.0>\n";
            }

        } else if (cmd.rfind("goto ", 0) == 0) {
            try {
                std::istringstream ss(cmd.substr(5));
                std::string first;
                ss >> first;

                float target;
                if (first == "all") {
                    ss >> target;
                    target = std::clamp(target, -(float)M_PI, (float)M_PI);
                    for (int i = 1; i <= 4; i++) {
                        swerve_pos_targets[i].store(target);
                        motor_in_pid[i].store(true);
                    }
                    std::cout << "All -> " << target << " rad\n";
                } else {
                    int id = std::stoi(first);
                    ss >> target;
                    target = std::clamp(target, -(float)M_PI, (float)M_PI);
                    if (id >= 1 && id <= 4) {
                        swerve_pos_targets[id].store(target);
                        motor_in_pid[id].store(true);
                        std::cout << "Motor " << id << " -> " << target << " rad\n";
                    } else {
                        std::cout << "Motor ID must be 1-4\n";
                    }
                }
            } catch (...) {
                std::cout << "Usage: goto <motor_id> <radians>  OR  goto all <radians>\n";
            }
        }
    }
}

// ---------------- MAIN ----------------
// Ctrl-C: stop accepting commands and let main() run the "return to 0" cleanup.
// Without this the process dies with the last duty still latched on the SparkMax.
void handle_sigint(int) {
    shutdown_requested = true;
}

int main() {
    std::signal(SIGINT, handle_sigint);

    for (auto& t : swerve_pos_targets) t.store(0.0f);
    for (auto& t : steer_cmds) t.store(0.0f);
    for (auto& t : encoder_offsets) t.store(0.0f);
    for (auto& m : motor_in_pid) m.store(false);

    // Init mirrors luna_debug (no SetMotorType, no onboard PID setup needed).
    try {
        for (int i = 1; i <= 4; i++) {
            swerve[i] = std::make_unique<SparkMax>(CAN_IFACE, i);
            swerve[i]->SetDataPortConfig(1);
            swerve[i]->SetAltEncoderCountsPerRev(8192);
            swerve[i]->SetAltEncoderPositionFactor(2.0f * (float)M_PI);
        }
        swerve[1]->SetAltEncoderInverted(false);
        swerve[2]->SetAltEncoderInverted(false);
        swerve[3]->SetAltEncoderInverted(false);
        swerve[4]->SetAltEncoderInverted(false);
    } catch (const std::exception& e) {
        std::cerr << "SparkMax init failed: " << e.what() << "\n";
        return 1;
    }

    // Send heartbeats for 2 seconds before accepting commands
    std::cout << "Warming up CAN...\n";
    for (int i = 0; i < 1000; i++) {
        for (int j = 1; j <= 4; j++) swerve[j]->Heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    std::cout << "Initialized in duty cycle mode. KP=" << KP.load()
              << " KI=" << KI.load() << " KD=" << KD.load()
              << " LIMIT=" << OUTPUT_LIMIT.load() << "\n";

    std::thread t1(control_thread);
    // Detached so a SIGINT during getline doesn't leave us stuck on join().
    std::thread(command_thread).detach();

    while (!shutdown_requested) std::this_thread::sleep_for(50ms);

    std::cout << "\nReturning wheels to 0...\n";
    for (int i = 1; i <= 4; i++) {
        swerve_pos_targets[i].store(0.0f);
        motor_in_pid[i].store(true);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    running = false;

    t1.join();
    std::cout << "Exited cleanly\n";
}
