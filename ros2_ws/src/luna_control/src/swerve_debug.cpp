/*
 * Swerve motor tuning/debug tool
 *
 * Commands:
 *   duty <id> <val>             open-loop duty cycle (-1.0 to 1.0)
 *   pid                         configure PID on all motors and switch to position mode
 *   goto <motor_id> <radians>   steer one motor (1-4) — requires pid mode
 *   goto all <radians>          steer all motors
 *   n                           print current encoder positions
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

const float KP           = 1.0f;
const float OUTPUT_LIMIT = 1.0f;

// ---------------- STATE ----------------
std::unique_ptr<SparkMax> swerve[5];
std::array<std::atomic<float>, 5> swerve_pos_targets;
std::array<std::atomic<float>, 5> steer_cmds;
std::array<std::atomic<float>, 5> encoder_offsets;
std::atomic<bool> position_mode(false);
std::atomic<bool> running(true);

// per-motor encoder sign: -1 if positive duty causes decreasing encoder
const float ENC_SIGN[5] = {0, 1.0f, 1.0f, -1.0f, 1.0f};

// ---------------- CONTROL LOOP ----------------
void control_thread() {
    while (running) {
        for (int i = 1; i <= 4; i++) {
            swerve[i]->Heartbeat();

            if (position_mode.load()) {
                float current = ENC_SIGN[i] * swerve[i]->GetAltEncoderPosition() - encoder_offsets[i].load();
                float error = swerve_pos_targets[i].load() - current;
                float output = std::clamp(KP * error, -OUTPUT_LIMIT, OUTPUT_LIMIT);
                swerve[i]->SetSetpoint(output);
            } else {
                float duty = std::clamp(steer_cmds[i].load(), -1.0f, 1.0f);
                swerve[i]->SetDutyCycle(duty);
            }
        }
        std::this_thread::sleep_for(2ms);
    }
}

// ---------------- INPUT ----------------
void command_thread() {
    std::string cmd;
    std::cout << "Commands: duty <id> <val> | pid | goto <id> <rad> | goto all <rad> | n | reset <id> | q\n";

    while (running) {
        std::getline(std::cin, cmd);

        if (cmd == "q") {
            running = false;

        } else if (cmd == "n") {
            for (int i = 1; i <= 4; i++) {
                float pos = ENC_SIGN[i] * swerve[i]->GetAltEncoderPosition() - encoder_offsets[i].load();
                std::cout << "Pos" << i << ": " << pos << "  ";
            }
            std::cout << "\n";

        } else if (cmd == "zero") {
            for (int i = 1; i <= 4; i++) {
                encoder_offsets[i].store(ENC_SIGN[i] * swerve[i]->GetAltEncoderPosition());
            }
            for (int i = 1; i <= 4; i++) swerve_pos_targets[i].store(0.0f);
            std::cout << "Zeroed. Current position is now 0 for all motors.\n";

        } else if (cmd == "pid") {
            position_mode.store(true);
            std::cout << "Software PID enabled. KP=" << KP << " OUTPUT_LIMIT=" << OUTPUT_LIMIT << "\n";

        } else if (cmd == "faults") {
            for (int i = 1; i <= 4; i++) {
                swerve[i]->ResetFaults();
                std::cout << "Faults cleared on motor " << i << "\n";
            }

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
                    position_mode.store(false);
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
                    for (int i = 1; i <= 4; i++) swerve_pos_targets[i].store(target);
                    position_mode.store(true);
                    std::cout << "All -> " << target << " rad\n";
                } else {
                    int id = std::stoi(first);
                    ss >> target;
                    target = std::clamp(target, -(float)M_PI, (float)M_PI);
                    if (id >= 1 && id <= 4) {
                        swerve_pos_targets[id].store(target);
                        position_mode.store(true);
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
int main() {
    for (auto& t : swerve_pos_targets) t.store(0.0f);
    for (auto& t : steer_cmds) t.store(0.0f);
    for (auto& t : encoder_offsets) t.store(0.0f);

    try {
        for (int i = 1; i <= 4; i++) {
            swerve[i] = std::make_unique<SparkMax>(CAN_IFACE, i);
            swerve[i]->SetDataPortConfig(1);
            swerve[i]->SetAltEncoderCountsPerRev(8192);
            swerve[i]->SetAltEncoderPositionFactor(2.0f * (float)M_PI);
        }
        swerve[1]->SetAltEncoderInverted(false);
        swerve[2]->SetAltEncoderInverted(true);
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

    std::cout << "Initialized in duty cycle mode. Type 'pid' to enable position control.\n";

    std::thread t1(control_thread);
    std::thread t2(command_thread);

    t2.join();
    running = false;

    std::cout << "Returning wheels to 0...\n";
    for (int i = 1; i <= 4; i++) swerve_pos_targets[i].store(0.0f);
    position_mode.store(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    t1.join();
    std::cout << "Exited cleanly\n";
}
