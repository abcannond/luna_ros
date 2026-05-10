/*
 * Swerve motor tuning/debug tool
 * Uses SparkMax position PID with alt encoder feedback
 *
 * Commands:
 *   goto <motor_id> <radians>   steer one motor (1-4)
 *   goto all <radians>          steer all motors
 *   n                           print current encoder positions
 *   dutycycle                   switch to open-loop duty cycle mode
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
const char* CAN_IFACE = "can1";

const float KP           = 2.0f;   // tune this
const float OUTPUT_LIMIT = 0.5f;   // max duty cycle output from PID

// ---------------- STATE ----------------
std::unique_ptr<SparkMax> swerve[5];
std::array<std::atomic<float>, 5> swerve_pos_targets;
std::array<std::atomic<float>, 5> steer_cmds;
std::atomic<bool> position_mode(false);
std::atomic<bool> running(true);

// ---------------- CONTROL LOOP ----------------
void control_thread() {
    while (running) {
        for (int i = 1; i <= 4; i++) {
            swerve[i]->Heartbeat();

            if (position_mode.load()) {
                swerve[i]->SetPosition(swerve_pos_targets[i].load());
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
    std::cout << "Commands: goto <id> <rad> | goto all <rad> | n | dutycycle | q\n";

    while (running) {
        std::getline(std::cin, cmd);

        if (cmd == "q") {
            running = false;

        } else if (cmd == "n") {
            for (int i = 1; i <= 4; i++) {
                std::cout << "Pos" << i << ": " << swerve[i]->GetAltEncoderPosition() << "  ";
            }
            std::cout << "\n";

        } else if (cmd == "dutycycle") {
            position_mode.store(false);
            std::cout << "Duty cycle mode\n";

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

    try {
        for (int i = 1; i <= 4; i++) {
            swerve[i] = std::make_unique<SparkMax>(CAN_IFACE, i);
            swerve[i]->SetDataPortConfig(1);
            swerve[i]->SetAltEncoderCountsPerRev(8192);
            swerve[i]->SetAltEncoderPositionFactor(2.0f * (float)M_PI);
        }
        swerve[1]->SetAltEncoderInverted(false);
        swerve[2]->SetAltEncoderInverted(true);
        swerve[3]->SetAltEncoderInverted(true);
        swerve[4]->SetAltEncoderInverted(false);

        // PID config (disabled until duty cycle is confirmed working)
        // for (int i = 1; i <= 4; i++) {
        //     swerve[i]->SetFeedbackSensorPID0(2);
        //     swerve[i]->SetPositionPIDWrapEnable(false);
        //     swerve[i]->SetP(0, KP);
        //     swerve[i]->SetI(0, 0.0f);
        //     swerve[i]->SetD(0, 0.0f);
        //     swerve[i]->SetOutputMin(0, -OUTPUT_LIMIT);
        //     swerve[i]->SetOutputMax(0,  OUTPUT_LIMIT);
        // }
    } catch (const std::exception& e) {
        std::cerr << "SparkMax init failed: " << e.what() << "\n";
        return 1;
    }

    std::cout << "Initialized. KP=" << KP << " OUTPUT_LIMIT=" << OUTPUT_LIMIT << "\n";

    std::thread t1(control_thread);
    std::thread t2(command_thread);

    t2.join();
    running = false;

    // return wheels to 0 before exit
    std::cout << "Returning wheels to 0...\n";
    for (int i = 1; i <= 4; i++) swerve_pos_targets[i].store(0.0f);
    position_mode.store(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    t1.join();
    std::cout << "Exited cleanly\n";
}
