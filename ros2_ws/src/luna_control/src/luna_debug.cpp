// C620 CAN control

#include <iostream>
#include <thread>
#include <map>
#include <atomic>
#include <vector>
#include <chrono>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <array>
#include <csignal>
#include <algorithm>
#include "luna_control/SparkMax.hpp" 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

// ---------------- CONFIG ----------------
const char* CAN_IFACE = "can0";
const int ARB_ID = 0x200;
// hopefully we can resolve 3 turning into 5 soon 
const std::vector<int> MOTOR_IDS = {1,2,3,4};

const int STEP = 10;
const int LOOP_DELAY_US = 2000;
const int MAX_CURRENT = 16000; //mA
const float MAX_SPEED = 2.5; //rad/s
const float MM_PER_RAD = 203; // mm/rad 
const float WHEEL_RADIUS_MM = 101.6f; 
const int GEAR_REDUCTION = 192;
const float RPM_TO_MM_S =
    static_cast<float>((2.0 * M_PI * WHEEL_RADIUS_MM) / 60.0);

// ---------------- STATE ----------------
std::array<std::atomic<int>, 5> currents; //motor currents 
std::map<int,bool> reverse_flags;
std::array<std::atomic<int>, 5> targets; //motor targets, index 0 not used to match with motor ids 
std::array<std::atomic<int>, 5> vel_targets; //motor target rpm, index 0 not used to match with motor ids 
std::array<std::atomic<float>, 5> steer_cmds; //swerve targets, currently in duty cycle
std::array<std::atomic<int>, 5> drive_vels; //drive motor velocities
std::array<std::atomic<int>, 5> drive_pos; //drive motor positions
std::atomic<bool> running(true);
std::unique_ptr<SparkMax> swerve[5];

int can_sock = -1;
int TARGET_CURRENT = 0;
int TARGET_VELOCITY = 0;
auto last_time = std::chrono::steady_clock::now();

// ---------------- CAN ----------------

//Function to try to establish CAN interface 
//returns -1 if error 
//returns the CAN interface if it exists
int open_can(const char* iface) {
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("socket(PF_CAN)");
        return -1;
    }

    struct ifreq ifr{};
    strncpy(ifr.ifr_name, iface, IFNAMSIZ);

    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("SIOCGIFINDEX (interface not found or down)");
        close(s);
        return -1;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind(CAN)");
        close(s);
        return -1;
    }

    std::cout << "CAN opened successfully on " << iface << "\n";
    return s;
}

can_frame build_msg() {
    can_frame fr{};
    fr.can_id = ARB_ID;
    fr.can_dlc = 8;

    int i = 0;
    for (int m : MOTOR_IDS) {
        int val = currents[m].load();
        fr.data[i++] = static_cast<unsigned char>((val >> 8) & 0xFF);
        fr.data[i++] = static_cast<unsigned char>(val & 0xFF);
    }
    return fr;
}

// ---------------- FEEDBACK ----------------
void feedback_thread() {
    while (running) {
        can_frame fr{};
        int n = static_cast<int>(read(can_sock, &fr, sizeof(fr)));
        if (n <= 0) {
            continue;
        }
        if (n > 0) {
            int id = fr.can_id - 0x200;
            if (id >= 1 && id <= 4) {
                //record encoder position and velocity for drive motors
                int enc = (fr.data[0] << 8) | fr.data[1];
                drive_pos[id].store(enc);

                int rpm = (int16_t)((fr.data[2] << 8) | fr.data[3]);
                //float vel_mm_s = rpm * RPM_TO_MM_S;
                drive_vels[id].store(rpm);
            }
        }
    }
}

// ---------------- CONTROL LOOP ----------------
void control_thread() {
    while (running) {
        
        // ---- SparkMax ---- (swerve motor control)
        for (int i = 1; i <= 4; i++) {
            swerve[i]->Heartbeat();

            float duty = steer_cmds[i].load();

            // clamp 
            duty = std::clamp(duty, -1.0f, 1.0f);

            swerve[i]->SetDutyCycle(duty);
        }
        /*
        //velocity control
        for(int m: MOTOR_IDS) {
            int cur_vel = drive_vels[m].load();
            int tgt_vel = vel_targets[m].load()*GEAR_REDUCTION; //need to convert our target wheel vel to motor vel
            
            if(reverse_flags[m] == true) {
                tgt_vel = -tgt_vel;
            }

            int error = tgt_vel - cur_vel;

            int current = currents[m].load();
            
            if(error >= GEAR_REDUCTION or error <= -GEAR_REDUCTION) {
                //get time delta 
                auto now = std::chrono::steady_clock::now();
                float dt_s = std::chrono::duration<float>(now - last_time).count();
                last_time = now;

                //change current 
                float kP = 1.0f;  //still tuning 
                int delta = static_cast<int>(kP * (float)error * dt_s);

                //clamp current 
                current = std::clamp(current + delta, -MAX_CURRENT, MAX_CURRENT);
            }

            currents[m].store(current);
        } */
        
        //set currents to targets 
        for (int m : MOTOR_IDS) {
            int tgt = targets[m].load();
            int cur = currents[m].load();

            if(tgt > cur) {
                cur += STEP; 
            }
            else if(tgt < cur) {
                cur -= STEP;
            }

            currents[m].store(cur);
        }
        
        auto msg = build_msg();
        
        write(can_sock, &msg, sizeof(msg));

        std::this_thread::sleep_for(std::chrono::microseconds(LOOP_DELAY_US));
    }
}

// ---------------- INPUT ----------------
// Make sure this is commented out when running full stack, this is only for testing 
void command_thread() {
    std::string cmd;

    std::cout << "Command interface ready:\n";
    std::cout << "w/s/a/d = tank drive\n";
    std::cout << "stop = zero all\n";
    std::cout << "q = quit\n";

    while (running) {
        std::getline(std::cin, cmd);

        if (cmd == "w") {
            TARGET_CURRENT += 500;
            std::cout << TARGET_CURRENT;
            TARGET_VELOCITY += 5;
            //std::cout << TARGET_VELOCITY;
        }
        if (cmd == "q") {
            TARGET_CURRENT -= 500;
            //std::cout << TARGET_CURRENT;
            TARGET_VELOCITY -= 5;
            std::cout << TARGET_VELOCITY;
        }
        if (cmd == "f") {
            std::cout << TARGET_CURRENT;
            //targets[1].store(TARGET_CURRENT);
            //targets[2].store(-TARGET_CURRENT);
            //targets[3].store(-TARGET_CURRENT);
            //targets[4].store(TARGET_CURRENT);

            //set all motors to go forwards
            for(int m: MOTOR_IDS) {
                vel_targets[m].store(TARGET_VELOCITY);
            }
        }
        if (cmd == "b") {
            TARGET_CURRENT = -TARGET_CURRENT;
            std::cout << TARGET_CURRENT;
            targets[1].store(TARGET_CURRENT);
            targets[2].store(-TARGET_CURRENT);
            targets[3].store(-TARGET_CURRENT);
            targets[4].store(TARGET_CURRENT);
        }
        else if (cmd == "s") {
            TARGET_CURRENT = 0;
            //stop all
            for (int m : MOTOR_IDS) {
                targets[m].store(0);
            }
            steer_cmds[1].store(0);
            steer_cmds[2].store(0);
            steer_cmds[3].store(0);
            steer_cmds[4].store(0);

            //set all motors to go forwards
            for(int m: MOTOR_IDS) {
                vel_targets[m].store(0);
            }
        }
        else if (cmd == "a") {
            std::cout << "m2";
            targets[2].store(-TARGET_CURRENT);
            vel_targets[2].store(TARGET_VELOCITY);
        }
        else if (cmd == "d") {
            std::cout << "m1";
            targets[1].store(TARGET_CURRENT);
            vel_targets[1].store(TARGET_VELOCITY);
        }
        else if (cmd == "q") {
            running = false;
        }
        else if (cmd == "z") {
            std::cout << "m3";
            targets[3].store(-TARGET_CURRENT);
            vel_targets[3].store(TARGET_VELOCITY);
        }
        else if (cmd == "x") {
            std::cout << "m4";
            targets[4].store(TARGET_CURRENT);
            vel_targets[4].store(TARGET_VELOCITY);
        }
        else if (cmd == "m") {
            for (int m : MOTOR_IDS) {
                std::cout << "M" << m << ": " << currents[m].load() << " ";
            }
            std::cout << std::endl; 
            for (int m : MOTOR_IDS) {
                std::cout << "V" << m << ": " << drive_vels[m].load()/GEAR_REDUCTION << " ";
            }
            std::cout << std::endl; 
        }
        //swerve motor controls
        else if (cmd == "p") {
            std::cout << "swerve";
            steer_cmds[4].store(0.1f); 
            
        }
        else if (cmd == "l") {
            std::cout << "swerve";
            steer_cmds[4].store(-0.1f); 
            
        }
        else if (cmd == "o") {
            std::cout << "swerve";
            steer_cmds[3].store(0.1f); 
            
        }
        else if (cmd == "k") {
            std::cout << "swerve";
            steer_cmds[3].store(-0.1f); 
            
        }
        else if (cmd == "i") {
            std::cout << "swerve";
            steer_cmds[2].store(0.1f); 
            
        }
        else if (cmd == "j") {
            std::cout << "swerve";
            steer_cmds[2].store(-0.1f); 
            
        }
        else if (cmd == "u") {
            std::cout << "swerve";
            steer_cmds[1].store(0.1f); 
            
        }
        else if (cmd == "h") {
            std::cout << "swerve";
            steer_cmds[1].store(-0.1f); 
            
        }
    }
}

// ---------------- SHUTDOWN ----------------
void safe_shutdown() {
    std::cout << "Shutting down...\n";

    bool ramp = true;
    while (ramp) {
        ramp = false;

        for (int m : MOTOR_IDS) {
            if (currents[m] > 0) {
                currents[m] -= STEP;
                ramp = true;
            } else if (currents[m] < 0) {
                currents[m] += STEP;
                ramp = true;
            }
        }

        auto msg = build_msg();
        write(can_sock, &msg, sizeof(msg));

        std::this_thread::sleep_for(2ms);
    }
}

// ---------------- MAIN ----------------
int main() {
    //set all target values to 0 to start
    for (auto &t : targets) t.store(0);
    
    //initialize currents
    for (int m : MOTOR_IDS) {
        currents[m].store(0);
    }

    //init reverse flags 
    for (int m : MOTOR_IDS) {
        reverse_flags[m] = false;
    }

    //init velocity targets
    for (int m : MOTOR_IDS) {
        vel_targets[m].store(0);
    }

    //init velocity feedback
    for (int m : MOTOR_IDS) {
        drive_vels[m].store(0);
    }

    //2 and 3 are reversed 
    reverse_flags[2] = true;
    reverse_flags[3] = true;
    
    //set up CAN connection 
    can_sock = open_can(CAN_IFACE);

    //set up SparkMax stuff for swerve motors

    try {
        for (int i = 1; i <= 4; i++) {
            swerve[i] = std::make_unique<SparkMax>(CAN_IFACE, i);
        }
    } 
    catch (const std::exception& e) {
        std::cerr << "SparkMax init failed: " << e.what() << std::endl;
        return 1;
    }

    //setup timeout 
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 50000;
    setsockopt(can_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    std::thread t1(feedback_thread);
    std::thread t2(control_thread);
    std::thread t3(command_thread);
    
    t3.join();

    running = false;

    //t3.join();
    t1.join();
    t2.join();

    safe_shutdown();
    close(can_sock);

    std::cout << "Exited cleanly\n";
}