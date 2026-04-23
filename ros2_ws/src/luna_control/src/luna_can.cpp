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
const std::vector<int> MOTOR_IDS = {1,2,3,4};

const int STEP = 100;
const int LOOP_DELAY_US = 2000;
const int MAX_CURRENT = 16000; //mA
const float MAX_SPEED = 2.5; //rad/s
const float MM_PER_RAD = 203; // mm/rad 
const float WHEEL_RADIUS_MM = 101.6f; 
const float RPM_TO_MM_S =
    static_cast<float>((2.0 * M_PI * WHEEL_RADIUS_MM) / 60.0);

// ---------------- STATE ----------------
std::array<std::atomic<int>, 5> currents; //motor currents 
std::map<int,bool> reverse_flags;
std::array<std::atomic<int>, 5> targets; //motor targets, index 0 not used to match with motor ids 
std::array<std::atomic<float>, 5> steer_cmds; //swerve targets, currently in duty cycle
std::array<std::atomic<float>, 5> drive_vels; //drive motor velocities
std::array<std::atomic<int>, 5> drive_pos; //drive motor positions
std::atomic<bool> running(true);
std::unique_ptr<SparkMax> swerve[5];

int can_sock = -1;

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

                float rpm = (int16_t)((fr.data[2] << 8) | fr.data[3]);
                float vel_mm_s = rpm * RPM_TO_MM_S;
                drive_vels[id].store(vel_mm_s);
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

        /*/drive motor control 
        //TODO: figure out PID and do proper rad/s conversion 
        //max speed is going to be 2.5 rad/s (0.5 m/s) for now
        for (int m : MOTOR_IDS) {

            float vel = drive_vels[m].load();   // make sure this is rad/s
            float tgt = static_cast<float>(targets[m].load());

            //if (reverse_flags[m]) {
               // vel = -vel;
            //}

            float error = tgt - vel;

            // P controller
            float Kp = 8.0f;   //TODO: TUNE
            int current_cmd = (int)(Kp * error);

            // clamp
            current_cmd = std::clamp(current_cmd, -MAX_CURRENT, MAX_CURRENT);

            if (reverse_flags[m]) {
                current_cmd = -current_cmd;
            }

            int cur = currents[m].load();

            if (current_cmd > cur + STEP) {
                cur += STEP;
            }
            else if (current_cmd < cur - STEP){
                cur -= STEP;
            }
            else {
                cur = current_cmd;
            }

            currents[m].store(cur);

        }
        */

       for (int m : MOTOR_IDS) {

            int target_current = MAX_CURRENT - 14000;

            // apply direction HERE (before control logic)
            if (reverse_flags[m]) {
                target_current = -target_current;
            }

            int cur = currents[m].load();

            if (cur < target_current) {
                cur += STEP;
            } else if (cur > target_current) {
                cur -= STEP;
            }

            cur = std::clamp(cur, -MAX_CURRENT, MAX_CURRENT);

            currents[m].store(cur);
        }
        
        //std::cout << currents[1].load() << std::endl;
        for (int m : MOTOR_IDS) {
             std::cout << "M" << m << ": " << currents[m].load() << " ";
        }
        std::cout << std::endl;

        auto msg = build_msg();
        
        write(can_sock, &msg, sizeof(msg));
        //only uncomment this if doing individual node testing
        /*if (write(can_sock, &msg, sizeof(msg)) != sizeof(msg)) {
            perror("CAN write failed");
        }
        */
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
            for (int m : MOTOR_IDS) targets[m] = 500;
            std::cout << "fwd";
        }
        else if (cmd == "s") {
            for (int m : MOTOR_IDS) targets[m] = -5000;
        }
        else if (cmd == "a") {
            targets[1] = -3000; targets[2] = -3000;
            targets[3] = 3000;  targets[4] = 3000;
        }
        else if (cmd == "d") {
            targets[1] = 3000; targets[2] = 3000;
            targets[3] = -3000; targets[4] = -3000;
        }
        else if (cmd == "stop") {
            for (auto &t : targets) t = 0;
        }
        else if (cmd == "q") {
            running = false;
        }
        else if (cmd == "t") {
            std::cout << "m1";
            targets[1] = 2000;
        }
         else if (cmd == "y") {
            std::cout << "m2";
            targets[2] = 2000;
        }
         else if (cmd == "u") {
            std::cout << "m3";
            targets[3] = 2000;
        }
         else if (cmd == "i") {
            std::cout << "m4";
            targets[4] = 2000;
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
    
    //set up pointer, node, and subscriber 
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<rclcpp::Node>("can_interface");
    auto sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/wheel_cmds",
    10,
    [](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 4) return;
        
        //convert data received from message (currently rad/s -> mm/s)
        for(int i=1; i<=4; i++){
            float raw_rad_s = static_cast<float>(msg->data[i - 1]);

            // clamp rad/s
            raw_rad_s = std::clamp(raw_rad_s, -MAX_SPEED, MAX_SPEED);

            // convert rad/s → mm/s
            float mm_s = raw_rad_s * MM_PER_RAD;

            //assign converted value to wheel targets
            targets[i].store((int)mm_s);
        }        
    }
    );
    
    //initialize currents
    for (int m : MOTOR_IDS) {
        currents[m] = 0;
    }

    //init reverse flags 
    for (int m : MOTOR_IDS) {
        reverse_flags[m] = false;
    }

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

    
    std::thread ros_thread([&](){
    rclcpp::spin(node);
    });
    
    t3.join();

    running = false;

    //t3.join();
    t1.join();
    t2.join();
    ros_thread.join();
    rclcpp::shutdown();

    safe_shutdown();
    close(can_sock);

    std::cout << "Exited cleanly\n";
}