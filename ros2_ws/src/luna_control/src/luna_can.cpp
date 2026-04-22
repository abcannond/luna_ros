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
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

// ---------------- CONFIG ----------------
const char* CAN_IFACE = "can0";
const int ARB_ID = 0x200;
const std::vector<int> MOTOR_IDS = {1,2,3,4};

const int STEP = 100;
const int LOOP_DELAY_US = 2000;
const int MAX_CURRENT = 16000;

// ---------------- STATE ----------------
std::array<std::atomic<int>, 5> currents; //motor currents 
std::map<int,bool> reverse_flags;
std::array<std::atomic<int>, 5> targets; //motor targets, index 0 not used to match with motor ids 
std::atomic<bool> running(true);

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
        int val = currents[m];
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
                int enc = (fr.data[0] << 8) | fr.data[1];
                int rpm = (int16_t)((fr.data[2] << 8) | fr.data[3]);

            }
        }
    }
}

// ---------------- CONTROL LOOP ----------------
void control_thread() {
    while (running) {
        for (int m : MOTOR_IDS) {

            //apply reverse flags
            int tgt = targets[m].load();
            if (reverse_flags[m]) tgt = -tgt;

            if (currents[m] < tgt)
                currents[m] += STEP;
            else if (currents[m] > tgt)
                currents[m] -= STEP;

            //clamp using max current to make sure motors dont blow up 
            currents[m] = std::clamp(currents[m], -MAX_CURRENT, MAX_CURRENT);
        }

        auto msg = build_msg();

        if (write(can_sock, &msg, sizeof(msg)) != sizeof(msg)) {
            perror("CAN write failed");
        }

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
            for (int m : MOTOR_IDS) targets[m] = 1000;
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
            for (auto &t : targets) t.second = 0;
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
        
        //assign message data to wheel targets
        targets[1].store((int)msg->data[0]);
        targets[2].store((int)msg->data[1]);
        targets[3].store((int)msg->data[2]);
        targets[4].store((int)msg->data[3]);
    }
    );

    //initialize currents and reverse flags
    //currently motors 2 and 3 are reversed
    for (int m : MOTOR_IDS) {
        currents[m] = 0;
        if(m == 2 or m ==3){
            reverse_flags[m] = true;
        }
        else{
            reverse_flags[m] = false;
        }
    }
    
    //set up CAN connection 
    can_sock = open_can(CAN_IFACE);

    //setup timeout 
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 50000;
    setsockopt(can_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    std::thread t1(feedback_thread);
    std::thread t2(control_thread);
    //std::thread t3(command_thread);

    std::thread ros_thread([&](){
    rclcpp::spin(node);
    });

    ros_thread.join();
    //t3.join();
    t1.join();
    t2.join();

    running = false;

    safe_shutdown();
    close(can_sock);

    std::cout << "Exited cleanly\n";
}