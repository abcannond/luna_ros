// C620 CAN control
//MOTORS 2 and 3 ARE REVERSED 

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

using namespace std::chrono_literals;

// ---------------- CONFIG ----------------
const char* CAN_IFACE = "can0";
const int ARB_ID = 0x200;
const std::vector<int> MOTOR_IDS = {1,2,3,4};

const int STEP = 100;
const int LOOP_DELAY_US = 2000;
const int MAX_CURRENT = 16000;

// ---------------- STATE ----------------
std::map<int,int> currents;
std::map<int,int> targets;
std::map<int,bool> reverse_flags;

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
            int tgt = reverse_flags[m] ? -targets[m] : targets[m];

            if (currents[m] < tgt)
                currents[m] += STEP;
            else if (currents[m] > tgt)
                currents[m] -= STEP;
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
    std::cout << "test";
    for (int m : MOTOR_IDS) {
        currents[m] = 0;
        targets[m] = 0;
        reverse_flags[m] = false;
    }
    
    can_sock = open_can(CAN_IFACE);

    std::thread t1(feedback_thread);
    std::thread t2(control_thread);
    std::thread t3(command_thread);

    t3.join();

    running = false;

    t1.join();
    t2.join();

    safe_shutdown();
    close(can_sock);

    std::cout << "Exited cleanly\n";
}


/*/ old control file 
// Unified controller: brushless (C620 via SocketCAN) + brushed Spark MAX (via SparkMax API)

#include <iostream>
#include <thread>
#include <atomic>
#include <map>
#include <vector>
#include <chrono>
#include <csignal>
#include <cstring>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <algorithm>
#include <cerrno>

#include "luna_control/SparkMax.hpp"

using namespace std::chrono_literals;motor-test.py


// ---------- Configuration ----------
const char *CAN_IFACE = "can0";
const std::vector<int> DRIVE_IDS = {5, 6, 7, 8};  // wheel motors
const int DUMP_ID = 4;                            // dump mechanism motor (must be 1..8 for C620)
const std::map<int,int> INVERT = { {5,-1}, {6,-1}, {7,1}, {8,1} }; // left inverted
const int RAMP_STEP = 100;       // step per loop toward target current
const int RAMP_DELAY_MS = 10;    // send rate for brushless (10 ms -> 100 Hz)
const int DRIVE_CURRENT = 8000;  // nominal current
const int MAX_CURRENT = 16000;   // clamp if needed
const int STATUS_INTERVAL_MS = 200; // how often to print status
const double STEER_DUTY = 0.05;  // steering strength (tweakable)

// ---------- Global state ----------
std::atomic<bool> running(true);
std::map<int,int> bl_targets; // keyed by motor CAN ID
std::map<int,int> bl_actual;

SparkMax *steer_left = nullptr;
SparkMax *steer_right = nullptr;
SparkMax *mechanism = nullptr;

std::atomic<double> steer_left_duty(0.0);
std::atomic<double> steer_right_duty(0.0);
std::atomic<double> mech_duty(0.0);

int can_sock = -1;

// ---------- Utility: open CAN socket ----------
int open_can_socket(const char *ifname) {
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);motor-test.py

8 KB
    if (s < 0) { perror("socket(PF_CAN)"); return -1; }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, ifname, IFNAMSIZ-1);
    ifr.ifr_name[IFNAMSIZ-1] = '\0';

    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl(SIOCGIFINDEX)");
        close(s);
        return -1;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(s);
        return -1;
    }

    // Try to increase socket send buffer (best-effort)
    int sndbuf = 256 * 1024;
    if (setsockopt(s, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf)) < 0) {
        // non-fatal; just warn
        std::cerr << "Warning: setsockopt(SO_SNDBUF) failed: " << strerror(errno) << "\n";
    }motor-test.py


    return s;
}

// ---------- Build C620 frame for a group base_id (1 or 5) ----------
struct can_frame build_bl_frame_group(const std::map<int,int>& actual, int base_id) {
    struct can_frame fr{};
    // Group base 1 => 0x200, base 5 => 0x1FF
    // Keep the same mapping used previously:
    fr.can_id = (base_id == 1) ? 0x200 : 0x1FF;
    fr.can_dlc = 8;
    for (int i = 0; i < 4; ++i) {
        int id = base_id + i;
        int val = actual.count(id) ? actual.at(id) : 0;
        // clamp safety
        if (val > MAX_CURRENT) val = MAX_CURRENT;
        if (val < -MAX_CURRENT) val = -MAX_CURRENT;
        int16_t sval = static_cast<int16_t>(val);
        fr.data[2*i]   = static_cast<unsigned char>((sval >> 8) & 0xFF); //static cast to prevent build error
        fr.data[2*i+1] = static_cast<unsigned char>(sval & 0xFF);
    }
    return fr;
}

// ---------- Send grouped frames (1-4 and 5-8) ----------
void send_bl_frames_now() {
    if (can_sock < 0) return;
    struct can_frame fr14 = build_bl_frame_group(bl_actual, 1);
    struct can_frame fr58 = build_bl_frame_group(bl_actual, 5);

    ssize_t n = write(can_sock, &fr14, sizeof(fr14));
    if (n != (ssize_t)sizeof(fr14)) {
        if (errno != ENOBUFS) perror("write CAN 1-4");
        else std::cerr << "Warning: CAN TX buffer full (1-4)\n";motor-test.py

    }

    n = write(can_sock, &fr58, sizeof(fr58));
    if (n != (ssize_t)sizeof(fr58)) {
        if (errno != ENOBUFS) perror("write CAN 5-8");
        else std::cerr << "Warning: CAN TX buffer full (5-8)\n";
    }
}

// ---------- Ramp + send thread ----------
void brushless_ramp_thread() {
    while (running) {
        // Ramp toward target for every motor key present
        for (auto &kv : bl_targets) {
            int id = kv.first;
            int tgt = kv.second;
            int cur = bl_actual[id];
            if (cur < tgt) cur = std::min(cur + RAMP_STEP, tgt);
            else if (cur > tgt) cur = std::max(cur - RAMP_STEP, tgt);
            bl_actual[id] = cur;
        }

        // send both frames
        send_bl_frames_now();

        std::this_thread::sleep_for(std::chrono::milliseconds(RAMP_DELAY_MS));motor-test.py

    }

    // ramp down on exit
    for (auto &kv : bl_targets) kv.second = 0;
    bool ramping = true;
    while (ramping) {
        ramping = false;
        for (auto &kv : bl_targets) {
            int id = kv.first;
            int tgt = kv.second;
            int cur = bl_actual[id];
            if (cur != tgt) {
                ramping = true;
                if (cur < tgt) cur = std::min(cur + RAMP_STEP, tgt);
                else cur = std::max(cur - RAMP_STEP, tgt);
                bl_actual[id] = cur;
            }
        }
        send_bl_frames_now();
        std::this_thread::sleep_for(std::chrono::milliseconds(RAMP_DELAY_MS));
    }
}

// ---------- Terminal input helpers ----------
void enable_nonblocking_terminal() {
    struct termios t;
    tcgetattr(STDIN_FILENO, &t);
    t.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}
void restore_terminal() {
    struct termios t;
    tcgetattr(STDIN_FILENO, &t);
    t.c_lflag |= (ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
}

// ---------- Signal ----------
void sigint_handler(int) { running = false; }

// ---------- Main ----------
int main() {
    // initialize all motors 1..8 (C620 address space)
    for (int id = 1; id <= 8; ++id) {
        bl_targets[id] = 0;
        bl_actual[id] = 0;
    }

    // quick debug output
    std::cout << "Step: Opening CAN socket (" << CAN_IFACE << ")...\n";
    can_sock = open_can_socket(CAN_IFACE);
    if (can_sock < 0) {
        std::cerr << "ERROR: Failed to open CAN socket on " << CAN_IFACE << ".\n";
        std::cerr << "Check: is the interface up? run: sudo ip link set can0 up type can bitrate 1000000\n";
        return 1;
    }
    std::cout << "CAN socket opened OK.\n";

    // Optionally increase tx queue length from userspace if you want:
    // system("sudo ip link set can0 txqueuelen 1000"); // uncomment if desired

    // Create SparkMax objects for brushed controllers
    try {
        SparkMax sm1("can0", 1), sm2("can0", 2), sm3("can0", 3);
        steer_left = &sm1;
        steer_right = &sm2;
        mechanism = &sm3;

        for (SparkMax* s : {steer_left, steer_right, mechanism}) {
            s->SetIdleMode(IdleMode::kBrake);
            s->SetMotorType(MotorType::kBrushed);
            s->SetInverted(false);
            s->BurnFlash();
        }

        // start ramp thread
        std::thread rampThread(brushless_ramp_thread);

        enable_nonblocking_terminal();
        std::signal(SIGINT, sigint_handler);

        std::cout << "Unified control running.\n"
                  << "W/S: brushless drive (group)\n"
                  << "A/D: steer brushed (differential)\n"
                  << "I/K: mechanism\n"
                  << "P/L: dump motor forward/back\n"
                  << "Space: stop all\n"
                  << "Q: quit\n";

        auto lastStatus = std::chrono::steady_clock::now();

        while (running) {
            int c = getchar();
            if (c != EOF) {
                switch (c) {
                    // drive group
                    case 'w': case 'W':
                        for (int id : DRIVE_IDS) bl_targets[id] = DRIVE_CURRENT * (INVERT.count(id) ? INVERT.at(id) : 1);
                        break;
                    case 's': case 'S':
                        for (int id : DRIVE_IDS) bl_targets[id] = -DRIVE_CURRENT * (INVERT.count(id) ? INVERT.at(id) : 1);
                        break;

                    // dump motor independent
                    case 'p': case 'P':
                        bl_targets[DUMP_ID] = DRIVE_CURRENT;
                        break;
                    case 'l': case 'L':
                        bl_targets[DUMP_ID] = -DRIVE_CURRENT;
                        break;

                    // steering differential
                    case 'a': case 'A':
                        steer_left_duty.store(-STEER_DUTY);
                        steer_right_duty.store( STEER_DUTY);
                        break;
                    case 'd': case 'D':
                        steer_left_duty.store( STEER_DUTY);
                        steer_right_duty.store(-STEER_DUTY);
                        break;
                    case 'j': case 'J':
                        steer_left_duty.store(0.0);
                        break;
                    case 'f': case 'F':
                        steer_right_duty.store(0.0);
                        break;

                    // mechanism brushed
                    case 'i': case 'I':
                        mech_duty.store(0.2);
                        break;
                    case 'k': case 'K':
                        mech_duty.store(-0.2);
                        break;

                    // stop all
                    case ' ':
                        for (auto &kv : bl_targets) kv.second = 0;
                        steer_left_duty.store(0.0);
                        steer_right_duty.store(0.0);
                        mech_duty.store(0.0);
                        break;

                    case 'q': case 'Q':
                        running = false;
                        break;
                    default:
                        break;
                }
            }

            // apply brushed outputs
            steer_left->SetDutyCycle(static_cast<float>(steer_left_duty.load())); //static cast to prevent build errror
            steer_right->SetDutyCycle(static_cast<float>(steer_right_duty.load()));
            mechanism->SetDutyCycle(static_cast<float>(mech_duty.load()));
            steer_left->Heartbeat();
            steer_right->Heartbeat();
            mechanism->Heartbeat();

            // print status at reduced rate
            auto now = std::chrono::steady_clock::now();
            if (now - lastStatus > std::chrono::milliseconds(STATUS_INTERVAL_MS)) {
                std::cout << "\rBL targets: [ ";
                for (int id = 1; id <= 8; ++id) std::cout << bl_targets[id] << " ";
                std::cout << "] Dump(" << DUMP_ID << "): " << bl_targets[DUMP_ID]
                          << "  SteerL: " << (steer_left_duty.load() * 100.0)
                          << "% SteerR: " << (steer_right_duty.load() * 100.0)
                          << "% Mech: " << (mech_duty.load() * 100.0) << "%    " << std::flush;
                lastStatus = now;
            }

            std::this_thread::sleep_for(20ms);
        }

        // shutdown
        std::cout << "\nShutting down: ramping BL to zero & stopping brushed motors...\n";
        running = false;
        if (rampThread.joinable()) rampThread.join();

        for (SparkMax* s : {steer_left, steer_right, mechanism}) {
            s->SetDutyCycle(0.0);
            s->Heartbeat();
        }

        // final frames
        send_bl_frames_now();

        restore_terminal();
        if (can_sock >= 0) close(can_sock);
        std::cout << "Exited cleanly.\n";
        return 0;
    } catch (const std::exception &ex) {
        std::cerr << "Exception constructing SparkMax or during runtime: " << ex.what() << "\n";
        if (can_sock >= 0) close(can_sock);
        return 1;
    }
}
*/