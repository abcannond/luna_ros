#pragma once

#include <array>
#include <cstdint>

namespace luna_control::teleop_can {

// SocketCAN bursts matching menu_loop.sh cmd2 / cmd3 / cmd4 (extended IDs).

class TeleopCan {
public:
    TeleopCan() = default;
    ~TeleopCan();

    TeleopCan(const TeleopCan&) = delete;
    TeleopCan& operator=(const TeleopCan&) = delete;
    TeleopCan(TeleopCan&&) = delete;
    TeleopCan& operator=(TeleopCan&&) = delete;

    bool open(const char* iface);
    void close();

    [[nodiscard]] bool is_open() const { return sock_ >= 0; }

    // NOT_TESTED until verified on hardware.
    bool move_linkages_up();
    bool move_linkages_down();
    bool move_linkages_stop();

private:
    bool send_ext_frame(std::uint32_t id29, const std::array<std::uint8_t, 8>& data);

    int sock_{-1};
};

}  // namespace luna_control::teleop_can
