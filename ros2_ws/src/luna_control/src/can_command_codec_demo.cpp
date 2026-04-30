#include "luna_control/can_command_codec.hpp"

#include <cassert>
#include <cstdint>
#include <iostream>

// TEMPORARY demo/sanity-check.
// This file is intentionally standalone so you can compile it manually without
// integrating the codec into existing control code yet.
//
// Example compile (from the luna_ros repo root):
//   g++ -std=c++17 -Iros2_ws/src/luna_control/include \
//     ros2_ws/src/luna_control/src/can_command_codec.cpp \
//     ros2_ws/src/luna_control/src/can_command_codec_demo.cpp \
//     -o /tmp/can_codec_demo
//
// Run:
//   /tmp/can_codec_demo

static void print_payload(const luna_control::can_codec::Payload8& p) {
    std::cout << "payload: ";
    for (std::uint8_t b : p) {
        std::cout << std::hex << static_cast<int>(b) << " ";
    }
    std::cout << std::dec << "\n";
}

int main() {
    using luna_control::can_codec::Payload8;
    using luna_control::can_codec::change_msg;
    using luna_control::can_codec::read_bits;
    using luna_control::can_codec::write_bits;

    const Payload8 base{0x00, 0xFF, 0xA5, 0x5A, 0x00, 0x00, 0x00, 0x00};

    // Whole-byte edit: change byte 1 from 0xFF -> 0x12, byte 4 -> 0x34.
    const Payload8 p1 = change_msg(base, {1, 4}, {0x12, 0x34});
    assert(p1[0] == 0x00);
    assert(p1[1] == 0x12);
    assert(p1[2] == 0xA5);
    assert(p1[3] == 0x5A);
    assert(p1[4] == 0x34);

    // Bitfield edit inside a byte: set bits 0..2 of byte 2 to 0b101.
    Payload8 p2 = base;
    const bool ok1 = write_bits(p2, /*byte_index=*/2, /*bit_offset=*/0, /*bit_length=*/3, /*value=*/0b101);
    assert(ok1);
    std::uint64_t r1 = 0;
    assert(read_bits(p2, 2, 0, 3, r1));
    assert(r1 == 0b101);

    // Ensure we didn't clobber the upper bits of byte 2 (base byte 2 was 0xA5 = 1010'0101).
    // After writing bits 0..2 to 101, byte2 should still have bits 3..7 = 1010'0.
    assert((p2[2] & 0xF8) == (base[2] & 0xF8));

    // Bitfield spanning bytes: write 12 bits starting at byte 3 bit 4.
    Payload8 p3 = base;
    const bool ok2 = write_bits(p3, /*byte_index=*/3, /*bit_offset=*/4, /*bit_length=*/12, /*value=*/0xABC);
    assert(ok2);
    std::uint64_t r2 = 0;
    assert(read_bits(p3, 3, 4, 12, r2));
    assert(r2 == 0xABC);

    std::cout << "OK: can_command_codec demo passed.\n";
    // Uncomment if you want to inspect bytes:
    // print_payload(p1);
    // print_payload(p2);
    // print_payload(p3);
    return 0;
}

