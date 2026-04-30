#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>

namespace luna_control::can_codec {

using Payload8 = std::array<std::uint8_t, 8>;

// Apply whole-byte overrides to a base 8-byte payload.
// - `byte_indexes` and `byte_values` must be the same length.
// - Each index must be in [0,7].
Payload8 change_msg(
    const Payload8& base,
    std::initializer_list<std::size_t> byte_indexes,
    std::initializer_list<std::uint8_t> byte_values);

// Write a packed bitfield into the payload, preserving all other bits.
//
// Bit numbering:
// - `byte_index` selects the starting byte in the payload.
// - `bit_offset` is the bit position within that byte, where 0 = LSB, 7 = MSB.
// - The bitfield may span across subsequent bytes in little-endian bit order.
//
// Example: write_bits(p, 2, 0, 3, 0b101) sets bits 0..2 of p[2] to 101.
//
// Returns false if:
// - byte_index out of range
// - bit_offset >= 8
// - bit_length == 0
// - the bitfield would extend past the 8-byte payload
// - `value` does not fit in `bit_length` bits
bool write_bits(
    Payload8& payload,
    std::size_t byte_index,
    std::uint8_t bit_offset,
    std::uint8_t bit_length,
    std::uint64_t value);

// Convenience: read a packed bitfield using the same numbering rules as write_bits().
// Returns false on invalid ranges (same validation as write_bits()).
bool read_bits(
    const Payload8& payload,
    std::size_t byte_index,
    std::uint8_t bit_offset,
    std::uint8_t bit_length,
    std::uint64_t& out_value);

}  // namespace luna_control::can_codec

