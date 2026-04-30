#include "luna_control/can_command_codec.hpp"

#include <algorithm>

namespace luna_control::can_codec {

Payload8 change_msg(
    const Payload8& base,
    std::initializer_list<std::size_t> byte_indexes,
    std::initializer_list<std::uint8_t> byte_values) {
    Payload8 out = base;

    const std::size_t n = byte_indexes.size();
    if (n != byte_values.size()) {
        return out;  // no partial edits if caller messed up lengths
    }

    auto idx_it = byte_indexes.begin();
    auto val_it = byte_values.begin();
    for (std::size_t i = 0; i < n; ++i, ++idx_it, ++val_it) {
        const std::size_t idx = *idx_it;
        if (idx >= out.size()) {
            continue;
        }
        out[idx] = *val_it;
    }

    return out;
}

static bool validate_range(
    std::size_t byte_index,
    std::uint8_t bit_offset,
    std::uint8_t bit_length) {
    if (byte_index >= 8) return false;
    if (bit_offset >= 8) return false;
    if (bit_length == 0) return false;

    const std::size_t start_bit = byte_index * 8 + bit_offset;
    const std::size_t end_bit_exclusive = start_bit + bit_length;
    return end_bit_exclusive <= 64;
}

bool write_bits(
    Payload8& payload,
    std::size_t byte_index,
    std::uint8_t bit_offset,
    std::uint8_t bit_length,
    std::uint64_t value) {
    if (!validate_range(byte_index, bit_offset, bit_length)) return false;

    if (bit_length < 64) {
        const std::uint64_t max_value = (1ULL << bit_length) - 1ULL;
        if (value > max_value) return false;
    }

    const std::size_t start_bit = byte_index * 8 + bit_offset;

    // Simple, explicit bit loop (safe, easy to reason about; fast enough for 8 bytes).
    for (std::size_t i = 0; i < bit_length; ++i) {
        const std::size_t abs_bit = start_bit + i;
        const std::size_t byte_i = abs_bit / 8;
        const std::size_t bit_i = abs_bit % 8;  // 0 = LSB

        const std::uint8_t bit = static_cast<std::uint8_t>((value >> i) & 0x1ULL);
        const std::uint8_t mask = static_cast<std::uint8_t>(1U << bit_i);

        if (bit) {
            payload[byte_i] = static_cast<std::uint8_t>(payload[byte_i] | mask);
        } else {
            payload[byte_i] = static_cast<std::uint8_t>(payload[byte_i] & static_cast<std::uint8_t>(~mask));
        }
    }

    return true;
}

bool read_bits(
    const Payload8& payload,
    std::size_t byte_index,
    std::uint8_t bit_offset,
    std::uint8_t bit_length,
    std::uint64_t& out_value) {
    if (!validate_range(byte_index, bit_offset, bit_length)) return false;

    const std::size_t start_bit = byte_index * 8 + bit_offset;

    std::uint64_t v = 0;
    for (std::size_t i = 0; i < bit_length; ++i) {
        const std::size_t abs_bit = start_bit + i;
        const std::size_t byte_i = abs_bit / 8;
        const std::size_t bit_i = abs_bit % 8;

        const std::uint8_t bit = static_cast<std::uint8_t>((payload[byte_i] >> bit_i) & 0x1U);
        v |= (static_cast<std::uint64_t>(bit) << i);
    }

    out_value = v;
    return true;
}

}  // namespace luna_control::can_codec

