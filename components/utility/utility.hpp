#ifndef UTILITY_UTILITY_HPP
#define UTILITY_UTILITY_HPP

#include "quaternion3d.hpp"
#include "vector3d.hpp"
#include <algorithm>
#include <array>
#include <bit>
#include <bitset>
#include <cassert>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace utility {

    template <std::floating_point T>
    inline T differentiate(T value, T prev_value, T sampling_time, T prev_derivative, T time_ant)
    {
        if (time_ant + sampling_time == static_cast<T>(0)) {
            throw std::runtime_error{"Division by 0"};
        }
        return (value - prev_value + prev_derivative * time_ant) / (time_ant + sampling_time);
    }

    template <std::floating_point T>
    inline T differentiate(T value, T prev_value, T sampling_time)
    {
        if (sampling_time == static_cast<T>(0)) {
            throw std::runtime_error{"Division by 0"};
        }
        return (value - prev_value) / sampling_time;
    }

    template <std::floating_point T>
    inline T integrate(T value, T prev_value, T sampling_time) noexcept
    {
        return (value + prev_value) * static_cast<T>(0.5F) * sampling_time;
    }

    template <std::floating_point T>
    inline T degrees_to_radians(T degrees) noexcept
    {
        return degrees * std::numbers::pi_v<T> / static_cast<T>(180.0);
    }

    template <std::floating_point T>
    inline T radians_to_degrees(T radians) noexcept
    {
        return radians * static_cast<T>(180.0) / std::numbers::pi_v<T>;
    }

    template <std::unsigned_integral UInt>
    inline void write_bits(UInt& data,
                           UInt write_data,
                           std::uint8_t write_size,
                           std::uint8_t write_position) noexcept
    {
        UInt mask = ((1U << write_size) - 1U) << (write_position - write_size + 1U);
        UInt temp = (write_data << (write_position - write_size + 1U));
        temp &= mask;
        data &= ~mask;
        data |= temp;
    }

    template <std::unsigned_integral UInt>
    inline void write_bit(UInt& data, bool write_data, std::uint8_t write_position) noexcept
    {
        write_data ? (data |= (1U << write_position)) : (data &= ~(1U << write_position));
    }

    template <std::unsigned_integral UInt>
    inline UInt read_bits(UInt data, std::uint8_t read_size, std::uint8_t read_position) noexcept
    {
        UInt mask = ((1U << read_size) - 1U) << (read_position - read_size + 1U);
        data &= mask;
        data >>= (read_position - read_size + 1U);

        return data;
    }

    template <std::unsigned_integral UInt>
    inline bool read_bit(UInt data, std::uint8_t read_position) noexcept
    {
        return (data & (1U << read_position)) ? true : false;
    }

    inline std::uint16_t big_endian_bytes_to_word(std::array<std::uint8_t, 2UL> bytes) noexcept
    {
        return static_cast<std::uint16_t>(bytes[0UL] << 8UL) |
               static_cast<std::uint16_t>(bytes[1UL]);
    }

    inline std::uint16_t little_endian_bytes_to_word(std::array<std::uint8_t, 2UL> bytes) noexcept
    {
        return static_cast<std::uint16_t>(bytes[0UL]) |
               static_cast<std::uint16_t>(bytes[1UL] << 8UL);
    }

    inline std::array<std::uint8_t, 2UL> word_to_big_endian_bytes(std::uint16_t word) noexcept
    {
        return std::array<std::uint8_t, 2UL>{static_cast<std::uint8_t>(word >> 8UL),
                                             static_cast<std::uint8_t>(word)};
    }

    inline std::array<std::uint8_t, 2UL> word_to_little_endian_bytes(std::uint16_t word) noexcept
    {
        return std::array<std::uint8_t, 2UL>{static_cast<std::uint8_t>(word),
                                             static_cast<std::uint8_t>(word >> 8UL)};
    }

    inline std::uint32_t big_endian_bytes_to_dword(std::array<std::uint8_t, 4UL> bytes) noexcept
    {
        return static_cast<std::uint32_t>(bytes[0UL] << 24UL) |
               static_cast<std::uint32_t>(bytes[1UL] << 16UL) |
               static_cast<std::uint32_t>(bytes[2UL] << 8UL) |
               static_cast<std::uint16_t>(bytes[3UL]);
    }

    inline std::uint32_t little_endian_bytes_to_dword(std::array<std::uint8_t, 4UL> bytes) noexcept
    {
        return static_cast<std::uint32_t>(bytes[0UL]) |
               static_cast<std::uint32_t>(bytes[1UL] << 8UL) |
               static_cast<std::uint32_t>(bytes[2UL] << 16UL) |
               static_cast<std::uint32_t>(bytes[3UL] << 24UL);
    }

    inline std::array<std::uint8_t, 4UL> dword_to_big_endian_bytes(std::uint32_t dword) noexcept
    {
        return std::array<std::uint8_t, 4UL>{static_cast<std::uint8_t>(dword >> 24UL),
                                             static_cast<std::uint8_t>(dword >> 16UL),
                                             static_cast<std::uint8_t>(dword >> 8UL),
                                             static_cast<std::uint8_t>(dword)};
    }

    inline std::array<std::uint8_t, 4UL> dword_to_little_endian_bytes(std::uint32_t dword) noexcept
    {
        return std::array<std::uint8_t, 4UL>{static_cast<std::uint8_t>(dword),
                                             static_cast<std::uint8_t>(dword >> 8UL),
                                             static_cast<std::uint8_t>(dword >> 16UL),
                                             static_cast<std::uint8_t>(dword >> 24UL)};
    }

    template <std::floating_point From, std::floating_point To>
    inline To rescale(From from_value, From from_min, From from_max, To to_min, To to_max) noexcept
    {
        return (std::clamp(from_value, from_min, from_max) - from_min) * (to_max - to_min) /
                   (from_max - from_min) +
               to_min;
    }

    template <std::unsigned_integral UInt>
    inline UInt reflection(UInt data) noexcept
    {
        UInt reflection = 0U;

        for (std::uint8_t i = 0U; i < std::bit_width(data); ++i) {
            write_bit(reflection, read_bit(data, i), std::bit_width(data) - 1U - i);
        }

        return reflection;
    }

    template <std::unsigned_integral UInt, std::size_t SIZE>
    inline UInt calculate_crc(std::array<std::uint8_t, SIZE>& data,
                              UInt init,
                              UInt polynomial,
                              UInt xor_out,
                              bool reflect_in,
                              bool reflect_out) noexcept
    {
        UInt crc = init;
        UInt msb_mask = 1U << (std::bit_width(crc) - 1U);
        UInt crc_mask = (1U << std::bit_width(crc)) - 1U;

        for (std::uint8_t byte : data) {
            if (reflect_in) {
                byte = reflection(byte);
            }
            crc ^= byte << (std::bit_width(crc) - 8U);

            for (std::uint8_t bit{}; bit < 8U; ++bit) {
                if (crc & msb_mask) {
                    crc = (crc << 1U) ^ polynomial;
                } else {
                    crc <<= 1U;
                }
            }
        }

        if (reflect_out) {
            crc = reflect(crc);
        }
        crc ^= xor_out;

        return crc & crc_mask;
    }

    inline std::uint32_t counter_to_freq_hz(std::uint32_t count,
                                            std::uint32_t prescaler,
                                            std::uint32_t clock_freq_hz,
                                            std::uint32_t clock_divider = 0UL) noexcept
    {
        return clock_freq_hz / ((count + 1UL) * (prescaler + 1UL) * (clock_divider + 1UL));
    }

    inline std::uint32_t counter_to_time_us(std::uint32_t count,
                                            std::uint32_t prescaler,
                                            std::uint32_t clock_freq_hz,
                                            std::uint32_t clock_divider = 0UL) noexcept
    {
        return 1000000UL / clock_freq_hz *
               ((count + 1UL) * (prescaler + 1UL) * (clock_divider + 1UL));
    }

    inline std::uint32_t freq_hz_to_counter(std::uint32_t freq_hz,
                                            std::uint32_t prescaler,
                                            std::uint32_t clock_freq_hz,
                                            std::uint32_t clock_divider = 0UL) noexcept
    {
        return clock_freq_hz / ((prescaler + 1UL) * (clock_divider + 1UL) * (freq_hz)) - 1UL;
    }

    inline std::uint32_t time_us_to_counter(std::uint32_t time_us,
                                            std::uint32_t prescaler,
                                            std::uint32_t clock_freq_hz,
                                            std::uint32_t clock_divider = 0UL) noexcept
    {
        return time_us / 1000000UL * clock_freq_hz / ((prescaler + 1UL) * (clock_divider + 1UL)) -
               1UL;
    }

    template <std::floating_point T>
    inline T accel_to_roll(Vector3D<T> const& accel) noexcept
    {
        return std::atan2(accel.y, accel.z);
    }

    template <std::floating_point T>
    inline T accel_to_pitch(Vector3D<T> const& accel) noexcept
    {
        return -std::atan2(accel.x, std::sqrt(accel.y * accel.y + accel.z * accel.z));
    }

    template <std::floating_point T>
    inline T accel_to_yaw([[maybe_unused]] Vector3D<T> const& accel) noexcept
    {
        return static_cast<T>(0);
    }

    template <std::floating_point T>
    inline Vector3D<T> accel_to_roll_pitch_yaw(Vector3D<T> const& accel) noexcept
    {
        return Vector3D<T>{.x = accel_to_roll(accel),
                           .y = accel_to_pitch(accel),
                           .z = accel_to_yaw(accel)};
    }

    template <std::floating_point T>
    inline Vector3D<T> quaternion_to_gravity(Quaternion3D<T> const& quaternion) noexcept
    {
        return Vector3D<T>{
            .x = static_cast<T>(2) * (quaternion.x * quaternion.z - quaternion.w * quaternion.y),
            .y = static_cast<T>(2) * (quaternion.w * quaternion.x + quaternion.y * quaternion.z),
            .z = quaternion.w * quaternion.w - quaternion.x * quaternion.x -
                 quaternion.y * quaternion.y + quaternion.z * quaternion.z};
    }

    template <std::floating_point T>
    inline T quaternion_to_roll(Quaternion3D<T> const& quaternion) noexcept
    {
        auto gravity = quaternion_to_gravity(quaternion);

        return std::atan2(gravity.y, gravity.z);
    }

    template <std::floating_point T>
    inline T quaternion_to_pitch(Quaternion3D<T> const& quaternion) noexcept
    {
        auto gravity = quaternion_to_gravity(quaternion);
        auto pitch =
            std::atan2(gravity.x, std::sqrt(gravity.y * gravity.y + gravity.z * gravity.z));

        return (gravity.z < static_cast<T>(0))
                   ? (pitch > static_cast<T>(0) ? std::numbers::pi_v<T> - pitch
                                                : -std::numbers::pi_v<T> - pitch)
                   : pitch;
    }

    template <std::floating_point T>
    inline T quaternion_to_yaw(Quaternion3D<T> const& quaternion) noexcept
    {
        return std::atan2(static_cast<T>(2) * quaternion.x * quaternion.y -
                              static_cast<T>(2) * quaternion.w * quaternion.z,
                          static_cast<T>(2) * quaternion.w * quaternion.w +
                              static_cast<T>(2) * quaternion.x * quaternion.x - static_cast<T>(1));
    }

    template <std::floating_point T>
    inline Vector3D<T> quaternion_to_roll_pitch_yaw(Quaternion3D<T> const& quaternion) noexcept
    {
        return Vector3D<T>{.x = quaternion_to_roll(quaternion),
                           .y = quaternion_to_pitch(quaternion),
                           .z = quaternion_to_yaw(quaternion)};
    }

    template <std::floating_point T>
    inline T state_feedback(T Ki, T int_e, T Kx, T x) noexcept
    {
        return Ki * int_e - Kx * x;
    }

    template <std::floating_point T>
    inline T x(T y_ref, T Kx, T x) noexcept
    {
        return y_ref - Kx * x;
    }

    template <std::floating_point T>
    inline T state_feedback(T y_ref, T Kx, T x, T Ki, T int_e) noexcept
    {
        if (std::abs(Ki) < 0.001F32) {
            return state_feedback(y_ref, Kx, x);
        } else {
            return state_feedback(Ki, int_e, Kx, x);
        }
    }

    inline void frequency_to_prescaler_and_period(std::uint32_t frequency,
                                                  std::uint32_t clock_hz,
                                                  std::uint32_t clock_div,
                                                  std::uint32_t max_prescaler,
                                                  std::uint32_t max_period,
                                                  std::uint32_t& prescaler,
                                                  std::uint32_t& period) noexcept
    {
        if (frequency > 0UL) {
            period = clock_hz / frequency;
            prescaler = 0UL;

            while (period > max_period && prescaler < max_prescaler) {
                prescaler += 1UL;
                period = clock_hz / ((prescaler + 1UL) * (clock_div + 1UL) * frequency);
            }

            period = std::min(period, max_period);

            if (period == max_period) {
                prescaler = clock_hz / (max_period * frequency) - 1UL;
                if (prescaler > max_prescaler) {
                    prescaler = max_prescaler;
                }
            }

            prescaler = std::min(prescaler, max_prescaler);
        }
    }

}; // namespace utility

#endif // UTILITY_UTILITY_HPP