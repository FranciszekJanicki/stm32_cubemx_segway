#ifndef SEGWAY_HPP
#define SEGWAY_HPP

#include "segway_config.hpp"
#include "sfo.hpp"
#include "sfr.hpp"
#include "wheel.hpp"
#include <array>

namespace Segway {

    using SFR = ::Utility::SFR<std::float32_t, 6UL, 2UL>;
    using SFO = ::Utility::SFO<std::float32_t, 6UL, 2UL>;

    struct Segway {
        void update_step_count(this Segway& self, WheelType const wheel_type) noexcept;

        void operator()(this Segway& self,
                        std::float32_t const dot_tilt,
                        std::float32_t const tilt,
                        std::float32_t const dot_rotation,
                        std::float32_t const rotation,
                        std::float32_t const position,
                        std::float32_t const whells_speed,
                        std::float32_t const sampling_time) noexcept;

        void run_segway(this Segway& self,
                        std::float32_t const dot_tilt,
                        std::float32_t const tilt,
                        std::float32_t const dot_rotation,
                        std::float32_t const rotation,
                        std::float32_t const position,
                        std::float32_t const whells_speed,
                        std::float32_t const sampling_time) noexcept;

        IMU sensor = IMU{};

        SFR regulator = SFR{};
        SFO observer = SFO{};

        std::array<Wheel, 2UL> wheels = std::array<Wheel, 2UL>{};

        std::float32_t prev_tilt = 0.0F;
        std::float32_t prev_rotation = 0.0F;
        std::float32_t prev_position = 0.0F;

    private:
        std::float32_t get_tilt(this Segway& self) noexcept;
        std::float32_t get_rotation(this Segway& self) noexcept;
        std::float32_t get_position(this Segway& self, std::uint32_t const sampling_time) noexcept;

        std::float32_t
        get_wheel_position(this Segway& self, WheelType const wheel_type, std::float32_t const sampling_time) noexcept;
        void set_wheel_speed(this Segway& self,
                             WheelType const wheel_type,
                             std::float32_t const wheel_speed,
                             std::float32_t sampling_time) noexcept;

        WheelDriver& get_wheel_driver(this Segway& self, WheelType const wheel_type) noexcept;
    };

}; // namespace Segway

#endif // SEGWAY_HPP