#ifndef SEGWAY_HPP
#define SEGWAY_HPP

#include "imu.hpp"
#include "segway_config.hpp"
#include "wheels.hpp"

namespace Segway {

    struct Segway {
        void update_step_count(this Segway& self, WheelType const wheel_type) noexcept;

        void
        operator()(this Segway& self, std::array<std::float32_t, 6UL> const& x_ref, std::float32_t const dt) noexcept;

        void
        run_segway(this Segway& self, std::array<std::float32_t, 6UL> const& x_ref, std::float32_t const dt) noexcept;

        IMU imu = {};
        SFR regulator = {};
        SFO observer = {};
        Config config = {};
        Wheels wheels = {};
    };

}; // namespace Segway

#endif // SEGWAY_HPP