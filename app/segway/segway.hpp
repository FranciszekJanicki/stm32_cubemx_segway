#ifndef SEGWAY_HPP
#define SEGWAY_HPP

#include "imu.hpp"
#include "segway_config.hpp"
#include "wheels.hpp"

namespace segway {

    struct Segway {
        void initialize(this Segway& self) noexcept;
        void deinitialize(this Segway& self) noexcept;

        void update_step_count(this Segway& self, WheelType const wheel_type) noexcept;

        void
        operator()(this Segway& self, std::array<std::float64_t, 6UL> const& x_ref, std::float64_t const dt) noexcept;

        void run_segway_lqr(this Segway& self,
                            std::array<std::float64_t, 6UL> const& x_ref,
                            std::float64_t const dt) noexcept;

        void run_segway_pid(this Segway& self, std::float64_t const tilt_ref, std::float64_t const dt) noexcept;

        IMU imu = {};
        Wheels wheels = {};
        Regulator regulator = {};
        Config config = {};
        bool is_stopped = {};

    private:
        void start(this Segway& self) noexcept;
        void stop(this Segway& self) noexcept;

        bool should_start(this Segway& self, std::float64_t const tilt, std::float64_t const wheel_speed) noexcept;
        bool should_stop(this Segway& self, std::float64_t const tilt, std::float64_t const wheel_speed) noexcept;
    };

}; // namespace segway

#endif // SEGWAY_HPP