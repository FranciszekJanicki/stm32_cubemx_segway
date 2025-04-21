#ifndef SEGWAY_HPP
#define SEGWAY_HPP

#include "imu.hpp"
#include "segway_config.hpp"
#include "wheels.hpp"

namespace Segway {

    struct Segway {
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

    private:
        void check_fault(this Segway& self, std::float64_t const tilt) noexcept;
    };

}; // namespace Segway

#endif // SEGWAY_HPP