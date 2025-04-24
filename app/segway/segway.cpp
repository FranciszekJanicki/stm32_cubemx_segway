#include "segway.hpp"
#include "log.hpp"
#include <cassert>

namespace {

    constexpr auto TAG = "Segway";

}; // namespace

namespace segway {

    void Segway::update_step_count(this Segway& self, WheelType const wheel_type) noexcept
    {
        update_wheel_step_count(self.wheels, wheel_type);
    }

    void Segway::operator()(this Segway& self,
                            std::array<std::float64_t, 6UL> const& x_ref,
                            std::float64_t const dt) noexcept
    {
        self.run_segway_lqr(x_ref, dt);
    }

    void Segway::run_segway_lqr(this Segway& self,
                                std::array<std::float64_t, 6UL> const& x_ref,
                                std::float64_t const dt) noexcept
    {
        if (std::holds_alternative<LQR>(self.regulator)) {
            auto& [Kx, Ki, prev_x, prev_e, int_e, x, e, u] = std::get<LQR>(self.regulator);

            x[1] = get_tilt_angle(self.imu);
            x[0] = utility::differentiate(x[1], prev_x[1], dt);
            x[3] = get_wheel_diff_rotation(self.wheels, dt, self.config.wheel_distance);
            x[2] = utility::differentiate(x[3], prev_x[3], dt);
            x[4] = get_wheel_diff_position(self.wheels, dt);
            x[5] = utility::differentiate(x[4], prev_x[4], dt);

            std::copy(x.begin(), x.end(), prev_x.begin());

            e[0] = x_ref[0] - x[0];
            e[1] = x_ref[1] - x[1];
            e[2] = x_ref[2] - x[2];
            e[3] = x_ref[3] - x[3];
            e[4] = x_ref[4] - x[4];
            e[5] = x_ref[5] - x[5];

            int_e[0] += utility::integrate(e[0], prev_e[0], dt);
            int_e[1] += utility::integrate(e[1], prev_e[1], dt);
            int_e[2] += utility::integrate(e[2], prev_e[2], dt);
            int_e[3] += utility::integrate(e[3], prev_e[3], dt);
            int_e[4] += utility::integrate(e[4], prev_e[4], dt);
            int_e[5] += utility::integrate(e[5], prev_e[5], dt);

            std::copy(e.begin(), e.end(), prev_e.begin());

            u[0] = utility::state_feedback(Ki[0], int_e[0], Kx[0], x[0]) +
                   utility::state_feedback(Ki[1], int_e[1], Kx[1], x[1]) +
                   utility::state_feedback(Ki[2], int_e[2], Kx[2], x[2]) +
                   utility::state_feedback(Ki[3], int_e[3], Kx[3], x[3]) +
                   utility::state_feedback(Ki[4], int_e[4], Kx[4], x[4]) +
                   utility::state_feedback(Ki[5], int_e[5], Kx[5], x[5]);

            u[1] = utility::state_feedback(Ki[0], int_e[0], Kx[0], x[0]) +
                   utility::state_feedback(Ki[1], int_e[1], -Kx[1], x[1]) +
                   utility::state_feedback(Ki[2], int_e[2], -Kx[2], x[2]) +
                   utility::state_feedback(Ki[3], int_e[3], Kx[3], x[3]) +
                   utility::state_feedback(Ki[4], int_e[4], Kx[4], x[4]) +
                   utility::state_feedback(Ki[5], int_e[5], Kx[5], x[5]);
        }
    }

    void Segway::run_segway_pid(this Segway& self, std::float64_t const tilt_ref, std::float64_t const dt) noexcept
    {
        if (std::holds_alternative<PID>(self.regulator)) {
            auto& regulator = std::get<PID>(self.regulator);

            auto tilt = get_tilt_angle(self.imu);
            LOG(TAG, "Measured ilt: %f", tilt);
            auto error_tilt = tilt_ref - tilt;
            LOG(TAG, "Error tilt: %f", error_tilt);
            auto control_speed = regulator.get_sat_u(error_tilt, dt);
            LOG(TAG, "Control speed: %f", control_speed);

            // if (self.should_start(tilt, control_speed)) {
            //     self.start();
            // } else if (self.should_stop(tilt, control_speed)) {
            //     self.stop();
            // }

            set_wheels_speed(self.wheels, control_speed, -control_speed, dt);
        }
    }

    void Segway::start(this Segway& self) noexcept
    {
        if (self.is_stopped) {
            self.is_stopped = false;
            start_wheels(self.wheels);
        }
    }

    void Segway::stop(this Segway& self) noexcept
    {
        if (!self.is_stopped) {
            self.is_stopped = true;
            stop_wheels(self.wheels);
        }
    }

    bool Segway::should_start(this Segway& self, std::float64_t const tilt, std::float64_t const wheel_speed) noexcept
    {
        return (std::abs(tilt) < self.config.imu_fault_thresh_low) &&
               (std::abs(wheel_speed) < self.config.wheel_fault_thresh_low) &&
               (std::abs(tilt) < self.config.tilt_fault_thresh_low) && self.is_stopped;
    }

    bool Segway::should_stop(this Segway& self, std::float64_t const tilt, std::float64_t const wheel_speed) noexcept
    {
        return (std::abs(tilt) > self.config.imu_fault_thresh_high) &&
               (std::abs(wheel_speed) > self.config.wheel_fault_thresh_high) &&
               (std::abs(tilt) > self.config.tilt_fault_thresh_high) && !self.is_stopped;
    }

}; // namespace segway