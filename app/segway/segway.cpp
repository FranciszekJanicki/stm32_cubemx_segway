#include "segway.hpp"
#include "log.hpp"
#include <cassert>

namespace {

    constexpr auto TAG = "Segway";

}; // namespace

namespace Segway {

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
            x[0] = Utility::differentiate(x[1], prev_x[1], dt);
            x[3] = get_wheel_diff_rotation(self.wheels, dt, self.config.wheel_distance);
            x[2] = Utility::differentiate(x[3], prev_x[3], dt);
            x[4] = get_wheel_diff_position(self.wheels, dt);
            x[5] = Utility::differentiate(x[4], prev_x[4], dt);

            self.check_fault(x[1]);

            std::copy(x.begin(), x.end(), prev_x.begin());

            e[0] = x_ref[0] - x[0];
            e[1] = x_ref[1] - x[1];
            e[2] = x_ref[2] - x[2];
            e[3] = x_ref[3] - x[3];
            e[4] = x_ref[4] - x[4];
            e[5] = x_ref[5] - x[5];

            int_e[0] += Utility::integrate(e[0], prev_e[0], dt);
            int_e[1] += Utility::integrate(e[1], prev_e[1], dt);
            int_e[2] += Utility::integrate(e[2], prev_e[2], dt);
            int_e[3] += Utility::integrate(e[3], prev_e[3], dt);
            int_e[4] += Utility::integrate(e[4], prev_e[4], dt);
            int_e[5] += Utility::integrate(e[5], prev_e[5], dt);

            std::copy(e.begin(), e.end(), prev_e.begin());

            u[0] = Utility::state_feedback(Ki[0], int_e[0], Kx[0], x[0]) +
                   Utility::state_feedback(Ki[1], int_e[1], Kx[1], x[1]) +
                   Utility::state_feedback(Ki[2], int_e[2], Kx[2], x[2]) +
                   Utility::state_feedback(Ki[3], int_e[3], Kx[3], x[3]) +
                   Utility::state_feedback(Ki[4], int_e[4], Kx[4], x[4]) +
                   Utility::state_feedback(Ki[5], int_e[5], Kx[5], x[5]);

            u[1] = Utility::state_feedback(Ki[0], int_e[0], Kx[0], x[0]) +
                   Utility::state_feedback(Ki[1], int_e[1], -Kx[1], x[1]) +
                   Utility::state_feedback(Ki[2], int_e[2], -Kx[2], x[2]) +
                   Utility::state_feedback(Ki[3], int_e[3], Kx[3], x[3]) +
                   Utility::state_feedback(Ki[4], int_e[4], Kx[4], x[4]) +
                   Utility::state_feedback(Ki[5], int_e[5], Kx[5], x[5]);

            set_wheels_speed(self.wheels, u[0], u[1], dt);
        }
    }

    void Segway::run_segway_pid(this Segway& self, std::float64_t const tilt_ref, std::float64_t const dt) noexcept
    {
        if (std::holds_alternative<PID>(self.regulator)) {
            auto& regulator = std::get<PID>(self.regulator);

            auto tilt = get_tilt_angle(self.imu);
            self.check_fault(tilt);

            auto e = tilt_ref - tilt;
            LOG(TAG, "Error tilt: %f", e);

            auto u = regulator.get_sat_u(e, dt);
            LOG(TAG, "Control speed: %f", u);

            set_wheels_speed(self.wheels, u, -u, dt);
        }
    }

    void Segway::check_fault(this Segway& self, std::float64_t const tilt) noexcept
    {
        auto& [_, fault_enter, fault_exit, has_fault_occured] = self.config;

        if (std::abs(tilt) > fault_enter) {
            if (!has_fault_occured) {
                stop_wheels(self.wheels);

                has_fault_occured = true;
                LOG(TAG, "FAULT ENTERED. Stopping wheels.");
            }
        } else if (std::abs(tilt) < fault_exit) {
            if (has_fault_occured) {
                start_wheels(self.wheels);

                has_fault_occured = false;
                LOG(TAG, "FAULT CLEARED. Starting wheels.");
            }
        }
    }

}; // namespace Segway