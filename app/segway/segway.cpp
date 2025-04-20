#include "segway.hpp"
#include "log.hpp"
#include <cassert>

namespace {

    constexpr auto TAG = "Segway";

};

namespace Segway {

    void Segway::update_step_count(this Segway& self, WheelType const wheel_type) noexcept
    {
        self.wheels.update_wheel_step_count(wheel_type);
    }

    void Segway::operator()(this Segway& self,
                            std::array<std::float32_t, 6UL> const& x_ref,
                            std::float32_t const dt) noexcept
    {
        self.run_segway(x_ref, dt);
    }

    void Segway::run_segway(this Segway& self,
                            std::array<std::float32_t, 6UL> const& x_ref,
                            std::float32_t const dt) noexcept
    {
        auto& [Ki, Kx, prev_x, prev_e, int_e, x, e, u] = self.config;

        x[1] = self.imu.get_tilt();
        x[0] = Utility::differentiate(x[1], prev_x[1], dt);
        x[3] = self.wheels.get_wheel_diff_rotation(dt);
        x[2] = Utility::differentiate(x[3], prev_x[3], dt);
        x[4] = self.wheels.get_wheel_diff_position(dt);
        x[5] = Utility::differentiate(x[4], prev_x[4], dt);

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

        std::copy(e.begin(), e.begin(), prev_e.data());

        u[std::to_underlying(WheelType::LEFT)] = (Ki[0] * int_e[0] - Kx[0] * x[0]) + (Ki[1] * int_e[1] - Kx[1] * x[1]) +
                                                 (Ki[2] * int_e[2] - Kx[2] * x[2]) + (Ki[3] * int_e[3] - Kx[3] * x[3]) +
                                                 (Ki[4] * int_e[4] - Kx[4] * x[4]) + (Ki[5] * int_e[5] - Kx[5] * x[5]);

        u[std::to_underlying(WheelType::RIGHT)] =
            (Ki[0] * int_e[0] + Kx[0] * x[0]) + (Ki[1] * int_e[1] + Kx[1] * x[1]) + (Ki[2] * int_e[2] + Kx[2] * x[2]) +
            (Ki[3] * int_e[3] + Kx[3] * x[3]) + (Ki[4] * int_e[4] + Kx[4] * x[4]) + (Ki[5] * int_e[5] + Kx[5] * x[5]);

        for (auto u_val : u) {
            LOG(TAG, "Control signal: %f\n\r", u_val);
        }

        self.wheels.set_wheel_speed(WheelType::LEFT, u[std::to_underlying(WheelType::LEFT)], dt);
        self.wheels.set_wheel_speed(WheelType::RIGHT, u[std::to_underlying(WheelType::RIGHT)], dt);
    }

}; // namespace Segway