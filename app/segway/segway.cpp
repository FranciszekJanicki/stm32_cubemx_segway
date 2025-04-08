#include "segway.hpp"

namespace Segway {

    void Segway::update_step_count(this Segway& self, WheelType const wheel_type) noexcept
    {
        auto& wheel_driver = self.get_wheel_driver(wheel_type);
        std::printf("Updating step count: %s\n\r", wheel_type_to_string(wheel_type));

        wheel_driver.driver.update_step_count();
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

        x[1] = self.get_tilt();
        x[0] = Utility::differentiate(x[0], prev_x[0], dt);
        x[3] = self.get_rotation();
        x[2] = Utility::differentiate(x[3], prev_x[3], dt);
        x[4] = self.get_position(dt);
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

        std::copy(e.begin(), e.end(), prev_e.begin());

        u[0] = (Ki[0] * int_e[0] - Kx[0][0] * x[0]) + (Ki[1] * int_e[1] - Kx[0][1] * x[1]) +
               (Ki[2] * int_e[2] - Kx[0][2] * x[2]) + (Ki[3] * int_e[3] - Kx[0][3] * x[3]) +
               (Ki[4] * int_e[4] - Kx[0][4] * x[4]) + (Ki[5] * int_e[5] - Kx[0][5] * x[5]);

        u[1] = (Ki[0] * int_e[0] - Kx[1][0] * x[0]) + (Ki[1] * int_e[1] - Kx[1][1] * x[1]) +
               (Ki[2] * int_e[2] - Kx[1][2] * x[2]) + (Ki[3] * int_e[3] - Kx[1][3] * x[3]) +
               (Ki[4] * int_e[4] - Kx[1][4] * x[4]) + (Ki[5] * int_e[5] - Kx[1][5] * x[5]);

        self.set_wheel_speed(WheelType::LEFT, u[0], dt);
        self.set_wheel_speed(WheelType::RIGHT, u[1], dt);
    }

    std::float32_t Segway::get_tilt(this Segway& self) noexcept
    {
        auto const measured_tilt = std::visit([](auto& imu) { return imu.get_roll().value_or(0.0F); }, self.sensor);
        std::printf("Tilt: %f\n\r", measured_tilt);

        return measured_tilt;
    }

    std::float32_t Segway::get_rotation(this Segway& self) noexcept
    {
        auto const measured_rotation = std::visit([](auto& imu) { return imu.get_yaw().value_or(0.0F); }, self.sensor);
        std::printf("Rotation: %f\n\r", measured_rotation);

        return measured_rotation;
    }

    std::float32_t
    Segway::get_wheel_position(this Segway& self, WheelType const wheel_type, std::float32_t const dt) noexcept
    {
        auto& wheel_driver = self.get_wheel_driver(wheel_type);
        return wheel_driver.get_wheel_position(dt);
    }

    std::float32_t Segway::get_position(this Segway& self, std::uint32_t const dt) noexcept
    {
        auto const position =
            self.get_wheel_position(WheelType::LEFT, dt) - self.get_wheel_position(WheelType::RIGHT, dt);
        std::printf("Wheel diff position: %f\n\r", position);

        return position;
    }

    void Segway::set_wheel_speed(this Segway& self,
                                 WheelType const wheel_type,
                                 std::float32_t const wheel_speed,
                                 std::float32_t const dt) noexcept
    {
        auto& wheel_driver = self.get_wheel_driver(wheel_type);
        std::printf("Setting %s speed to %f\n\r", wheel_type_to_string(wheel_type), wheel_speed);

        wheel_driver.set_wheel_speed(wheel_speed, dt);
    }

    WheelDriver& Segway::get_wheel_driver(this Segway& self, WheelType const wheel_type) noexcept
    {
        auto* wheel =
            std::ranges::find_if(self.wheels, [wheel_type](Wheel const& wheel) { return wheel.type == wheel_type; });

        return wheel->driver;
    }

}; // namespace Segway