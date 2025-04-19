#include "segway.hpp"
#include "log.hpp"
#include <cassert>

namespace {

    constexpr auto TAG = "Segway";

};

namespace Segway {

    void Segway::update_step_count(this Segway& self, WheelType const wheel_type) noexcept
    {
        auto& wheel_driver = self.get_wheel_driver(wheel_type);
        LOG(TAG, "Updating step count: %s\n\r", wheel_type_to_string(wheel_type));

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
        LOG(TAG, "Run segway!\n\r");

        auto& [Ki, Kx, prev_x, prev_e, int_e, x, e, u] = self.config;

        x[1] = self.get_tilt();
        x[0] = Utility::differentiate(x[0], prev_x[0], dt);
        x[3] = self.get_rotation();
        x[2] = Utility::differentiate(x[3], prev_x[3], dt);
        x[4] = self.get_position(dt);
        x[5] = Utility::differentiate(x[4], prev_x[4], dt);

        std::memcpy(prev_x.data(), x.data(), x.size() * sizeof(e[0]));

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

        std::memcpy(prev_e.data(), e.data(), e.size() * sizeof(e[0]));

        u[std::to_underlying(WheelType::LEFT)] = (Ki[0] * int_e[0] - Kx[0] * x[0]) + (Ki[1] * int_e[1] - Kx[1] * x[1]) +
                                                 (Ki[2] * int_e[2] - Kx[2] * x[2]) + (Ki[3] * int_e[3] - Kx[3] * x[3]) +
                                                 (Ki[4] * int_e[4] - Kx[4] * x[4]) + (Ki[5] * int_e[5] - Kx[5] * x[5]);

        u[std::to_underlying(WheelType::RIGHT)] =
            (Ki[0] * int_e[0] + Kx[0] * x[0]) + (Ki[1] * int_e[1] + Kx[1] * x[1]) + (Ki[2] * int_e[2] + Kx[2] * x[2]) +
            (Ki[3] * int_e[3] + Kx[3] * x[3]) + (Ki[4] * int_e[4] + Kx[4] * x[4]) + (Ki[5] * int_e[5] + Kx[5] * x[5]);

        self.set_wheel_speed(WheelType::LEFT, u[std::to_underlying(WheelType::LEFT)], dt);
        self.set_wheel_speed(WheelType::RIGHT, u[std::to_underlying(WheelType::RIGHT)], dt);
    }

    std::float32_t Segway::get_tilt(this Segway& self) noexcept
    {
        auto const measured_tilt = std::visit([](auto& imu) { return imu.get_roll().value(); }, self.sensor);
        LOG(TAG, "Tilt: %f\n\r", measured_tilt);

        return measured_tilt;
    }

    std::float32_t Segway::get_rotation(this Segway& self) noexcept
    {
        auto const measured_rotation = std::visit([](auto& imu) { return imu.get_yaw().value(); }, self.sensor);
        LOG(TAG, "Rotation: %f\n\r", measured_rotation);

        return measured_rotation;
    }

    std::float32_t
    Segway::get_wheel_position(this Segway& self, WheelType const wheel_type, std::float32_t const dt) noexcept
    {
        auto& wheel_driver = self.get_wheel_driver(wheel_type);
        auto const wheel_position = wheel_driver.get_wheel_position(dt);
        LOG(TAG, "%s position: %f\n\r", wheel_type_to_string(wheel_type), wheel_position);

        return wheel_position;
    }

    std::float32_t Segway::get_position(this Segway& self, std::uint32_t const dt) noexcept
    {
        auto const position =
            self.get_wheel_position(WheelType::LEFT, dt) - self.get_wheel_position(WheelType::RIGHT, dt);
        LOG(TAG, "Wheel diff position: %f\n\r", position);

        return position;
    }

    void Segway::set_wheel_speed(this Segway& self,
                                 WheelType const wheel_type,
                                 std::float32_t const wheel_speed,
                                 std::float32_t const dt) noexcept
    {
        auto& wheel_driver = self.get_wheel_driver(wheel_type);
        LOG(TAG, "Setting %s speed to %f\n\r", wheel_type_to_string(wheel_type), wheel_speed);

        wheel_driver.set_wheel_speed(wheel_speed, dt);
    }

    WheelDriver& Segway::get_wheel_driver(this Segway& self, WheelType const wheel_type) noexcept
    {
        auto* wheel =
            std::ranges::find_if(self.wheels, [wheel_type](Wheel const& wheel) { return wheel.type == wheel_type; });
        assert(wheel);

        return wheel->driver;
    }

}; // namespace Segway