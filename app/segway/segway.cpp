#include "segway.hpp"

namespace Segway {

    void Segway::update_step_count(this Segway& self, WheelType const wheel_type) noexcept
    {
        auto& wheel_driver = self.get_wheel_driver(wheel_type);
        wheel_driver.driver.update_step_count();
    }

    void Segway::operator()(this Segway& self,
                            std::float32_t const dot_tilt,
                            std::float32_t const tilt,
                            std::float32_t const dot_rotation,
                            std::float32_t const rotation,
                            std::float32_t const position,
                            std::float32_t const step_diff,
                            std::float32_t const sampling_time) noexcept
    {
        self.run_segway(dot_tilt, tilt, dot_rotation, rotation, position, step_diff, sampling_time);
    }

    void Segway::run_segway(this Segway& self,
                            std::float32_t const dot_tilt,
                            std::float32_t const tilt,
                            std::float32_t const dot_rotation,
                            std::float32_t const rotation,
                            std::float32_t const position,
                            std::float32_t const step_diff,
                            std::float32_t const sampling_time) noexcept
    {
        auto const measured_tilt =
            std::visit([](auto& imu) { return imu.get_roll().value_or(0.0F); }, self.sensor);
        std::printf("roll: %f\n\r", measured_tilt);

        auto const measured_rotation =
            std::visit([](auto& imu) { return imu.get_yaw().value_or(0.0F); }, self.sensor);
        std::printf("roll: %f\n\r", measured_rotation);

        auto const left_wheel_speed = 0.0F;
        auto const right_wheel_speed = 0.0F;

        self.set_wheel_speed(WheelType::LEFT, left_wheel_speed, sampling_time);
        self.set_wheel_speed(WheelType::RIGHT, right_wheel_speed, sampling_time);
    }

    void Segway::set_wheel_speed(this Segway& self,
                                 WheelType const wheel_type,
                                 std::float32_t const wheel_speed,
                                 std::float32_t const sampling_time) noexcept
    {
        auto& wheel_driver = self.get_wheel_driver(wheel_type);
        wheel_driver.set_wheel_speed(wheel_speed, sampling_time);
    }

    WheelDriver& Segway::get_wheel_driver(this Segway& self, WheelType const wheel_type) noexcept
    {
        auto* wheel = std::ranges::find_if(self.wheels, [wheel_type](Wheel const& wheel) {
            return wheel.type == wheel_type;
        });

        return wheel->driver;
    }

}; // namespace Segway