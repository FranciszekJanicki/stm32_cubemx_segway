#include "segway.hpp"

namespace Segway {

    void Segway::update_step_count(this Segway& self, WheelType const wheel_type) noexcept
    {
        auto& wheel_driver = self.get_wheel_driver(wheel_type);
        std::printf("Updating step count: %s\n\r", wheel_type_to_string(wheel_type));

        wheel_driver.driver.update_step_count();
    }

    void Segway::operator()(this Segway& self,
                            std::float32_t const dot_tilt,
                            std::float32_t const tilt,
                            std::float32_t const dot_rotation,
                            std::float32_t const rotation,
                            std::float32_t const wheels_position,
                            std::float32_t const whells_speed,
                            std::float32_t const sampling_time) noexcept
    {
        self.run_segway(dot_tilt,
                        tilt,
                        dot_rotation,
                        rotation,
                        wheels_position,
                        whells_speed,
                        sampling_time);
    }

    void Segway::run_segway(this Segway& self,
                            std::float32_t const dot_tilt,
                            std::float32_t const tilt,
                            std::float32_t const dot_rotation,
                            std::float32_t const rotation,
                            std::float32_t const wheels_position,
                            std::float32_t const whells_speed,
                            std::float32_t const sampling_time) noexcept
    {
        auto const dot_tilt_K1 = 0.0F;
        auto const tilt_K1 = 0.0F;
        auto const dot_rotation_K1 = 0.0F;
        auto const rotation_K1 = 0.0F;
        auto const wheels_position_K1 = 0.0F;
        auto const whells_speed_K1 = 0.0F;

        auto const dot_tilt_K2 = 0.0F;
        auto const tilt_K2 = 0.0F;
        auto const dot_rotation_K2 = 0.0F;
        auto const rotation_K2 = 0.0F;
        auto const wheels_position_K2 = 0.0F;
        auto const whells_speed_K2 = 0.0F;

        auto const meas_tilt = self.get_tilt();
        auto const meas_dot_tilt = Utility::differentiate(meas_tilt,
                                                          std::exchange(self.prev_tilt, meas_tilt),
                                                          sampling_time);
        auto const meas_rotation = self.get_rotation();
        auto const meas_dot_rotation =
            Utility::differentiate(meas_rotation,
                                   std::exchange(self.prev_rotation, meas_rotation),
                                   sampling_time);
        auto const meas_wheels_position = self.get_wheels_position(sampling_time);
        auto const meas_whells_speed =
            Utility::differentiate(meas_wheels_position,
                                   std::exchange(self.prev_whells_position, meas_wheels_position),
                                   sampling_time);

        auto const left_wheel_speed =
            (dot_tilt - dot_tilt_K1 * meas_dot_tilt) + (tilt - tilt_K1 * meas_tilt) +
            (dot_rotation - dot_rotation_K1 * meas_dot_rotation) +
            (wheels_position - wheels_position_K1 * meas_wheels_position) +
            (whells_speed - whells_speed_K1 * meas_whells_speed);

        auto const right_wheel_speed =
            (dot_tilt - dot_tilt_K2 * meas_dot_tilt) + (tilt - tilt_K2 * meas_tilt) +
            (dot_rotation - dot_rotation_K2 * meas_dot_rotation) +
            (wheels_position - wheels_position_K2 * meas_wheels_position) +
            (whells_speed - whells_speed_K2 * meas_whells_speed);

        self.set_wheel_speed(WheelType::LEFT, left_wheel_speed, sampling_time);
        self.set_wheel_speed(WheelType::RIGHT, right_wheel_speed, sampling_time);
    }

    std::float32_t Segway::get_tilt(this Segway& self) noexcept
    {
        auto const measured_tilt =
            std::visit([](auto& imu) { return imu.get_roll().value_or(0.0F); }, self.sensor);
        std::printf("Tilt: %f\n\r", measured_tilt);

        return measured_tilt;
    }

    std::float32_t Segway::get_rotation(this Segway& self) noexcept
    {
        auto const measured_rotation =
            std::visit([](auto& imu) { return imu.get_yaw().value_or(0.0F); }, self.sensor);
        std::printf("Rotation: %f\n\r", measured_rotation);

        return measured_rotation;
    }

    std::float32_t Segway::get_wheel_position(this Segway& self,
                                              WheelType const wheel_type,
                                              std::float32_t const sampling_time) noexcept
    {
        auto& wheel_driver = self.get_wheel_driver(wheel_type);
        return wheel_driver.get_wheel_position(sampling_time);
    }

    std::float32_t Segway::get_wheels_position(this Segway& self,
                                               std::uint32_t const sampling_time) noexcept
    {
        auto const wheels_position = self.get_wheel_position(WheelType::LEFT, sampling_time) -
                                     self.get_wheel_position(WheelType::RIGHT, sampling_time);
        std::printf("Wheel diff position: %f\n\r", wheels_position);

        return wheels_position;
    }

    void Segway::set_wheel_speed(this Segway& self,
                                 WheelType const wheel_type,
                                 std::float32_t const wheel_speed,
                                 std::float32_t const sampling_time) noexcept
    {
        auto& wheel_driver = self.get_wheel_driver(wheel_type);
        std::printf("Setting %s speed to %f\n\r", wheel_type_to_string(wheel_type), wheel_speed);

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