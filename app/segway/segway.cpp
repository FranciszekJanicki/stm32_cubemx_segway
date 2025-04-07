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
                            std::float32_t const position,
                            std::float32_t const whells_speed,
                            std::float32_t const sampling_time) noexcept
    {
        self.run_segway(dot_tilt, tilt, dot_rotation, rotation, position, whells_speed, sampling_time);
    }

    void Segway::run_segway(this Segway& self,
                            std::float32_t const dot_tilt,
                            std::float32_t const tilt,
                            std::float32_t const dot_rotation,
                            std::float32_t const rotation,
                            std::float32_t const position,
                            std::float32_t const whells_speed,
                            std::float32_t const sampling_time) noexcept
    {
        static auto prev_tilt = 0.0F32;
        static auto prev_rotation = 0.0F32;
        static auto prev_position = 0.0F32;

        static auto int_e_dot_tilt = 0.0F32;
        static auto int_e_tilt = 0.0F32;
        static auto int_e_dot_rotation = 0.0F32;
        static auto int_e_rotation = 0.0F32;
        static auto int_e_position = 0.0F32;
        static auto int_e_whells_speed = 0.0F32;

        static auto prev_e_dot_tilt = 0.0F32;
        static auto prev_e_tilt = 0.0F32;
        static auto prev_e_dot_rotation = 0.0F32;
        static auto prev_e_rotation = 0.0F32;
        static auto prev_e_position = 0.0F32;
        static auto prev_e_whells_speed = 0.0F32;

        static auto const Ki = std::array<std::float32_t, 6UL>{};
        static auto const Kx =
            std::array{std::array<std::float32_t, 6UL>{0.0F32, 0.0F32, 0.0F32, 0.0F32, 0.0F32, 0.0F32},
                       std::array<std::float32_t, 6UL>{0.0F32, 0.0F32, 0.0F32, 0.0F32, 0.0F32, 0.0F32}};

        auto const meas_tilt = self.get_tilt();
        auto const meas_dot_tilt = Utility::differentiate(meas_tilt, prev_tilt, sampling_time);
        auto const meas_rotation = self.get_rotation();
        auto const meas_dot_rotation = Utility::differentiate(meas_rotation, prev_rotation, sampling_time);
        auto const meas_position = self.get_position(sampling_time);
        auto const meas_whells_speed = Utility::differentiate(meas_position, prev_position, sampling_time);

        prev_tilt = meas_tilt;
        prev_rotation = rotation;
        prev_position = meas_position;

        auto const e_dot_tilt = dot_tilt - meas_dot_tilt;
        auto const e_tilt = tilt - meas_tilt;
        auto const e_dot_rotation = dot_rotation - meas_dot_rotation;
        auto const e_rotation = rotation - meas_rotation;
        auto const e_position = position - meas_position;
        auto const e_whells_speed = whells_speed - meas_whells_speed;

        int_e_dot_tilt += Utility::integrate(e_dot_tilt, prev_e_dot_tilt, sampling_time);
        int_e_tilt += Utility::integrate(e_tilt, prev_e_tilt, sampling_time);
        int_e_dot_rotation += Utility::integrate(e_dot_tilt, prev_e_dot_tilt, sampling_time);
        int_e_rotation += Utility::integrate(e_dot_tilt, prev_e_dot_tilt, sampling_time);
        int_e_position += Utility::integrate(e_dot_tilt, prev_e_dot_tilt, sampling_time);
        int_e_whells_speed += Utility::integrate(e_dot_tilt, prev_e_dot_tilt, sampling_time);

        prev_e_dot_tilt = e_dot_tilt;
        prev_e_tilt = e_tilt;
        prev_e_dot_rotation = e_dot_rotation;
        prev_e_rotation = e_rotation;
        prev_e_position = e_position;
        prev_e_whells_speed = e_whells_speed;

        auto const left_wheel_speed =
            (Ki[0] * int_e_dot_tilt - Kx[0][0] * meas_dot_tilt) + (Ki[1] * int_e_tilt - Kx[0][1] * meas_tilt) +
            (Ki[2] * int_e_dot_rotation - Kx[0][2] * meas_dot_rotation) +
            (Ki[3] * int_e_rotation - Kx[0][3] * meas_rotation) + (Ki[4] * int_e_position - Kx[0][4] * meas_position) +
            (Ki[5] * int_e_whells_speed - Kx[0][5] * meas_whells_speed);

        auto const right_wheel_speed = (dot_tilt - Kx[1][0] * meas_dot_tilt) + (tilt - Kx[1][1] * meas_tilt) +
                                       (dot_rotation - Kx[1][2] * meas_dot_rotation) +
                                       (rotation - Kx[1][3] * meas_rotation) + (position - Kx[1][4] * meas_position) +
                                       (whells_speed - Kx[1][5] * meas_whells_speed);

        self.set_wheel_speed(WheelType::LEFT, left_wheel_speed, sampling_time);
        self.set_wheel_speed(WheelType::RIGHT, right_wheel_speed, sampling_time);
    }

    std::float32_t Segway::get_tilt(this Segway& self) noexcept
    {
        auto const measured_tilt = std::visit([](auto& imu) { return imu.get_roll().value_or(0.0F32); }, self.sensor);
        std::printf("Tilt: %f\n\r", measured_tilt);

        return measured_tilt;
    }

    std::float32_t Segway::get_rotation(this Segway& self) noexcept
    {
        auto const measured_rotation =
            std::visit([](auto& imu) { return imu.get_yaw().value_or(0.0F32); }, self.sensor);
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

    std::float32_t Segway::get_position(this Segway& self, std::uint32_t const sampling_time) noexcept
    {
        auto const position = self.get_wheel_position(WheelType::LEFT, sampling_time) -
                              self.get_wheel_position(WheelType::RIGHT, sampling_time);
        std::printf("Wheel diff position: %f\n\r", position);

        return position;
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
        auto* wheel =
            std::ranges::find_if(self.wheels, [wheel_type](Wheel const& wheel) { return wheel.type == wheel_type; });

        return wheel->driver;
    }

}; // namespace Segway