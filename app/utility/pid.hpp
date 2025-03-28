#ifndef PID_HPP
#define PID_HPP

#include "utility.hpp"
#include <concepts>
#include <utility>

namespace Utility {

    template <typename T>
    struct PID {
        T operator()(T const error, T const sampling_time) noexcept
        {
            return this->get_sat_control(error, sampling_time);
        }

        T get_sat_control(T const error, T const sampling_time) noexcept
        {
            auto const control = this->get_control(error, sampling_time);
            auto const sat_control = std::clamp(control, -this->saturation, this->saturation);

            this->prev_sat_error = std::exchange(this->sat_error, control - sat_control);
            this->prev_error = error;

            return sat_control;
        }

        T get_control(T const error, T const sampling_time) noexcept
        {
            return this->get_proportion(error) + this->get_integral(error, sampling_time) +
                   this->get_derivative(error, sampling_time);
        }

        T get_proportion(T const error) noexcept
        {
            return this->proportion_gain * error;
        }

        T get_derivative(T const error, T const sampling_time) noexcept
        {
            this->error_derivative = Utility::differentiate(error,
                                                            this->prev_error,
                                                            sampling_time,
                                                            this->error_derivative,
                                                            this->time_constant);

            return this->derivative_gain * this->error_derivative;
        }

        T get_integral(T const error, T const sampling_time) noexcept
        {
            this->error_integral += Utility::integrate(error, this->prev_error, sampling_time);
            this->sat_error_integral += Utility::integrate(this->sat_error, this->prev_sat_error, sampling_time);

            return this->integral_gain * this->error_integral - this->control_gain * this->sat_error_integral;
        }

        // basic PID parameters
        T proportion_gain{};
        T integral_gain{};
        T derivative_gain{};
        T time_constant{};

        // anti windup parameters
        T control_gain{};
        T saturation{};

        // basic PID internal state
        T prev_error{};
        T error_integral{};
        T error_derivative{};

        // anti windup internal state
        T sat_error{};
        T prev_sat_error{};
        T sat_error_integral{};
    };

}; // namespace Utility

#endif // PID_HPP
