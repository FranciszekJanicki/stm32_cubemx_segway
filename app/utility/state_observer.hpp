#ifndef STATE_OBSERVER_HPP
#define STATE_OBSERVER_HPP

#include "stack_matrix.hpp"

namespace Utility {

    template <std::floating_point T, std::size_t STATES, std::size_t CONTROLS = 1UL, std::size_t MEASUREMENTS = 1UL>
    struct StateObserver {
    public:
        template <std::size_t N, std::size_t M>
        using Matrix = StackMatrix<T, N, M>;

        Matrix<STATES, 1UL> operator()(Matrix<1UL, CONTROLS> const& control,
                                       Matrix<1UL, MEASUREMENTS> const& measurement)
        {
            try {
                this->predict(control);
                this->correct(measurement);
            }
            catch (std::runtime_error const& error) {
                throw error;
            }
            return this->state;
        }

        void predict(Matrix<1UL, CONTROLS> const& control)
        {
            try {
                this->state = this->state_transition * this->state + this->control_transition * control;
            }
            catch (std::runtime_error const& error) {
                throw error;
            }
        }

        void predict(Matrix<1UL, MEASUREMENTS> const& measurement)
        {
            try {
                this->state =
                    this->state + this->state_gain * (measurement - this->measurement_transition * this->state);
            }
            catch (std::runtime_error const& error) {
                throw error;
            }
        }

        Matrix<STATES, 1UL> state{};
        Matrix<STATES, 1UL> state_gain{};
        Matrix<STATES, STATES> state_transition{};
        Matrix<STATES, CONTROLS> control_transition{};
        Matrix<MEASUREMENTS, 1UL> measurement_transition{};
    };
}; // namespace Utility

#endif // STATE_OBSERVER_HPP