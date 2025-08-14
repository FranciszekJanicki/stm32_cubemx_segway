#ifndef UTILITY_KALMAN_HPP
#define UTILITY_KALMAN_HPP

#include "matrix.hpp"

namespace utility {

    template <std::floating_point T, std::size_t nX, std::size_t nY = 1UL, std::size_t nU = nY>
    struct Kalman {
        template <std::size_t N, std::size_t M>
        using Mtx = Matrix<T, N, M>;

        [[nodiscard]] Mtx<nX, 1UL> operator()(this Kalman& self,
                                              Mtx<nU, 1UL> const& u,
                                              Mtx<nY, 1UL> const& y)
        {
            try {
                self.predict(u, y);
                self.correct(u, y);
                return self.x;
            } catch (std::runtime_error const& error) {
                throw error;
            }
        }

        void predict(this Kalman& self, Mtx<nU, 1UL> const& u, Mtx<nY, 1UL> const& y)
        {
            try {
                self.x = (self.A * self.x) + (self.B * u);
                self.x_covar = (self.A * self.x_covar * matrix_transpose(self.A)) + self.x_noise;
            } catch (std::runtime_error const& error) {
                throw error;
            }
        }

        void correct(this Kalman& self, Mtx<nU, 1UL> const& u, Mtx<nY, 1UL> const& y)
        {
            try {
                auto innovation = y - (self.C * self.x + self.D * u);
                auto res_covar = (self.C * self.x_covar * matrix_transpose(self.C)) + self.y_noise;
                auto K = self.x_covar * matrix_transpose(self.C) * matrix_inverse(res_covar);
                self.x = self.x + (K * innovation);
                self.x_covar = (make_eye<T, nX>() - K * self.C) * self.x_covar;
            } catch (std::runtime_error const& error) {
                throw error;
            }
        }

        /* state */
        Mtx<nX, 1UL> x = {};
        Mtx<nX, nX> x_covar = {};

        /* state model */
        Mtx<nX, nX> A = {};
        Mtx<nX, nU> B = {};
        Mtx<nY, nX> C = {};
        Mtx<1UL, nU> D = {};

        /* state noise */
        Mtx<nY, nY> y_noise = {};
        Mtx<nX, nX> x_noise = {};
    };

}; // namespace utility

#endif // UTILITY_KALMAN_HPP