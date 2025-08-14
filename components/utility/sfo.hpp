#ifndef UTILITY_SFO_HPP
#define UTILITY_SFO_HPP

#include "matrix.hpp"

namespace utility {

    template <std::floating_point T, std::size_t nX, std::size_t nY = 1UL, std::size_t nU = nY>
    struct SFO {
    public:
        template <std::size_t N, std::size_t M>
        using Mtx = Matrix<T, N, M>;

        Mtx<nX, 1UL> get_estimated_state(this SFO& self,
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

        Mtx<nX, 1UL> operator()(this SFO& self, Mtx<nU, 1UL> const& u, Mtx<nY, 1UL> const& y)
        {
            try {
                return self.get_estimated_state(u, y);
            } catch (std::runtime_error const& error) {
                throw error;
            }
        }

        void predict(this SFO& self, Mtx<nU, 1UL> const& u, Mtx<nY, 1UL> const& y)
        {
            try {
                auto x_hat = self.A * self.x + self.B * u;
                self.x = x_hat;
            } catch (std::runtime_error const& error) {
                throw error;
            }
        }

        void update(this SFO& self, Mtx<nU, 1UL> const& u, Mtx<nY, 1UL> const& y)
        {
            try {
                auto y_hat = self.C * self.x + self.D * u;
                auto x = self.x + self.K * (y - y_hat);
                self.x = x;
            } catch (std::runtime_error const& error) {
                throw error;
            }
        }

        /* state */
        Mtx<nX, 1UL> x = {};

        /* state model */
        Mtx<nX, nX> A = {};
        Mtx<nX, nU> B = {};
        Mtx<nY, nX> C = {};
        Mtx<1UL, nU> D = {};

        /* state gain */
        Mtx<nU, nX> K = {};
    };

}; // namespace utility

#endif // UTILITY_SFO_HPP