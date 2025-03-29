#ifndef STACK_MATRIX_HPP
#define STACK_MATRIX_HPP

#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <exception>
#include <print>
#include <stdexcept>
#include <utility>

namespace Utility {

    template <std::floating_point T, std::size_t N, std::size_t M>
    struct StackMatrix {
        using Data = std::array<std::array<T, M>, N>;
        using Init = std::initializer_list<std::initializer_list<T>>;
        using Row = std::array<T, M>;
        using Column = std::array<T, N>;

        static Data make_data(Init const init) noexcept
        {
            Data result;
            for (std::size_t i{}; i < N; ++i) {
                for (std::size_t j{}; j < M; ++j) {
                    result[i][j] = *((init.begin() + i)->begin() + j);
                }
            }
            return result;
        }

        Matrix() noexcept = default;

        Matrix(Init const init) noexcept : data{make_data(init)}
        {}

        Matrix(Matrix&& other) noexcept = default;
        Matrix(Matrix const& other) noexcept = default;

        Matrix& operator=(Matrix&& other) noexcept = default;
        Matrix& operator=(Matrix const& other) noexcept = default;

        ~Matrix() noexcept = default;

        Row const& operator[](std::size_t const row)
        {
            if (row > N) {
                throw std::runtime_error{"Out of bounds"};
            }
            return this->data[row];
        }

        Row& operator[](std::size_t const row)
        {
            if (row > N) {
                throw std::runtime_error{"Out of bounds"};
            }
            return this->data[row];
        }

        T const& operator[](std::size_t const row, std::size_t const col)
        {
            if (row > N || col > M) {
                throw std::runtime_error{"Out of bounds"};
            }
            return this->data[row][col];
        }

        T& operator[](std::size_t const row, std::size_t const col)
        {
            if (row > N || col > M) {
                throw std::runtime_error{"Out of bounds"};
            }
            return this->data[row][col];
        }

        bool operator<=>(Matrix const& other) noexcept = default;

        Matrix& operator=(Init const init)
        {
            this->data = make_data(init);
            return *this;
        }

        void print() noexcept
        {
            std::print("[");
            if (!this->data.empty()) {
                for (auto const& row : this->data) {
                    std::print("[");
                    if (!row.empty()) {
                        for (auto const col : row) {
                            std::print("{}", col);
                            if (col != row.back()) {
                                std::print(", ");
                            }
                        }
                    }
                    std::print("]");
                    if (row != this->data.back()) {
                        std::print(",\n");
                    }
                }
            }
            std::print("]\n");
        }

        bool is_square() noexcept
        {
            return N == M;
        }

        std::size_t rows() noexcept
        {
            return N;
        }

        std::size_t cols() noexcept
        {
            return M;
        }

        Matrix& operator+=(Matrix const& other) noexcept
        {
            *this = matrix_sum(*this, other);
            return *this;
        }

        Matrix& operator-=(Matrix const& other) noexcept
        {
            *this = matrix_difference(*this, other);
            return *this;
        }

        Matrix& operator*=(Matrix const& other)
        {
            *this = matrix_product(*this, other);
            return *this;
        }

        Matrix& operator*=(T const scale) noexcept
        {
            *this = matrix_scale(*this, scale);
            return *this;
        }

        Matrix& operator/=(T const scale)
        {
            if (scale == static_cast<T>(0)) {
                throw std::runtime_error{"Division by 0!"};
            }

            *this = matrix_scale(*this, 1 / scale);
            return *this;
        }

        Matrix& operator/=(Matrix const& other)
        {
            try {
                *this = matrix_product(*this, matrix_inverse(other));
                return *this;
            }
            catch (std::runtime_error const& error) {
                throw error;
            }
        }

        Matrix& operator^=(T const power)
        {
            *this = matrix_power(*this, power);
            return *this;
        }

        Data data{};
    };

    template <std::floating_point T, std::size_t N>
    Matrix<T, N, N> make_eye() noexcept
    {
        Matrix<T, N, N> result;
        for (std::size_t i{}; i < N; ++i) {
            for (std::size_t j{}; j < N; ++j) {
                result[i, j] = (i == j) ? T{1} : static_cast<T>(0);
            }
        }
        return result;
    }

    template <std::floating_point T, std::size_t N>
    Matrix<T, N - 1, N - 1> matrix_minor(Matrix<T, N, N> const& matrix, std::size_t const row, std::size_t const column)
    {
        if (row >= N || column >= N) {
            throw std::runtime_error{"Wrong dimensions"};
        }
        if constexpr (N == 1) {
            return matrix;
        }

        if constexpr (N > 1) {
            std::size_t cof_i{};
            std::size_t cof_j{};
            Matrix<T, N - 1, N - 1> result{};
            for (std::size_t i{}; i < N - 1; ++i) {
                for (std::size_t j{}; j < N - 1; ++j) {
                    if (i != row && j != column) {
                        result[cof_i, cof_j++] = matrix[i, j];
                        if (cof_j == N - 1) {
                            cof_j = 0;
                            ++cof_i;
                        }
                    }
                }
            }
            return result;
        }
    }

    template <std::floating_point T, std::size_t N>
    Matrix<T, N, N> matrix_complement(Matrix<T, N, N> const& matrix)
    {
        if constexpr (N == 1) {
            return matrix;
        }

        if constexpr (N > 1) {
            Matrix<T, N, N> result{};
            for (std::size_t i{}; i < N; ++i) {
                for (std::size_t j{}; j < N; ++j) {
                    try {
                        result[i, j] = ((i + j) % 2 == 0 ? 1 : -1) * matrix_det(matrix_minor(matrix, i, j));
                    }
                    catch (std::runtime_error const& error) {
                        throw error;
                    }
                }
            }
            return result;
        }
    }

    template <std::floating_point T, std::size_t N>
    Matrix<T, N, N> matrix_adjoint(Matrix<T, N, N> const& matrix)
    {
        try {
            return matrix_transpose(matrix_complement(matrix));
        }
        catch (std::runtime_error const& error) {
            throw error;
        }
    }

    template <std::floating_point T, std::size_t N, std::size_t M>
    Matrix<T, M, N> matrix_transpose(Matrix<T, N, M> const& matrix) noexcept
    {
        Matrix<T, M, N> result{};
        for (std::size_t i{}; i < N; ++i) {
            for (std::size_t j{}; j < M; ++j) {
                result[i, j] = matrix[j, i];
            }
        }
        return result;
    }

    template <std::floating_point T, std::size_t N>
    T matrix_det(Matrix<T, N, N> const& matrix)
    {
        if constexpr (N == 1) {
            return matrix[0, 0];
        }

        if constexpr (N == 2) {
            return (matrix[0, 0] * matrix[1, 1]) - (matrix[1, 0] * matrix[0, 1]);
        }

        if constexpr (N > 2) {
            try {
                T det{};
                for (std::size_t i{}; i < N; ++i) {
                    det += (i % 2 == 0 ? 1 : -1) * matrix[0, i] * matrix_det(matrix_minor(matrix, 0, i));
                }
                return det;
            }
            catch (std::runtime_error const& error) {
                throw error;
            }
        }
    }

    template <std::floating_point T, std::size_t N>
    Matrix<T, N, N> matrix_inverse(Matrix<T, N, N> const& matrix)
    {
        try {
            return matrix_scale(matrix_adjoint(matrix), 1 / matrix_det(matrix));
        }
        catch (std::runtime_error const& error) {
            throw error;
        }
    }

    template <std::floating_point T, std::size_t N>
    Matrix<T, N, N> matrix_upper_triangular(Matrix<T, N, N> const& matrix)
    {
        try {
            return matrix_transpose(matrix_lower_triangular(matrix));
        }
        catch (std::runtime_error const& error) {
            throw error;
        }
    }

    template <std::floating_point T, std::size_t N>
    Matrix<T, N, N> matrix_lower_triangular(Matrix<T, N, N> const& matrix)
    {
        if constexpr (N == 1) {
            return matrix;
        }

        if constexpr (N > 1) {
            Matrix<T, N, N> result;
            for (std::size_t i{}; i < N; ++i) {
                for (std::size_t j{}; j <= i; ++j) {
                    T sum{};
                    for (std::size_t k{}; k < j; ++k) {
                        if (j == i) {
                            sum += std::pow(result[j, k], 2);
                            result[j, j] = std::sqrt(matrix[j, j] - sum);
                        } else {
                            sum += (result[i, k] * result[j, k]);
                            result[i, j] = (matrix[i, j] - sum) / result[j, j];
                        }
                    }
                }
            }
            return result;
        }
    }

    template <std::floating_point T, std::size_t N, std::size_t M>
    Matrix<T, N, M> matrix_sum(Matrix<T, N, M> const& left, Matrix<T, N, M> const& right) noexcept

    {
        Matrix<T, N, M> result;
        for (std::size_t i{}; i < N; ++i) {
            for (std::size_t j{}; j < M; ++j) {
                result[i, j] = left[i, j] + right[i, j];
            }
        }
        return result;
    }

    template <std::floating_point T, std::size_t N, std::size_t M>
    Matrix<T, N, M> matrix_difference(Matrix<T, N, M> const& left, Matrix<T, N, M> const& right) noexcept

    {
        Matrix<T, N, M> result;
        for (std::size_t i{}; i < N; ++i) {
            for (std::size_t j{}; j < M; ++j) {
                result[i, j] = left[i, j] - right[i, j];
            }
        }
        return result;
    }

    template <std::floating_point T, std::size_t N, std::size_t M>
    Matrix<T, N, M> matrix_scale(Matrix<T, N, M> const& matrix, T const scale) noexcept
    {
        Matrix<T, N, M> result;
        for (std::size_t i{}; i < N; ++i) {
            for (std::size_t j{}; j < M; ++j) {
                result[i, j] = matrix[i, j] * scale;
            }
        }
        return result;
    }

    template <std::floating_point T, std::size_t N, std::size_t M, std::size_t P>
    Matrix<T, N, P> matrix_product(Matrix<T, N, M> const& left, Matrix<T, M, P> const& right) noexcept

    {
        Matrix<T, N, P> result;
        for (std::size_t i{}; i < N; ++i) {
            for (std::size_t j{}; j < P; ++j) {
                T sum{};
                for (std::size_t k{}; k < M; ++k) {
                    sum += left[i, k] * right[k, j];
                }
                result[i, j] = sum;
            }
        }
        return result;
    }

    template <std::floating_point T, std::size_t N>
    Matrix<T, N, N> matrix_power(Matrix<T, N, N> const& matrix, T const power) noexcept
    {
        if (power == 1) {
            return matrix;
        }

        Matrix<T, N, N> result{matrix};
        for (std::size_t i{}; i < power - 1; ++i) {
            result = matrix_product(result, matrix);
        }
        return result;
    }

    template <std::floating_point T, std::size_t N, std::size_t M>
    std::size_t matrix_rank(Matrix<T, N, M> const& matrix) noexcept
    {
        if constexpr (N == M) {
            if (auto det{matrix_det(matrix)}; det <= std::numeric_limits<T>::epsilon()) {
                return N - 1UL;
            }
            return N;
        }
        if constexpr (N > M) {
            return M;
        }
        if constexpr (N < M) {
            return N;
        }
    }

    template <std::floating_point T, std::size_t N>
    std::array<T, N> matrix_eigvals(Matrix<T, N, N> const& matrix) noexcept
    {}

    template <std::floating_point T, std::size_t N, std::size_t M>
    Matrix<T, N, M> operator+(Matrix<T, N, M> const& left, Matrix<T, N, M> const& right) noexcept

    {
        return matrix_sum(left, right);
    }

    template <std::floating_point T, std::size_t N, std::size_t M>
    Matrix<T, N, M> operator-(Matrix<T, N, M> const& left, Matrix<T, N, M> const& right) noexcept

    {
        return matrix_difference(left, right);
    }

    template <std::floating_point T, std::size_t N, std::size_t M, std::size_t P>
    Matrix<T, N, P> operator*(Matrix<T, N, M> const& left, Matrix<T, M, P> const& right) noexcept

    {
        return matrix_product(left, right);
    }

    template <std::floating_point T, std::size_t N, std::size_t M>
    Matrix<T, N, M> operator*(T const scale, Matrix<T, N, M> const& matrix) noexcept
    {
        return matrix_scale(matrix, scale);
    }

    template <std::floating_point T, std::size_t N, std::size_t M>
    Matrix<T, N, M> operator*(Matrix<T, N, M> const& matrix, T const scale) noexcept
    {
        return matrix_scale(matrix, scale);
    }

    template <std::floating_point T, std::size_t N, std::size_t M>
    Matrix<T, N, M> operator/(Matrix<T, N, M> const& matrix, T const scale)
    {
        if (scale == static_cast<T>(0)) {
            throw std::runtime_error{"Division by 0!"};
        }
        return matrix_scale(matrix, 1 / scale);
    }

    template <std::floating_point T, std::size_t N>
    Matrix<T, N, N> operator/(Matrix<T, N, N> const& left, Matrix<T, N, N> const& right)
    {
        try {
            return matrix_product(left, matrix_inverse(right));
        }
        catch (std::runtime_error const& error) {
            throw error;
        }
    }

    template <std::floating_point T, std::size_t N>
    Matrix<T, N, N> operator^(Matrix<T, N, N> const& matrix, T const power)
    {
        return matrix_power(matrix, power);
    }

}; // namespace Utility

#endif // STACK_MATRIX_HPP