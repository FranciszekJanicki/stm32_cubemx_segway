#ifndef ROTATION3D_HPP
#define ROTATION3D_HPP

#include "vecto r3d.hpp"
#include <array>
#include <cmath>
#include <compare>
#include <concepts>
#include <cstdlib>
#include <utility>

namespace Utility {

    template <std::floating_point T>
    struct Rotation3D {
        Rotation3D& operator+=(Rotation3D const& other) noexcept
        {
            this->x += other.x;
            this->y += other.y;
            this->z += other.z;
            return *this;
        }

        Rotation3D& operator-=(Rotation3D const& other) noexcept
        {
            this->x -= other.x;
            this->y -= other.y;
            this->z -= other.z;
            return *this;
        }

        Rotation3D& operator*=(Rotation3D const& other) noexcept
        {
            auto new_x = Vector3D<T>{this->x.x * other.x.x + this->x.y * other.y.x + this->x.z * other.z.x,
                                     this->x.x * other.x.y + this->x.y * other.y.y + this->x.z * other.z.y,
                                     this->x.x * other.x.z + this->x.y * other.y.z + this->x.z * other.z.z};

            auto new_y = Vector3D<T>{this->y.x * other.x.x + this->y.y * other.y.x + this->y.z * other.z.x,
                                     this->y.x * other.x.y + this->y.y * other.y.y + this->y.z * other.z.y,
                                     this->y.x * other.x.z + this->y.y * other.y.z + this->y.z * other.z.z};

            auto new_z = Vector3D<T>{this->z.x * other.x.x + this->z.y * other.y.x + this->z.z * other.z.x,
                                     this->z.x * other.x.y + this->z.y * other.y.y + this->z.z * other.z.y,
                                     this->z.x * other.x.z + this->z.y * other.y.z + this->z.z * other.z.z};

            this->x = new_x;
            this->y = new_y;
            this->z = new_z;

            return *this;
        }

        Rotation3D& operator*=(T const factor) noexcept
        {
            this->x *= factor;
            this->y *= factor;
            this->z *= factor;
            return *this;
        }

        Rotation3D& operator/=(T const factor) noexcept
        {
            this->x /= factor;
            this->y /= factor;
            this->z /= factor;
            return *this;
        }

        bool operator<=>(Rotation3D const& other) noexcept = default;

        Vector3D<T> x{};
        Vector3D<T> y{};
        Vector3D<T> z{};
    };

    template <std::floating_point T>
    Rotation3D<T> operator+(Rotation3D<T> const& left, Rotation3D<T> const& right) noexcept
    {
        return Rotation3D<T>{left.x + right.x, left.y + right.y, left.z + right.z};
    }

    template <std::floating_point T>
    Rotation3D<T> operator-(Rotation3D<T> const& left, Rotation3D<T> const& right) noexcept
    {
        return Rotation3D<T>{left.x - right.x, left.y - right.y, left.z - right.z};
    }

    template <std::floating_point T>
    Rotation3D<T> operator*(Rotation3D<T> const& left, Rotation3D<T> const& right) noexcept
    {
        auto new_x = Vector3D<T>{left.x.x * right.x.x + left.x.y * right.y.x + left.x.z * right.z.x,
                                 left.x.x * right.x.y + left.x.y * right.y.y + left.x.z * right.z.y,
                                 left.x.x * right.x.z + left.x.y * right.y.z + left.x.z * right.z.z};

        auto new_y = Vector3D<T>{left.y.x * right.x.x + left.y.y * right.y.x + left.y.z * right.z.x,
                                 left.y.x * right.x.y + left.y.y * right.y.y + left.y.z * right.z.y,
                                 left.y.x * right.x.z + left.y.y * right.y.z + left.y.z * right.z.z};

        auto new_z = Vector3D<T>{left.z.x * right.x.x + left.z.y * right.y.x + left.z.z * right.z.x,
                                 left.z.x * right.x.y + left.z.y * right.y.y + left.z.z * right.z.y,
                                 left.z.x * right.x.z + left.z.y * right.y.z + left.z.z * right.z.z};

        return Rotation3D<T>{new_x, new_y, new_z};
    }

    template <std::floating_point T>
    Rotation3D<T> operator*(T const factor, Rotation3D<T> const& matrix) noexcept
    {
        return Rotation3D<T>{matrix.x * factor, matrix.y * factor, matrix.z * factor};
    }

    template <std::floating_point T>
    Rotation3D<T> operator*(Rotation3D<T> const& matrix, T const factor) noexcept
    {
        return Rotation3D<T>{matrix.x * factor, matrix.y * factor, matrix.z * factor};
    }

    template <std::floating_point T>
    Rotation3D<T> operator/(Rotation3D<T> const& matrix, T const factor) noexcept
    {
        return Rotation3D<T>{matrix.x / factor, matrix.y / factor, matrix.z / factor};
    }

}; // namespace Utility

#endif // ROTATION3D_HPP