#ifndef UNIT_TEST_HPP
#define UNIT_TEST_HPP

#include <cstdint>

namespace Segway {

    enum struct TestType : std::uint8_t {
        ICM20948,
        MPU6050,
        A4988_1,
        A4988_2,
        SEGWAY,
    };

    void test(TestType const test_type) noexcept;
    void test_icm20948() noexcept;
    void test_mpu6050() noexcept;
    void test_a4988_1() noexcept;
    void test_a4988_2() noexcept;
    void test_segway() noexcept;

}; // namespace Segway

#endif // UNIT_TEST_HPP