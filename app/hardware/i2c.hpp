#ifndef I2C_HPP
#define I2C_HPP

#include "i2c.h"

namespace Hardware {

    void initialize_i2c1() noexcept;

    void deinitialize_i2c1() noexcept;

}; // namespace Hardware

#endif // I2C_HPP