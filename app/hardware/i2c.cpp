#include "i2c.hpp"

namespace Hardware {

    void initialize_i2c1() noexcept
    {
        MX_I2C1_Init();
    }

    void deinitialize_i2c1() noexcept
    {}

}; // namespace Hardware