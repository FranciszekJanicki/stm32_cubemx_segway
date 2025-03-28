#include "main.h"
#include "../segway/unit_test.hpp"

int main()
{
    HAL_Init();
    SystemClock_Config();

    Segway::test_step_driver_1();
}
