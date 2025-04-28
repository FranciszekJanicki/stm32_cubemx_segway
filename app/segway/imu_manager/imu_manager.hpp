#ifndef SEGWAY_IMU_MANAGER_HPP
#define SEGWAY_IMU_MANAGER_HPP

namespace segway {

    void imu_manager_init() noexcept;
    void imu_manager_process() noexcept;
    
}; // namespace segway

#endif // SEGWAY_IMU_MANAGER_HPP