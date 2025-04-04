#include "icm20948.hpp"

namespace ICM20948 {

    ICM_20948_Status_e write_bytes(std::uint8_t const reg_address,
                                   std::uint8_t* const bytes,
                                   std::uint32_t const size,
                                   void* const user) noexcept
    {
        auto i2c_device = static_cast<I2CDevice const*>(user);
        i2c_device->write_bytes(reg_address, bytes, size);

        return ICM_20948_Stat_Ok;
    }

    ICM_20948_Status_e read_bytes(std::uint8_t const reg_address,
                                  std::uint8_t* const bytes,
                                  std::uint32_t const size,
                                  void* const user) noexcept
    {
        auto i2c_device = static_cast<I2CDevice const*>(user);
        i2c_device->read_bytes(reg_address, bytes, size);

        return ICM_20948_Stat_Ok;
    }

    ICM20948::ICM20948(I2CDevice&& i2c_device) noexcept :
        i2c_device_{std::forward<I2CDevice>(i2c_device)},
        icm_20948_serif_{.write = &write_bytes, .read = &read_bytes, .user = &i2c_device_},
        icm_20948_device_{._serif = &icm_20948_serif_,
                          ._dmp_firmware_available = true,
                          ._firmware_loaded = false,
                          ._last_bank = 255,
                          ._last_mems_bank = 255}
    {
        this->initialize();
    }

    ICM20948::~ICM20948() noexcept
    {
        this->deinitialize();
    }

    void ICM20948::initialize() noexcept
    {
        ICM_20948_sw_reset(&this->icm_20948_device_);
        HAL_Delay(500);

        ICM_20948_sleep(&this->icm_20948_device_, false);
        ICM_20948_low_power(&this->icm_20948_device_, false);

        HAL_Delay(500);

        ICM_20948_i2c_master_passthrough(&this->icm_20948_device_, false);
        ICM_20948_i2c_master_enable(&this->icm_20948_device_, true);

        uint8_t tmp = 1;
        ICM_20948_i2c_master_single_w(&this->icm_20948_device_, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL3, &tmp);

        for (uint8_t tries = 0; tries < 100; ++tries) {
            uint8_t test[2] = {0};
            ICM_20948_i2c_master_single_r(&this->icm_20948_device_, MAG_AK09916_I2C_ADDR, AK09916_REG_WIA1, &test[1]);
            ICM_20948_i2c_master_single_r(&this->icm_20948_device_, MAG_AK09916_I2C_ADDR, AK09916_REG_WIA2, &test[0]);

            if (MAG_AK09916_WHO_AM_I == *(uint16_t*)test)
                break;

            ICM_20948_i2c_master_reset(&this->icm_20948_device_);
            HAL_Delay(5);
        }

        HAL_Delay(100);
        this->initialize_dmp();

        // Output rate = (DMP running rate / ODR ) - 1
        inv_icm20948_enable_dmp_sensor(&this->icm_20948_device_, INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR, 1);
        inv_icm20948_set_dmp_sensor_period(&this->icm_20948_device_, DMP_ODR_Reg_Quat6, 0);

        inv_icm20948_enable_dmp_sensor(&this->icm_20948_device_, INV_ICM20948_SENSOR_RAW_ACCELEROMETER, 1);
        inv_icm20948_set_dmp_sensor_period(&this->icm_20948_device_, DMP_ODR_Reg_Accel, 0);

        inv_icm20948_enable_dmp_sensor(&this->icm_20948_device_, INV_ICM20948_SENSOR_RAW_GYROSCOPE, 1);
        inv_icm20948_set_dmp_sensor_period(&this->icm_20948_device_, DMP_ODR_Reg_Gyro, 0);
        inv_icm20948_set_dmp_sensor_period(&this->icm_20948_device_, DMP_ODR_Reg_Gyro_Calibr, 0);

        ICM_20948_enable_FIFO(&this->icm_20948_device_, true);
        ICM_20948_enable_DMP(&this->icm_20948_device_, true);

        ICM_20948_reset_FIFO(&this->icm_20948_device_);
        ICM_20948_reset_DMP(&this->icm_20948_device_);

        this->initialized_ = true;
    }

    void ICM20948::initialize_dmp() noexcept
    {
        ICM_20948_i2c_controller_configure_peripheral(&this->icm_20948_device_,
                                                      0,
                                                      MAG_AK09916_I2C_ADDR,
                                                      AK09916_REG_RSV2,
                                                      10,
                                                      true,
                                                      true,
                                                      false,
                                                      true,
                                                      true,
                                                      0);
        ICM_20948_i2c_controller_configure_peripheral(&this->icm_20948_device_,
                                                      1,
                                                      MAG_AK09916_I2C_ADDR,
                                                      AK09916_REG_CNTL2,
                                                      1,
                                                      false,
                                                      true,
                                                      false,
                                                      false,
                                                      false,
                                                      AK09916_mode_single);

        uint8_t tmp;

        tmp = 0x04; // MAG ODR = 1k1 / 2^tmp
        ICM_20948_set_bank(&this->icm_20948_device_, 3);
        ICM_20948_execute_w(&this->icm_20948_device_, AGB3_REG_I2C_MST_ODR_CONFIG, &tmp, 1);

        ICM_20948_set_clock_source(&this->icm_20948_device_, ICM_20948_Clock_Auto);

        tmp = 0x40;
        ICM_20948_set_bank(&this->icm_20948_device_, 0);
        ICM_20948_execute_w(&this->icm_20948_device_, AGB0_REG_PWR_MGMT_2, &tmp, 1);

        ICM_20948_set_sample_mode(&this->icm_20948_device_, ICM_20948_Internal_Mst, ICM_20948_Sample_Mode_Cycled);
        HAL_Delay(15);

        ICM_20948_enable_FIFO(&this->icm_20948_device_, false);
        ICM_20948_enable_DMP(&this->icm_20948_device_, false);

        const ICM_20948_fss_t fss_cfg = {
            .a = gpm4,
            .g = dps2000,
        };

        ICM_20948_set_full_scale(
            &this->icm_20948_device_,
            static_cast<ICM_20948_InternalSensorID_bm>(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
            fss_cfg);

        ICM_20948_enable_dlpf(&this->icm_20948_device_, ICM_20948_Internal_Gyr, true);

        tmp = 0;

        ICM_20948_set_bank(&this->icm_20948_device_, 0);
        ICM_20948_execute_w(&this->icm_20948_device_, AGB0_REG_FIFO_EN_1, &tmp, 1);
        ICM_20948_set_bank(&this->icm_20948_device_, 0);
        ICM_20948_execute_w(&this->icm_20948_device_, AGB0_REG_FIFO_EN_2, &tmp, 1);

        ICM_20948_INT_enable_t en;
        ICM_20948_int_enable(&this->icm_20948_device_, NULL, &en);
        en.RAW_DATA_0_RDY_EN = 0;
        ICM_20948_int_enable(&this->icm_20948_device_, &en, NULL);

        ICM_20948_reset_FIFO(&this->icm_20948_device_);
        // GYRO ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]).
        // ACCEL ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]).
        const ICM_20948_smplrt_t smp_cfg = {
            .a = 4, // 225HZ
            .g = 4  // 225HZ
        };

        ICM_20948_set_sample_rate(
            &this->icm_20948_device_,
            static_cast<ICM_20948_InternalSensorID_bm>(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
            smp_cfg);

        ICM_20948_set_dmp_start_address(&this->icm_20948_device_, DMP_START_ADDRESS);

        ICM_20948_firmware_load(&this->icm_20948_device_);

        ICM_20948_set_dmp_start_address(&this->icm_20948_device_, DMP_START_ADDRESS);

        tmp = 0x48;

        ICM_20948_set_bank(&this->icm_20948_device_, 0);
        ICM_20948_execute_w(&this->icm_20948_device_, AGB0_REG_HW_FIX_DISABLE, &tmp, 1);

        ICM_20948_set_bank(&this->icm_20948_device_, 0);
        tmp = 0xE4;
        ICM_20948_execute_w(&this->icm_20948_device_, AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &tmp, 1);

        const uint8_t acc_scale1[] = {0x04, 0x00, 0x00, 0x00};
        const uint8_t acc_scale2[] = {0x00, 0x04, 0x00, 0x00};

        inv_icm20948_write_mems(&this->icm_20948_device_, ACC_SCALE, 4, acc_scale1);
        inv_icm20948_write_mems(&this->icm_20948_device_, ACC_SCALE2, 4, acc_scale2);

        const uint8_t mount_multiplier_zero[] = {0x00, 0x00, 0x00, 0x00};
        const uint8_t mount_multiplier_plus[] = {0x09, 0x99, 0x99, 0x99};
        const uint8_t mount_multiplier_minus[] = {0xF6, 0x66, 0x66, 0x67};

        inv_icm20948_write_mems(&this->icm_20948_device_, CPASS_MTX_00, 4, mount_multiplier_plus);
        inv_icm20948_write_mems(&this->icm_20948_device_, CPASS_MTX_01, 4, mount_multiplier_zero);
        inv_icm20948_write_mems(&this->icm_20948_device_, CPASS_MTX_02, 4, mount_multiplier_zero);
        inv_icm20948_write_mems(&this->icm_20948_device_, CPASS_MTX_10, 4, mount_multiplier_zero);
        inv_icm20948_write_mems(&this->icm_20948_device_, CPASS_MTX_11, 4, mount_multiplier_minus);
        inv_icm20948_write_mems(&this->icm_20948_device_, CPASS_MTX_12, 4, mount_multiplier_zero);
        inv_icm20948_write_mems(&this->icm_20948_device_, CPASS_MTX_20, 4, mount_multiplier_zero);
        inv_icm20948_write_mems(&this->icm_20948_device_, CPASS_MTX_21, 4, mount_multiplier_zero);
        inv_icm20948_write_mems(&this->icm_20948_device_, CPASS_MTX_22, 4, mount_multiplier_minus);

        const uint8_t b2s_mount_multiplier_zero[] = {0x00, 0x00, 0x00, 0x00};
        const uint8_t b2s_mount_multiplier_plus[] = {0x40, 0x00, 0x00, 0x00};

        inv_icm20948_write_mems(&this->icm_20948_device_, B2S_MTX_00, 4, b2s_mount_multiplier_plus);
        inv_icm20948_write_mems(&this->icm_20948_device_, B2S_MTX_01, 4, b2s_mount_multiplier_zero);
        inv_icm20948_write_mems(&this->icm_20948_device_, B2S_MTX_02, 4, b2s_mount_multiplier_zero);
        inv_icm20948_write_mems(&this->icm_20948_device_, B2S_MTX_10, 4, b2s_mount_multiplier_zero);
        inv_icm20948_write_mems(&this->icm_20948_device_, B2S_MTX_11, 4, b2s_mount_multiplier_plus);
        inv_icm20948_write_mems(&this->icm_20948_device_, B2S_MTX_12, 4, b2s_mount_multiplier_zero);
        inv_icm20948_write_mems(&this->icm_20948_device_, B2S_MTX_20, 4, b2s_mount_multiplier_zero);
        inv_icm20948_write_mems(&this->icm_20948_device_, B2S_MTX_21, 4, b2s_mount_multiplier_zero);
        inv_icm20948_write_mems(&this->icm_20948_device_, B2S_MTX_22, 4, b2s_mount_multiplier_plus);

        inv_icm20948_set_gyro_sf(&this->icm_20948_device_, 4, 3);

        const uint8_t gyro_full_scale[] = {0x10, 0x00, 0x00, 0x00};
        inv_icm20948_write_mems(&this->icm_20948_device_, GYRO_FULLSCALE, 4, gyro_full_scale);

        const uint8_t accel_only_gain[] = {0x00, 0xE8, 0xBA, 0x2E};
        inv_icm20948_write_mems(&this->icm_20948_device_, ACCEL_ONLY_GAIN, 4, accel_only_gain);

        const uint8_t accel_alpha_var[] = {0x3D, 0x27, 0xD2, 0x7D};
        inv_icm20948_write_mems(&this->icm_20948_device_, ACCEL_ALPHA_VAR, 4, accel_alpha_var);

        const uint8_t accel_a_var[] = {0x02, 0xD8, 0x2D, 0x83};
        inv_icm20948_write_mems(&this->icm_20948_device_, ACCEL_A_VAR, 4, accel_a_var);

        const uint8_t accel_cal_rate[] = {0x00, 0x00};
        inv_icm20948_write_mems(&this->icm_20948_device_, ACCEL_CAL_RATE, 2, accel_cal_rate);

        const uint8_t compass_rate[] = {0x00, 0x45};
        inv_icm20948_write_mems(&this->icm_20948_device_, CPASS_TIME_BUFFER, 2, compass_rate);

        en.DMP_INT1_EN = true;
        ICM_20948_int_enable(&this->icm_20948_device_, &en, &en);
    }

    void ICM20948::deinitialize() noexcept
    {
        this->initialized_ = false;
    }

    std::optional<std::float32_t> ICM20948::get_roll() noexcept
    {
        return this->get_roll_pitch_yaw().transform([](Vec3D<std::float32_t> const& rpy) { return rpy.x; });
    }

    std::optional<std::float32_t> ICM20948::get_pitch() noexcept
    {
        return this->get_roll_pitch_yaw().transform([](Vec3D<std::float32_t> const& rpy) { return rpy.y; });
    }

    std::optional<std::float32_t> ICM20948::get_yaw() noexcept
    {
        return this->get_roll_pitch_yaw().transform([](Vec3D<std::float32_t> const& rpy) { return rpy.z; });
    }

    std::optional<Vec3D<std::float32_t>> ICM20948::get_roll_pitch_yaw() noexcept
    {
        auto dmp_data = icm_20948_DMP_data_t{};
        inv_icm20948_read_dmp_data(&this->icm_20948_device_, &dmp_data);
        ICM_20948_reset_FIFO(&this->icm_20948_device_);

        return (dmp_data.header & DMP_header_bitmap_Quat6)
                   ? std::optional<Vec3D<std::float32_t>>{std::in_place,
                                                          dmp_data.Quat6.Data.Q1 / QUAT_SCALE,
                                                          dmp_data.Quat6.Data.Q2 / QUAT_SCALE,
                                                          dmp_data.Quat6.Data.Q3 / QUAT_SCALE}
                   : std::optional<Vec3D<std::float32_t>>{std::nullopt};
    }

} // namespace ICM20948