#include "icm_sensor.h"
#include "AK09916_ENUMERATIONS.h"
#include "AK09916_REGISTERS.h"
#include "ICM_20948_C.h"
#include "ICM_20948_DMP.h"
#include "ICM_20948_ENUMERATIONS.h"
#include "ICM_20948_REGISTERS.h"
#include "i2c.h"
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

    static ICM_20948_Status_e write(uint8_t reg, uint8_t* data, uint32_t len, void* user)
    {
        (void)user;
        HAL_I2C_Mem_Write(&hi2c1, ICM_20948_I2C_ADDR_AD0 << 1, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
        return ICM_20948_Stat_Ok;
    }
    static ICM_20948_Status_e read(uint8_t reg, uint8_t* data, uint32_t len, void* user)
    {
        (void)user;
        HAL_I2C_Mem_Read(&hi2c1, ICM_20948_I2C_ADDR_AD0 << 1, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
        return ICM_20948_Stat_Ok;
    }

    static const ICM_20948_Serif_t icm_serif_h = {.read = read, .write = write, .user = NULL};

    static ICM_20948_Device_t icm_dev = {._serif = &icm_serif_h,
                                         ._last_bank = 255,
                                         ._last_mems_bank = 255,
                                         ._dmp_firmware_available = true,
                                         ._firmware_loaded = false};

    static void set_magnetometr(void)
    {
        ICM_20948_i2c_controller_configure_peripheral(&icm_dev,
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
        ICM_20948_i2c_controller_configure_peripheral(&icm_dev,
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
        ICM_20948_set_bank(&icm_dev, 3);
        ICM_20948_execute_w(&icm_dev, AGB3_REG_I2C_MST_ODR_CONFIG, &tmp, 1);
    }

    static void enable_acc_and_gyro()
    {
        uint8_t tmp = 0x40;
        ICM_20948_set_bank(&icm_dev, 0);
        ICM_20948_execute_w(&icm_dev, AGB0_REG_PWR_MGMT_2, &tmp, 1);
    }

    static void stop_writing_perph_data_to_fifo(void)
    {
        uint8_t tmp = 0;

        ICM_20948_set_bank(&icm_dev, 0);
        ICM_20948_execute_w(&icm_dev, AGB0_REG_FIFO_EN_1, &tmp, 1);
        ICM_20948_set_bank(&icm_dev, 0);
        ICM_20948_execute_w(&icm_dev, AGB0_REG_FIFO_EN_2, &tmp, 1);
    }

    static void disable_raw_data_ready_int(void)
    {
        ICM_20948_INT_enable_t en;
        ICM_20948_int_enable(&icm_dev, NULL, &en);
        en.RAW_DATA_0_RDY_EN = 0;
        ICM_20948_int_enable(&icm_dev, &en, NULL);
    }

    static void disable_hardware_fix(void)
    {
        uint8_t tmp = 0x48;
        ICM_20948_set_bank(&icm_dev, 0);
        ICM_20948_execute_w(&icm_dev, AGB0_REG_HW_FIX_DISABLE, &tmp, 1);
    }

    static void enable_single_fifo_priority(void)
    {
        ICM_20948_set_bank(&icm_dev, 0);
        uint8_t tmp = 0xE4;
        ICM_20948_execute_w(&icm_dev, AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &tmp, 1);
    }

    static void set_dmp_accel_scalling(void)
    {
        const uint8_t acc_scale1[] = {0x04, 0x00, 0x00, 0x00};
        const uint8_t acc_scale2[] = {0x00, 0x04, 0x00, 0x00};

        inv_icm20948_write_mems(&icm_dev, ACC_SCALE, 4, acc_scale1);
        inv_icm20948_write_mems(&icm_dev, ACC_SCALE2, 4, acc_scale2);
    }

    static void set_dmp_compass_scale(void)
    {
        const uint8_t mount_multiplier_zero[] = {0x00, 0x00, 0x00, 0x00};
        const uint8_t mount_multiplier_plus[] = {0x09, 0x99, 0x99, 0x99};
        const uint8_t mount_multiplier_minus[] = {0xF6, 0x66, 0x66, 0x67};

        inv_icm20948_write_mems(&icm_dev, CPASS_MTX_00, 4, mount_multiplier_plus);
        inv_icm20948_write_mems(&icm_dev, CPASS_MTX_01, 4, mount_multiplier_zero);
        inv_icm20948_write_mems(&icm_dev, CPASS_MTX_02, 4, mount_multiplier_zero);
        inv_icm20948_write_mems(&icm_dev, CPASS_MTX_10, 4, mount_multiplier_zero);
        inv_icm20948_write_mems(&icm_dev, CPASS_MTX_11, 4, mount_multiplier_minus);
        inv_icm20948_write_mems(&icm_dev, CPASS_MTX_12, 4, mount_multiplier_zero);
        inv_icm20948_write_mems(&icm_dev, CPASS_MTX_20, 4, mount_multiplier_zero);
        inv_icm20948_write_mems(&icm_dev, CPASS_MTX_21, 4, mount_multiplier_zero);
        inv_icm20948_write_mems(&icm_dev, CPASS_MTX_22, 4, mount_multiplier_minus);
    }

    static void set_dmp_b2s_matrix(void)
    {
        const uint8_t mount_multiplier_zero[] = {0x00, 0x00, 0x00, 0x00};
        const uint8_t mount_multiplier_plus[] = {0x09, 0x99, 0x99, 0x99};
        const uint8_t mount_multiplier_minus[] = {0xF6, 0x66, 0x66, 0x67};

        inv_icm20948_write_mems(&icm_dev, CPASS_MTX_00, 4, mount_multiplier_plus);
        inv_icm20948_write_mems(&icm_dev, CPASS_MTX_01, 4, mount_multiplier_zero);
        inv_icm20948_write_mems(&icm_dev, CPASS_MTX_02, 4, mount_multiplier_zero);
        inv_icm20948_write_mems(&icm_dev, CPASS_MTX_10, 4, mount_multiplier_zero);
        inv_icm20948_write_mems(&icm_dev, CPASS_MTX_11, 4, mount_multiplier_minus);
        inv_icm20948_write_mems(&icm_dev, CPASS_MTX_12, 4, mount_multiplier_zero);
        inv_icm20948_write_mems(&icm_dev, CPASS_MTX_20, 4, mount_multiplier_zero);
        inv_icm20948_write_mems(&icm_dev, CPASS_MTX_21, 4, mount_multiplier_zero);
        inv_icm20948_write_mems(&icm_dev, CPASS_MTX_22, 4, mount_multiplier_minus);
    }

    static void init_dmp(void)
    {
        set_magnetometr();

        ICM_20948_set_clock_source(&icm_dev, ICM_20948_Clock_Auto);

        enable_acc_and_gyro();

        ICM_20948_set_sample_mode(&icm_dev, ICM_20948_Internal_Mst, ICM_20948_Sample_Mode_Cycled);
        HAL_Delay(5);
        ICM_20948_enable_FIFO(&icm_dev, false);
        ICM_20948_enable_DMP(&icm_dev, false);

        const ICM_20948_fss_t fss_cfg = {
            .a = gpm4,
            .g = dps2000,
        };
        ICM_20948_set_full_scale(&icm_dev, (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fss_cfg);

        ICM_20948_enable_dlpf(&icm_dev, ICM_20948_Internal_Gyr, true);

        stop_writing_perph_data_to_fifo();

        disable_raw_data_ready_int();

        ICM_20948_reset_FIFO(&icm_dev);
        // GYRO ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]).
        // ACCEL ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]).
        const ICM_20948_smplrt_t smp_cfg = {
            .a = 4, // 225HZ
            .g = 4  // 225HZ
        };

        ICM_20948_set_sample_rate(&icm_dev, (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), smp_cfg);

        ICM_20948_set_dmp_start_address(&icm_dev, DMP_START_ADDRESS);
        ICM_20948_firmware_load(&icm_dev);
        ICM_20948_set_dmp_start_address(&icm_dev, DMP_START_ADDRESS);

        disable_hardware_fix();

        enable_single_fifo_priority();

        set_dmp_accel_scalling();

        set_dmp_compass_scale();

        set_dmp_b2s_matrix();

        inv_icm20948_set_gyro_sf(&icm_dev, 4, 3);

        const uint8_t gyro_full_scale[] = {0x10, 0x00, 0x00, 0x00};
        inv_icm20948_write_mems(&icm_dev, GYRO_FULLSCALE, 4, gyro_full_scale);

        const uint8_t accel_only_gain[] = {0x00, 0xE8, 0xBA, 0x2E};
        inv_icm20948_write_mems(&icm_dev, ACCEL_ONLY_GAIN, 4, accel_only_gain);

        const uint8_t accel_alpha_var[] = {0x3D, 0x27, 0xD2, 0x7D};
        inv_icm20948_write_mems(&icm_dev, ACCEL_ALPHA_VAR, 4, accel_alpha_var);

        const uint8_t accel_a_var[] = {0x02, 0xD8, 0x2D, 0x83};
        inv_icm20948_write_mems(&icm_dev, ACCEL_A_VAR, 4, accel_a_var);

        const uint8_t accel_cal_rate[] = {0x00, 0x00};
        inv_icm20948_write_mems(&icm_dev, ACCEL_CAL_RATE, 2, accel_cal_rate);

        const uint8_t compass_rate[] = {0x00, 0x45};
        inv_icm20948_write_mems(&icm_dev, CPASS_TIME_BUFFER, 2, compass_rate);
    }

    void icm_sensor_init(void)
    {
        ICM_20948_sw_reset(&icm_dev);
        HAL_Delay(100);

        ICM_20948_sleep(&icm_dev, false);
        ICM_20948_low_power(&icm_dev, false);

        ICM_20948_i2c_master_passthrough(&icm_dev, false);
        ICM_20948_i2c_master_enable(&icm_dev, true);

        uint8_t tmp = 1;
        ICM_20948_i2c_master_single_w(&icm_dev, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL3, &tmp);

        for (uint8_t tries = 0; tries < 5; ++tries) {
            uint8_t test[2] = {0};
            ICM_20948_i2c_master_single_r(&icm_dev, MAG_AK09916_I2C_ADDR, AK09916_REG_WIA1, &test[1]);
            ICM_20948_i2c_master_single_r(&icm_dev, MAG_AK09916_I2C_ADDR, AK09916_REG_WIA2, &test[0]);

            if (MAG_AK09916_WHO_AM_I == *(uint16_t*)test)
                break;

            ICM_20948_i2c_master_reset(&icm_dev);
            HAL_Delay(5);
        }

        init_dmp();

        // Output rate = (DMP running rate / ODR ) - 1
        inv_icm20948_enable_dmp_sensor(&icm_dev, INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR, 1);
        inv_icm20948_set_dmp_sensor_period(&icm_dev, DMP_ODR_Reg_Quat6, 0);

        inv_icm20948_enable_dmp_sensor(&icm_dev, INV_ICM20948_SENSOR_RAW_ACCELEROMETER, 1);
        inv_icm20948_set_dmp_sensor_period(&icm_dev, DMP_ODR_Reg_Accel, 0);

        inv_icm20948_enable_dmp_sensor(&icm_dev, INV_ICM20948_SENSOR_RAW_GYROSCOPE, 1);
        inv_icm20948_set_dmp_sensor_period(&icm_dev, DMP_ODR_Reg_Gyro, 0);

        inv_icm20948_enable_dmp_sensor(&icm_dev, INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED, 1);
        inv_icm20948_set_dmp_sensor_period(&icm_dev, DMP_ODR_Reg_Cpass, 0);

        ICM_20948_enable_FIFO(&icm_dev, true);
        ICM_20948_enable_DMP(&icm_dev, true);

        ICM_20948_reset_FIFO(&icm_dev);
        ICM_20948_reset_DMP(&icm_dev);
    }

    void icm_sensor_read(icm_sensor_data_t* data)
    {
        icm_20948_DMP_data_t reading = {0};
        inv_icm20948_read_dmp_data(&icm_dev, &reading);

        if (reading.header & DMP_header_bitmap_Quat6) {
            data->q1 = reading.Quat6.Data.Q1;
            data->q2 = reading.Quat6.Data.Q1;
            data->q3 = reading.Quat6.Data.Q1;
        }
        if (reading.header & DMP_header_bitmap_Gyro) {
            data->gyr_x = reading.Raw_Gyro.Data.X;
            data->gyr_y = reading.Raw_Gyro.Data.Y;
            data->gyr_z = reading.Raw_Gyro.Data.Z;
        }
        if (reading.header & DMP_header_bitmap_Accel) {
            data->acc_x = reading.Raw_Accel.Data.X;
            data->acc_y = reading.Raw_Accel.Data.Y;
            data->acc_z = reading.Raw_Accel.Data.Z;
        }

        if (reading.header & DMP_header_bitmap_Compass) {
            data->comp_x = reading.Compass.Data.X;
            data->comp_y = reading.Compass.Data.Y;
            data->comp_z = reading.Compass.Data.Z;
        }

        ICM_20948_reset_FIFO(&icm_dev);
    }

#ifdef __cplusplus
}
#endif