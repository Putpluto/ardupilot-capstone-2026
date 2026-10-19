/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see <http://www.gnu.org/licenses/>.
 */
/*
  driver for ST LSM6DSV16X IMU

  Uses HAODR mode-1 for high-accuracy ODR (1000-8000 Hz) and
  continuous FIFO for burst reads.  Digital filters auto-adapt to ODR
  so no per-rate AAF register reconfiguration is needed.

  fast sampling is controlled via INS_FAST_SAMPLE / INS_GYRO_RATE
  with base rate 1000 Hz.

  On the WHO_AM_I 0x73 parts the high-g accelerometer is batched next to
  the low-g one and replaces it while a shock is beyond the low-g range.
  Its offset is learnt only while disarmed and still, see select_accel().
 */

#include "AP_InertialSensor_LSM6DSV.h"

#include <stdio.h>
#include <utility>

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

namespace {

// Enable the second-stage digital low-pass filter (LPF2) on the
// accelerometer output.  Bandwidth is set by LSM6DSV_ACCEL_LPF2_BW.
#ifndef LSM6DSV_ACCEL_LPF2_ENABLED
#define LSM6DSV_ACCEL_LPF2_ENABLED 1
#endif

// LPF2 bandwidth selection written to CTRL8 HP_LPF2_XL_BW bits [7:5].
// When CTRL9.LPF2_XL_EN = 1 these select the second-stage cutoff:
//   0x00 = ODR/4   0x20 = ODR/10   0x40 = ODR/20   0x60 = ODR/45
//   0x80 = ODR/100 0xA0 = ODR/200  0xC0 = ODR/400  0xE0 = ODR/800
#ifndef LSM6DSV_ACCEL_LPF2_BW
#define LSM6DSV_ACCEL_LPF2_BW 0x20  // ODR/10 → 200 Hz @ 2 kHz ODR
#endif

// High-g accelerometer on the 0x73 parts. The low-g channel saturates at 16 g ahead of
// LPF2, which smooths the clipped peaks to below full scale, so a hard landing loses
// most of its deceleration without one sample reaching the clip limit. The high-g
// channel has no LPF2 and is batched in the FIFO next to the low-g samples.
#ifndef LSM6DSV_HIGH_G_ENABLED
#define LSM6DSV_HIGH_G_ENABLED 1
#endif

// High-g full scale in g. FS_XL_HG 000/001 (32/64 g) mean the same on every 0x73 part;
// 010 is 128 g on the LSM6DSV320X and ISM6HG256X but 80 g on the LSM6DSV80X, so only
// select 128 on a board known to carry one of the former.
#ifndef LSM6DSV_HIGH_G_FS_G
#define LSM6DSV_HIGH_G_FS_G 64
#endif
static_assert(LSM6DSV_HIGH_G_FS_G == 32 || LSM6DSV_HIGH_G_FS_G == 64 || LSM6DSV_HIGH_G_FS_G == 128,
              "LSM6DSV_HIGH_G_FS_G must be 32, 64 or 128");

// use high-g from the first sample with any high-g axis at or above this until this
// long after the last one
#define LSM6DSV_HIGH_G_SWITCH_G             10.0f
#define LSM6DSV_HIGH_G_HOLD_MS              10
// also switch when a low-g axis is this close to 16 g (only reachable with LPF2 off)
#define LSM6DSV_LOW_G_FULL_SCALE_G          15.5f
// the high-g zero-g offset (typ. ±1.5 g) is learnt against low-g while both are below this
#define LSM6DSV_HIGH_G_TRACK_G              8.0f
// ...and only while disarmed with vibration (VIBE, m/s/s) below this. Under motor vibration
// the mean of (high-g - low-g) over the samples inside that window is biased, because the two
// channels are filtered differently: on 2026-10-19 it drifted by 2 m/s/s in flight while half
// the samples came from high-g, and the EKF integrated the error into a false climb.
#define LSM6DSV_HIGH_G_LEARN_VIBE           3.0f
// high-g is not used before the offset has this many samples, or while it is above this
#define LSM6DSV_HIGH_G_OFFSET_MIN_SAMPLES   64
#define LSM6DSV_HIGH_G_OFFSET_MAX_G         4.0f
// stop holding low-g samples back for a high-g partner after this many in a row had none
#define LSM6DSV_HIGH_G_MISSING_MAX          8

// The 0x73 parts run the gyro at 4000 dps. At 2000 dps it sat on its 2293 dps output
// limit for up to 15 samples in hard landings, leaving the attitude 5-12 deg off.
#ifndef LSM6DSV320X_GYRO_4000DPS
#define LSM6DSV320X_GYRO_4000DPS 1
#endif

// ---- FIFO control registers (R/W) ----
#define LSM6DSV_REG_FIFO_CTRL1              0x07  // FIFO watermark threshold [7:0]
#define LSM6DSV_REG_FIFO_CTRL2              0x08  // FIFO watermark threshold [8], compression
#define LSM6DSV_REG_FIFO_CTRL3              0x09  // FIFO gyro/accel batch data rate
#define LSM6DSV_REG_FIFO_CTRL4              0x0A  // FIFO mode selection

#define LSM6DSV_FIFO_CTRL3_BDR_DISABLED     0x00
#define LSM6DSV_FIFO_CTRL4_MODE_BYPASS       0x00
#define LSM6DSV_FIFO_CTRL4_MODE_CONTINUOUS   0x06

// ---- Counter batch data rate register 1 (R/W) ----
#define LSM6DSV_REG_COUNTER_BDR_REG1        0x0B
#define LSM6DSV_COUNTER_BDR_REG1_XL_HG_BATCH_EN (1U << 3)  // batch high-g accel (0x73 parts)

// ---- WHO_AM_I register (R) ----
#define LSM6DSV_REG_WHO_AM_I                0x0F
#define LSM6DSV_ID_LSM6DSV16X               0x70
// LSM6DSV320X, LSM6DSV80X and ISM6HG256X all report 0x73. Their gyro and low-g accel
// registers match the LSM6DSV16X except CTRL6, see LSM6DSV_CTRL6_RESERVED_BIT3
#define LSM6DSV_ID_LSM6DSV320X              0x73

// ---- Accelerometer control register 1 (R/W) ----
// [6:4] OP_MODE_XL: operating mode   [3:0] ODR_XL: output data rate
#define LSM6DSV_REG_CTRL1                   0x10

// ---- Gyroscope control register 2 (R/W) ----
// [6:4] OP_MODE_G: operating mode   [3:0] ODR_G: output data rate
#define LSM6DSV_REG_CTRL2                   0x11

// ---- Control register 3 (R/W) ----
#define LSM6DSV_REG_CTRL3                   0x12
#define LSM6DSV_CTRL3_BDU                   (1U << 6)  // block data update
#define LSM6DSV_CTRL3_IF_INC                (1U << 2)  // auto-increment address
#define LSM6DSV_CTRL3_SW_RESET              (1U << 0)  // software reset

// [4] OP_MODE: 0=high-performance, 1=high-accuracy ODR
#define LSM6DSV_CTRL_MODE_HAODR             0x10

// ---- Control register 6 — gyro full-scale selection (R/W) ----
// [3:0] FS_G: gyroscope full-scale
#define LSM6DSV_REG_CTRL6                   0x15
#define LSM6DSV_CTRL6_FS_G_125DPS           0x00
#define LSM6DSV_CTRL6_FS_G_250DPS           0x01
#define LSM6DSV_CTRL6_FS_G_500DPS           0x02
#define LSM6DSV_CTRL6_FS_G_1000DPS          0x03
#define LSM6DSV_CTRL6_FS_G_2000DPS          0x04
#define LSM6DSV_CTRL6_FS_G_4000DPS          0x0C
// On the 0x73 parts FS_G is bits [2:0] only (2000dps is still 0x04). Bit 3 is
// reserved, resets to 1 and must be kept set
#define LSM6DSV_CTRL6_RESERVED_BIT3         0x08
#define LSM6DSV320X_CTRL6_FS_G_4000DPS      0x05

// ---- Control register 8 — accel full-scale & LPF2 BW (R/W) ----
// [7:5] HP_LPF2_XL_BW   [1:0] FS_XL: accelerometer full-scale
#define LSM6DSV_REG_CTRL8                   0x17
#define LSM6DSV_CTRL8_FS_XL_2G              0x00
#define LSM6DSV_CTRL8_FS_XL_4G              0x01
#define LSM6DSV_CTRL8_FS_XL_8G              0x02
#define LSM6DSV_CTRL8_FS_XL_16G             0x03

// ---- Control register 9 — accel LPF2 enable (R/W) ----
#define LSM6DSV_REG_CTRL9                   0x18
#define LSM6DSV_CTRL9_LPF2_XL_EN            (1U << 3)

// ---- FIFO status registers (R) ----
#define LSM6DSV_REG_FIFO_STATUS1            0x1B  // DIFF_FIFO [7:0]
#define LSM6DSV_REG_FIFO_STATUS2            0x1C  // flags + DIFF_FIFO [8]
#define LSM6DSV_FIFO_STATUS2_DIFF_FIFO_8    (1U << 0)

// ---- Status register (R) ----
#define LSM6DSV_REG_STATUS                  0x1E
#define LSM6DSV_STATUS_XLDA                 (1U << 0)  // accel data available
#define LSM6DSV_STATUS_GDA                  (1U << 1)  // gyro data available
#define LSM6DSV_STATUS_TDA                  (1U << 2)  // temperature data available

// ---- Data output registers (R) ----
#define LSM6DSV_REG_OUT_TEMP_L              0x20  // temperature output (16-bit)
#define LSM6DSV_REG_OUTX_L_G                0x22  // gyro XYZ output (6 bytes)
#define LSM6DSV_REG_OUTX_L_A                0x28  // accel XYZ output (6 bytes)

// ---- High-g accelerometer control register 1 (R/W, 0x73 parts) ----
// [7] XL_HG_REGOUT_EN  [6] HG_USR_OFF_ON_OUT  [5:3] ODR_XL_HG  [2:0] FS_XL_HG
#define LSM6DSV_REG_CTRL1_XL_HG             0x4E
#define LSM6DSV_CTRL1_XL_HG_REGOUT_EN       (1U << 7)

// ---- HAODR configuration register (R/W) ----
// [1:0] HAODR_SEL: high-accuracy ODR mode selection
#define LSM6DSV_REG_HAODR_CFG               0x62
#define LSM6DSV_HAODR_CFG_MODE1             0x01

// HAODR mode-1 ODR codes (written to CTRL1/CTRL2 ODR_XL/ODR_G [3:0])
#define LSM6DSV_MODE1_ODR_125HZ             0x06
#define LSM6DSV_MODE1_ODR_250HZ             0x07
#define LSM6DSV_MODE1_ODR_500HZ             0x08
#define LSM6DSV_MODE1_ODR_1000HZ            0x09
#define LSM6DSV_MODE1_ODR_2000HZ            0x0A
#define LSM6DSV_MODE1_ODR_4000HZ            0x0B
#define LSM6DSV_MODE1_ODR_8000HZ            0x0C

// ---- FIFO data output registers (R) ----
#define LSM6DSV_REG_FIFO_DATA_OUT_TAG       0x78  // FIFO tag byte
#define LSM6DSV_REG_FIFO_DATA_OUT_X_L       0x79  // FIFO data start

// ---- SPI protocol ----
#define LSM6DSV_SPI_READ_FLAG               0x80

// ---- Driver timing constants ----
#define LSM6DSV_DEFAULT_BACKEND_RATE_HZ     1000
#define LSM6DSV_INIT_MAX_TRIES              5
#define LSM6DSV_RESET_TIMEOUT_MS            100
#define LSM6DSV_DATA_READY_TIMEOUT_MS       20
#define LSM6DSV_POWERUP_DELAY_MS            5

// ---- FIFO sizing ----
#define LSM6DSV_PRIMARY_FIFO_WATERMARK_WORDS 2
#define LSM6DSV_FIFO_MAX_DRAIN_WORDS        32
#define LSM6DSV_FIFO_BURST_WORDS            16

// temperature update interval in milliseconds
#define LSM6DSV_TEMPERATURE_UPDATE_MS       100

// ---- Temperature conversion ----
#define LSM6DSV_TEMPERATURE_ZERO_C          25.0f
#define LSM6DSV_TEMPERATURE_SENSITIVITY     256.0f  // LSB/°C

// ---- Accelerometer sensitivity (mg/LSB → m/s²) ----
#define LSM6DSV_ACCEL_SCALE_2G              (GRAVITY_MSS *  2.0f / 32768.0f)
#define LSM6DSV_ACCEL_SCALE_4G              (GRAVITY_MSS *  4.0f / 32768.0f)
#define LSM6DSV_ACCEL_SCALE_8G              (GRAVITY_MSS *  8.0f / 32768.0f)
#define LSM6DSV_ACCEL_SCALE_16G             (GRAVITY_MSS * 16.0f / 32768.0f)

// ---- Gyroscope sensitivity per ST datasheet (mdps/LSB → rad/s) ----
#define LSM6DSV_GYRO_SCALE_125DPS           radians(4.375f  / 1000.0f)
#define LSM6DSV_GYRO_SCALE_250DPS           radians(8.75f   / 1000.0f)
#define LSM6DSV_GYRO_SCALE_500DPS           radians(17.50f  / 1000.0f)
#define LSM6DSV_GYRO_SCALE_1000DPS          radians(35.0f   / 1000.0f)
#define LSM6DSV_GYRO_SCALE_2000DPS          radians(70.0f   / 1000.0f)
#define LSM6DSV_GYRO_SCALE_4000DPS          radians(140.0f  / 1000.0f)

struct PACKED RawFifoWord {
    uint8_t tag;
    le16_t axis[3];
};

static_assert(sizeof(RawFifoWord) == 7, "RawFifoWord must be 7 bytes");
constexpr uint16_t LSM6DSV_FIFO_BURST_BUFFER_SIZE = LSM6DSV_FIFO_BURST_WORDS * sizeof(RawFifoWord) + 1;

float abs_max(const Vector3f &v)
{
    return MAX(fabsf(v.x), MAX(fabsf(v.y), fabsf(v.z)));
}

}

AP_InertialSensor_LSM6DSV::AP_InertialSensor_LSM6DSV(AP_InertialSensor &imu,
                                                     AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                     enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
    , _rotation(rotation)
    , _accel_scale(LSM6DSV_ACCEL_SCALE_16G)
    , _gyro_scale(LSM6DSV_GYRO_SCALE_2000DPS)
{
}

AP_InertialSensor_LSM6DSV::~AP_InertialSensor_LSM6DSV()
{
    if (_fifo_buffer != nullptr) {
        hal.util->free_type(_fifo_buffer, LSM6DSV_FIFO_BURST_BUFFER_SIZE,
                            AP_HAL::Util::MEM_DMA_SAFE);
    }
}

AP_InertialSensor_Backend *AP_InertialSensor_LSM6DSV::probe(AP_InertialSensor &imu,
                                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                                            enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    auto *sensor = NEW_NOTHROW AP_InertialSensor_LSM6DSV(imu, std::move(dev), rotation);
    if (sensor == nullptr) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_LSM6DSV::start()
{
    // pre-fetch instance numbers for checking fast sampling settings
    if (!_imu.get_gyro_instance(gyro_instance) || !_imu.get_accel_instance(accel_instance)) {
        return;
    }

    // determine fast sampling rate (SPI only)
    _backend_rate_hz = LSM6DSV_DEFAULT_BACKEND_RATE_HZ;
    if (enable_fast_sampling(accel_instance) && get_fast_sampling_rate() > 1) {
        _fast_sampling = (_dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI);
    }
    if (_fast_sampling) {
        _backend_rate_hz = calculate_backend_rate(LSM6DSV_DEFAULT_BACKEND_RATE_HZ);
    }
    _backend_period_us = 1000000UL / _backend_rate_hz;

    if (!_imu.register_accel(accel_instance, _backend_rate_hz, _dev->get_bus_id_devtype(DEVTYPE_INS_LSM6DSV)) ||
        !_imu.register_gyro(gyro_instance, _backend_rate_hz, _dev->get_bus_id_devtype(DEVTYPE_INS_LSM6DSV))) {
        return;
    }

    {
        WITH_SEMAPHORE(_dev->get_semaphore());
        if (!write_register(LSM6DSV_REG_FIFO_CTRL1, 0x00, true) ||
            !write_register(LSM6DSV_REG_FIFO_CTRL2, 0x00, true) ||
            !write_register(LSM6DSV_REG_FIFO_CTRL3, LSM6DSV_FIFO_CTRL3_BDR_DISABLED, true) ||
            !write_register(LSM6DSV_REG_FIFO_CTRL4, LSM6DSV_FIFO_CTRL4_MODE_BYPASS, true)) {
            return;
        }
        if (!configure_primary_fifo()) {
            return;
        }

        // re-configure ODR registers for the target sampling rate
        const uint8_t odr = odr_code_for_rate(_backend_rate_hz);
        write_register(LSM6DSV_REG_CTRL1, LSM6DSV_CTRL_MODE_HAODR | odr, true);
        write_register(LSM6DSV_REG_CTRL2, LSM6DSV_CTRL_MODE_HAODR | odr, true);
        configure_primary_fifo();
        _high_g = configure_high_g();
    }

    set_gyro_orientation(gyro_instance, _rotation);
    set_accel_orientation(accel_instance, _rotation);

    _fifo_buffer = static_cast<uint8_t *>(hal.util->malloc_type(LSM6DSV_FIFO_BURST_BUFFER_SIZE,
                                                                 AP_HAL::Util::MEM_DMA_SAFE));
    if (_fifo_buffer == nullptr) {
        AP_HAL::panic("LSM6DSV: Unable to allocate FIFO buffer");
    }

    periodic_handle = _dev->register_periodic_callback(_backend_period_us,
                                                       FUNCTOR_BIND_MEMBER(&AP_InertialSensor_LSM6DSV::poll_data, void));
}

bool AP_InertialSensor_LSM6DSV::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
#if HAL_LOGGING_ENABLED
    log_high_g();
#endif
    return true;
}

bool AP_InertialSensor_LSM6DSV::get_output_banner(char* banner, uint8_t banner_len)
{
    char high_g[12] {};
    if (_high_g) {
        snprintf(high_g, sizeof(high_g), " hg%ug", unsigned(LSM6DSV_HIGH_G_FS_G));
    }
    snprintf(banner, banner_len, "IMU%u: %s %s sampling %.1fkHz%s",
             gyro_instance,
             _lsm6dsv_type == LSM6DSV_Type::LSM6DSV320X ? "LSM6DSV320X" : "LSM6DSV16X",
             _fast_sampling ? "fast" : "normal",
             _backend_rate_hz * 0.001f,
             high_g);
    return true;
}

bool AP_InertialSensor_LSM6DSV::init()
{
    _dev->set_read_flag(LSM6DSV_SPI_READ_FLAG);
    return hardware_init();
}

bool AP_InertialSensor_LSM6DSV::hardware_init()
{
    hal.scheduler->delay(LSM6DSV_POWERUP_DELAY_MS);

    WITH_SEMAPHORE(_dev->get_semaphore());
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);
    if (!_dev->setup_checked_registers(14, 20)) {
        return false;
    }

    for (uint8_t attempt = 0; attempt < LSM6DSV_INIT_MAX_TRIES; attempt++) {
        if (!check_whoami()) {
            continue;
        }

        switch (_lsm6dsv_type) {
        case LSM6DSV_Type::LSM6DSV16X:
            _gyro_scale = LSM6DSV_GYRO_SCALE_2000DPS;
            _accel_scale = LSM6DSV_ACCEL_SCALE_16G;
            break;
        case LSM6DSV_Type::LSM6DSV320X:
#if LSM6DSV320X_GYRO_4000DPS
            _gyro_scale = LSM6DSV_GYRO_SCALE_4000DPS;
#else
            _gyro_scale = LSM6DSV_GYRO_SCALE_2000DPS;
#endif
            _accel_scale = LSM6DSV_ACCEL_SCALE_16G;
            break;
        }

        if (!reset_device()) {
            continue;
        }

        if (!configure_gyro()) {
            continue;
        }

        if (!configure_accel()) {
            continue;
        }

        if (!write_register(LSM6DSV_REG_HAODR_CFG, LSM6DSV_HAODR_CFG_MODE1, true)) {
            continue;
        }

        if (!write_register(LSM6DSV_REG_CTRL1, LSM6DSV_CTRL_MODE_HAODR | LSM6DSV_MODE1_ODR_1000HZ, true)) {
            continue;
        }

        if (!write_register(LSM6DSV_REG_CTRL2, LSM6DSV_CTRL_MODE_HAODR | LSM6DSV_MODE1_ODR_1000HZ, true)) {
            continue;
        }

        if (!write_register(LSM6DSV_REG_CTRL3, LSM6DSV_CTRL3_BDU | LSM6DSV_CTRL3_IF_INC, true)) {
            continue;
        }

        if (!write_register(LSM6DSV_REG_FIFO_CTRL4, 0x00, true)) {
            continue;
        }

        if (!wait_for_data_ready()) {
            continue;
        }

        _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
        return true;
    }

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    return false;
}

bool AP_InertialSensor_LSM6DSV::check_whoami()
{
    if (!read_registers(LSM6DSV_REG_WHO_AM_I, &_whoami, 1)) {
        return false;
    }

    switch (_whoami) {
    case LSM6DSV_ID_LSM6DSV16X:
        _lsm6dsv_type = LSM6DSV_Type::LSM6DSV16X;
        return true;
    case LSM6DSV_ID_LSM6DSV320X:
        _lsm6dsv_type = LSM6DSV_Type::LSM6DSV320X;
        return true;
    }

    return false;
}

bool AP_InertialSensor_LSM6DSV::reset_device()
{
    if (!write_register(LSM6DSV_REG_CTRL3, LSM6DSV_CTRL3_SW_RESET)) {
        return false;
    }

    const uint32_t start_ms = AP_HAL::millis();
    while (AP_HAL::millis() - start_ms < LSM6DSV_RESET_TIMEOUT_MS) {
        uint8_t ctrl3 = 0;
        hal.scheduler->delay(1);
        if (!read_registers(LSM6DSV_REG_CTRL3, &ctrl3, 1)) {
            continue;
        }
        if ((ctrl3 & LSM6DSV_CTRL3_SW_RESET) == 0) {
            return true;
        }
    }

    return false;
}

bool AP_InertialSensor_LSM6DSV::configure_gyro()
{
    uint8_t ctrl6 = LSM6DSV_CTRL6_FS_G_2000DPS;
    if (_lsm6dsv_type == LSM6DSV_Type::LSM6DSV320X) {
#if LSM6DSV320X_GYRO_4000DPS
        ctrl6 = LSM6DSV320X_CTRL6_FS_G_4000DPS;
#endif
        ctrl6 |= LSM6DSV_CTRL6_RESERVED_BIT3;
    }
    return write_register(LSM6DSV_REG_CTRL6, ctrl6, true);
}

bool AP_InertialSensor_LSM6DSV::configure_accel()
{
#if LSM6DSV_ACCEL_LPF2_ENABLED
    const uint8_t ctrl8 = LSM6DSV_CTRL8_FS_XL_16G | LSM6DSV_ACCEL_LPF2_BW;
    return write_register(LSM6DSV_REG_CTRL8, ctrl8, true) &&
           write_register(LSM6DSV_REG_CTRL9, LSM6DSV_CTRL9_LPF2_XL_EN, true);
#else
    return write_register(LSM6DSV_REG_CTRL8, LSM6DSV_CTRL8_FS_XL_16G, true);
#endif
}

bool AP_InertialSensor_LSM6DSV::configure_primary_fifo()
{
    const uint8_t odr = odr_code_for_rate(_backend_rate_hz);
    const uint8_t fifo_ctrl3 = uint8_t((odr << 4) | odr);

    return write_register(LSM6DSV_REG_FIFO_CTRL1,
                          uint8_t(LSM6DSV_PRIMARY_FIFO_WATERMARK_WORDS & 0xFFU),
                          true) &&
           write_register(LSM6DSV_REG_FIFO_CTRL2, 0x00, true) &&
           write_register(LSM6DSV_REG_FIFO_CTRL3, fifo_ctrl3, true) &&
           write_register(LSM6DSV_REG_FIFO_CTRL4, LSM6DSV_FIFO_CTRL4_MODE_CONTINUOUS, true);
}

// run the high-g accelerometer at the low-g rate and batch it in the FIFO (0x73 parts)
bool AP_InertialSensor_LSM6DSV::configure_high_g()
{
#if LSM6DSV_HIGH_G_ENABLED
    if (_lsm6dsv_type != LSM6DSV_Type::LSM6DSV320X) {
        return false;
    }

    // ODR_XL_HG 100..111 = 960 Hz..7.68 kHz, which HAODR mode-1 makes 1..8 kHz like ODR_XL
    uint8_t odr;
    switch (_backend_rate_hz) {
    case 8000: odr = 0x07; break;
    case 4000: odr = 0x06; break;
    case 2000: odr = 0x05; break;
    case 1000:
    default:   odr = 0x04; break;
    }
    // FS_XL_HG 000 = 32 g, 001 = 64 g, 010 = 128 g; sensitivity is full scale / 32768
    const uint8_t fs = LSM6DSV_HIGH_G_FS_G == 32 ? 0x00 : (LSM6DSV_HIGH_G_FS_G == 64 ? 0x01 : 0x02);
    if (!write_register(LSM6DSV_REG_CTRL1_XL_HG, LSM6DSV_CTRL1_XL_HG_REGOUT_EN | (odr << 3) | fs, true) ||
        !write_register(LSM6DSV_REG_COUNTER_BDR_REG1, LSM6DSV_COUNTER_BDR_REG1_XL_HG_BATCH_EN, true)) {
        return false;
    }

    _high_g_scale = GRAVITY_MSS * LSM6DSV_HIGH_G_FS_G / 32768.0f;
    _high_g_hold_samples = _backend_rate_hz * LSM6DSV_HIGH_G_HOLD_MS / 1000U;
    return true;
#else
    return false;
#endif
}

uint8_t AP_InertialSensor_LSM6DSV::odr_code_for_rate(uint16_t rate_hz) const
{
    switch (rate_hz) {
    case 8000: return LSM6DSV_MODE1_ODR_8000HZ;
    case 4000: return LSM6DSV_MODE1_ODR_4000HZ;
    case 2000: return LSM6DSV_MODE1_ODR_2000HZ;
    case 1000:
    default:   return LSM6DSV_MODE1_ODR_1000HZ;
    }
}

// calculate the backend sample rate accounting for fast sampling
// multiplier and loop rate constraints
uint16_t AP_InertialSensor_LSM6DSV::calculate_backend_rate(uint16_t base_rate_hz) const
{
    // constrain the gyro rate to be at least the loop rate
    uint8_t min_mult = 1;
    if (get_loop_rate_hz() > base_rate_hz) {
        min_mult = 2;
    }
    if (get_loop_rate_hz() > base_rate_hz * 2) {
        min_mult = 4;
    }
    const uint8_t mult = constrain_int16(get_fast_sampling_rate(), min_mult, 8);
    return constrain_int16(base_rate_hz * mult, base_rate_hz, 8000);
}

bool AP_InertialSensor_LSM6DSV::fifo_tag_supported_for_primary(const FifoTag tag)
{
    return tag == FifoTag::GyroNC || tag == FifoTag::AccelNC || tag == FifoTag::AccelHG;
}

AP_InertialSensor_LSM6DSV::FifoTag AP_InertialSensor_LSM6DSV::decode_fifo_tag(const uint8_t raw_tag)
{
    switch ((raw_tag >> 3) & 0x1FU) {
    case 0x00:
        return FifoTag::Empty;
    case 0x01:
        return FifoTag::GyroNC;
    case 0x02:
        return FifoTag::AccelNC;
    case 0x03:
        return FifoTag::Temperature;
    case 0x04:
        return FifoTag::Timestamp;
    case 0x05:
        return FifoTag::CfgChange;
    case 0x06:
        return FifoTag::AccelNC_T2;
    case 0x07:
        return FifoTag::AccelNC_T1;
    case 0x08:
        return FifoTag::Accel2xC;
    case 0x09:
        return FifoTag::Accel3xC;
    case 0x0A:
        return FifoTag::GyroNC_T2;
    case 0x0B:
        return FifoTag::GyroNC_T1;
    case 0x0C:
        return FifoTag::Gyro2xC;
    case 0x0D:
        return FifoTag::Gyro3xC;
    case 0x1D:
        return FifoTag::AccelHG;
    default:
        return FifoTag::Unsupported;
    }
}

uint8_t AP_InertialSensor_LSM6DSV::decode_fifo_tag_count(const uint8_t raw_tag)
{
    return (raw_tag >> 1) & 0x03U;
}

bool AP_InertialSensor_LSM6DSV::wait_for_data_ready()
{
    for (uint8_t i = 0; i < LSM6DSV_DATA_READY_TIMEOUT_MS; i++) {
        uint8_t status = 0;
        hal.scheduler->delay(1);
        if (!read_registers(LSM6DSV_REG_STATUS, &status, 1)) {
            continue;
        }
        if ((status & LSM6DSV_STATUS_GDA) != 0 &&
            (status & LSM6DSV_STATUS_XLDA) != 0) {
            return true;
        }
    }

    return false;
}

bool AP_InertialSensor_LSM6DSV::read_registers(uint8_t reg, uint8_t *data, uint8_t len)
{
    return _dev->read_registers(reg, data, len);
}

bool AP_InertialSensor_LSM6DSV::write_register(uint8_t reg, uint8_t value, bool checked)
{
    return _dev->write_register(reg, value, checked);
}

bool AP_InertialSensor_LSM6DSV::read_fifo_status(FifoFrame &frame, uint32_t now_us)
{
    uint8_t fifo_status[2] {};
    if (!read_registers(LSM6DSV_REG_FIFO_STATUS1, fifo_status, sizeof(fifo_status))) {
        _inc_accel_error_count(accel_instance);
        _inc_gyro_error_count(gyro_instance);
        return false;
    }

    frame.unread_words = fifo_status[0] |
                         (((fifo_status[1] & LSM6DSV_FIFO_STATUS2_DIFF_FIFO_8) != 0U) ? 0x100U : 0U);

    return true;
}

bool AP_InertialSensor_LSM6DSV::read_fifo_words_block(const uint16_t n_words, uint32_t now_us)
{
    if (_fifo_buffer == nullptr || n_words == 0 || n_words > LSM6DSV_FIFO_BURST_WORDS) {
        return false;
    }

    _fifo_buffer[0] = LSM6DSV_REG_FIFO_DATA_OUT_TAG | LSM6DSV_SPI_READ_FLAG;
    // zero MOSI payload for SPI full-duplex read
    memset(_fifo_buffer + 1, 0, n_words * sizeof(RawFifoWord));
    if (!_dev->transfer_fullduplex(_fifo_buffer, n_words * sizeof(RawFifoWord) + 1)) {
        _inc_accel_error_count(accel_instance);
        _inc_gyro_error_count(gyro_instance);
        return false;
    }

    return true;
}

bool AP_InertialSensor_LSM6DSV::consume_fifo_word(FifoFrame &frame, SampleFrame &sample, const uint8_t *raw_word)
{
    RawFifoWord raw;
    memcpy(&raw, raw_word, sizeof(raw));

    frame.tag = decode_fifo_tag(raw.tag);
    frame.tag_count = decode_fifo_tag_count(raw.tag);

    if (!fifo_tag_supported_for_primary(frame.tag)) {
        return true;
    }

    const Vector3f axes{
        float(int16_t(le16toh(raw.axis[0]))),
        float(int16_t(le16toh(raw.axis[1]))),
        float(int16_t(le16toh(raw.axis[2]))),
    };

    switch (frame.tag) {
    case FifoTag::GyroNC:
        sample.gyro = axes * _gyro_scale;
        break;
    case FifoTag::AccelNC:
        sample.accel = axes * _accel_scale;
        break;
    case FifoTag::AccelHG:
        sample.accel_hg = axes * _high_g_scale;
        break;
    default:
        return false;
    }

    return true;
}

uint16_t AP_InertialSensor_LSM6DSV::drain_fifo(uint32_t now_us)
{
    FifoFrame frame{};

    if (!read_fifo_status(frame, now_us)) {
        return 0;
    }

    if (frame.unread_words == 0) {
        return 0;
    }

    uint16_t samples_published = 0;
    uint16_t drained = 0;

    while (drained < frame.unread_words && drained < LSM6DSV_FIFO_MAX_DRAIN_WORDS) {
        const uint16_t remaining = MIN(uint16_t(frame.unread_words - drained),
                                       uint16_t(LSM6DSV_FIFO_MAX_DRAIN_WORDS - drained));
        const uint16_t block_words = MIN(remaining, LSM6DSV_FIFO_BURST_WORDS);

        if (!read_fifo_words_block(block_words, now_us)) {
            break;
        }

        const uint8_t *raw_word = _fifo_buffer + 1;
        for (uint16_t i = 0; i < block_words; i++, raw_word += sizeof(RawFifoWord)) {
            if (!consume_fifo_word(frame, frame.sample, raw_word)) {
                return samples_published;
            }
            drained++;
            if (frame.tag == FifoTag::GyroNC) {
                publish_gyro_sample(frame.sample);
                samples_published++;
                frame.sample.gyro.zero();
            } else if (frame.tag == FifoTag::AccelNC) {
                handle_low_g(frame.sample.accel, frame.tag_count);
                frame.sample.accel.zero();
            } else if (frame.tag == FifoTag::AccelHG) {
                handle_high_g(frame.sample.accel_hg, frame.tag_count);
            }
        }
    }

    if (samples_published > 0) {
        _dev->adjust_periodic_callback(periodic_handle, _backend_period_us);
    }

    return samples_published;
}

void AP_InertialSensor_LSM6DSV::publish_gyro_sample(SampleFrame &sample)
{
    _rotate_and_correct_gyro(gyro_instance, sample.gyro);
    _notify_new_gyro_raw_sample(gyro_instance, sample.gyro);
}

void AP_InertialSensor_LSM6DSV::publish_accel_sample(Vector3f accel)
{
    _rotate_and_correct_accel(accel_instance, accel);
    _notify_new_accel_raw_sample(accel_instance, accel);
    update_temperature();
}

/*
  pair each low-g sample with the high-g sample of the same FIFO time slot (same
  TAG_CNT), whichever of the two arrives first. A low-g sample whose partner never
  comes is published on its own when the next low-g or high-g word shows up.
 */
void AP_InertialSensor_LSM6DSV::handle_low_g(const Vector3f &lg, uint8_t tag_cnt)
{
    if (!_high_g) {
        publish_accel_sample(lg);
        return;
    }
    if (_pending_lg.valid) {
        publish_unpaired_low_g();
    }
    if (_pending_hg.valid && _pending_hg.tag_cnt == tag_cnt) {
        _pending_hg.valid = false;
        publish_accel_sample(select_accel(lg, &_pending_hg.accel));
        return;
    }
    _pending_hg.valid = false;
    _pending_lg.accel = lg;
    _pending_lg.tag_cnt = tag_cnt;
    _pending_lg.valid = true;
    if (_high_g_missing >= LSM6DSV_HIGH_G_MISSING_MAX) {
        // high-g words have stopped arriving, don't hold low-g back for them
        publish_unpaired_low_g();
    }
}

void AP_InertialSensor_LSM6DSV::handle_high_g(const Vector3f &hg, uint8_t tag_cnt)
{
    if (!_high_g) {
        return;
    }
    _high_g_missing = 0;
    if (_pending_lg.valid && _pending_lg.tag_cnt == tag_cnt) {
        _pending_lg.valid = false;
        publish_accel_sample(select_accel(_pending_lg.accel, &hg));
        return;
    }
    if (_pending_lg.valid) {
        publish_unpaired_low_g();
    }
    _pending_hg.accel = hg;
    _pending_hg.tag_cnt = tag_cnt;
    _pending_hg.valid = true;
}

void AP_InertialSensor_LSM6DSV::publish_unpaired_low_g()
{
    _pending_lg.valid = false;
    _high_g_unpaired++;
    if (_high_g_missing < UINT8_MAX) {
        _high_g_missing++;
    }
    publish_accel_sample(select_accel(_pending_lg.accel, nullptr));
}

/*
  return the low-g sample, or the high-g one while a shock is beyond the low-g range.
  The low-g channel clips at 16 g ahead of its LPF2, which then smooths the clipped
  peaks to below full scale, so the switch is made on the high-g reading (no LPF2,
  64 g or more). The high-g zero-g offset (typ. ±1.5 g) is learnt against low-g while
  disarmed, still and both well within range: a cumulative mean at first, then a 1 s
  time constant. Armed, the estimate from the ground is kept.
 */
Vector3f AP_InertialSensor_LSM6DSV::select_accel(const Vector3f &lg, const Vector3f *hg_raw)
{
    if (hg_raw == nullptr) {
        if (_high_g_hold > 0) {
            _high_g_hold--;
        }
        return lg;
    }

    const Vector3f hg = *hg_raw - _high_g_offset;
    const bool offset_ok = _high_g_offset_n >= LSM6DSV_HIGH_G_OFFSET_MIN_SAMPLES &&
                           abs_max(_high_g_offset) < LSM6DSV_HIGH_G_OFFSET_MAX_G * GRAVITY_MSS;
    if (offset_ok &&
        (abs_max(hg) >= LSM6DSV_HIGH_G_SWITCH_G * GRAVITY_MSS ||
         abs_max(lg) >= LSM6DSV_LOW_G_FULL_SCALE_G * GRAVITY_MSS)) {
        _high_g_hold = _high_g_hold_samples;
    }
    if (_high_g_hold > 0) {
        _high_g_hold--;
        _high_g_used++;
        return hg;
    }

    if (!hal.util->get_soft_armed() &&
        abs_max(lg) < LSM6DSV_HIGH_G_TRACK_G * GRAVITY_MSS &&
        abs_max(hg) < LSM6DSV_HIGH_G_TRACK_G * GRAVITY_MSS &&
        abs_max(_imu.get_vibration_levels(accel_instance)) < LSM6DSV_HIGH_G_LEARN_VIBE) {
        if (_high_g_offset_n < _backend_rate_hz) {
            _high_g_offset_n++;
        }
        _high_g_offset += (*hg_raw - lg - _high_g_offset) / float(_high_g_offset_n);
        _high_g_learnt = true;
    }
    return lg;
}

#if HAL_LOGGING_ENABLED
// log the high-g offset estimate and how often high-g was used, once a second
void AP_InertialSensor_LSM6DSV::log_high_g()
{
    const uint32_t now_ms = AP_HAL::millis();
    if (!_high_g || now_ms - _high_g_log_ms < 1000) {
        return;
    }
    _high_g_log_ms = now_ms;

// @LoggerMessage: IHGA
// @Description: LSM6DSV high-g accelerometer channel
// @Field: TimeUS: Time since system startup
// @Field: I: accelerometer sensor instance number
// @Field: OX: high-g minus low-g offset, sensor X axis
// @Field: OY: high-g minus low-g offset, sensor Y axis
// @Field: OZ: high-g minus low-g offset, sensor Z axis
// @Field: Used: samples taken from the high-g channel since boot
// @Field: Unp: low-g samples published without a high-g partner since boot
// @Field: Lrn: 1 if the offset was learnt since the previous message (disarmed and still only)
    AP::logger().Write("IHGA", "TimeUS,I,OX,OY,OZ,Used,Unp,Lrn", "QBfffIIB",
                       AP_HAL::micros64(),
                       accel_instance,
                       _high_g_offset.x,
                       _high_g_offset.y,
                       _high_g_offset.z,
                       _high_g_used,
                       _high_g_unpaired,
                       uint8_t(_high_g_learnt));
    _high_g_learnt = false;
}
#endif

void AP_InertialSensor_LSM6DSV::check_register_monitor()
{
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);
    AP_HAL::Device::checkreg reg;
    if (!_dev->check_next_register(reg)) {
        log_register_change(_dev->get_bus_id(), reg);
        _inc_accel_error_count(accel_instance);
        _inc_gyro_error_count(gyro_instance);
    }
    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
}

void AP_InertialSensor_LSM6DSV::update_temperature()
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _temperature_last_ms < LSM6DSV_TEMPERATURE_UPDATE_MS) {
        return;
    }
    _temperature_last_ms = now_ms;

    uint8_t tbuf[2];
    if (!read_registers(LSM6DSV_REG_OUT_TEMP_L, tbuf, sizeof(tbuf))) {
        _inc_accel_error_count(accel_instance);
        return;
    }
    const int16_t temperature_raw = int16_t(uint16_t(tbuf[0] | (tbuf[1] << 8)));
    const float temp_degc = LSM6DSV_TEMPERATURE_ZERO_C + temperature_raw / LSM6DSV_TEMPERATURE_SENSITIVITY;
    _publish_temperature(accel_instance, temp_degc);
}

void AP_InertialSensor_LSM6DSV::poll_data()
{
    drain_fifo(AP_HAL::micros());
    check_register_monitor();
}
