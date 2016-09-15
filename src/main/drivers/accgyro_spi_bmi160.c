/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */


/*
 * Authors:
 * Dominic Clifton - Cleanflight implementation
 * John Ihlein - Initial FF32 code
 * AJ Christensen - port from d-Ronin / Bosch reference driver
*/

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#include "system.h"
#include "io.h"
#include "exti.h"
#include "bus_spi.h"

#include "gyro_sync.h"

#include "sensor.h"
#include "accgyro.h"
#include "accgyro_mpu.h"

#if defined(USE_GYRO_SPI_BMI160) || defined(USE_ACC_SPI_BMI160)

#include "accgyro_spi_bmi160.h"

static void bmi160AccAndGyroInit(void);
static bool bmiSPI160InitDone = false;

/* BMI160 Registers */
#define BMI160_REG_CHIPID 0x00
#define BMI160_REG_PMU_STAT 0x03
#define BMI160_REG_GYR_DATA_X_LSB 0x0C
#define BMI160_REG_STATUS 0x1B
#define BMI160_REG_TEMPERATURE_0 0x20
#define BMI160_REG_ACC_CONF 0x40
#define BMI160_REG_ACC_RANGE 0x41
#define BMI160_REG_GYR_CONF 0x42
#define BMI160_REG_GYR_RANGE 0x43
#define BMI160_REG_INT_EN1 0x51
#define BMI160_REG_INT_OUT_CTRL 0x53
#define BMI160_REG_INT_MAP1 0x56
#define BMI160_REG_FOC_CONF 0x69
#define BMI160_REG_CONF 0x6A
#define BMI160_REG_OFFSET_0 0x77
#define BMI160_REG_CMD 0x7E

/* Register values */
#define BMI160_PMU_CMD_PMU_ACC_NORMAL 0x11
#define BMI160_PMU_CMD_PMU_GYR_NORMAL 0x15
#define BMI160_INT_EN1_DRDY 0x10
#define BMI160_INT_OUT_CTRL_INT1_CONFIG 0x0A
#define BMI160_REG_INT_MAP1_INT1_DRDY 0x80
#define BMI160_CMD_START_FOC 0x03
#define BMI160_CMD_PROG_NVM 0xA0
#define BMI160_REG_STATUS_NVM_RDY 0x10
#define BMI160_REG_STATUS_FOC_RDY 0x08
#define BMI160_REG_CONF_NVM_PROG_EN 0x02

#define DISABLE_BMI160 IOHi(bmiSPI160CsPin)
#define ENABLE_BMI160        IOLo(bmiSPI160CsPin)

static IO_t bmiSPI160CsPin = IO_NONE;

bool bmi160WriteRegister(uint8_t reg, uint8_t data)
{
    ENABLE_BMI160;

    spiTransferByte(BMI160_SPI_INSTANCE, reg);
    spiTransferByte(BMI160_SPI_INSTANCE, data);

    DISABLE_BMI160;

    return true;
}

bool bmi160ReadRegister(uint8_t reg, uint8_t length, uint8_t *data)
{
    ENABLE_BMI160;

    spiTransferByte(BMI160_SPI_INSTANCE, reg | 0x80); // read transaction
    spiTransfer(BMI160_SPI_INSTANCE, data, NULL, length);

    DISABLE_BMI160;

    return true;
}

void bmi160SpiGyroInit(uint8_t lpf)
{
    mpuIntExtiInit();

    bmi160AccAndGyroInit();

    spiSetDivisor(BMI160_SPI_INSTANCE, SPI_CLOCK_INITIALIZATON);

    // Accel and Gyro DLPF Setting
    bmi160WriteRegister(BMI160_CONFIG, lpf);
    delayMicroseconds(1);

    spiSetDivisor(BMI160_SPI_INSTANCE, SPI_CLOCK_FAST);  // 18 MHz SPI clock

    int16_t data[3];
    mpuGyroRead(data);

    if (((int8_t)data[1]) == -1 && ((int8_t)data[0]) == -1) {
        failureMode(FAILURE_GYRO_INIT_FAILED);
    }
}

void bmi160SpiAccInit(acc_t *acc)
{
    mpuIntExtiInit();

    acc->acc_1G = 512 * 4;
}

bool bmi160SpiDetect(void)
{
    uint8_t in;
    uint8_t attemptsRemaining = 5;

#ifdef BMI160_CS_PIN
    bmiSPI160CsPin = IOGetByTag(IO_TAG(BMI160_CS_PIN));
#endif
    IOInit(bmiSPI160CsPin, OWNER_MPU, RESOURCE_SPI_CS, 0);
    IOConfigGPIO(bmiSPI160CsPin, SPI_IO_CS_CFG);

    spiSetDivisor(BMI160_SPI_INSTANCE, SPI_CLOCK_INITIALIZATON);

    bmi160WriteRegister(MPU_RA_PWR_MGMT_1, BIT_H_RESET);

    do {
        delay(150);

        bmi160ReadRegister(MPU_RA_WHO_AM_I, 1, &in);
        if (in == BMI160_WHO_AM_I_CONST) {
            break;
        }
        if (!attemptsRemaining) {
            return false;
        }
    } while (attemptsRemaining--);


    bmi160ReadRegister(MPU_RA_PRODUCT_ID, 1, &in);

    /* look for a product ID we recognise */

    // verify product revision
    switch (in) {
        case BMI160ES_REV_C4:
        case BMI160ES_REV_C5:
        case BMI160_REV_C4:
        case BMI160_REV_C5:
        case BMI160ES_REV_D6:
        case BMI160ES_REV_D7:
        case BMI160ES_REV_D8:
        case BMI160_REV_D6:
        case BMI160_REV_D7:
        case BMI160_REV_D8:
        case BMI160_REV_D9:
        case BMI160_REV_D10:
            return true;
    }

    return false;
}

static void bmi160AccAndGyroInit(void) {

    if (mpuSpi6000InitDone) {
        return;
    }

    spiSetDivisor(BMI160_SPI_INSTANCE, SPI_CLOCK_INITIALIZATON);

    // Device Reset
    bmi160WriteRegister(MPU_RA_PWR_MGMT_1, BIT_H_RESET);
    delay(150);

    bmi160WriteRegister(MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
    delay(150);

    // Clock Source PPL with Z axis gyro reference
    bmi160WriteRegister(MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
    delayMicroseconds(15);

    // Disable Primary I2C Interface
    bmi160WriteRegister(MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
    delayMicroseconds(15);

    bmi160WriteRegister(MPU_RA_PWR_MGMT_2, 0x00);
    delayMicroseconds(15);

    // Accel Sample Rate 1kHz
    // Gyroscope Output Rate =  1kHz when the DLPF is enabled
    bmi160WriteRegister(MPU_RA_SMPLRT_DIV, gyroMPU6xxxGetDividerDrops());
    delayMicroseconds(15);

    // Gyro +/- 1000 DPS Full Scale
    bmi160WriteRegister(MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
    delayMicroseconds(15);

    // Accel +/- 8 G Full Scale
    bmi160WriteRegister(MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);
    delayMicroseconds(15);


    bmi160WriteRegister(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR
    delayMicroseconds(15);

#ifdef USE_MPU_DATA_READY_SIGNAL
    bmi160WriteRegister(MPU_RA_INT_ENABLE, MPU_RF_DATA_RDY_EN);
    delayMicroseconds(15);
#endif

    spiSetDivisor(BMI160_SPI_INSTANCE, SPI_CLOCK_FAST);
    delayMicroseconds(1);

    mpuSpi6000InitDone = true;
}

bool bmi160SpiAccDetect(acc_t *acc)
{
    if (mpuDetectionResult.sensor != MPU_60x0_SPI) {
        return false;
    }

    acc->init = bmi160SpiAccInit;
    acc->read = mpuAccRead;

    return true;
}

bool bmi160SpiGyroDetect(gyro_t *gyro)
{
    if (mpuDetectionResult.sensor != MPU_60x0_SPI) {
        return false;
    }

    gyro->init = bmi160SpiGyroInit;
    gyro->read = mpuGyroRead;
    gyro->intStatus = checkMPUDataReady;
    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;

    return true;
}

#endif
