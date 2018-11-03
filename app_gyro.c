/*
 Copyright 2016 Benjamin Vedder    benjamin@vedder.se

 This file is part of the VESC firmware.

 The VESC firmware is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 The VESC firmware is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "app.h"
#include "ch.h"
#include "hal.h"
#include "hw.h"
#include "mc_interface.h"
#include "commands.h"
#include "utils.h"
#include "timeout.h"
#include <string.h>
#include <math.h>
#include "led_external.h"
#include "datatypes.h"
#include "comm_can.h"
#include "terminal.h"
#include "kalman.h"

// Registers
#define MPU_ADDR        0x68
//#define MPU_ADDR        0x72
#define WHO_AM_I        0x75

#define PWR_MGMT_1        0x6B    /* Default value 0x40 */
#define PWR_MGMT_2        0x6C

#define ACCEL_XOUT_H    0x3B    /* Read Only Register */
#define ACCEL_XOUT_L    0X3C    /* Read Only Register */
#define ACCEL_YOUT_H    0x3D    /* Read Only Register */
#define ACCEL_YOUT_L    0X3E    /* Read Only Register */
#define ACCEL_ZOUT_H    0x3F    /* Read Only Register */
#define ACCEL_ZOUT_L    0X40    /* Read Only Register */
#define TEMP_OUT_H        0x41    /* Read Only Register */
#define TEMP_OUT_L        0x42    /* Read Only Register */
#define GYRO_XOUT_H        0x43    /* Read Only Register */
#define GYRO_XOUT_L        0x44    /* Read Only Register */
#define GYRO_YOUT_H        0x45    /* Read Only Register */
#define GYRO_YOUT_L        0x46    /* Read Only Register */
#define GYRO_ZOUT_H        0x47    /* Read Only Register */
#define GYRO_ZOUT_L        0x48    /* Read Only Register */

// Settings
#define OUTPUT_ITERATION_TIME_MS        1
#define MAX_CAN_AGE                        0.1
#define RPM_FILTER_SAMPLES                8
#define LOCAL_TIMEOUT                    2000

#define RESTRICT_PITCH                    1 // restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#define RAD_TO_DEG                        (180 / M_PI)

// Threads
static THD_FUNCTION(gyro_thread, arg);
static THD_WORKING_AREA(gyro_thread_wa, 1024);
static THD_FUNCTION(output_thread, arg);
static THD_WORKING_AREA(output_thread_wa, 1024);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile gyro_data gyro_d;
static volatile int gyro_error = 0;
static volatile chuk_config config;
static volatile bool output_running = false;
static volatile systime_t last_update_time;
static volatile int whoAmItook = 0;
static volatile int initPowerMgmTook = 0;
static volatile int noDataReads = 0;
static volatile i2cflags_t errors = 0;

/* IMU Data */
static volatile int16_t accX, accY, accZ;
static volatile int16_t gyroX, gyroY, gyroZ;
static volatile int16_t tempRaw;

// Kalman instances
static volatile Kalman* kalmanX;
static volatile Kalman* kalmanY;

static volatile float gyroXangle, gyroYangle; // Angle calculate using the gyro only
static volatile float compAngleX, compAngleY; // Calculated angle using a complementary filter
static volatile float kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

static volatile uint32_t timer;

// Private functions
static void terminal_cmd_gyro_status(int argc, const char **argv);

void app_gyro_configure(chuk_config *conf) {
    config = *conf;

    terminal_register_command_callback("gyro_status",
            "Print the status of the gyro app", 0, terminal_cmd_gyro_status);
}

void app_gyro_start(void) {
    kalmanX = newKalman();
    kalmanY = newKalman();
    stop_now = false;
    hw_start_i2c();
    chThdCreateStatic(gyro_thread_wa, sizeof(gyro_thread_wa), NORMALPRIO, gyro_thread, NULL);
}

void app_gyro_stop(void) {
    stop_now = true;

    if (is_running) {
        hw_stop_i2c();
    }

    while (is_running) {
        chThdSleepMilliseconds(1);
    }
}

float app_gyro_get_decoded_chuk(void) {
}

void app_gyro_update_output(gyro_data *data) {
    if (!output_running) {
        last_update_time = 0;
        output_running = true;
        chThdCreateStatic(output_thread_wa, sizeof(output_thread_wa), NORMALPRIO, output_thread, NULL);
    }

    gyro_d = *data;
    last_update_time = chVTGetSystemTime();
    timeout_reset();
}

/**
 * Converts data from 2complemented representation to signed integer
 */
int16_t complement2signed(uint8_t msb, uint8_t lsb) {
    uint16_t word = 0;
    word = (msb << 8) + lsb;
    if (msb > 0x7F) {
        return -1 * ((int16_t) ((~word) + 1));
    }
    return (int16_t) word;
}

static THD_FUNCTION(gyro_thread, arg) {
    (void) arg;

    chRegSetThreadName("Gyro i2c");
    is_running = true;

    uint8_t rxbuf[14];
    uint8_t txbuf[2];
    msg_t status = MSG_OK;
    systime_t tmo = MS2ST(5);
    i2caddr_t gyro_addr = MPU_ADDR;
    gyro_data gyro_d_tmp;

    bool is_ok = true;

    volatile bool initPowerMgm = true;

    for (;;) {
        if (stop_now) {
            is_running = false;
            gyro_error = 0;
            return;
        }

        if (initPowerMgm) {
            // init power management
            txbuf[0] = PWR_MGMT_1;
            txbuf[1] = 0x01;
            i2cAcquireBus(&HW_I2C_DEV);
            status = i2cMasterTransmitTimeout(&HW_I2C_DEV, gyro_addr, txbuf, 2, rxbuf, 0, tmo);
            i2cReleaseBus(&HW_I2C_DEV);
            is_ok = status == MSG_OK;

            if (!is_ok) {
                errors = i2cGetErrors(&HW_I2C_DEV);
                // print errors
                commands_printf("PWR_MGMT_ERR: %d", errors);
                initPowerMgmTook++;
                gyro_error = 2;
                hw_try_restore_i2c();
                initPowerMgm = true;
                chThdSleepMilliseconds(100);
                continue;
            }

            initPowerMgm = false;

            // Wait for sensor to stabilize
            chThdSleepMilliseconds(100);

            /* Set kalman and gyro starting angle */
            txbuf[0] = ACCEL_XOUT_H;
            i2cAcquireBus(&HW_I2C_DEV);
            status = i2cMasterTransmitTimeout(&HW_I2C_DEV, gyro_addr, txbuf, 1, rxbuf, 6, tmo);
            i2cReleaseBus(&HW_I2C_DEV);
            is_ok = status == MSG_OK;

            if (!is_ok) {
                errors = i2cGetErrors(&HW_I2C_DEV);
                // print errors
                commands_printf("INITIAL_ANGLE_ERR: %d", errors);
                gyro_error = 2;
                //hw_try_restore_i2c();
                initPowerMgm = true;
                chThdSleepMilliseconds(100);
                continue;
            }

            accX = complement2signed(rxbuf[0], rxbuf[1]);
            accY = complement2signed(rxbuf[2], rxbuf[3]);
            accZ = complement2signed(rxbuf[4], rxbuf[5]);

            float roll = 0;
            float pitch = 0;
            // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
            // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
            // It is then converted from radians to degrees
            if (RESTRICT_PITCH) { // Eq. 25 and 26
                roll = atan2f(accY, accZ) * RAD_TO_DEG;
                pitch = atanf(-accX / sqrtf(accY * accY + accZ * accZ)) * RAD_TO_DEG; //atan
            } else { // Eq. 28 and 29
                roll = atanf(accY / sqrtf(accX * accX + accZ * accZ)) * RAD_TO_DEG; //atan
                pitch = atan2f(-accX, accZ) * RAD_TO_DEG;
            }

            setAngle(kalmanX, roll); // Set starting angle
            setAngle(kalmanY, pitch);
            gyroXangle = roll;
            gyroYangle = pitch;
            compAngleX = roll;
            compAngleY = pitch;

            timer = ST2MS(chVTGetSystemTime());
            commands_printf("%d", timer);
        }

        // read data
        txbuf[0] = ACCEL_XOUT_H;
        i2cAcquireBus(&HW_I2C_DEV);
        status = i2cMasterTransmitTimeout(&HW_I2C_DEV, gyro_addr, txbuf, 1, rxbuf, 14, tmo);
        i2cReleaseBus(&HW_I2C_DEV);
        is_ok = status == MSG_OK;

        if (!is_ok) {
            errors = i2cGetErrors(&HW_I2C_DEV);
            // print errors
            commands_printf("DATA_ERR: %d", errors);
            noDataReads++;
            gyro_error = 2;
//            hw_try_restore_i2c();
            initPowerMgm = true;
            chThdSleepMilliseconds(100);
            continue;
        }

        static uint8_t last_buffer[14];
        int same = 1;

        for (int i = 0; i < 14; i++) {
            if (last_buffer[i] != rxbuf[i]) {
                same = 0;
            }
        }

        memcpy(last_buffer, rxbuf, 14);

        if (1) {
            gyro_error = 0;
            accX = complement2signed(rxbuf[0], rxbuf[1]);
            accY = complement2signed(rxbuf[2], rxbuf[3]);
            accZ = complement2signed(rxbuf[4], rxbuf[5]);
            tempRaw = complement2signed(rxbuf[6], rxbuf[7]);
            gyroX = complement2signed(rxbuf[8], rxbuf[9]);
            gyroY = complement2signed(rxbuf[10], rxbuf[11]);
            gyroZ = complement2signed(rxbuf[12], rxbuf[13]);

            uint32_t now = ST2MS(chVTGetSystemTime());
            float dt = (float) (now - timer) / 1000.; // Calculate delta time
            timer = now;

            commands_printf("dt - %.2f; timer - %d", dt, timer);

            float roll = 0;
            float pitch = 0;
            // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
            // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
            // It is then converted from radians to degrees
            if (RESTRICT_PITCH) { // Eq. 25 and 26
                roll = atan2f(accY, accZ) * RAD_TO_DEG;
                pitch = atanf(-accX / sqrtf(accY * accY + accZ * accZ)) * RAD_TO_DEG; //atan
            } else { // Eq. 28 and 29
                roll = atanf(accY / sqrtf(accX * accX + accZ * accZ)) * RAD_TO_DEG; //atan
                pitch = atan2f(-accX, accZ) * RAD_TO_DEG;
            }

            float gyroXrate = gyroX / 131.0; // Convert to deg/s
            float gyroYrate = gyroY / 131.0; // Convert to deg/s

            if (RESTRICT_PITCH) {
                // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
                if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
                    setAngle(kalmanX, roll);
                    compAngleX = roll;
                    kalAngleX = roll;
                    gyroXangle = roll;
                } else {
                    kalAngleX = getAngle(kalmanX, roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
                }

                if (fabsf(kalAngleX) > 90) {
                    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
                }
                kalAngleY = getAngle(kalmanY, pitch, gyroYrate, dt);
            } else {
                // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
                if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
                    setAngle(kalmanY, pitch);
                    compAngleY = pitch;
                    kalAngleY = pitch;
                    gyroYangle = pitch;
                } else {
                    kalAngleY = getAngle(kalmanY, pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
                }

                if (fabsf(kalAngleY) > 90) {
                    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
                }
                kalAngleX = getAngle(kalmanX, roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
            }

            gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
            gyroYangle += gyroYrate * dt;
            //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
            //gyroYangle += kalmanY.getRate() * dt;

            compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
            compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

            // Reset the gyro angle when it has drifted too much
            if (gyroXangle < -90 || gyroXangle > 90) {
                gyroXangle = kalAngleX;
            }

            if (gyroYangle < -90 || gyroYangle > 90) {
                gyroYangle = kalAngleY;
            }

            float temperature = (float) tempRaw / 340.0 + 36.53;

            commands_printf("x%d y%d z%d x%d y%d z%d t%d", accX, accY, accZ, gyroX, gyroY, gyroZ, tempRaw);
            commands_printf("roll %.1f gyroX %.1f compX %.1f kalX %.1f temp %.1f", roll, gyroXangle, compAngleX, kalAngleX, temperature);
            commands_printf("pitch %.1f gyroY %.1f compY %.1f kalY %.1f temp %.1f", pitch, gyroYangle, compAngleY, kalAngleY, temperature);
            //app_gyro_update_output(&gyro_d_tmp);
        }

        if (timeout_has_timeout()) {
            gyro_error = 1;
//            commands_printf("TIMEOUT");
        }

        chThdSleepMilliseconds(20);
    }
}

static THD_FUNCTION(output_thread, arg) {
    (void) arg;

    chRegSetThreadName("Gyro output");

    for (;;) {
        chThdSleepMilliseconds(OUTPUT_ITERATION_TIME_MS);

        if (timeout_has_timeout() || gyro_error != 0
                || config.ctrl_type == CHUK_CTRL_TYPE_NONE) {
            continue;
        }

    }
}

static void terminal_cmd_gyro_status(int argc, const char **argv) {
    (void) argc;
    (void) argv;

    commands_printf("Gyro Status");
    commands_printf("Output: %s (error: %d)", output_running ? "On" : "Off",
            gyro_error);
    commands_printf("Who am I took: %d", whoAmItook);
    commands_printf("Init power mgm took: %d", initPowerMgmTook);
    commands_printf("No data reads: %d", noDataReads);
    commands_printf(" ");
}

