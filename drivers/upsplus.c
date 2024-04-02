/* upsplus.c Driver for the UPSPlus HAT (https://wiki.52pi.com/index.php/EP-0136), addressed via i2c.

    Copyright (C) 2019 Andrew Anderson <aander07@gmail.com>
    Copyright (C) 2024 David Carson <dacarson@gmail.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "main.h"

#include <sys/ioctl.h>
#include "nut_stdint.h"

/*
 * Linux I2C userland is a bit of a mess until distros refresh to
 * the i2c-tools 4.x release that profides i2c/smbus.h for userspace
 * instead of (re)using linux/i2c-dev.h, which conflicts with a
 * kernel header of the same name.
 *
 * See:
 * https://i2c.wiki.kernel.org/index.php/Plans_for_I2C_Tools_4
 */
#if HAVE_LINUX_SMBUS_H
#    include <i2c/smbus.h>
#endif
#if HAVE_LINUX_I2C_DEV_H
#    include <linux/i2c-dev.h>    /* for I2C_SLAVE */
#if !HAVE_LINUX_SMBUS_H
#ifndef I2C_FUNC_I2C
#    include <linux/i2c.h>
#endif
#endif
#endif

/*
 * i2c-tools pre-4.0 has a userspace header with a name that conflicts
 * with a kernel header, so it may be ignored/removed by distributions
 * when packaging i2c-tools.
 *
 * This will cause the driver to be un-buildable on certain
 * configurations, so include the necessary bits here to handle this
 * situation.
 */
#if WITH_LINUX_I2C
#if !HAVE_DECL_I2C_SMBUS_ACCESS
static inline __s32 i2c_smbus_access(int file, char read_write, __u8 command, int size, union i2c_smbus_data *data)
{
    struct i2c_smbus_ioctl_data args;
    __s32 err;

    args.read_write = read_write;
    args.command = command;
    args.size = size;
    args.data = data;

    err = ioctl(file, I2C_SMBUS, &args);
    if (err == -1)
        err = -errno;
    return err;
}
#endif

#if !HAVE_DECL_I2C_SMBUS_READ_BYTE_DATA
static inline __s32 i2c_smbus_read_byte_data(int file, __u8 command)
{
    union i2c_smbus_data data;
    int err;

    if ((err = i2c_smbus_access(file, I2C_SMBUS_READ, command, I2C_SMBUS_BYTE_DATA, &data)) < 0)
        return err;
    else
        return 0x0FF & data.byte;
}
#endif

#if !HAVE_DECL_I2C_SMBUS_WRITE_BYTE_DATA
static inline __s32 i2c_smbus_write_byte_data(int file, __u8 command, __u8 value)
{
    union i2c_smbus_data data;
    int err;

    data.byte = value;
    if ((err = i2c_smbus_access(file, I2C_SMBUS_WRITE, command, I2C_SMBUS_BYTE_DATA, &data)) < 0)
        return err;
    else
        return 0x0FF & data.byte;
}
#endif

#if !HAVE_DECL_I2C_SMBUS_READ_WORD_DATA
static inline __s32 i2c_smbus_read_word_data(int file, __u8 command)
{
    union i2c_smbus_data data;
    int err;

    if ((err = i2c_smbus_access(file, I2C_SMBUS_READ, command, I2C_SMBUS_WORD_DATA, &data)) < 0)
        return err;
    else
        return 0x0FFFF & data.word;
}
#endif

#if !HAVE_DECL_I2C_SMBUS_WRITE_WORD_DATA
static inline __s32 i2c_smbus_write_word_data(int file, __u8 command, __u16 value)
{
    union i2c_smbus_data data;
    int err;

    data.word = value;
    if ((err = i2c_smbus_access(file, I2C_SMBUS_WRITE, command, I2C_SMBUS_WORD_DATA, &data)) < 0)
        return err;
    else
        return 0x0FFFF & data.word;
}
#endif

#if !HAVE_DECL_I2C_SMBUS_READ_BLOCK_DATA
static inline __u8 *i2c_smbus_read_i2c_block_data(int file, __u8 command, __u8 length, __u8 * values)
{
    union i2c_smbus_data data;
    int err;

    if (length > I2C_SMBUS_BLOCK_MAX) {
        length = I2C_SMBUS_BLOCK_MAX;
    }

    data.block[0] = length;
    memcpy(data.block + 1, values, length);

    if ((err = i2c_smbus_access(file, I2C_SMBUS_READ, command, I2C_SMBUS_I2C_BLOCK_DATA, &data)) < 0)
        return NULL;
    else
        memcpy(values, &data.block[1], data.block[0]);

    return values;
}
#endif
#endif                /* if WITH_LINUX_I2C */

/*
 * UPSPlus i2c registers as provided in:
 * https://wiki.52pi.com/index.php/EP-0136
 */
#define MCU_VOLTAGE_CMD                     0x01
#define OUTPUT_VOLTAGE_CMD                  0x03
#define BATTERY_VOLTAGE_CMD                 0x05
#define USBC_VOLTAGE_CMD                    0x07
#define MICROUSB_VOLTAGE_CMD                0x09

#define BATTERY_TEMPERATURE_CMD             0x0B

#define BATTERY_FULL_CMD                    0x0D
#define BATTERY_EMPTY_CMD                   0x0F
#define BATTERY_LOW_CMD                     0x11
#define CHARGE_LEVEL_CMD                    0x13

#define BATTERY_SAMPLE_PERIOD_CMD           0x15

#define POWER_STATUS_CMD                    0x17

#define SHUTDOWN_TIMER_CMD                  0x18
#define WAKEUP_ON_CHARGE_CMD                0x19

#define RESTART_TIMER_CMD                   0x1A
#define RESET_TO_DEFAULT_CMD                0x1B

#define RUNNING_TIME_CMD                    0x1C
#define CHARGING_TIME_CMD                   0x20
#define UPTIME_CMD                          0x24

#define FIRMWARE_VERSION_CMD                0x28

#define BATTERY_PARAM_SET_BY_USER_CMD       0x2A

#define SERIAL_NUMBER_CMD                   0xF0

/*
 * Constants used internally
 */

#define POWER_NOT_CONNECTED                 0x0
#define USBC_POWER_CONNECTED                0x1
#define MICROUSB_POWER_CONNECTED            0x2

#define USB_LEVEL_THRESHOLD                 4000
#define CHARGE_CURRENT_THRESHOLD            0.20
#define USBC_NOMINAL_VOLTAGE                9.00
#define MICROUSB_NOMINAL_VOLTAGE            5.00
#define MIN_BATTERY_VOLTAGE                 2750/* Lower will kill the battery. */
#define MAX_BATTERY_VOLTAGE                 4500
#define MIN_SAMPLE_PERIOD                   1
#define MAX_SAMPLE_PERIOD                   1440
#define MAX_LOAD                            22.5/* (5V x 4.5A) */
#define MAX_PEAK_LOAD                       40.0/* (5V x 8A) */
#define AUTO_SHUTDOWN_TIME                  240

#define DRIVER_NAME                         "UPSPlus driver"
#define DRIVER_VERSION                      "1.0"

#define LENGTH_TEMP 256

#define UPSPLUS_I2C_ADDRESS                  0x17
#define INA219_OUTPUT_I2C_ADDRESS            0x40
#define INA219_BATTERY_I2C_ADDRESS           0x45

#define INA219_CONFIGURATION_CMD             0x00
#define INA219_SHUNTVOLTAGE_CMD              0x01
#define INA219_BUSVOLTAGE_CMD                0x02
#define INA219_POWER_CMD                     0x03
#define INA219_CURRENT_CMD                   0x04
#define INA219_CALIBRATION_CMD               0x05

#define INA219_CONVERSION_READY              0x02

/*
 * INA219_CONFIG_BVOLTAGERANGE_32V (0x2000 & 0x2000 Mask)
 * INA219_CONFIG_GAIN_1_40MV (0x0000 & 0x1800 Mask)
 * INA219_CONFIG_BADCRES_12BIT_1S_532US   (0x0018 & 0x0780 Mask)
 * INA219_CONFIG_SADCRES_12BIT_1S_532US   (0x0018 & 0x0078 Mask)
 */
#define INA219_CONFIGURATION_VALUE           0x219F

/*
* Rather than calculating the calibration value and the LSB
* multiplication factor for Current and Power with a INA219
* library, the value can be scraped from:
* "UPSPlus/Full-featured-demo-code.py" script with debugging turned
* on, and see the results of it's calculations.
* ina_supply = INA219(0.00725, busnum=DEVICE_BUS, address=0x40, log_level=logging.DEBUG)
* ina_batt = INA219(0.005, busnum=DEVICE_BUS, address=0x45, log_level=logging.DEBUG)
*/
#define INA219_CALIBRATION_VALUE_MAGIC       0x8388
#define OUTPUT_CURRENT_LSB_MAGIC             0.0001682
#define OUTPUT_POWER_LSB_MAGIC               0.003364
#define BATTERY_CURRENT_LSB_MAGIC            0.0002439
#define BATTERY_POWER_LSB_MAGIC              0.004878

static char *default_i2c_bus_path = "/dev/i2c-1";
static char *i2c_bus_path;

/*
 * Current charging levels
 */
static uint16_t usbC_power = 0;
static uint16_t microUsb_power = 0;

/*
 * Flag to track state
 */
static uint8_t power_state = 0;
static uint16_t firmware_version = 0;

/*
 * Battery values used to calculate percentages
 */
static uint16_t battery_full = 0;
static uint16_t battery_empty = 0;
static uint16_t battery_voltage = 0;
static uint16_t battery_low = 0;

/*
 * Battery current is positive, charging
 * Battery current is negative, discharging
 */
static float battery_current = 0;

/*
 * Smooth out i2c read errors by holding the most recent
 * battery charge level reading
 */
static float battery_charge_level = 0;

/*
 * Keep track of if the driver started the shutdown
 * so that if power is re-connected, it will stop
 * the shutdown.
 */
static uint8_t automatic_shutdown = 0;

/* driver description structure */
upsdrv_info_t upsdrv_info = {
    DRIVER_NAME,
    DRIVER_VERSION,
    "David Carson <dacarson@gmail.com>",
    DRV_BETA,
    { NULL }
};

/* The macros below all write into a "data" variable defined by the routine
 * scope which calls them, with respective type of uint8_t for "byte" and
 * uint16_t for "word" macros. Native i2c functions operate with __s32 type
 * (currently, signed 32-bit ints?) with negative values for error returns.
 * Note: some manpages refer to "s32" while headers on my and CI systems use
 * a "__s32" type. Maybe this is something to determine in configure script?
 * Code below was fixed to convert the valid values and avoid compiler
 * warnings about comparing whether unsigned ints happened to be negative.
 */
#define I2C_READ_BYTE(fd, cmd, label) \
    { \
        __s32 sData; \
        if ((sData = i2c_smbus_read_byte_data(fd, cmd)) < 0 ) { \
            upsdebugx(2, "Failure reading the i2c bus [%s]", label); \
            return; \
        } ; \
        data = (uint8_t) sData; \
    }

#define I2C_WRITE_BYTE(fd, cmd, value, label) \
    { \
        if ( i2c_smbus_write_byte_data(fd, cmd, value) < 0 ) { \
            upsdebugx(2, "Failure writing to the i2c bus [%s]", label); \
            return; \
        } ; \
    }

#define I2C_READ_WORD(fd, cmd, label) \
    { \
        __s32 sData; \
        if ((sData = i2c_smbus_read_word_data(fd, cmd)) < 0 ) { \
            upsdebugx(2, "Failure reading the i2c bus [%s]", label); \
            return; \
        } ; \
        data = (uint16_t) sData; \
    }

#define I2C_WRITE_WORD(fd, cmd, value, label) \
    { \
        if ( i2c_smbus_write_word_data(fd, cmd, value) < 0 ) { \
            upsdebugx(2, "Failure writing to the i2c bus [%s]", label); \
            return; \
        } ; \
    }

#define I2C_READ_BLOCK(fd, cmd, size, block, label) \
    if ((i2c_smbus_read_i2c_block_data(fd, cmd, size, block)) < 0 ) { \
        upsdebugx(2, "Failure reading the i2c bus [%s]", label); \
        return; \
    }

/*
* For some reason the INA219 registers seem to be in reverse order
* So swap MSB/LSB for INA219 operations.
*/
#define I2C_READ_WORD_INA219(fd, cmd, label) \
    { \
        __s32 sData; \
        if ((sData = i2c_smbus_read_word_data(fd, cmd)) < 0 ) { \
            upsdebugx(2, "Failure reading the i2c bus [%s]", label); \
            return; \
        } ; \
        data = (uint16_t)( (sData >> 8) | (sData << 8) ); \
    }

#define I2C_WRITE_WORD_INA219(fd, cmd, value, label) \
    { \
        if ( i2c_smbus_write_word_data(fd, cmd, (__u16)((value >> 8) | (value << 8))) < 0 ) { \
            upsdebugx(2, "Failure writing to the i2c bus [%s]", label); \
            return; \
        } ; \
    }
static inline int open_i2c_bus(char *path, uint8_t addr)
{
    int file;

    if ((file = open(path, O_RDWR)) < 0) {
        fatal_with_errno(EXIT_FAILURE, "Failed to open the i2c bus on %s", path);
    }

    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        fatal_with_errno(EXIT_FAILURE, "Failed to acquire the i2c bus and/or talk to the UPS");
    }

    return file;
}

static void get_charge_level(void)
{
    uint8_t cmd = CHARGE_LEVEL_CMD;
    uint16_t data = battery_charge_level;

    upsdebugx(3, __func__);

    I2C_READ_WORD(upsfd, cmd, __func__)

    battery_charge_level = data;

    upsdebugx(1, "Battery Charge Level: %d%%", data);
    dstate_setinfo("battery.charge", "%d", data);
}

static void get_output_voltage(void)
{
    uint8_t cmd = OUTPUT_VOLTAGE_CMD;
    uint16_t data;

    upsdebugx(3, __func__);

    I2C_READ_WORD(upsfd, cmd, __func__)

    upsdebugx(1, "Output voltage: %0.3fV", data / 1000.0);
    dstate_setinfo("output.voltage", "%0.3f", data / 1000.0);
}

static void get_UsbC_Voltage(void)
{
    uint8_t cmd = USBC_VOLTAGE_CMD;
    uint16_t data;

    upsdebugx(3, __func__);

    I2C_READ_WORD(upsfd, cmd, __func__)
	
	usbC_power = data;

    upsdebugx(2, "USB-C Voltage: %0.3fV", data / 1000.0);
}

static void get_MicroUsb_Voltage(void)
{
    uint8_t cmd = MICROUSB_VOLTAGE_CMD;
    uint16_t data;

    upsdebugx(3, __func__);

    I2C_READ_WORD(upsfd, cmd, __func__)
	
	microUsb_power = data;

    upsdebugx(2, "Micro USB Voltage: %0.3fV", data / 1000.0);
}

static void get_status(void)
{
    uint8_t cmd = POWER_STATUS_CMD, data;
    char status_buf[ST_MAX_VALUE_LEN];

    upsdebugx(3, __func__);

    memset(status_buf, 0, ST_MAX_VALUE_LEN);

    I2C_READ_BYTE(upsfd, cmd, __func__)

	if (data == 1) {
        upsdebugx(1, "Battery status: normal");
    } else {
        upsdebugx(1, "Battery status: off");
        status_set("OFF");
        /* If we are not running, then set no other Status. */
        return;
    }

    if (power_state == POWER_NOT_CONNECTED) {
        status_set("OB");
    } else {
        status_set("OL");
    }

    if (battery_voltage == 0) {
        upsdebugx(1, "Battery Status: Replace");
        status_set("RB");
    } else if (battery_voltage < battery_low) {
        upsdebugx(1, "Battery Status: Low");
        status_set("LB");
    } else if (battery_voltage > (1.1 * battery_full)) {
        upsdebugx(1, "Battery Status: High");
        status_set("HB");
    }

    if (battery_current > CHARGE_CURRENT_THRESHOLD) {
        upsdebugx(1, "Battery Status: Charging");
        dstate_setinfo("battery.charger.status", "%s", "charging");
        status_set("CHRG");
    } else if (battery_current < 0 && power_state == POWER_NOT_CONNECTED) {
        upsdebugx(1, "Battery Status: Discharging");
        dstate_setinfo("battery.charger.status", "%s", "discharging");
        status_set("DISCHRG");
    } else {
        upsdebugx(1, "Battery Status: Resting");
        dstate_setinfo("battery.charger.status", "%s", "resting");
    }

}

static void get_battery_temperature(void)
{
    uint8_t cmd = BATTERY_TEMPERATURE_CMD;
    int16_t data;

    upsdebugx(3, __func__);

    I2C_READ_WORD(upsfd, cmd, __func__)
    
	upsdebugx(1, "Battery Temperature: %d°C", data);
    dstate_setinfo("battery.temperature", "%d", data);
}

static void get_battery_voltage(void)
{
    uint8_t cmd = BATTERY_VOLTAGE_CMD;
    int16_t data = battery_voltage;

    upsdebugx(3, __func__);

    I2C_READ_WORD(upsfd, cmd, __func__)
    
	battery_voltage = data;

    upsdebugx(1, "Battery Voltage: %0.3fV", data / 1000.0);
    dstate_setinfo("battery.voltage", "%0.3f", data / 1000.0);
}

static void get_realtime_output_state(void)
{
    uint16_t data;

    extrafd = open_i2c_bus(i2c_bus_path, INA219_OUTPUT_I2C_ADDRESS);

/* Configure INA219 */
    I2C_WRITE_WORD_INA219(extrafd, INA219_CONFIGURATION_CMD, INA219_CONFIGURATION_VALUE, __func__)

/* Calibrate INA219 */
    I2C_WRITE_WORD_INA219(extrafd, INA219_CALIBRATION_CMD, INA219_CALIBRATION_VALUE_MAGIC, __func__)

/* Read values */
    int attempt = 3;
    do {
        I2C_READ_WORD_INA219(extrafd, INA219_BUSVOLTAGE_CMD, __func__)
    } while (!(data & INA219_CONVERSION_READY) && attempt--);

    data >>= 3;    /* Bits 3-15 */
    data *= 4;    /* LSB 4mV */
    upsdebugx(1, "INA219 Output Voltage: %0.3fV", data / 1000.0);
    dstate_setinfo("output.voltage", "%0.3f", data / 1000.0);

    I2C_READ_WORD_INA219(extrafd, INA219_POWER_CMD, __func__)
    upsdebugx(1, "INA219 Output Power: %0.3fW", data * OUTPUT_POWER_LSB_MAGIC);
    upsdebugx(1, "UPS Load: %0.3f%%", 100 * data * OUTPUT_POWER_LSB_MAGIC / MAX_LOAD);
    dstate_setinfo("ups.load", "%0.3f", 100 * data * OUTPUT_POWER_LSB_MAGIC / MAX_LOAD);

    I2C_READ_WORD_INA219(extrafd, INA219_CURRENT_CMD, __func__)
/* Current is a signed 16bit number */
    upsdebugx(1, "INA219 Output Current: %0.3fA", (int16_t) data * OUTPUT_CURRENT_LSB_MAGIC);
    dstate_setinfo("output.current", "%0.3f", (int16_t) data * OUTPUT_CURRENT_LSB_MAGIC);

    close(extrafd);
    extrafd = 0;
}

static void get_realtime_battery_state(void)
{
    uint16_t data = 0;

    extrafd = open_i2c_bus(i2c_bus_path, INA219_BATTERY_I2C_ADDRESS);

/* Configure INA219 */
    I2C_WRITE_WORD_INA219(extrafd, INA219_CONFIGURATION_CMD, INA219_CONFIGURATION_VALUE, __func__)

/* Calibrate INA219 */
    I2C_WRITE_WORD_INA219(extrafd, INA219_CALIBRATION_CMD, INA219_CALIBRATION_VALUE_MAGIC, __func__)
    
/* Read INA219 values */
    int attempt = 3;
    do {
        I2C_READ_WORD_INA219(extrafd, INA219_BUSVOLTAGE_CMD, __func__)
    } while (!(data & INA219_CONVERSION_READY) && attempt--);

    data >>= 3;    /* Bits 3-15 */
    data *= 4;    /* LSB 4mV */
    upsdebugx(1, "INA219 Battery Voltage: %0.3fV", data / 1000.0);
    dstate_setinfo("battery.voltage", "%0.3f", data / 1000.0);

    I2C_READ_WORD_INA219(extrafd, INA219_POWER_CMD, __func__)
    upsdebugx(1, "INA219 Battery Power: %0.3fW", data * BATTERY_POWER_LSB_MAGIC);
/* dstate_setinfo( "battery.power", "%0.3f", data * BATTERY_POWER_LSB_MAGIC ); */

    I2C_READ_WORD_INA219(extrafd, INA219_CURRENT_CMD, __func__)
/* Current is a signed 16bit number */
    upsdebugx(1, "INA219 Battery Current: %0.3fA", (int16_t) data * BATTERY_CURRENT_LSB_MAGIC);
    dstate_setinfo("battery.current", "%0.3f", (int16_t) data * BATTERY_CURRENT_LSB_MAGIC);
    battery_current = (int16_t) data * BATTERY_CURRENT_LSB_MAGIC;

    close(extrafd);
    extrafd = 0;
}

static void get_firmware_version(void)
{
    uint8_t cmd = FIRMWARE_VERSION_CMD;
    uint16_t data = 0;

    upsdebugx(3, __func__);

    I2C_READ_WORD(upsfd, cmd, __func__)
    
	firmware_version = data;

    upsdebugx(1, "UPS Firmware Version: %d", data);
    dstate_setinfo("ups.firmware", "%d", data);
}

static void get_serial_number(void)
{
    uint8_t cmd = SERIAL_NUMBER_CMD;
    __u8 block[I2C_SMBUS_BLOCK_MAX];
    char serial_number[LENGTH_TEMP];

    upsdebugx(3, __func__);

    I2C_READ_BLOCK(upsfd, cmd, 12, block, __func__)

    snprintf(serial_number, sizeof(serial_number), "%08X-%08X-%08X",
        (block[3] << 24 | block[2] << 16 | block[1] << 8 | block[0]),
        (block[7] << 24 | block[6] << 16 | block[5] << 8 | block[4]),
        (block[11] << 24 | block[10] << 16 | block[9] << 8 | block[8]));

    upsdebugx(1, "Serial Number: %s", serial_number);
    dstate_setinfo("device.serial", "%s", serial_number);
}

static void get_charge_low(void)
{
/* Calculate the low percentage as this UPS only has a low voltage */
    uint8_t cmd = BATTERY_LOW_CMD;
    int16_t data = battery_low;

    upsdebugx(3, __func__);

    I2C_READ_WORD(upsfd, cmd, __func__)

    battery_low = data;

    upsdebugx(1, "Low Charge Threshold: %0.3f%%", 100.0 * (data - battery_empty) / (battery_full - battery_empty));
    dstate_setinfo("battery.charge.low", "%0.3f", 100.0 * (data - battery_empty) / (battery_full - battery_empty));

}

static int set_charge_low(const char *value)
{
/* Calculate the low percentage as this UPS only has a low voltage */
    uint8_t cmd = BATTERY_LOW_CMD;
    int16_t data;

    upsdebugx(3, __func__);

/* Calculate a new low voltage based on percent */
    int percent;
    if (str_to_int(value, &percent, 10) && percent >= 0 && percent <= 100) {
        data = ((battery_full - battery_empty) * percent) / 100 + battery_empty;

        i2c_smbus_write_byte_data(upsfd, BATTERY_PARAM_SET_BY_USER_CMD, 0x1);
        i2c_smbus_write_byte_data(upsfd, cmd, data & 0xFF);
        i2c_smbus_write_byte_data(upsfd, cmd + 1, (data >> 8) & 0xFF);
        upsdebugx(1, "Low Charge Threshold: %0.3f%%", 100.0 * (data - battery_empty) / (battery_full - battery_empty));
        dstate_setinfo("battery.charge.low", "%0.3f", 100.0 * (data - battery_empty) / (battery_full - battery_empty));
        battery_low = data;
        return STAT_SET_HANDLED;
    }

    upsdebugx(1, "Unknown value for battery.charge.low: %s", value);
    return STAT_SET_UNKNOWN;
}

static void get_battery_full(void)
{
    uint8_t cmd = BATTERY_FULL_CMD;
    int16_t data = battery_full;

    upsdebugx(3, __func__);

    I2C_READ_WORD(upsfd, cmd, __func__)
    battery_full = data;

    upsdebugx(1, "Battery Voltage High: %0.3fV", data / 1000.0);
    dstate_setinfo("battery.voltage.high", "%0.3f", data / 1000.0);
}

static int set_battery_full(const char *value)
{
    uint8_t cmd = BATTERY_FULL_CMD;
    int16_t data;

    upsdebugx(3, __func__);

    double voltage;
    if (str_to_double(value, &voltage, 10)) {
        data = voltage * 1000;
        if (data >= MIN_BATTERY_VOLTAGE && data <= MAX_BATTERY_VOLTAGE) {
            i2c_smbus_write_byte_data(upsfd, BATTERY_PARAM_SET_BY_USER_CMD, 0x1);
            i2c_smbus_write_byte_data(upsfd, cmd, data & 0xFF);
            i2c_smbus_write_byte_data(upsfd, cmd + 1, (data >> 8) & 0xFF);
            upsdebugx(1, "Battery Voltage High: %0.3fV", data / 1000.0);
            dstate_setinfo("battery.voltage.high", "%0.3f", data / 1000.0);
            battery_full = data;
            return STAT_SET_HANDLED;
        }
    }

    upsdebugx(1, "Unknown value for battery.voltage.high: %s", value);
    return STAT_SET_UNKNOWN;
}

static void get_battery_nominal(void)
{
    int16_t data;

    upsdebugx(3, __func__);

/*
 * Supported Batteries are either 4.2V, 4.35V, 4.4V or 4.5V
 * So calculate it based on 'Full' battery value
 */
    data = 4500;
    if (battery_full - data < 0) {
        data = 4400;
        if (battery_full - data < 0) {
            data = 4350;
            if (battery_full - data < 0) {
                data = 4200;
            }
        }
    }

    upsdebugx(1, "Battery Voltage Nominal: %0.3fV", data / 1000.0);
    dstate_setinfo("battery.voltage.nominal", "%0.3f", data / 1000.0);
}

static void get_battery_empty(void)
{
    uint8_t cmd = BATTERY_EMPTY_CMD;
    int16_t data = battery_empty;

    upsdebugx(3, __func__);

    I2C_READ_WORD(upsfd, cmd, __func__)
    battery_empty = data;

    upsdebugx(1, "Battery Voltage Low: %0.3fV", data / 1000.0);
    dstate_setinfo("battery.voltage.low", "%0.3f", data / 1000.0);
}

static int set_battery_empty(const char *value)
{
    uint8_t cmd = BATTERY_EMPTY_CMD;
    int16_t data;

    upsdebugx(3, __func__);

    double voltage;
    if (str_to_double(value, &voltage, 10)) {
        data = voltage * 1000;
        if (data >= MIN_BATTERY_VOLTAGE && data <= MAX_BATTERY_VOLTAGE) {
            i2c_smbus_write_byte_data(upsfd, BATTERY_PARAM_SET_BY_USER_CMD, 0x1);
            i2c_smbus_write_byte_data(upsfd, cmd, data & 0xFF);
            i2c_smbus_write_byte_data(upsfd, cmd + 1, (data >> 8) & 0xFF);
            upsdebugx(1, "Battery Voltage Low: %0.3fV", data / 1000.0);
            dstate_setinfo("battery.voltage.low", "%0.3f", data / 1000.0);
            battery_empty = data;
            return STAT_SET_HANDLED;
        }
    }

    upsdebugx(1, "Unknown value for battery.voltage.low: %s", value);
    return STAT_SET_UNKNOWN;
}

static void get_power_off_timer(void)
{
    uint8_t cmd = SHUTDOWN_TIMER_CMD;
    uint8_t data;

    upsdebugx(3, __func__);

    I2C_READ_BYTE(upsfd, cmd, __func__)

    upsdebugx(1, "Shutdown Timer: %ds", data);
    dstate_setinfo("ups.timer.shutdown", "%d", data);
}

static int set_power_off_timer(const char *value)
{
    uint8_t cmd = SHUTDOWN_TIMER_CMD;
    short data;

    upsdebugx(3, __func__);

    if (str_to_short(value, &data, 10) && data >= 10) {
        i2c_smbus_write_byte_data(upsfd, cmd, data);
        upsdebugx(1, "Shutdown Timer: %ds", data);
        dstate_setinfo("ups.timer.shutdown", "%d", data);
        automatic_shutdown = 0;
        return STAT_SET_HANDLED;
    }

    upsdebugx(1, "Unknown value for ups.timer.shutdown: %s", value);
    return STAT_SET_UNKNOWN;
}

static void get_reboot_timer(void)
{
    uint8_t cmd = RESTART_TIMER_CMD;
    uint8_t data;

    upsdebugx(3, __func__);

    I2C_READ_BYTE(upsfd, cmd, __func__)

    upsdebugx(1, "Reboot Timer: %ds", data);
    dstate_setinfo("ups.timer.reboot", "%d", data);
}

static int set_reboot_timer(const char *value)
{
    uint8_t cmd = RESTART_TIMER_CMD;
    short data;

    upsdebugx(3, __func__);

    if (str_to_short(value, &data, 10) && data >= 10) {
        i2c_smbus_write_byte_data(upsfd, cmd, data);
        upsdebugx(1, "Reboot Timer: %ds", data);
        dstate_setinfo("ups.timer.reboot", "%d", data);
        return STAT_SET_HANDLED;
    }

    upsdebugx(1, "Unknown value for ups.timer.reboot: %s", value);
    return STAT_SET_UNKNOWN;
}

static void get_ups_auto_restart(void)
{
    uint8_t cmd = WAKEUP_ON_CHARGE_CMD;
    uint8_t data;

    upsdebugx(3, __func__);

    I2C_READ_BYTE(upsfd, cmd, __func__)

    if (data == 1) {
        upsdebugx(1, "Auto restart on external power: yes");
        dstate_setinfo("ups.start.auto", "%s", "yes");
    } else {
        upsdebugx(1, "Auto restart on external power: no");
        dstate_setinfo("ups.start.auto", "%s", "no");
    }
}

static int set_ups_auto_restart(const char *value)
{
    uint8_t cmd = WAKEUP_ON_CHARGE_CMD;

    upsdebugx(3, __func__);

    if (!strcasecmp(value, "yes")) {
        i2c_smbus_write_byte_data(upsfd, cmd, 0x1);
        dstate_setinfo("ups.start.auto", "yes");
        return STAT_SET_HANDLED;
    } else if (!strcasecmp(value, "no")) {
        i2c_smbus_write_byte_data(upsfd, cmd, 0x0);
        dstate_setinfo("ups.start.auto", "no");
        return STAT_SET_HANDLED;
    }

    upsdebugx(1, "Unknown value for ups.start.auto: %s", value);
    return STAT_SET_UNKNOWN;
}

static void get_ups_uptime(void)
{
    uint8_t cmd = UPTIME_CMD;
    __u8 block[I2C_SMBUS_BLOCK_MAX];

    upsdebugx(3, __func__);

    I2C_READ_BLOCK(upsfd, cmd, 4, block, __func__)

    upsdebugx(1, "Device uptime: %ds", (block[3] << 24 | block[2] << 16 | block[1] << 8 | block[0]));
    dstate_setinfo("device.uptime", "%d", (block[3] << 24 | block[2] << 16 | block[1] << 8 | block[0]));
}

static void check_operating_state(void)
{
    get_UsbC_Voltage();
    get_MicroUsb_Voltage();

    if (usbC_power > USB_LEVEL_THRESHOLD) {
        power_state = USBC_POWER_CONNECTED;
        upsdebugx(1, "Input Voltage: %0.3fV", usbC_power / 1000.0);
        dstate_setinfo("input.voltage.nominal", "%0.3f", USBC_NOMINAL_VOLTAGE);
        dstate_setinfo("input.voltage", "%0.3f", usbC_power / 1000.0);
    } else if (microUsb_power > USB_LEVEL_THRESHOLD) {
        power_state = MICROUSB_POWER_CONNECTED;
        upsdebugx(1, "Input Voltage: %0.3fV", microUsb_power / 1000.0);
        dstate_setinfo("input.voltage.nominal", "%0.3f", MICROUSB_NOMINAL_VOLTAGE);
        dstate_setinfo("input.voltage", "%0.3f", microUsb_power / 1000.0);
    } else {
        power_state = POWER_NOT_CONNECTED;
        upsdebugx(1, "Input Voltage: None");
        dstate_setinfo("input.voltage", "%0.3f", 0.0);
    }

/*
 * The UPSPlus does not shut down by itself based on the Protection Voltage. The UPSPlus
 * scripts manually check the current voltage and protection voltage and if the current
 * voltage is less that protection, start a 4 minute shutdown timer. So do the same here.
 */

/* If we have power, and we are shutting down and we started the automatic shutdown, then stop it */
    if (power_state != POWER_NOT_CONNECTED && automatic_shutdown == 1) {
        i2c_smbus_write_byte_data(upsfd, SHUTDOWN_TIMER_CMD, 0);
        automatic_shutdown = 0;
        upsdebugx(1, "Power connected, cancelled shutdown timer");
    }

/* If we don't have power, and we are not shutting down, then start the automatic shutdown. */
    if (power_state == POWER_NOT_CONNECTED && automatic_shutdown != 1 && battery_voltage < battery_low) {
        i2c_smbus_write_byte_data(upsfd, SHUTDOWN_TIMER_CMD, AUTO_SHUTDOWN_TIME);
        automatic_shutdown = 1;
        upsdebugx(1, "Low battery, starting shutdown timer for %d", AUTO_SHUTDOWN_TIME);
    }

/* Otherwise leave the current state because we could have started shutdown and the user stopped it. */
}

static void reset_factory()
{
    uint8_t cmd = RESET_TO_DEFAULT_CMD;

    I2C_WRITE_BYTE(upsfd, cmd, 0x1, __func__)
}

static void reset_battery()
{
    uint8_t cmd = RESET_TO_DEFAULT_CMD;

    I2C_WRITE_BYTE(upsfd, cmd, 0x0, __func__)
}

int upsplus_setvar(const char *key, const char *value)
{
    upsdebugx(2, "In %s for %s with %s...", __func__, key, value);

    if (!strcasecmp(key, "battery.voltage.high")) {
        return set_battery_full(value);
    }

    if (!strcasecmp(key, "battery.voltage.low")) {
        return set_battery_empty(value);
    }

    if (!strcasecmp(key, "battery.charge.low")) {
        return set_charge_low(value);
    }

    if (!strcasecmp(key, "ups.timer.shutdown")) {
        return set_power_off_timer(value);
    }

    if (!strcasecmp(key, "ups.timer.reboot")) {
        return set_reboot_timer(value);
    }

    if (!strcasecmp(key, "ups.start.auto")) {
        return set_ups_auto_restart(value);
    }

    return STAT_SET_UNKNOWN;
}

int upsplus_instcmd(const char *cmd, const char *reserved)
{
    upsdebugx(2, "In %s with %s and extra %s.", __func__, cmd, reserved);

    if (!strcasecmp(cmd, "load.off.delay")) {
        set_power_off_timer("10");
        return STAT_INSTCMD_HANDLED;
    }

    if (!strcasecmp(cmd, "shutdown.return")) {
        set_ups_auto_restart("yes");
        set_power_off_timer("10");
        return STAT_INSTCMD_HANDLED;
    }

    if (!strcasecmp(cmd, "shutdown.stayoff")) {
        set_ups_auto_restart("no");
        set_power_off_timer("10");
        return STAT_INSTCMD_HANDLED;
    }

    if (!strcasecmp(cmd, "shutdown.stop")) {
        i2c_smbus_write_byte_data(upsfd, RESTART_TIMER_CMD, 0x0);
        i2c_smbus_write_byte_data(upsfd, SHUTDOWN_TIMER_CMD, 0x0);
        return STAT_INSTCMD_HANDLED;
    }

    if (!strcasecmp(cmd, "shutdown.reboot.graceful")) {
        set_reboot_timer("10");
        return STAT_INSTCMD_HANDLED;
    }

    return STAT_INSTCMD_UNKNOWN;
}

void upsdrv_initinfo(void)
{

    dstate_setinfo("ups.mfr", "%s", "UPSPlus HAT");
    dstate_setinfo("ups.type", "%s", "ups");
    dstate_setinfo("ups.model", "%s", "EP-0136");

    /* note: for a transition period, these data are redundant */

    dstate_setinfo("device.mfr", "%s", "UPSPlus HAT");
    dstate_setinfo("device.type", "%s", "ups");
    dstate_setinfo("device.model", "%s", "EP-0136");

    dstate_setinfo("battery.packs", "%d", 2);
/* 18650 lithium battery are Li-ion */
    dstate_setinfo("battery.type", "%s", "Li-ion");

/* Attempt to detect the UPSPlus by reading the firmware */
/* version. */
    get_firmware_version();
    if (firmware_version == 0) {
        fatal_with_errno(-1, "Failed to find UPSPlus on i2c bus, exiting.\n");
    }
    get_serial_number();

/* Setup functions */
    upsh.setvar = upsplus_setvar;
    upsh.instcmd = upsplus_instcmd;

/* Setup editable fields */
    dstate_setinfo("battery.voltage.high", "4.5");
    dstate_setflags("battery.voltage.high", ST_FLAG_RW | ST_FLAG_NUMBER);

    dstate_setinfo("battery.voltage.low", "2.75");
    dstate_setflags("battery.voltage.low", ST_FLAG_RW | ST_FLAG_NUMBER);

    dstate_setinfo("battery.charge.low", "20");
    dstate_setflags("battery.charge.low", ST_FLAG_RW | ST_FLAG_NUMBER);

    dstate_setinfo("ups.timer.reboot", "0");
    dstate_setflags("ups.timer.reboot", ST_FLAG_RW | ST_FLAG_NUMBER);

    dstate_setinfo("ups.timer.shutdown", "0");
    dstate_setflags("ups.timer.shutdown", ST_FLAG_RW | ST_FLAG_NUMBER);

    dstate_setinfo("ups.start.auto", "yes");
    dstate_setflags("ups.start.auto", ST_FLAG_RW | ST_FLAG_STRING);
    dstate_setaux("ups.start.auto", 3);

/* Setup commands */
    dstate_addcmd("load.off.delay");/* Shutdown countdown (10) + default Auto Power up */
/* For the sake of coherence, shutdown commands will set ups.start.auto to the right value before issuing the command. */
    dstate_addcmd("shutdown.return");/* Shutdown countdown (10) +  Auto Power up ON */
    dstate_addcmd("shutdown.stayoff");/* Shutdown countdown (10) +  Auto Power up OFF */
    dstate_addcmd("shutdown.stop");/* // Shutdown countdown = 0 + Reboot countdown = 0 */
    dstate_addcmd("shutdown.reboot.graceful");/* Restart countdown (10) */
}

void upsdrv_updateinfo(void)
{
    get_battery_full();
    get_battery_empty();
    get_charge_low();
    get_battery_nominal();

    get_battery_temperature();
    get_battery_voltage();
    get_output_voltage();
    get_charge_level();
    get_realtime_battery_state();
    get_realtime_output_state();

    check_operating_state();

    get_ups_uptime();
    get_power_off_timer();
    get_reboot_timer();
    get_ups_auto_restart();

    status_init();
    get_status();
    status_commit();

    dstate_dataok();
}

void upsdrv_shutdown(void)
{
    set_ups_auto_restart("no");
    set_power_off_timer("10");
}

void upsdrv_help(void)
{
    printf("\n---------\nNOTE:\n");
    printf("By default UPSPlus appears on i2c bus 1, so port should be set to '/dev/i2c-1'.\n");
    printf("The /dev/i2c-1 device needs to be world RW permissions, aka 'sudo chmod a+rw /dev/i2c-1'.\n");
    printf("\n");
}

void upsdrv_makevartable(void)
{
    addvar(VAR_FLAG, "factoryreset", "Reset UPSPlus to factory settings and exit.");

    addvar(VAR_FLAG, "batteryreset", "Reset Battery min/max/low to automatic settings and exit.");

    addvar(VAR_VALUE, "sampleperiod", "Set number of minutes between sampling battery state (1 - 1440, default: 2)");
}

void upsdrv_initups(void)
{
/* Allow the port to be set to the keyword 'default' */
    i2c_bus_path = default_i2c_bus_path;
    if (strcmp(device_path, "default")) {
        i2c_bus_path = device_path;
    }

    upsfd = open_i2c_bus(i2c_bus_path, UPSPLUS_I2C_ADDRESS);

/* Handle commandline parameters */
    if (testvar("factoryreset")) {
        reset_factory();
        fatal_with_errno(0,"Reset to factory defaults");
    }

    if (testvar("batteryreset")) {
        reset_battery();
        fatal_with_errno(0, "Reset battery parameters to automatic");
    }

    if (getval("sampleperiod") != NULL) {
        int sampleperiod;
        if (str_to_int(getval("sampleperiod"), &sampleperiod, 10) && sampleperiod >= MIN_SAMPLE_PERIOD && sampleperiod <= MAX_SAMPLE_PERIOD) {
            i2c_smbus_write_byte_data(upsfd, BATTERY_SAMPLE_PERIOD_CMD, sampleperiod & 0xFF);
            i2c_smbus_write_byte_data(upsfd, BATTERY_SAMPLE_PERIOD_CMD + 1, (sampleperiod >> 8) & 0xFF);
            upsdebugx(1, "Updated sample period to: %d", sampleperiod);
        } else {
            upsdebugx(1, "Ignoring sampleperiod, out of range: %s", getval("sampleperiod"));
        }
    }
}

void upsdrv_cleanup(void)
{
    close(upsfd);
}
