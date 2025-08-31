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
#define OUTPUT_VOLTAGE_MINIMUM              0
#define OUTPUT_VOLTAGE_MAXIMUM              5500

#define BATTERY_VOLTAGE_CMD                 0x05
#define BATTERY_VOLTAGE_MINIMUM             0
#define BATTERY_VOLTAGE_MAXIMUM             4500

#define USBC_VOLTAGE_CMD                    0x07
#define MICROUSB_VOLTAGE_CMD                0x09
#define USB_VOLTAGE_MINIMUM                 4000
#define USB_VOLTAGE_MAXIMUM                 13500

#define BATTERY_TEMPERATURE_CMD             0x0B
#define BATTERY_TEMPERATURE_MINIMUM         -20
#define BATTERY_TEMPERATURE_MAXIMUM         65
/* Note: the forced temperature protection cannot be turned off, threshold: 65 degrees! */

#define BATTERY_FULL_CMD                    0x0D
#define BATTERY_EMPTY_CMD                   0x0F
#define BATTERY_PROTECTION_CMD              0x11/* UPS will FSD when this is hit */
#define MIN_BATTERY_VOLTAGE                 2750/* Lower will kill the battery. */
#define MAX_BATTERY_VOLTAGE                 4500

#define CHARGE_LEVEL_CMD                    0x13

#define BATTERY_SAMPLE_PERIOD_CMD           0x15
#define BATTERY_SAMPLE_PERIOD_MINIMUM       1
#define BATTERY_SAMPLE_PERIOD_MAXIMUM       1440

#define POWER_STATUS_CMD                    0x17

#define SHUTDOWN_TIMER_CMD                  0x18

#define WAKEUP_ON_CHARGE_CMD                0x19
#define WAKEUP_ON_CHARGE_DISABLE            0x0
#define WAKEUP_ON_CHARGE_ENABLE             0x1

#define RESTART_TIMER_CMD                   0x1A
#define RESET_TO_DEFAULT_CMD                0x1B

#define RUNNING_TIME_CMD                    0x1C
#define CHARGING_TIME_CMD                   0x20
#define UPTIME_CMD                          0x24

#define FIRMWARE_VERSION_CMD                0x28

#define BATTERY_PARAM_CUSTOM_CMD            0x2A
#define BATTERY_PARAM_CUSTOM_DISABLE        0x0
#define BATTERY_PARAM_CUSTOM_ENABLE         0x1

/* 0x2B - 0xEF are reserved, and don't seem to be used.
 * When new commands are added in FW updates, they are
 * added as the next available register. So to store
 * custom data, start at highest reserved register
 * and work back.
 */
 #define RESERVED_BATTERY_LOW_CHARGE_CMD    0xEF
 #define BATTERY_LOW_CHARGE_CONFIGURED      0x80
 #define BATTERY_LOW_CHARGE_MASK            0x7F

#define SERIAL_NUMBER_CMD                   0xF0

/* Bulk reading constants */
#define UPSPLUS_MEMORY_START                0x01
#define UPSPLUS_MEMORY_END                  0x2A  /* Only read up to the last useful register */
#define UPSPLUS_MEMORY_SIZE                 (UPSPLUS_MEMORY_END - UPSPLUS_MEMORY_START + 1)

/* Reserved registers that we skip */
#define RESERVED_START                      0x2B
#define RESERVED_END                        0xEF
#define SERIAL_NUMBER_END                   0xFB  /* Serial number is 12 bytes starting at 0xF0 */

/* Extended memory range if reading reserved registers */
#define EXTENDED_MEMORY_END                 0xF0
#define EXTENDED_MEMORY_SIZE                (EXTENDED_MEMORY_END - UPSPLUS_MEMORY_START + 1)

/*
 * Constants used internally
 */

#define POWER_NOT_CONNECTED                 0x0
#define USBC_POWER_CONNECTED                0x1
#define MICROUSB_POWER_CONNECTED            0x2

#define CHARGE_CURRENT_THRESHOLD            0.20
#define USBC_NOMINAL_VOLTAGE                9.00
#define MICROUSB_NOMINAL_VOLTAGE            5.00

#define MAX_LOAD                            22.5/* (5V x 4.5A) */
#define MAX_PEAK_LOAD                       40.0/* (5V x 8A) */
#define DEFAULT_BATTERY_CAPACITY_Ah         3.5
#define BATTERY_CELL_COUNT                  2
#define TIMER_MINIMUM                       10
#define SHUTDOWN_TIMER                      20
#define AUTO_SHUTDOWN_TIME                  240
#define DEFAULT_CHARGE_LOW                  10

#define DRIVER_NAME                         "UPSPlus driver"
#define DRIVER_VERSION                      "1.1"

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
 * Bulk memory buffer for efficient I2C reading
 */
static uint8_t upsplus_memory[UPSPLUS_MEMORY_SIZE];
static time_t last_memory_update = 0;
static int memory_initialized = 0; /* Track if memory has been read at least once */
static int memory_update_interval = 120; /* seconds between updates - default to 2 minutes to match battery sample period */
static int critical_update_interval = 2; /* seconds between critical updates (power status, input voltage) */
static int read_reserved_registers = 0; /* whether to try reading reserved registers */

/*
 * Flag to track state
 */
static uint8_t power_state = 0;
static uint16_t firmware_version = 0;

/*
 * These values don't change once set, so do
 * some caching. Values are in mV
 * -1 value means that it needs to be loaded from UPS
 */
static int16_t battery_full = -1;
static int16_t battery_low = -1;
static int8_t ups_auto_restart = -1;

/* Current battery voltage is used in multiple
 calculations, so cache it temporarily. mV
 */
static uint16_t battery_voltage = 0;

/*
 * Battery current is positive, charging
 * Battery current is negative, discharging
 */
static float battery_current = 0;

/* Function prototypes */
static void get_reserved_battery_low_charge(void);
static void get_mcu_voltage(void);
static void get_running_time(void);
static void get_charging_time(void);
static void get_battery_sample_period(void);
static void get_battery_empty(void);
static void get_battery_param_custom(void);
static int read_upsplus_memory(void);
static int read_critical_data(void);
static uint16_t get_memory_word(uint8_t offset);
static uint8_t get_memory_byte(uint8_t offset);

/*
 * If the battery is draining while power is connected
 * multiple times, then it is bad. If it is just a
 * once off, then it is just reading the battery charge
 * level.
 */
static time_t bad_battery_timer = 0;

/*
 * Smooth out i2c read errors by holding the most recent
 * battery charge level reading
 */
static uint16_t battery_charge_level = 0;

/*
 * Battery percentage for when to switch to LB, and
 * start automatic shutdown
 */
static uint8_t battery_charge_low = DEFAULT_CHARGE_LOW;

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

/*
 * Read the entire UPSPlus memory range into local buffer
 * This is much more efficient than individual register reads
 */
static int read_upsplus_memory(void)
{
  time_t now;
  int i, fd;
  uint8_t cmd = UPSPLUS_MEMORY_START;
  int success_count = 0;
  int memory_size = read_reserved_registers ? EXTENDED_MEMORY_SIZE : UPSPLUS_MEMORY_SIZE;
  int total_chunks = (memory_size + I2C_SMBUS_BLOCK_MAX - 1) / I2C_SMBUS_BLOCK_MAX;
  
  time(&now);
  
  /* Always update on first run or if enough time has passed */
  if (memory_initialized && (now - last_memory_update < memory_update_interval)) {
    upsdebugx(3, "Memory update not due yet. Last update: %lds ago, interval: %ds", 
              now - last_memory_update, memory_update_interval);
    return 0;
  }
  
  if (memory_initialized) {
    upsdebugx(2, "Memory update due. Last update: %lds ago, interval: %ds", 
              now - last_memory_update, memory_update_interval);
  } else {
    upsdebugx(2, "First memory read - initializing memory buffer");
  }
  
  upsdebugx(2, "Current memory update interval: %d seconds", memory_update_interval);
  
  if (read_reserved_registers) {
    upsdebugx(3, "Reading extended UPSPlus memory range (0x%02X-0x%02X) including reserved registers", 
              UPSPLUS_MEMORY_START, EXTENDED_MEMORY_END);
  } else {
    upsdebugx(3, "Reading UPSPlus memory range (0x%02X-0x%02X) - skipping reserved registers", 
              UPSPLUS_MEMORY_START, UPSPLUS_MEMORY_END);
  }
  
  fd = open_i2c_bus(i2c_bus_path, UPSPLUS_I2C_ADDRESS);
  if (fd < 0) {
    return -1;
  }
  
  /* Read memory in chunks since I2C block read has limitations */
  for (i = 0; i < memory_size; i += I2C_SMBUS_BLOCK_MAX) {
    int chunk_size = (i + I2C_SMBUS_BLOCK_MAX <= memory_size) ? 
                     I2C_SMBUS_BLOCK_MAX : memory_size - i;
    
    if (i2c_smbus_read_i2c_block_data(fd, cmd + i, chunk_size, 
                                      &upsplus_memory[i]) < 0) {
      upsdebugx(2, "Failed to read memory chunk starting at 0x%02X, filling with zeros", cmd + i);
      /* Fill failed chunk with zeros instead of failing completely */
      memset(&upsplus_memory[i], 0, chunk_size);
    } else {
      success_count++;
    }
  }
  
  close(fd);
  
  if (success_count > 0) {
    last_memory_update = now;
    memory_initialized = 1; /* Mark that we've successfully read memory at least once */
    upsdebugx(2, "Successfully read %d/%d chunks of UPSPlus memory", success_count, total_chunks);
    
    /* Log which registers we successfully read */
    if (read_reserved_registers) {
      upsdebugx(3, "Successfully read registers: 0x%02X-0x%02X", 
                UPSPLUS_MEMORY_START, EXTENDED_MEMORY_END);
      upsdebugx(3, "Memory buffer contains: %d bytes of data", memory_size);
    } else {
      upsdebugx(3, "Successfully read registers: 0x%02X-0x%02X", 
                UPSPLUS_MEMORY_START, UPSPLUS_MEMORY_END);
      upsdebugx(3, "Memory buffer contains: %d bytes of data", memory_size);
    }
    
    return 0; /* Success even with partial reads */
  } else {
    upsdebugx(1, "Failed to read any memory chunks");
    return -1; /* Only fail if no chunks were read */
  }
}

/*
 * Read only critical real-time data that can change at any moment
 * This includes power status, input voltage, and basic UPS state
 */
static int read_critical_data(void)
{
  time_t now;
  int fd;
  uint8_t data;
  
  time(&now);
  
  /* Only update if enough time has passed since last critical update */
  static time_t last_critical_update = 0;
  if (now - last_critical_update < critical_update_interval) {
    return 0;
  }
  
  upsdebugx(2, "Reading critical real-time data (power status, input voltage, battery state)");
  
  fd = open_i2c_bus(i2c_bus_path, UPSPLUS_I2C_ADDRESS);
  if (fd < 0) {
    return -1;
  }
  
  /* Read power status */
  if (i2c_smbus_read_byte_data(fd, POWER_STATUS_CMD) >= 0) {
    data = i2c_smbus_read_byte_data(fd, POWER_STATUS_CMD);
    upsdebugx(3, "Critical update: Power status: 0x%02X", data);
  }
  
  /* Read USB-C voltage */
  if (i2c_smbus_read_word_data(fd, USBC_VOLTAGE_CMD) >= 0) {
    uint16_t voltage = i2c_smbus_read_word_data(fd, USBC_VOLTAGE_CMD);
    usbC_power = voltage;
    upsdebugx(3, "Critical update: USB-C voltage: %0.3fV", voltage / 1000.0);
  }
  
  /* Read MicroUSB voltage */
  if (i2c_smbus_read_word_data(fd, MICROUSB_VOLTAGE_CMD) >= 0) {
    uint16_t voltage = i2c_smbus_read_word_data(fd, MICROUSB_VOLTAGE_CMD);
    microUsb_power = voltage;
    upsdebugx(3, "Critical update: MicroUSB voltage: %0.3fV", voltage / 1000.0);
  }
  
  /* Read battery voltage - critical for status determination */
  if (i2c_smbus_read_word_data(fd, BATTERY_VOLTAGE_CMD) >= 0) {
    int16_t voltage = i2c_smbus_read_word_data(fd, BATTERY_VOLTAGE_CMD);
    battery_voltage = voltage;
    upsdebugx(3, "Critical update: Battery voltage: %0.3fV", voltage / 1000.0);
  }
  
  /* Read battery charge level - critical for status determination */
  if (i2c_smbus_read_word_data(fd, CHARGE_LEVEL_CMD) >= 0) {
    uint16_t charge = i2c_smbus_read_word_data(fd, CHARGE_LEVEL_CMD);
    battery_charge_level = charge;
    upsdebugx(3, "Critical update: Battery charge: %d%%", charge);
  }
  
  /* Read output voltage - important for load status */
  if (i2c_smbus_read_word_data(fd, OUTPUT_VOLTAGE_CMD) >= 0) {
    uint16_t voltage = i2c_smbus_read_word_data(fd, OUTPUT_VOLTAGE_CMD);
    upsdebugx(3, "Critical update: Output voltage: %0.3fV", voltage / 1000.0);
  }
  
  close(fd);
  
  /* Read battery current from INA219 - critical for charging/discharging status */
  int battery_fd = open_i2c_bus(i2c_bus_path, INA219_BATTERY_I2C_ADDRESS);
  if (battery_fd >= 0) {
    /* Configure INA219 */
    i2c_smbus_write_word_data(battery_fd, INA219_CONFIGURATION_CMD, INA219_CONFIGURATION_VALUE);
    i2c_smbus_write_word_data(battery_fd, INA219_CALIBRATION_CMD, INA219_CALIBRATION_VALUE_MAGIC);
    
    /* Read current */
    int16_t current_data = i2c_smbus_read_word_data(battery_fd, INA219_CURRENT_CMD);
    if (current_data >= 0) {
      /* Swap bytes for INA219 */
      current_data = (current_data >> 8) | (current_data << 8);
      battery_current = (int16_t)current_data * BATTERY_CURRENT_LSB_MAGIC;
      upsdebugx(3, "Critical update: Battery current: %0.3fA", battery_current);
    }
    close(battery_fd);
  }
  
  last_critical_update = now;
  upsdebugx(2, "Critical data updated successfully");
  
  return 0;
}

/*
 * Helper function to read a word (16-bit) value from memory buffer
 */
static uint16_t get_memory_word(uint8_t offset)
{
  int memory_size = read_reserved_registers ? EXTENDED_MEMORY_SIZE : UPSPLUS_MEMORY_SIZE;
  
  if (offset + 1 >= memory_size) {
    upsdebugx(3, "Memory read offset 0x%02X out of range (max: 0x%02X)", offset, memory_size - 1);
    return 0;
  }
  return (upsplus_memory[offset + 1] << 8) | upsplus_memory[offset];
}

/*
 * Helper function to read a byte value from memory buffer
 */
static uint8_t get_memory_byte(uint8_t offset)
{
  int memory_size = read_reserved_registers ? EXTENDED_MEMORY_SIZE : UPSPLUS_MEMORY_SIZE;
  
  if (offset >= memory_size) {
    upsdebugx(3, "Memory read offset 0x%02X out of range (max: 0x%02X)", offset, memory_size - 1);
    return 0;
  }
  return upsplus_memory[offset];
}

static void get_charge_level(void)
{
  uint16_t data;
  
  upsdebugx(3, __func__);
  
  /* Read from memory buffer instead of I2C */
  data = get_memory_word(CHARGE_LEVEL_CMD - UPSPLUS_MEMORY_START);
  if (data == 0 && last_memory_update == 0) {
    /* Fallback to direct I2C if memory hasn't been read yet */
    upsdebugx(3, "Memory not available, falling back to direct I2C read for charge level");
    I2C_READ_WORD(upsfd, CHARGE_LEVEL_CMD, __func__)
  } else {
    battery_charge_level = data;
    upsdebugx(3, "Read charge level from memory buffer: %d%%", data);
  }
  
  upsdebugx(1, "Battery Charge Level: %d%%", battery_charge_level);
  if (battery_charge_level < 110) {
    dstate_setinfo("battery.charge", "%d", battery_charge_level);
  } else {
    dstate_setinfo("battery.charge", "%d", battery_charge_level);
    upsdebugx(2, "Battery Charge Level out of range, skipping");
  }
}

static void get_output_voltage(void)
{
  uint16_t data;
  
  upsdebugx(3, __func__);
  
  /* Read from memory buffer instead of I2C */
  data = get_memory_word(OUTPUT_VOLTAGE_CMD - UPSPLUS_MEMORY_START);
  if (data == 0 && last_memory_update == 0) {
    /* Fallback to direct I2C if memory hasn't been read yet */
    I2C_READ_WORD(upsfd, OUTPUT_VOLTAGE_CMD, __func__)
  }
  
  upsdebugx(1, "Output voltage: %0.3fV", data / 1000.0);
  if (data > OUTPUT_VOLTAGE_MINIMUM || data < OUTPUT_VOLTAGE_MAXIMUM) {
    dstate_setinfo("output.voltage", "%0.3f", data / 1000.0);
  } else {
    upsdebugx(2, "Output voltage out of range, skipping");
  }
}

static void get_battery_full(void)
{
  int16_t data = battery_full;
  
  upsdebugx(3, __func__);
  
  if (battery_full < 0) {
    /* Read from memory buffer instead of I2C */
    data = get_memory_word(BATTERY_FULL_CMD - UPSPLUS_MEMORY_START);
    if (data == 0 && last_memory_update == 0) {
      /* Fallback to direct I2C if memory hasn't been read yet */
      I2C_READ_WORD(upsfd, BATTERY_FULL_CMD, __func__)
    }
    battery_full = data;
  }
  
  upsdebugx(1, "Battery Voltage High: %0.3fV", data / 1000.0);
  if (data >= BATTERY_VOLTAGE_MINIMUM && data <= BATTERY_VOLTAGE_MAXIMUM) {
    dstate_setinfo("battery.voltage.high", "%0.3f", data / 1000.0);
  } else {
    upsdebugx(2, "Battery Voltage High out of range, skipping");
  }
}

static void set_battery_full(uint16_t data)
{
  uint8_t cmd = BATTERY_FULL_CMD;
  
  upsdebugx(3, __func__);
  
  I2C_WRITE_BYTE(upsfd, BATTERY_PARAM_CUSTOM_CMD,
                 BATTERY_PARAM_CUSTOM_ENABLE, __func__)
  I2C_WRITE_BYTE(upsfd, cmd, data & 0xFF, __func__)
  I2C_WRITE_BYTE(upsfd, cmd + 1, (data >> 8) & 0xFF, __func__)

  battery_full = -1;
  
  upsdebugx(1, "Battery Voltage High: %0.3fV", data / 1000.0);
  dstate_setinfo("battery.voltage.high", "%0.3f", data / 1000.0);
}

static void get_battery_low(void)
{
  int16_t data = battery_low;
  
  upsdebugx(3, __func__);
  
  if (battery_low < 0) {
    /* Read from memory buffer instead of I2C */
    data = get_memory_word(BATTERY_PROTECTION_CMD - UPSPLUS_MEMORY_START);
    if (data == 0 && last_memory_update == 0) {
      /* Fallback to direct I2C if memory hasn't been read yet */
      I2C_READ_WORD(upsfd, BATTERY_PROTECTION_CMD, __func__)
    }
    battery_low = data;
  }
  
  upsdebugx(1, "Battery Voltage Low: %0.3fV", data / 1000.0);
  if (data >= BATTERY_VOLTAGE_MINIMUM && data <= BATTERY_VOLTAGE_MAXIMUM) {
    dstate_setinfo("battery.voltage.low", "%0.3f", data / 1000.0);
  } else {
    upsdebugx(2, "Battery Voltage Low out of range, skipping");
  }
}

static void set_battery_empty(uint16_t data)
{
  uint8_t cmd = BATTERY_EMPTY_CMD;
  
  upsdebugx(3, __func__);
  
  I2C_WRITE_BYTE(upsfd, BATTERY_PARAM_CUSTOM_CMD,
                 BATTERY_PARAM_CUSTOM_ENABLE, __func__)
  I2C_WRITE_BYTE(upsfd, cmd, data & 0xFF, __func__)
  I2C_WRITE_BYTE(upsfd, cmd + 1, (data >> 8) & 0xFF, __func__)
  
  upsdebugx(1, "Battery Voltage Empty: %0.3fV", data / 1000.0);
  //dstate_setinfo("battery.voltage.empty", "%0.3f", data / 1000.0);
}

static void set_battery_low(uint16_t data)
{
  uint8_t cmd = BATTERY_PROTECTION_CMD;
  
  upsdebugx(3, __func__);
  
  I2C_WRITE_BYTE(upsfd, BATTERY_PARAM_CUSTOM_CMD,
                 BATTERY_PARAM_CUSTOM_ENABLE, __func__)
  I2C_WRITE_BYTE(upsfd, cmd, data & 0xFF, __func__)
  I2C_WRITE_BYTE(upsfd, cmd + 1, (data >> 8) & 0xFF, __func__)
  
  battery_low = -1;
  
  upsdebugx(1, "Battery Voltage Low: %0.3fV", data / 1000.0);
  dstate_setinfo("battery.voltage.low", "%0.3f", data / 1000.0);
  
  /* Set the Empty voltage to be the same as the low/FSD voltage */
  set_battery_empty(data);
}


static void get_charge_low(void)
{
  /* Use the separate function for reserved registers */
  get_reserved_battery_low_charge();
}

static void set_charge_low(int16_t data)
{
  uint8_t cmd = RESERVED_BATTERY_LOW_CHARGE_CMD;
  
  upsdebugx(3, __func__);
  
  if (data < 0 || data > 100) {
    return;
  }
  
  battery_charge_low = data;
  
  data |= BATTERY_LOW_CHARGE_CONFIGURED;
  
  I2C_WRITE_BYTE(upsfd, cmd, data, __func__)

  upsdebugx(1, "Low Charge Threshold: %d%%", battery_charge_low);
  dstate_setinfo("battery.charge.low", "%d", battery_charge_low);
}





static void get_status(void)
{
  uint8_t data;
  char status_buf[ST_MAX_VALUE_LEN];
  time_t now;
  
  upsdebugx(3, __func__);
  
  memset(status_buf, 0, ST_MAX_VALUE_LEN);
  
  /* Use critical data that was already read, or fall back to memory buffer */
  if (last_memory_update > 0) {
    data = get_memory_byte(POWER_STATUS_CMD - UPSPLUS_MEMORY_START);
    if (data == 0) {
      /* Fallback to direct I2C if memory data is invalid */
      I2C_READ_BYTE(upsfd, POWER_STATUS_CMD, __func__)
    }
  } else {
    /* Fallback to direct I2C if memory hasn't been read yet */
    I2C_READ_BYTE(upsfd, POWER_STATUS_CMD, __func__)
  }
  
  if (data == 1) {
    upsdebugx(1, "Power status: normal");
  } else {
    upsdebugx(1, "Power status: off");
    status_set("OFF");
    /* If we are not running, then set no other Status. */
    return;
  }
  
  if (power_state == POWER_NOT_CONNECTED) {
    status_set("OB");
  } else {
    status_set("OL");
  }
  
  /* Battery low/full values are cached and don't change frequently */
  get_battery_low();
  get_battery_full();

  /* Check for 60secs of discharging on power */
  time(&now);
  if (battery_voltage == 0 || (bad_battery_timer && now - bad_battery_timer > 60)) {
    upsdebugx(1, "Battery Status: Replace");
    status_set("RB");
  } else if (battery_charge_level < battery_charge_low) {
    upsdebugx(1, "Battery Status: Low");
    status_set("LB");
  } else if (battery_voltage > (1.2 * battery_full)) {
    upsdebugx(1, "Battery Status: High");
    status_set("HB");
  }
  
    /* If we are discharging while power is connected for
   * 1 minute, then batteries are bad.
   * Need to check for multiple times discharging
   * because when the UPS circuitry calibrates, it samples
   * the battery which drains it.
   */
  if (battery_current < 0 && power_state != POWER_NOT_CONNECTED) {
    if (!bad_battery_timer) {
      time(&bad_battery_timer);
    }
    upsdebugx(1, "Battery Status: Calibrating");
    status_set("CAL");
  } else {
    bad_battery_timer = 0;
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
  int16_t data;
  
  upsdebugx(3, __func__);
  
  /* Read from memory buffer instead of I2C */
  data = get_memory_word(BATTERY_TEMPERATURE_CMD - UPSPLUS_MEMORY_START);
  if (data == 0 && last_memory_update == 0) {
    /* Fallback to direct I2C if memory hasn't been read yet */
    I2C_READ_WORD(upsfd, BATTERY_TEMPERATURE_CMD, __func__)
  }
  
  upsdebugx(1, "Battery Temperature: %d°C", data);
  if (data >=  BATTERY_TEMPERATURE_MINIMUM &&
      data <= BATTERY_TEMPERATURE_MAXIMUM) {
    dstate_setinfo("battery.temperature", "%d", data);
  } else {
    upsdebugx(2, "Battery Temperature out of range, skipping");
  }
}

static void get_battery_voltage(void)
{
  int16_t data = battery_voltage;
  
  upsdebugx(3, __func__);
  
  /* Read from memory buffer instead of I2C */
  data = get_memory_word(BATTERY_VOLTAGE_CMD - UPSPLUS_MEMORY_START);
  if (data == 0 && last_memory_update == 0) {
    /* Fallback to direct I2C if memory hasn't been read yet */
    I2C_READ_WORD(upsfd, BATTERY_VOLTAGE_CMD, __func__)
  }
  battery_voltage = data;
  
  upsdebugx(1, "Battery Voltage: %0.3fV", data / 1000.0);
  if (data >= BATTERY_VOLTAGE_MINIMUM && data <= BATTERY_VOLTAGE_MAXIMUM) {
    dstate_setinfo("battery.voltage", "%0.3f", data / 1000.0);
  } else {
    upsdebugx(2, "Battery Voltage out of range, skipping");
  }
}

static void estimate_battery_runtime(float power_consumption)
{
  float total_battery_capacity;
  float remaining_energy;
  
  upsdebugx(3, __func__);

  get_battery_full();
  
  total_battery_capacity = (battery_full / 1000.0) * DEFAULT_BATTERY_CAPACITY_Ah * BATTERY_CELL_COUNT;
  remaining_energy = total_battery_capacity * (float)battery_charge_level / 100.0;
  
  upsdebugx(1, "Battery runtime: %ds", (int)(remaining_energy * 60.0 * 60.0 / power_consumption));
  dstate_setinfo("battery.runtime", "%d", (int)(remaining_energy * 60.0 * 60.0 / power_consumption));
  
  return ;
}

static void get_realtime_output_state(void)
{
  uint16_t data;
  
  upsdebugx(3, __func__);

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
  
  if (attempt == 0) {
    upsdebugx(1, "INA219 Output Voltage value not ready");
    close(extrafd);
    extrafd = 0;
    return;
  }
  
  data >>= 3;    /* Bits 3-15 */
  data *= 4;    /* LSB 4mV */
  upsdebugx(1, "INA219 Output Voltage: %0.3fV", data / 1000.0);
  if (data > OUTPUT_VOLTAGE_MINIMUM || data < OUTPUT_VOLTAGE_MAXIMUM) {
    dstate_setinfo("output.voltage", "%0.3f", data / 1000.0);
  } else {
    upsdebugx(2, "Output voltage out of range, skipping");
  }
  
  I2C_READ_WORD_INA219(extrafd, INA219_POWER_CMD, __func__)
  upsdebugx(1, "INA219 Output Power: %0.3fW", data * OUTPUT_POWER_LSB_MAGIC);
    // Apparent Power and Real Power are the same for this DC UPS
  dstate_setinfo("ups.realpower", "%0.3f", data * OUTPUT_POWER_LSB_MAGIC);
  dstate_setinfo("ups.power", "%0.3f", data * OUTPUT_POWER_LSB_MAGIC);
  upsdebugx(1, "UPS Load: %0.3f%%", 100 * data * OUTPUT_POWER_LSB_MAGIC / MAX_LOAD);
  dstate_setinfo("ups.load", "%0.3f", 100 * data * OUTPUT_POWER_LSB_MAGIC / MAX_LOAD);
  
  // If charging, estimate time based on output power
  if (power_state != POWER_NOT_CONNECTED) {
    estimate_battery_runtime(data * OUTPUT_POWER_LSB_MAGIC);
  }
  
  I2C_READ_WORD_INA219(extrafd, INA219_CURRENT_CMD, __func__)
  /* Current is a signed 16bit number */
  upsdebugx(1, "INA219 Output Current: %0.3fA", (int16_t) data * OUTPUT_CURRENT_LSB_MAGIC);
  dstate_setinfo("output.current", "%0.3f", (int16_t) data * OUTPUT_CURRENT_LSB_MAGIC);
  
  close(extrafd);
  extrafd = 0;
}

static void get_realtime_battery_state(void)
{
  int16_t data = 0;
  
  upsdebugx(3, __func__);

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
  
  if (attempt == 0) {
    upsdebugx(1, "INA219 Battery Voltage value not ready");
    close(extrafd);
    extrafd = 0;
    return;
  }
  
  data >>= 3;    /* Bits 3-15 */
  data *= 4;    /* LSB 4mV */
  upsdebugx(1, "INA219 Battery Voltage: %0.3fV", data / 1000.0);
  if (data >= BATTERY_VOLTAGE_MINIMUM && data <= BATTERY_VOLTAGE_MAXIMUM) {
    battery_voltage = data;
    dstate_setinfo("battery.voltage", "%0.3f", data / 1000.0);
  } else {
    upsdebugx(2, "INA219 Battery Voltage out of range, skipping");
  }
  
  I2C_READ_WORD_INA219(extrafd, INA219_POWER_CMD, __func__)
  upsdebugx(1, "INA219 Battery Power: %0.3fW", data * BATTERY_POWER_LSB_MAGIC);
  /* dstate_setinfo( "battery.power", "%0.3f", data * BATTERY_POWER_LSB_MAGIC ); */
  
  // If discharging, estimate time based on battery power
  if (power_state == POWER_NOT_CONNECTED) {
    estimate_battery_runtime(data * BATTERY_POWER_LSB_MAGIC);
  }
  
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
  uint16_t data = 0;
  
  upsdebugx(3, __func__);
  
  /* Read from memory buffer instead of I2C */
  data = get_memory_word(FIRMWARE_VERSION_CMD - UPSPLUS_MEMORY_START);
  if (data == 0 && last_memory_update == 0) {
    /* Fallback to direct I2C if memory hasn't been read yet */
    I2C_READ_WORD(upsfd, FIRMWARE_VERSION_CMD, __func__)
  }
  
  firmware_version = data;
  
  upsdebugx(1, "UPS Firmware Version: %d", data);
  dstate_setinfo("ups.firmware", "%d", data);
}

static void get_serial_number(void)
{
  __u8 block[12];
  char serial_number[LENGTH_TEMP];
  
  upsdebugx(3, __func__);
  
  /* Read from memory buffer instead of I2C */
  if (last_memory_update > 0) {
    /* Read 12 bytes from memory buffer */
    if (SERIAL_NUMBER_CMD - UPSPLUS_MEMORY_START + 11 < UPSPLUS_MEMORY_SIZE) {
      memcpy(block, &upsplus_memory[SERIAL_NUMBER_CMD - UPSPLUS_MEMORY_START], 12);
    } else {
      memset(block, 0, 12);
    }
  } else {
    /* Fallback to direct I2C if memory hasn't been read yet */
    I2C_READ_BLOCK(upsfd, SERIAL_NUMBER_CMD, 12, block, __func__)
  }
  
  snprintf(serial_number, sizeof(serial_number), "%08X-%08X-%08X",
           (block[3] << 24 | block[2] << 16 | block[1] << 8 | block[0]),
           (block[7] << 24 | block[6] << 16 | block[5] << 8 | block[4]),
           (block[11] << 24 | block[10] << 16 | block[9] << 8 | block[8]));
  
  upsdebugx(1, "Serial Number: %s", serial_number);
  dstate_setinfo("device.serial", "%s", serial_number);
}

static void get_battery_nominal(void)
{
  int16_t data;
  
  upsdebugx(3, __func__);
  
  /*
   * Supported Batteries are either 4.2V, 4.35V, 4.4V or 4.5V
   * So calculate it based on 'Full' battery value
   */
  get_battery_full();
  
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

/*
 * Read MCU voltage from memory buffer
 */
static void get_mcu_voltage(void)
{
  uint16_t data;
  
  upsdebugx(3, __func__);
  
  /* Read from memory buffer instead of I2C */
  data = get_memory_word(MCU_VOLTAGE_CMD - UPSPLUS_MEMORY_START);
  if (data == 0 && last_memory_update == 0) {
    /* Fallback to direct I2C if memory hasn't been read yet */
    I2C_READ_WORD(upsfd, MCU_VOLTAGE_CMD, __func__)
  }
  
  upsdebugx(1, "MCU Voltage: %0.3fV", data / 1000.0);
  if (data > 0 && data < 10000) { /* Reasonable range 0-10V */
    dstate_setinfo("ups.mcu.voltage", "%0.3f", data / 1000.0);
  }
}

/*
 * Read running time from memory buffer
 */
static void get_running_time(void)
{
  uint32_t running_time;
  
  upsdebugx(3, __func__);
  
  /* Read from memory buffer instead of I2C */
  if (last_memory_update > 0) {
    /* Read 4 bytes from memory buffer */
    if (RUNNING_TIME_CMD - UPSPLUS_MEMORY_START + 3 < UPSPLUS_MEMORY_SIZE) {
      running_time = (upsplus_memory[RUNNING_TIME_CMD - UPSPLUS_MEMORY_START + 3] << 24) |
                     (upsplus_memory[RUNNING_TIME_CMD - UPSPLUS_MEMORY_START + 2] << 16) |
                     (upsplus_memory[RUNNING_TIME_CMD - UPSPLUS_MEMORY_START + 1] << 8) |
                     upsplus_memory[RUNNING_TIME_CMD - UPSPLUS_MEMORY_START];
    } else {
      running_time = 0;
    }
  } else {
    /* Fallback to direct I2C if memory hasn't been read yet */
    __u8 block[I2C_SMBUS_BLOCK_MAX];
    I2C_READ_BLOCK(upsfd, RUNNING_TIME_CMD, 4, block, __func__)
    running_time = (block[3] << 24) | (block[2] << 16) | (block[1] << 8) | block[0];
  }
  
  upsdebugx(1, "Running time: %ds", running_time);
  dstate_setinfo("ups.runtime", "%d", running_time);
}

/*
 * Read charging time from memory buffer
 */
static void get_charging_time(void)
{
  uint32_t charging_time;
  
  upsdebugx(3, __func__);
  
  /* Read from memory buffer instead of I2C */
  if (last_memory_update == 0) {
    /* Fallback to direct I2C if memory hasn't been read yet */
    __u8 block[I2C_SMBUS_BLOCK_MAX];
    I2C_READ_BLOCK(upsfd, CHARGING_TIME_CMD, 4, block, __func__)
    charging_time = (block[3] << 24) | (block[2] << 16) | (block[1] << 8) | block[0];
  } else {
    /* Read 4 bytes from memory buffer */
    if (CHARGING_TIME_CMD - UPSPLUS_MEMORY_START + 3 < UPSPLUS_MEMORY_SIZE) {
      charging_time = (upsplus_memory[CHARGING_TIME_CMD - UPSPLUS_MEMORY_START + 3] << 24) |
                      (upsplus_memory[CHARGING_TIME_CMD - UPSPLUS_MEMORY_START + 2] << 16) |
                      (upsplus_memory[CHARGING_TIME_CMD - UPSPLUS_MEMORY_START + 1] << 8) |
                      upsplus_memory[CHARGING_TIME_CMD - UPSPLUS_MEMORY_START];
    } else {
      charging_time = 0;
    }
  }
  
  upsdebugx(1, "Charging time: %ds", charging_time);
  dstate_setinfo("battery.charging.time", "%d", charging_time);
}

/*
 * Read battery sample period from memory buffer
 */
static void get_battery_sample_period(void)
{
  uint16_t data;
  
  upsdebugx(3, __func__);
  
  /* Read from memory buffer instead of I2C */
  data = get_memory_word(BATTERY_SAMPLE_PERIOD_CMD - UPSPLUS_MEMORY_START);
  if (data == 0 && last_memory_update == 0) {
    /* Fallback to direct I2C if memory hasn't been read yet */
    I2C_READ_WORD(upsfd, BATTERY_SAMPLE_PERIOD_CMD, __func__)
  }
  
  upsdebugx(1, "Battery sample period: %d minutes", data);
  if (data >= BATTERY_SAMPLE_PERIOD_MINIMUM && data <= BATTERY_SAMPLE_PERIOD_MAXIMUM) {
    dstate_setinfo("battery.sample.period", "%d", data);
    
    /* Align memory update interval with battery sample period for efficiency */
    int new_interval = data * 60; /* Convert minutes to seconds */
    
    /* Ensure minimum reasonable interval to avoid excessive I2C traffic */
    if (new_interval < 30) {
      new_interval = 30;
      upsdebugx(2, "Adjusted memory update interval to minimum 30 seconds (requested: %d seconds)", data * 60);
    }
    
    if (new_interval != memory_update_interval) {
      upsdebugx(2, "Adjusting memory update interval from %d to %d seconds to match battery sample period", 
                memory_update_interval, new_interval);
      memory_update_interval = new_interval;
    }
  }
}

/*
 * Read battery empty voltage from memory buffer
 */
static void get_battery_empty(void)
{
  int16_t data;
  
  upsdebugx(3, __func__);
  
  /* Read from memory buffer instead of I2C */
  data = get_memory_word(BATTERY_EMPTY_CMD - UPSPLUS_MEMORY_START);
  if (data == 0 && last_memory_update == 0) {
    /* Fallback to direct I2C if memory hasn't been read yet */
    I2C_READ_WORD(upsfd, BATTERY_EMPTY_CMD, __func__)
  }
  
  upsdebugx(1, "Battery Voltage Empty: %0.3fV", data / 1000.0);
  if (data >= BATTERY_VOLTAGE_MINIMUM && data <= BATTERY_VOLTAGE_MAXIMUM) {
    dstate_setinfo("battery.voltage.empty", "%0.3f", data / 1000.0);
  }
}

/*
 * Read battery parameter custom setting from memory buffer
 */
static void get_battery_param_custom(void)
{
  uint8_t data;
  
  upsdebugx(3, __func__);
  
  /* Read from memory buffer instead of I2C */
  data = get_memory_byte(BATTERY_PARAM_CUSTOM_CMD - UPSPLUS_MEMORY_START);
  if (data == 0 && last_memory_update == 0) {
    /* Fallback to direct I2C if memory hasn't been read yet */
    I2C_READ_BYTE(upsfd, BATTERY_PARAM_CUSTOM_CMD, __func__)
  }
  
  upsdebugx(1, "Battery parameter custom: %s", 
            data == BATTERY_PARAM_CUSTOM_ENABLE ? "enabled" : "disabled");
  dstate_setinfo("battery.param.custom", "%s", 
                 data == BATTERY_PARAM_CUSTOM_ENABLE ? "enabled" : "disabled");
}

/*
 * Read the reserved battery low charge register separately
 * since it's outside our main memory range
 */
static void get_reserved_battery_low_charge(void)
{
  uint16_t data;
  
  upsdebugx(3, __func__);
  
  /* This register is outside our main memory range, so always read directly */
  I2C_READ_WORD(upsfd, RESERVED_BATTERY_LOW_CHARGE_CMD, __func__)
  
  if (data & BATTERY_LOW_CHARGE_CONFIGURED) {
    upsdebugx(3, "Found Low Charge Threshold in Reserved Register");
    if ((data & BATTERY_LOW_CHARGE_MASK) < 100) {
      upsdebugx(3, "Low Charge Threshold is within range");
      battery_charge_low = (data & BATTERY_LOW_CHARGE_MASK);
    }
  }
  
  upsdebugx(1, "Low Charge Threshold: %d%%", battery_charge_low);
  if (battery_charge_low <= 100) {
    dstate_setinfo("battery.charge.low", "%d", battery_charge_low);
  } else {
    dstate_setinfo("battery.charge.low", "%d", battery_charge_low);
    upsdebugx(2, "Low Charge Threshold out of range, skipping");
  }
}

static void reset_shutdown_restart_timers(void)
{
  upsdebugx(3, __func__);
  
  I2C_WRITE_BYTE(upsfd, RESTART_TIMER_CMD, 0x0, __func__)
  I2C_WRITE_BYTE(upsfd, SHUTDOWN_TIMER_CMD, 0x0, __func__)
}

static void get_power_off_timer(void)
{
  uint8_t data;
  
  upsdebugx(3, __func__);
  
  /* Read from memory buffer instead of I2C */
  data = get_memory_byte(SHUTDOWN_TIMER_CMD - UPSPLUS_MEMORY_START);
  if (data == 0 && last_memory_update == 0) {
    /* Fallback to direct I2C if memory hasn't been read yet */
    I2C_READ_BYTE(upsfd, SHUTDOWN_TIMER_CMD, __func__)
  }
  
  upsdebugx(1, "Shutdown Timer: %ds", data);
  dstate_setinfo("ups.timer.shutdown", "%d", data);
}

static void set_power_off_timer(const short data)
{
  uint8_t cmd = SHUTDOWN_TIMER_CMD;
  
  upsdebugx(3, __func__);
  
  reset_shutdown_restart_timers();
  I2C_WRITE_BYTE(upsfd, cmd, data, __func__)
  
  upsdebugx(1, "Set Shutdown Timer: %ds", data);
  dstate_setinfo("ups.timer.shutdown", "%d", data);
}

static void get_reboot_timer(void)
{
  uint8_t data;
  
  upsdebugx(3, __func__);
  
  /* Read from memory buffer instead of I2C */
  data = get_memory_byte(RESTART_TIMER_CMD - UPSPLUS_MEMORY_START);
  if (data == 0 && last_memory_update == 0) {
    /* Fallback to direct I2C if memory hasn't been read yet */
    I2C_READ_BYTE(upsfd, RESTART_TIMER_CMD, __func__)
  }
  
  upsdebugx(1, "Reboot Timer: %ds", data);
  dstate_setinfo("ups.timer.reboot", "%d", data);
}

static void set_reboot_timer(const short data)
{
  uint8_t cmd = RESTART_TIMER_CMD;
  
  upsdebugx(3, __func__);
  
  reset_shutdown_restart_timers();
  I2C_WRITE_BYTE(upsfd, cmd, data, __func__)
  
  upsdebugx(1, "Set Reboot Timer: %ds", data);
  dstate_setinfo("ups.timer.reboot", "%d", data);
}

static void get_ups_auto_restart(void)
{
  uint8_t data = ups_auto_restart;
  
  if (ups_auto_restart < 0) {
    upsdebugx(3, __func__);
    /* Read from memory buffer instead of I2C */
    data = get_memory_byte(WAKEUP_ON_CHARGE_CMD - UPSPLUS_MEMORY_START);
    if (data == 0 && last_memory_update == 0) {
      /* Fallback to direct I2C if memory hasn't been read yet */
      I2C_READ_BYTE(upsfd, WAKEUP_ON_CHARGE_CMD, __func__)
    }
  }
  
  if (data == WAKEUP_ON_CHARGE_ENABLE) {
    upsdebugx(1, "Auto restart on external power: yes");
    dstate_setinfo("ups.start.auto", "%s", "yes");
  } else {
    upsdebugx(1, "Auto restart on external power: no");
    dstate_setinfo("ups.start.auto", "%s", "no");
  }
}

static void set_ups_auto_restart(const short data)
{
  uint8_t cmd = WAKEUP_ON_CHARGE_CMD;
  
  upsdebugx(3, __func__);
  
  I2C_WRITE_BYTE(upsfd, cmd, data, __func__)
  
  if (data == WAKEUP_ON_CHARGE_ENABLE) {
    upsdebugx(1, "Set auto restart on external power: yes");
    dstate_setinfo("ups.start.auto", "yes");
  } else {
    upsdebugx(1, "Set auto restart on external power: no");
    dstate_setinfo("ups.start.auto", "no");
  }
  
  ups_auto_restart = -1;
}

static void get_ups_uptime(void)
{
  uint32_t uptime;
  
  upsdebugx(3, __func__);
  
  /* Read from memory buffer instead of I2C */
  if (last_memory_update > 0) {
    /* Read 4 bytes from memory buffer */
    if (UPTIME_CMD - UPSPLUS_MEMORY_START + 3 < UPSPLUS_MEMORY_SIZE) {
      uptime = (upsplus_memory[UPTIME_CMD - UPSPLUS_MEMORY_START + 3] << 24) |
               (upsplus_memory[UPTIME_CMD - UPSPLUS_MEMORY_START + 2] << 16) |
               (upsplus_memory[UPTIME_CMD - UPSPLUS_MEMORY_START + 1] << 8) |
               upsplus_memory[UPTIME_CMD - UPSPLUS_MEMORY_START];
    } else {
      uptime = 0;
    }
  } else {
    /* Fallback to direct I2C if memory hasn't been read yet */
    __u8 block[I2C_SMBUS_BLOCK_MAX];
    I2C_READ_BLOCK(upsfd, UPTIME_CMD, 4, block, __func__)
    uptime = (block[3] << 24) | (block[2] << 16) | (block[1] << 8) | block[0];
  }
  
  upsdebugx(1, "Device uptime: %ds", uptime);
  dstate_setinfo("device.uptime", "%d", uptime);
}

static void check_operating_state(void)
{
  /* Use the critical data that was already read, don't read again */
  upsdebugx(3, "Checking operating state with USB-C: %0.3fV, MicroUSB: %0.3fV", 
            usbC_power / 1000.0, microUsb_power / 1000.0);
  
  if (usbC_power > USB_VOLTAGE_MINIMUM) {
    power_state = USBC_POWER_CONNECTED;
    upsdebugx(1, "USB-C Input Voltage: %0.3fV", usbC_power / 1000.0);
    /* Skip bad readings */
    if (usbC_power <= USB_VOLTAGE_MAXIMUM) {
      dstate_setinfo("input.voltage.nominal", "%0.3f", USBC_NOMINAL_VOLTAGE);
      dstate_setinfo("input.voltage", "%0.3f", usbC_power / 1000.0);
    } else {
      upsdebugx(2, "Input Voltage out of range, skipping.");
    }
  } else if (microUsb_power > USB_VOLTAGE_MINIMUM) {
    power_state = MICROUSB_POWER_CONNECTED;
    upsdebugx(1, "MicroUSB Input Voltage: %0.3fV", microUsb_power / 1000.0);
    /* Skip bad readings */
    if (microUsb_power <= USB_VOLTAGE_MAXIMUM) {
      dstate_setinfo("input.voltage.nominal", "%0.3f", MICROUSB_NOMINAL_VOLTAGE);
      dstate_setinfo("input.voltage", "%0.3f", microUsb_power / 1000.0);
    } else {
      upsdebugx(2, "Input Voltage out of range, skipping.");
    }
  } else {
    power_state = POWER_NOT_CONNECTED;
    upsdebugx(1, "Input Voltage: None");
    dstate_setinfo("input.voltage", "0.0");
  }
}

static void reset_factory(void)
{
  uint8_t cmd = RESET_TO_DEFAULT_CMD;
  
  upsdebugx(3, __func__);
  
  I2C_WRITE_BYTE(upsfd, cmd, 0x1, __func__)
}

static void reset_battery(void)
{
  uint8_t cmd = BATTERY_PARAM_CUSTOM_CMD;
  
  upsdebugx(3, __func__);
  
  I2C_WRITE_BYTE(upsfd, cmd, BATTERY_PARAM_CUSTOM_DISABLE, __func__)
}

/*
 Always reset timers back to zero before trying to set them. Others
 have reported issues with setting a timer that is already running with
 a value other than zero.
 */
int upsplus_setvar(const char *key, const char *value)
{
  short data;
  double voltage;
  int percent;
  
  upsdebugx(2, "In %s for %s with %s...", __func__, key, value);
  
  if (!strcasecmp(key, "battery.voltage.high")) {
    if (str_to_double(value, &voltage, 10)) {
      if (voltage * 1000 >= MIN_BATTERY_VOLTAGE
          && voltage * 1000 <= MAX_BATTERY_VOLTAGE) {
        set_battery_full(voltage * 1000);
        return STAT_SET_HANDLED;
      }
    }
    upsdebugx(1, "Unknown value for battery.voltage.high: %s", value);
    return STAT_SET_UNKNOWN;
  }
  
  if (!strcasecmp(key, "battery.voltage.low")) {
    if (str_to_double(value, &voltage, 10)) {
      if (voltage * 1000 >= MIN_BATTERY_VOLTAGE
          && voltage * 1000 <= MAX_BATTERY_VOLTAGE) {
        set_battery_low(voltage * 1000);
        return STAT_SET_HANDLED;
      }
    }
    upsdebugx(1, "Unknown value for battery.voltage.low: %s", value);
    return STAT_SET_UNKNOWN;
  }
  
  if (!strcasecmp(key, "battery.charge.low")) {
    if (str_to_int(value, &percent, 10) && percent >= 0 && percent <= 100) {
      set_charge_low(percent);
      return STAT_SET_HANDLED;
    }
    upsdebugx(1, "Unknown value for battery.charge.low: %s", value);
    return STAT_SET_UNKNOWN;
  }
  
  if (!strcasecmp(key, "ups.timer.shutdown")) {
    if (str_to_short(value, &data, 10) && data >= TIMER_MINIMUM) {
      set_power_off_timer(data);
      return STAT_SET_HANDLED;
    }
    upsdebugx(1, "Unknown value for ups.timer.shutdown: %s", value);
    return STAT_SET_UNKNOWN;
  }
  
  if (!strcasecmp(key, "ups.timer.reboot")) {
    if (str_to_short(value, &data, 10) && data >= TIMER_MINIMUM) {
      set_reboot_timer(data);
      return STAT_SET_HANDLED;
    }
    return STAT_SET_UNKNOWN;
  }
  
  if (!strcasecmp(key, "ups.start.auto")) {
    if (!strcasecmp(value, "yes")) {
      set_ups_auto_restart(WAKEUP_ON_CHARGE_ENABLE);
      return STAT_SET_HANDLED;
    } else if (!strcasecmp(value, "no")) {
      set_ups_auto_restart(WAKEUP_ON_CHARGE_DISABLE);
      return STAT_SET_HANDLED;
    }
    
    upsdebugx(1, "Unknown value for ups.start.auto: %s", value);
    return STAT_SET_UNKNOWN;
  }
  
  return STAT_SET_UNKNOWN;
}

/*
 Always reset timers back to zero before trying to set them. Others
 have reported issues with setting a timer that is already running with
 a value other than zero.
 */
int upsplus_instcmd(const char *cmd, const char *reserved)
{
  upsdebugx(2, "In %s with %s and extra %s.", __func__, cmd, reserved);
  
  if (!strcasecmp(cmd, "load.off.delay")) {
    set_power_off_timer(TIMER_MINIMUM);
    return STAT_INSTCMD_HANDLED;
  }
  
  if (!strcasecmp(cmd, "shutdown.return")) {
    set_ups_auto_restart(WAKEUP_ON_CHARGE_ENABLE);
    set_power_off_timer(TIMER_MINIMUM);
    return STAT_INSTCMD_HANDLED;
  }
  
  if (!strcasecmp(cmd, "shutdown.stayoff")) {
    set_ups_auto_restart(WAKEUP_ON_CHARGE_DISABLE);
    set_power_off_timer(TIMER_MINIMUM);
    return STAT_INSTCMD_HANDLED;
  }
  
  if (!strcasecmp(cmd, "shutdown.stop")) {
    reset_shutdown_restart_timers();
    return STAT_INSTCMD_HANDLED;
  }
  
  if (!strcasecmp(cmd, "shutdown.reboot.graceful")) {
    set_reboot_timer(TIMER_MINIMUM);
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
  
  dstate_setinfo("battery.packs", "%d", BATTERY_CELL_COUNT);
  dstate_setinfo("battery.capacity", "%.2f", DEFAULT_BATTERY_CAPACITY_Ah);
  /* 18650 lithium battery are Li-ion */
  dstate_setinfo("battery.type", "%s", "Li-ion");
  
  /* Specs say:
  Linear compensation range discharge capacity: 5V 4.5A. (Nominal)
  */
  dstate_setinfo("output.voltage.nominal", "5.0");
  dstate_setinfo("output.current.nominal", "4.5");
  dstate_setinfo("ups.realpower.nominal", "%f", MAX_LOAD);
  dstate_setinfo("ups.power.nominal", "%f", MAX_LOAD);
  
  /* Attempt to detect the UPSPlus by reading the firmware */
  /* version. */
  get_firmware_version();
  if (firmware_version == 0) {
    fatal_with_errno(-1, "Failed to find UPSPlus on i2c bus, exiting.\n");
  }
  get_serial_number();
  
  /* Read initial battery sample period to set memory update interval */
  uint16_t initial_sample_period;
  upsdebugx(1, "Initial memory update interval: %d seconds", memory_update_interval);
  
  if (i2c_smbus_read_word_data(upsfd, BATTERY_SAMPLE_PERIOD_CMD) > 0) {
    initial_sample_period = i2c_smbus_read_word_data(upsfd, BATTERY_SAMPLE_PERIOD_CMD);
    upsdebugx(1, "Read initial battery sample period: %d minutes", initial_sample_period);
    
    if (initial_sample_period >= BATTERY_SAMPLE_PERIOD_MINIMUM && initial_sample_period <= BATTERY_SAMPLE_PERIOD_MAXIMUM) {
      memory_update_interval = initial_sample_period * 60;
      
      /* Ensure minimum reasonable interval */
      if (memory_update_interval < 30) {
        memory_update_interval = 30;
        upsdebugx(1, "Initial memory update interval adjusted to minimum 30 seconds");
      }
      
      upsdebugx(1, "Initial memory update interval set to %d seconds based on UPS battery sample period", memory_update_interval);
    }
  } else {
    upsdebugx(1, "Could not read initial battery sample period, keeping default: %d seconds", memory_update_interval);
  }
  
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
  dstate_addcmd("shutdown.stop");/* Shutdown countdown = 0 + Reboot countdown = 0 */
  dstate_addcmd("shutdown.reboot.graceful");/* Restart countdown (10) */
}

void upsdrv_updateinfo(void)
{
  static int update_count = 0;
  static time_t last_update_call = 0;
  time_t now;
  
  update_count++;
  time(&now);
  
  if (last_update_call > 0) {
    upsdebugx(2, "upsdrv_updateinfo() called #%d times, %lds since last call", 
              update_count, now - last_update_call);
  } else {
    upsdebugx(2, "upsdrv_updateinfo() called #%d times (first call)", update_count);
  }
  last_update_call = now;
  
  /* Read critical real-time data (power status, input voltage) - updated frequently */
  int critical_result = read_critical_data();
  upsdebugx(2, "read_critical_data() returned: %d", critical_result);
  
  /* Read entire memory buffer for efficient access - updated at battery sample interval */
  /* Note: Critical data (power status, battery voltage/current, input/output voltage) */
  /* is already updated every 2 seconds, so status determination uses fresh data */
  int memory_result = read_upsplus_memory();
  upsdebugx(2, "read_upsplus_memory() returned: %d", memory_result);
  
  get_battery_full();
  get_battery_low();
  get_battery_empty();
  get_charge_low();
  get_battery_nominal();
  get_battery_param_custom();
  
  get_battery_temperature();
  get_battery_voltage();
  get_output_voltage();
  get_charge_level();
  get_mcu_voltage();
  get_realtime_battery_state();
  get_realtime_output_state();
  
  check_operating_state();
  
  get_ups_uptime();
  get_running_time();
  get_charging_time();
  get_battery_sample_period();
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
  upsdebugx(3, __func__);
  
  set_ups_auto_restart(WAKEUP_ON_CHARGE_DISABLE);
  set_power_off_timer(SHUTDOWN_TIMER);
}

void upsdrv_help(void)
{
  printf("\n---------\nNOTE:\n");
  printf("By default UPSPlus appears on i2c bus 1, so port should be set to '/dev/i2c-1'.\n");
  printf("The /dev/i2c-1 device needs to be world RW permissions, aka 'sudo chmod a+rw /dev/i2c-1'.\n");
  printf("\n");
  printf("OPTIMIZATION:\n");
  printf("The driver uses a hybrid approach for optimal performance:\n");
  printf("- Critical data (power status, battery voltage/current, input/output voltage) is updated every 2 seconds\n");
  printf("- Battery data is updated at the UPS battery sample interval (typically 2+ minutes)\n");
  printf("- This ensures fast response to power events while minimizing I2C traffic\n");
  printf("\n");
}

void upsdrv_makevartable(void)
{
  addvar(VAR_FLAG, "factoryreset", "Reset UPSPlus to factory settings and exit.");
  
  addvar(VAR_FLAG, "batteryreset", "Reset Battery min/max/low to automatic settings and exit.");
  
  addvar(VAR_VALUE, "sampleperiod", "Set number of minutes between sampling battery state (1 - 1440, default: 2)");
  
  addvar(VAR_VALUE, "memoryinterval", "Set memory update interval in seconds (1 - 60, default: 1)");
  
  addvar(VAR_VALUE, "criticalinterval", "Set critical update interval in seconds (1 - 30, default: 2)");
  
  addvar(VAR_FLAG, "readreserved", "Try to read reserved registers (may cause errors)");
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
    if (str_to_int(getval("sampleperiod"), &sampleperiod, 10) && sampleperiod >= BATTERY_SAMPLE_PERIOD_MINIMUM && sampleperiod <= BATTERY_SAMPLE_PERIOD_MAXIMUM) {
      i2c_smbus_write_byte_data(upsfd, BATTERY_SAMPLE_PERIOD_CMD, sampleperiod & 0xFF);
      i2c_smbus_write_byte_data(upsfd, BATTERY_SAMPLE_PERIOD_CMD + 1, (sampleperiod >> 8) & 0xFF);
      upsdebugx(1, "Updated sample period to: %d", sampleperiod);
      
      /* Also update memory update interval to match */
      memory_update_interval = sampleperiod * 60;
      
      /* Ensure minimum reasonable interval */
      if (memory_update_interval < 30) {
        memory_update_interval = 30;
        upsdebugx(1, "Memory update interval adjusted to minimum 30 seconds");
      }
      
      upsdebugx(1, "Memory update interval set to %d seconds to match battery sample period", memory_update_interval);
    } else {
      upsdebugx(1, "Ignoring sampleperiod, out of range: %s", getval("sampleperiod"));
    }
  }
  
  if (getval("memoryinterval") != NULL) {
    int interval;
    if (str_to_int(getval("memoryinterval"), &interval, 10) && interval >= 1 && interval <= 60) {
      memory_update_interval = interval;
      upsdebugx(1, "Updated memory update interval to: %d seconds", interval);
    } else {
      upsdebugx(1, "Ignoring memoryinterval, out of range: %s", getval("memoryinterval"));
    }
  }
  
  if (getval("criticalinterval") != NULL) {
    int interval;
    if (str_to_int(getval("criticalinterval"), &interval, 10) && interval >= 1 && interval <= 30) {
      critical_update_interval = interval;
      upsdebugx(1, "Updated critical update interval to: %d seconds", interval);
    } else {
      upsdebugx(1, "Ignoring criticalinterval, out of range: %s", getval("criticalinterval"));
    }
  }
  
  if (testvar("readreserved")) {
    read_reserved_registers = 1;
    upsdebugx(1, "Will attempt to read reserved registers");
  }
}

void upsdrv_cleanup(void)
{
  close(upsfd);
}
