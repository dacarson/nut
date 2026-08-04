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
 
 /*
 * Sampling model:
 * - Registers 0x01-0x0C are sample-timed by the UPS MCU and refresh only at the configured Battery Sample Period.
 * - Registers 0x0D-0x2A are treated as live (RW) and should be read frequently.
 * - We bulk-read two memory blocks: sampled (0x01-0x0C) at most once per sample period, and live (0x0D-0x2A) on every update.
 * - For firmware < 29, this driver talks directly to the two INA219 sensors on the output/battery
 *   rails for real-time readings (current, power, bus voltage).
 * - For firmware >= 29, the UPS MCU itself reads the INA219 chips and publishes calibrated current
 *   (registers 0x2E-0x32, part of the same bulk memory read) so this driver no longer touches the
 *   INA219 chips directly; power is derived here as voltage * current.
  */

#include "main.h"

#include <math.h>
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
#define BATTERY_TEMPERATURE_ALARM           65/* IP5328's own NTC-based over-temp protection (see documents/UPSPlus_Hardware_Chips.md); doesn't cut RPi power (RPi runs off VREG, independent of the discharge path it protects), so there is no separate forced-shutdown event to warn ahead of */

#define BATTERY_FULL_CMD                    0x0D
#define BATTERY_EMPTY_CMD                   0x0F
#define BATTERY_PROTECTION_CMD              0x11/* UPS will FSD when this is hit */
#define MIN_BATTERY_VOLTAGE                 2750/* Lower will kill the battery. */
#define MAX_BATTERY_VOLTAGE                 4500

#define CHARGE_LEVEL_CMD                    0x13

#define BATTERY_SAMPLE_PERIOD_CMD           0x15
#define BATTERY_SAMPLE_PERIOD_MINIMUM       2
#define BATTERY_SAMPLE_PERIOD_MAXIMUM       1440

#define POWER_STATUS_CMD                    0x17
#define POWER_STATUS_FLAG_ON                0x01
#define POWER_STATUS_FLAG_CALIBRATING       0x02

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

/* 0x2C - 0xEF are reserved, and don't seem to be used.
 * When new commands are added in FW updates, they are
 * added as the next available register. So to store
 * custom data, start at highest reserved register
 * and work back.
 * Low charge threshold lives at 0xEF (FW < 20) or 0x2B (FW >= 20).
 */
 #define RESERVED_BATTERY_LOW_CHARGE_CMD_LEGACY  0xEF
 #define BATTERY_LOW_CHARGE_CONFIGURED           0x80
 #define BATTERY_LOW_CHARGE_MASK                 0x7F

 /* Firmware 2.0 commands */
 #define BATTERY_LOW_CHARGE_CMD_V20         0x2B
 #define LOAD_ON_DELAY_CMD_V20              0x2C

#define SERIAL_NUMBER_CMD                   0xF0

/* Bulk reading constants */
#define UPSPLUS_MEMORY_START                0x01
#define UPSPLUS_MEMORY_END                  0xFB  /* Only read up to the last useful register */
#define UPSPLUS_MEMORY_SIZE                 (UPSPLUS_MEMORY_END - UPSPLUS_MEMORY_START + 1)

/*
 * Constants used internally
 */

#define POWER_NOT_CONNECTED                 0x0
#define USBC_POWER_CONNECTED                0x1
#define MICROUSB_POWER_CONNECTED            0x2

#define USBC_NOMINAL_VOLTAGE                9.00
#define MICROUSB_NOMINAL_VOLTAGE            5.00

#define MAX_LOAD                            22.5/* (5V x 4.5A) */
#define MAX_PEAK_LOAD                       40.0/* (5V x 8A) */
#define DEFAULT_BATTERY_CAPACITY_Ah         3.5
#define BATTERY_CELL_COUNT                  2
#define TIMER_MINIMUM_LEGACY_FW              5   /* Original vendor firmware (< 20): shutdown/reboot timers accept values from 5s */
#define TIMER_MINIMUM_FIXED_FW               10  /* Corrected firmware (>= 20): valid range is 0 (off) or 10-255s */
#define SHUTDOWN_TIMER                      20
#define AUTO_SHUTDOWN_TIME                  240
#define DEFAULT_CHARGE_LOW                  10

#define DRIVER_NAME                         "UPSPlus driver"
#define DRIVER_VERSION                      "2.3"

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

/*
 * Firmware >= 29 reads the INA219 chips itself and publishes the calibrated
 * current (mA, signed, 1 mA/LSB) at these registers, which fall within the
 * bulk 0x01-0xFB memory read, so no direct INA219 i2c access is needed.
 * There is no dedicated power register; power is derived here as V * I using
 * the sample-timed output/battery voltage already in the memory buffer.
 */
#define FIRMWARE_VERSION_INA219_IN_FW        29
#define OUTPUT_CURRENT_CMD_V29               0x2E
#define BATTERY_CURRENT_CMD_V29              0x30
#define CURRENT_VALID_FLAGS_CMD_V29          0x32
#define OUTPUT_CURRENT_VALID_FLAG            0x01
#define BATTERY_CURRENT_VALID_FLAG           0x02

static char *default_i2c_bus_path = "/dev/i2c-1";
static char *i2c_bus_path;

/*
 * Current charging levels
 */

/*
 * Bulk memory buffer for efficient I2C reading with double-read validation
 * All registers (0x01-0xFB) are updated frequently, so we read everything
 * and validate with double-read to handle device update cycle corruption
 */
static uint8_t upsplus_memory[UPSPLUS_MEMORY_SIZE];
static int memory_initialized = 0; /* Track if memory has been read at least once */
static int critical_update_interval = 2; /* seconds between critical updates (power status, input voltage) */

/*
 * Flag to track state
 */
static uint8_t power_state = 0;
static uint16_t firmware_version = 0;

/* Serial number is immutable at runtime; cache after first read */
static char serial_cached[LENGTH_TEMP];
static int serial_initialized = 0;

static uint8_t get_battery_low_charge_cmd(void)
{
  return (firmware_version >= 20) ?
         BATTERY_LOW_CHARGE_CMD_V20 :
         RESERVED_BATTERY_LOW_CHARGE_CMD_LEGACY;
}

/*
 * Store last-read values for calculations. Values are in mV.
 */
static int16_t battery_full = -1;
static int16_t battery_low = -1;
static int16_t battery_empty = -1;
static int8_t ups_auto_restart = -1;

/* Last-read battery temperature (C), cached for get_status() to check against
 * BATTERY_TEMPERATURE_ALARM. Below BATTERY_TEMPERATURE_MINIMUM until first
 * valid read, so ALARM can't be raised on stale/uninitialized data. */
static int16_t battery_temperature = BATTERY_TEMPERATURE_MINIMUM - 1;

/* Whether the user has overridden full/empty/low battery voltages
 * (BATTERY_PARAM_CUSTOM_ENABLE), disabling the firmware's own
 * self-programming. When set, battery.charge may never reach 100%
 * (or may reach it early) against real voltage, so it can't be trusted
 * to gate charging status. */
static uint8_t battery_param_custom = BATTERY_PARAM_CUSTOM_DISABLE;

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
static int bad_battery_count = 0; /* consecutive polls with timer expired */

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

/*
 * Output load as a percentage of MAX_LOAD, updated each poll cycle
 * by get_realtime_output_state(). Used to detect OVER condition.
 */
static float output_load_percent = 0.0;

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



/*
 * For some reason the INA219 registers seem to be in reverse order
 * So swap MSB/LSB for INA219 operations.
 */
/* Unlike I2C_READ_WORD/I2C_WRITE_WORD (used with the driver's long-lived upsfd, which must
 * never be closed here), these _INA219 macros are only ever used with a per-call fd
 * (ina219_fd) opened just above in the caller -- close it before the early return so an I2C
 * failure can't leak the descriptor. */
#define I2C_READ_WORD_INA219(fd, cmd, label) \
{ \
  __s32 sData; \
  if ((sData = i2c_smbus_read_word_data(fd, cmd)) < 0 ) { \
    upsdebugx(2, "Failure reading the i2c bus [%s]", label); \
    close(fd); \
    return; \
  } ; \
  data = (uint16_t)( (sData >> 8) | (sData << 8) ); \
}

#define I2C_WRITE_WORD_INA219(fd, cmd, value, label) \
{ \
  if ( i2c_smbus_write_word_data(fd, cmd, (__u16)((value >> 8) | (value << 8))) < 0 ) { \
    upsdebugx(2, "Failure writing to the i2c bus [%s]", label); \
    close(fd); \
    return; \
  } ; \
}

/* Helper: check if a 16-bit register matches desired value, tolerant to byte order */
static inline int reg16_matches(__s32 readv, uint16_t desired)
{
  uint16_t v, vs;
  if (readv < 0) return 0; /* read failure means not matching */
  v = (uint16_t)readv;
  if (v == desired) return 1;
  /* Check byteswapped too (INA219 quirk vs host order) */
  vs = (uint16_t)((v >> 8) | (v << 8));
  return (vs == desired);
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

  /* Limit per-transaction wait to 500ms (value is in units of 10ms).
   * Without this, a hung I2C bus can stall each transfer for 15-20 seconds,
   * causing the driver to miss its dstate_dataok() deadline and NUT to mark
   * all data stale.
   */
  ioctl(file, I2C_TIMEOUT, 50);

  return file;
}

/* Backoff between retry attempts in read_upsplus_memory(). The STM32 briefly (<=2ms) drops
 * its own slave-address recognition while it grabs the I2C bus as master to poll the INA219
 * current sensors; a chunk read that lands in that window fails with EIO/EREMOTEIO. Without a
 * delay here, a failed attempt was immediately retried (all chunks, up to 3 times) with
 * essentially no gap -- confirmed via strace as bursts of ~24 back-to-back failing ioctls in
 * under 25ms. That retry storm was largely self-inflicted: it kept the bus busy long past the
 * originating collision, made further collisions (with our own or its own retries) more
 * likely, and was directly visible in the STM32's internal diagnostics as chronic bus activity
 * far more frequent than this driver's actual ~2s poll cadence should produce. A short pause
 * gives any in-flight STM32 master-window time to finish before retrying. */
#define MEMORY_READ_RETRY_DELAY_US 20000 /* 20ms */

/*
 * Read the entire UPSPlus memory range.
 * For firmware < 20, use double-read validation to handle update-cycle corruption.
 * For firmware >= 20, skip double-read and use a single pass.
 */
static int read_upsplus_memory(void)
{
  int i, fd, attempt;
  uint8_t base = UPSPLUS_MEMORY_START;
  uint8_t temp_buffer[UPSPLUS_MEMORY_SIZE];
  int success_count = 0;
  int validation_passes = 0;

  fd = open_i2c_bus(i2c_bus_path, UPSPLUS_I2C_ADDRESS);
  if (fd < 0) return -1;

  /* Firmware >= 20 does not need double-read validation */
  if (firmware_version >= 20) {
    int total_chunks = (UPSPLUS_MEMORY_SIZE + I2C_SMBUS_BLOCK_MAX - 1) / I2C_SMBUS_BLOCK_MAX;
    for (attempt = 0; attempt < 3; attempt++) {
      success_count = 0;

      for (i = 0; i < UPSPLUS_MEMORY_SIZE; i += I2C_SMBUS_BLOCK_MAX) {
        int chunk = (i + I2C_SMBUS_BLOCK_MAX <= UPSPLUS_MEMORY_SIZE) ? I2C_SMBUS_BLOCK_MAX : UPSPLUS_MEMORY_SIZE - i;
        if (i2c_smbus_read_i2c_block_data(fd, base + i, chunk, &temp_buffer[i]) < 0) {
          upsdebugx(2, "Failed to read memory chunk at 0x%02X (attempt %d)", base + i, attempt + 1);
        } else {
          success_count++;
        }
      }

      if (success_count == total_chunks) {
        memcpy(upsplus_memory, temp_buffer, UPSPLUS_MEMORY_SIZE);
        memory_initialized = 1;
        upsdebugx(2, "Memory refreshed (%d chunks)", success_count);
        close(fd);
        return 0;
      }

      upsdebugx(2, "Incomplete memory read on attempt %d (%d/%d chunks)", attempt + 1, success_count, total_chunks);
      if (attempt < 2) {
        usleep(MEMORY_READ_RETRY_DELAY_US);
      }
    }

    close(fd);
    upsdebugx(1, "Memory read failed after 3 attempts - using cached data if available");
    return memory_initialized ? 0 : -1;
  }

  /* Perform double-read validation to handle device update cycle corruption */
  {
  int total_chunks = (UPSPLUS_MEMORY_SIZE + I2C_SMBUS_BLOCK_MAX - 1) / I2C_SMBUS_BLOCK_MAX;
  uint8_t second_buffer[UPSPLUS_MEMORY_SIZE];
  for (attempt = 0; attempt < 3; attempt++) {
    success_count = 0;
    
    /* First read into temporary buffer */
    for (i = 0; i < UPSPLUS_MEMORY_SIZE; i += I2C_SMBUS_BLOCK_MAX) {
      int chunk = (i + I2C_SMBUS_BLOCK_MAX <= UPSPLUS_MEMORY_SIZE) ? I2C_SMBUS_BLOCK_MAX : UPSPLUS_MEMORY_SIZE - i;
      if (i2c_smbus_read_i2c_block_data(fd, base + i, chunk, &temp_buffer[i]) < 0) {
        upsdebugx(2, "Failed to read memory chunk at 0x%02X (attempt %d)", base + i, attempt + 1);
      } else {
        success_count++;
      }
    }

    if (success_count < total_chunks) {
      upsdebugx(2, "Incomplete first read on attempt %d (%d/%d chunks)", attempt + 1, success_count, total_chunks);
      if (attempt < 2) {
        usleep(MEMORY_READ_RETRY_DELAY_US);
      }
      continue;
    }
    
    /* Small delay to ensure device has moved past any update cycle */
    usleep(10000); /* 10ms delay */

    /* Second read into separate buffer */
    success_count = 0;
    for (i = 0; i < UPSPLUS_MEMORY_SIZE; i += I2C_SMBUS_BLOCK_MAX) {
      int chunk = (i + I2C_SMBUS_BLOCK_MAX <= UPSPLUS_MEMORY_SIZE) ? I2C_SMBUS_BLOCK_MAX : UPSPLUS_MEMORY_SIZE - i;
      if (i2c_smbus_read_i2c_block_data(fd, base + i, chunk, &second_buffer[i]) < 0) {
        upsdebugx(2, "Failed to read memory chunk at 0x%02X on validation read", base + i);
      } else {
        success_count++;
      }
    }

    if (success_count < total_chunks) {
      upsdebugx(2, "Incomplete second read on attempt %d (%d/%d chunks)", attempt + 1, success_count, total_chunks);
      if (attempt < 2) {
        usleep(MEMORY_READ_RETRY_DELAY_US);
      }
      continue;
    }
    
    /* Compare the two reads - if they match, we have good data */
    if (memcmp(temp_buffer, second_buffer, UPSPLUS_MEMORY_SIZE) == 0) {
      validation_passes = 1;
      upsdebugx(2, "Memory validation passed on attempt %d", attempt + 1);
      break;
    } else {
      upsdebugx(2, "Memory validation failed on attempt %d (data differs between reads)", attempt + 1);
      /* Use second read as first read for next attempt */
      memcpy(temp_buffer, second_buffer, UPSPLUS_MEMORY_SIZE);
      if (attempt < 2) {
        usleep(MEMORY_READ_RETRY_DELAY_US);
      }
    }
  }
  }

  close(fd);

  if (validation_passes) {
    memcpy(upsplus_memory, temp_buffer, UPSPLUS_MEMORY_SIZE);
    memory_initialized = 1;
    upsdebugx(2, "Memory refreshed and validated (%d chunks)", success_count);
    return 0;
  } else {
    upsdebugx(1, "Memory validation failed after 3 attempts - using cached data if available");
    return memory_initialized ? 0 : -1; /* Return 0 if we have cached data, -1 if no data at all */
  }
}

/*
 * Read only critical real-time data that can change at any moment
 * This includes power status, input voltage, and basic UPS state
 */
static int read_critical_data(void)
{
  time_t now;
  static time_t last_critical_update = 0;

  time(&now);

  /* Only update if enough time has passed since last critical update */
  if (now - last_critical_update < critical_update_interval) {
    return 0;
  }

  if (firmware_version >= FIRMWARE_VERSION_INA219_IN_FW) {
    /* Firmware reads the INA219 chips itself and publishes current via the
     * bulk memory buffer (refreshed every upsdrv_updateinfo() call), and
     * output voltage is already read from memory by get_output_voltage().
     * No direct INA219 i2c access needed here.
     */
    last_critical_update = now;
    return 0;
  }

  upsdebugx(2, "Reading critical real-time data: prioritizing INA219; minimal MCU reads");

  /* Real-time: battery current from INA219 on battery rail */
  {
  int battery_fd = open_i2c_bus(i2c_bus_path, INA219_BATTERY_I2C_ADDRESS);
  if (battery_fd >= 0) {
    __s32 cfg, cal;
    int16_t current_data;
    cfg = i2c_smbus_read_word_data(battery_fd, INA219_CONFIGURATION_CMD);
    if (!reg16_matches(cfg, INA219_CONFIGURATION_VALUE)) {
      i2c_smbus_write_word_data(battery_fd, INA219_CONFIGURATION_CMD, INA219_CONFIGURATION_VALUE);
    }
    cal = i2c_smbus_read_word_data(battery_fd, INA219_CALIBRATION_CMD);
    if (!reg16_matches(cal, INA219_CALIBRATION_VALUE_MAGIC)) {
      i2c_smbus_write_word_data(battery_fd, INA219_CALIBRATION_CMD, INA219_CALIBRATION_VALUE_MAGIC);
    }
    current_data = i2c_smbus_read_word_data(battery_fd, INA219_CURRENT_CMD);
    if (current_data >= 0) {
      current_data = (current_data >> 8) | (current_data << 8);
      battery_current = (int16_t)current_data * BATTERY_CURRENT_LSB_MAGIC;
      upsdebugx(3, "Critical update: Battery current: %0.3fA", battery_current);
    }
    close(battery_fd);
  }
  }

  /* Real-time: output bus voltage via INA219 */
  {
  int output_fd = open_i2c_bus(i2c_bus_path, INA219_OUTPUT_I2C_ADDRESS);
  if (output_fd >= 0) {
    __s32 ocfg, ocal, v;
    ocfg = i2c_smbus_read_word_data(output_fd, INA219_CONFIGURATION_CMD);
    if (!reg16_matches(ocfg, INA219_CONFIGURATION_VALUE)) {
      i2c_smbus_write_word_data(output_fd, INA219_CONFIGURATION_CMD, INA219_CONFIGURATION_VALUE);
    }
    ocal = i2c_smbus_read_word_data(output_fd, INA219_CALIBRATION_CMD);
    if (!reg16_matches(ocal, INA219_CALIBRATION_VALUE_MAGIC)) {
      i2c_smbus_write_word_data(output_fd, INA219_CALIBRATION_CMD, INA219_CALIBRATION_VALUE_MAGIC);
    }
    v = i2c_smbus_read_word_data(output_fd, INA219_BUSVOLTAGE_CMD);
    if (v >= 0) {
      uint16_t w = (uint16_t)((v >> 8) | (v << 8));
      w >>= 3; /* Bits 3-15 */
      w *= 4;  /* LSB 4mV */
      dstate_setinfo("output.voltage", "%0.3f", w / 1000.0);
    }
    close(output_fd);
  }
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
  uint8_t reg = UPSPLUS_MEMORY_START + offset;
  uint16_t i = reg - UPSPLUS_MEMORY_START;

  if (i + 1 >= UPSPLUS_MEMORY_SIZE || !memory_initialized) return 0;
  return (upsplus_memory[i + 1] << 8) | upsplus_memory[i];
}

/*
 * Helper function to read a byte value from memory buffer
 */
static uint8_t get_memory_byte(uint8_t offset)
{
  uint8_t reg = UPSPLUS_MEMORY_START + offset;
  uint16_t i = reg - UPSPLUS_MEMORY_START;

  if (i >= UPSPLUS_MEMORY_SIZE || !memory_initialized) return 0;
  return upsplus_memory[i];
}

/*
 * Helper function to read a 32-bit value from memory buffer
 */
static uint32_t get_memory_dword(uint8_t offset)
{
  uint8_t reg = UPSPLUS_MEMORY_START + offset;
  uint16_t i = reg - UPSPLUS_MEMORY_START;

  if (i + 3 >= UPSPLUS_MEMORY_SIZE || !memory_initialized) return 0;
  return (upsplus_memory[i + 3] << 24) |
         (upsplus_memory[i + 2] << 16) |
         (upsplus_memory[i + 1] << 8) |
         upsplus_memory[i];
}

static void get_charge_level(void)
{
  uint16_t data;
  
  upsdebugx(3, __func__);
  
  /* Read from memory buffer instead of I2C */
  data = get_memory_word(CHARGE_LEVEL_CMD - UPSPLUS_MEMORY_START);
  if (data == 0 && !memory_initialized) {
    upsdebugx(2, "Memory buffer not available for charge level, skipping");
    return;
  }

  upsdebugx(3, "Read charge level from memory buffer: %d%%", data);
  
  if ((data > 100) || (data == 0 && battery_charge_level > battery_charge_low)) {
    upsdebugx(1, "Battery Charge Level out of range, skipping: %d%%", data);
  } else {
	battery_charge_level = data;
	upsdebugx(1, "Battery Charge Level: %d%%", battery_charge_level);
    dstate_setinfo("battery.charge", "%d", battery_charge_level);
  }
}

static void get_output_voltage(void)
{
  uint16_t data;
  
  upsdebugx(3, __func__);
  
  /* Read from memory buffer instead of I2C */
  data = get_memory_word(OUTPUT_VOLTAGE_CMD - UPSPLUS_MEMORY_START);
  if (data == 0) {
    upsdebugx(2, "Output voltage read as zero, skipping");
    return;
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
  int16_t data;
  
  upsdebugx(3, __func__);
  
  /* Read from memory buffer instead of I2C */
  data = get_memory_word(BATTERY_FULL_CMD - UPSPLUS_MEMORY_START);
  if (data == 0 && !memory_initialized) {
    upsdebugx(2, "Memory buffer not available for battery full voltage, skipping");
    return;
  }
  battery_full = data;
  
  upsdebugx(1, "Battery Voltage High: %0.3fV", data / 1000.0);
  dstate_setinfo("battery.voltage.high", "%0.3f", data / 1000.0);
}

static void set_battery_full(uint16_t data)
{
  uint8_t cmd = BATTERY_FULL_CMD;
  
  upsdebugx(3, __func__);
  
  I2C_WRITE_BYTE(upsfd, BATTERY_PARAM_CUSTOM_CMD,
                 BATTERY_PARAM_CUSTOM_ENABLE, __func__)
  I2C_WRITE_BYTE(upsfd, cmd, data & 0xFF, __func__)
  I2C_WRITE_BYTE(upsfd, cmd + 1, (data >> 8) & 0xFF, __func__)

  /* Invalidate live memory buffer since we wrote to a live register */
  memory_initialized = 0;
  upsdebugx(2, "Invalidated memory buffer after writing to register 0x%02X", cmd);
  
  upsdebugx(1, "Battery Voltage High: %0.3fV", data / 1000.0);
  dstate_setinfo("battery.voltage.high", "%0.3f", data / 1000.0);
}

static void get_battery_low(void)
{
  int16_t data;
  
  upsdebugx(3, __func__);
  
  /* Read from memory buffer instead of I2C */
  data = get_memory_word(BATTERY_PROTECTION_CMD - UPSPLUS_MEMORY_START);
  if (data == 0 && !memory_initialized) {
    upsdebugx(2, "Memory buffer not available for battery low voltage, skipping");
    return;
  }
  battery_low = data;
  
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
  
  /* Invalidate live memory buffer since we wrote to a live register */
  memory_initialized = 0;
  upsdebugx(2, "Invalidated memory buffer after writing to register 0x%02X", cmd);
  
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
  
  /* Invalidate live memory buffer since we wrote to a live register */
  memory_initialized = 0;
  upsdebugx(2, "Invalidated memory buffer after writing to register 0x%02X", cmd);
  
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
  uint8_t cmd = get_battery_low_charge_cmd();
  
  upsdebugx(3, __func__);
  
  if (data < 0 || data > 100) {
    return;
  }
  
  battery_charge_low = data;

  /* fw < 20 uses bit 7 as a "configured" flag in the legacy 0xEF register.
   * fw >= 20 stores the percentage directly in 0x2B - no flag bit needed.
   */
  if (firmware_version < 20) {
    data |= BATTERY_LOW_CHARGE_CONFIGURED;
  }

  I2C_WRITE_BYTE(upsfd, cmd, data, __func__)

  upsdebugx(1, "Low Charge Threshold: %d%%", battery_charge_low);
  dstate_setinfo("battery.charge.low", "%d", battery_charge_low);
}





static void get_status(void)
{
  uint8_t data;
  time_t now;
  
  upsdebugx(3, __func__);
  
  /* Use critical data that was already read, or fall back to memory buffer */
  if (memory_initialized) {
    data = get_memory_byte(POWER_STATUS_CMD - UPSPLUS_MEMORY_START);
    upsdebugx(2, "Power status from memory buffer: 0x%02X (%d)", data, data);
  } else {
    upsdebugx(2, "Memory buffer not available for power status, skipping");
    return;
  }
  
  if (firmware_version >= 20) {
    if ((data & POWER_STATUS_FLAG_ON) == 0) {
      upsdebugx(1, "Power status: off");
      status_set("OFF");
      /* If we are not running, then set no other Status. */
      return;
    }
    upsdebugx(1, "Power status: normal");
  } else {
    if (data == 1) {
      upsdebugx(1, "Power status: normal");
    } else if (data == 0) {
      upsdebugx(1, "Power status: off");
      status_set("OFF");
      /* If we are not running, then set no other Status. */
      return;
    } else {
      upsdebugx(1, "Power status: unknown");
      return;
    }
  }
  
  if (power_state == POWER_NOT_CONNECTED) {
    status_set("OB");
  } else {
    status_set("OL");
  }
  
  /* Battery low/full values are read from the memory buffer */
  get_battery_low();
  get_battery_full();

  /* Update bad_battery_timer and CAL before RB check so that a calibration
   * event resets the timer before the 60-second expiry is evaluated.
   * Without this ordering, CAL and RB would both fire in the same poll cycle
   * when calibration starts exactly as the timer expires.
   *
   * If we are discharging while power is connected for 1 minute, then
   * batteries are bad. Need to check for multiple times discharging because
   * when the UPS circuitry calibrates, it samples the battery which drains it.
   */
  if (firmware_version >= 20) {
    if (data & POWER_STATUS_FLAG_CALIBRATING) {
      bad_battery_timer = 0;
      upsdebugx(1, "Battery Status: Calibrating");
      status_set("CAL");
    } else if (battery_current < 0 && power_state != POWER_NOT_CONNECTED) {
      if (!bad_battery_timer) {
        time(&bad_battery_timer);
      }
    } else {
      bad_battery_timer = 0;
    }
  } else if (battery_current < 0 && power_state != POWER_NOT_CONNECTED) {
    if (!bad_battery_timer) {
      time(&bad_battery_timer);
    }
    upsdebugx(1, "Battery Status: Calibrating");
    status_set("CAL");
  } else {
    bad_battery_timer = 0;
  }

  /* Require 2 consecutive polls with the timer expired before setting RB.
   * A single poll above the threshold is suppressed - this covers the case
   * where the UPS samples the battery just before the hardware CAL bit
   * appears, causing one poll of spurious discharge. When CAL fires it
   * resets bad_battery_timer to 0, which in turn resets bad_battery_count,
   * so the counter never reaches 2 unless there is a genuine fault.
   */
  time(&now);
  if (bad_battery_timer && now - bad_battery_timer > 60) {
    bad_battery_count++;
  } else {
    bad_battery_count = 0;
  }
  if (battery_voltage == 0 || bad_battery_count >= 2) {
    upsdebugx(1, "Battery Status: Replace (count=%d)", bad_battery_count);
    status_set("RB");
  } else if (battery_charge_level < battery_charge_low) {
    upsdebugx(1, "Battery Status: Low");
    status_set("LB");
  } else if (battery_full > 0 && battery_voltage > (1.2 * battery_full)) {
    upsdebugx(1, "Battery Status: High");
    status_set("HB");
  }
  
  /* Only trust battery_charge_level to gate CHRG off at 100% when the
   * firmware determined full/empty itself. If the user overrode those
   * voltages, the percentage may hit (or miss) 100% independent of whether
   * the battery is actually still taking charge, so fall back to current
   * direction alone. */
  if (battery_current > 0 &&
      (battery_param_custom == BATTERY_PARAM_CUSTOM_ENABLE || battery_charge_level < 100)) {
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

  if (output_load_percent > 100.0) {
    upsdebugx(1, "UPS Status: Overloaded (%0.1f%%)", output_load_percent);
    status_set("OVER");
  }

  /* Battery over-temperature: matches IP5328's own NTC protection point.
   * This does not directly cut RPi power (RPi runs off VREG, independent
   * of the discharge path IP5328's NTC governs) - see documents/
   * UPSPlus_Hardware_Chips.md, "Thermal protections". */
  if (battery_temperature >= BATTERY_TEMPERATURE_ALARM) {
    upsdebugx(1, "UPS Status: Alarm (battery temperature %dC)", battery_temperature);
    dstate_setinfo("ups.alarm", "High battery temperature (%dC)", battery_temperature);
    status_set("ALARM");
  } else {
    dstate_setinfo("ups.alarm", "%s", "");
  }

  {
    uint8_t shutdown_timer = get_memory_byte(SHUTDOWN_TIMER_CMD - UPSPLUS_MEMORY_START);
    uint8_t shutdown_timer_min = (firmware_version < 20) ? TIMER_MINIMUM_LEGACY_FW : TIMER_MINIMUM_FIXED_FW;
    if (shutdown_timer >= shutdown_timer_min && shutdown_timer < 255) {
      upsdebugx(1, "UPS Status: Forced Shutdown (timer %ds)", shutdown_timer);
      status_set("FSD");
    }
  }

}

static void get_battery_temperature(void)
{
  int16_t data;
  
  upsdebugx(3, __func__);
  
  /* Read from memory buffer instead of I2C */
  data = get_memory_word(BATTERY_TEMPERATURE_CMD - UPSPLUS_MEMORY_START);
  if (data == 0 && !memory_initialized) {
    upsdebugx(2, "Memory buffer not available for battery temperature, skipping");
    return;
  }
  
  upsdebugx(1, "Battery Temperature: %dC", data);
  if (data >=  BATTERY_TEMPERATURE_MINIMUM &&
      data <= BATTERY_TEMPERATURE_MAXIMUM) {
    battery_temperature = data;
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
  if (data == 0) {
    upsdebugx(2, "Battery voltage read as zero, retaining last value (%dmV)", battery_voltage);
    return;
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
  float nominal_voltage;
  float total_battery_capacity;
  float remaining_energy;
  int runtime_seconds;
  static float smoothed_power = -1.0;

  upsdebugx(3, __func__);

  /* Use a fixed nominal pack voltage (midpoint of the configured full/empty
   * thresholds) rather than the live, load-sagging battery voltage.
   * battery_charge_level (%) is itself derived from that same full/empty
   * voltage range by the firmware, so scaling capacity by *both* the live,
   * sagging voltage *and* the charge percentage double-counts voltage sag:
   * a momentary sag under load knocks down both terms at once, producing
   * an exaggerated, jumpy runtime estimate (e.g. a cliff-drop the instant
   * a load turns on, even though the charge percentage barely moved).
   * Keeping the capacity term fixed leaves runtime tracking charge_level
   * cleanly.
   */
  if (battery_full > 0 && battery_empty > 0 && battery_full > battery_empty) {
    nominal_voltage = ((battery_full + battery_empty) / 2) / 1000.0;
  } else {
    /* Full/empty thresholds not yet available; fall back to live voltage */
    nominal_voltage = battery_voltage / 1000.0;
  }
  total_battery_capacity = nominal_voltage * DEFAULT_BATTERY_CAPACITY_Ah * BATTERY_CELL_COUNT;
  remaining_energy = total_battery_capacity * (float)battery_charge_level / 100.0;
  
  /* Handle edge cases that cause extreme runtime spikes */
  if (power_consumption <= 0.001) { /* Less than 1mW */
    upsdebugx(2, "Power consumption too low (%0.6fW) for runtime calculation, skipping", power_consumption);
    return;
  }

  /* Smooth the instantaneous power reading (it is a single INA219 sample and
   * spikes with transient load, e.g. WiFi/USB bursts) so the runtime estimate
   * reflects sustained load rather than jittering with every poll cycle.
   */
  if (smoothed_power < 0.0) {
    smoothed_power = power_consumption;
  } else {
    smoothed_power = (smoothed_power * 0.8) + (power_consumption * 0.2);
  }

  runtime_seconds = (int)(remaining_energy * 60.0 * 60.0 / smoothed_power);

  /* Runtime over 7 days is unreasonable, skip reporting for this cycle */
  if (runtime_seconds > 604800) {
    upsdebugx(2, "Calculated runtime (%ds) exceeds 7 days, ignoring", runtime_seconds);
    return;
  }

  upsdebugx(1, "Battery runtime: %ds (power %0.3fW, smoothed %0.3fW)", runtime_seconds, power_consumption, smoothed_power);
  dstate_setinfo("battery.runtime", "%d", runtime_seconds);
}

static void get_realtime_output_state(void)
{
  uint16_t data;
  int ina219_fd;
  int attempt;

  upsdebugx(3, __func__);

  if (firmware_version >= FIRMWARE_VERSION_INA219_IN_FW) {
    uint8_t valid_flags;
    int16_t current_mA;
    uint16_t voltage_mV;
    float current_A, power_W;

    valid_flags = get_memory_byte(CURRENT_VALID_FLAGS_CMD_V29 - UPSPLUS_MEMORY_START);
    if (!(valid_flags & OUTPUT_CURRENT_VALID_FLAG)) {
      upsdebugx(2, "Output current not valid (stale), skipping");
      return;
    }

    current_mA = (int16_t) get_memory_word(OUTPUT_CURRENT_CMD_V29 - UPSPLUS_MEMORY_START);
    voltage_mV = get_memory_word(OUTPUT_VOLTAGE_CMD - UPSPLUS_MEMORY_START);
    current_A = current_mA / 1000.0;
    power_W = fabs((voltage_mV / 1000.0) * current_A);

    upsdebugx(1, "Output Current: %0.3fA", current_A);
    dstate_setinfo("output.current", "%0.3f", current_A);

    upsdebugx(1, "Output Power: %0.3fW", power_W);
    /* Apparent Power and Real Power are the same for this DC UPS */
    dstate_setinfo("ups.realpower", "%0.3f", power_W);
    dstate_setinfo("ups.power", "%0.3f", power_W);
    output_load_percent = 100 * power_W / MAX_LOAD;
    upsdebugx(1, "UPS Load: %0.3f%%", output_load_percent);
    dstate_setinfo("ups.load", "%0.3f", output_load_percent);

    if (power_state != POWER_NOT_CONNECTED) {
      estimate_battery_runtime(power_W);
    }
    return;
  }

  ina219_fd = open_i2c_bus(i2c_bus_path, INA219_OUTPUT_I2C_ADDRESS);
  
  /* Configure/Calibrate INA219 only if needed */
  {
    __s32 cfg, cal;
    cfg = i2c_smbus_read_word_data(ina219_fd, INA219_CONFIGURATION_CMD);
    if (!reg16_matches(cfg, INA219_CONFIGURATION_VALUE)) {
      I2C_WRITE_WORD_INA219(ina219_fd, INA219_CONFIGURATION_CMD, INA219_CONFIGURATION_VALUE, __func__)
    }
    cal = i2c_smbus_read_word_data(ina219_fd, INA219_CALIBRATION_CMD);
    if (!reg16_matches(cal, INA219_CALIBRATION_VALUE_MAGIC)) {
      I2C_WRITE_WORD_INA219(ina219_fd, INA219_CALIBRATION_CMD, INA219_CALIBRATION_VALUE_MAGIC, __func__)
    }
  }

  /* Read values */
  attempt = 3;
  do {
    I2C_READ_WORD_INA219(ina219_fd, INA219_BUSVOLTAGE_CMD, __func__)
  } while (!(data & INA219_CONVERSION_READY) && attempt--);

  if (attempt == 0) {
    upsdebugx(1, "INA219 Output Voltage value not ready");
    close(ina219_fd);
    ina219_fd = 0;
    return;
  }
  
  data >>= 3;    /* Bits 3-15 */
  data *= 4;    /* LSB 4mV */
  upsdebugx(1, "INA219 Output Voltage: %0.3fV", data / 1000.0);
  if (data == 0) {
    upsdebugx(2, "INA219 output voltage read as zero, skipping");
  } else if (data > OUTPUT_VOLTAGE_MINIMUM || data < OUTPUT_VOLTAGE_MAXIMUM) {
    dstate_setinfo("output.voltage", "%0.3f", data / 1000.0);
  } else {
    upsdebugx(2, "Output voltage out of range, skipping");
  }
  
  I2C_READ_WORD_INA219(ina219_fd, INA219_POWER_CMD, __func__)
  upsdebugx(1, "INA219 Output Power: %0.3fW", data * OUTPUT_POWER_LSB_MAGIC);
    // Apparent Power and Real Power are the same for this DC UPS
  dstate_setinfo("ups.realpower", "%0.3f", data * OUTPUT_POWER_LSB_MAGIC);
  dstate_setinfo("ups.power", "%0.3f", data * OUTPUT_POWER_LSB_MAGIC);
  output_load_percent = 100 * data * OUTPUT_POWER_LSB_MAGIC / MAX_LOAD;
  upsdebugx(1, "UPS Load: %0.3f%%", output_load_percent);
  dstate_setinfo("ups.load", "%0.3f", output_load_percent);
  
  // If charging, estimate time based on output power
  if (power_state != POWER_NOT_CONNECTED) {
    estimate_battery_runtime(data * OUTPUT_POWER_LSB_MAGIC);
  }
  
  I2C_READ_WORD_INA219(ina219_fd, INA219_CURRENT_CMD, __func__)
  /* Current is a signed 16bit number */
  upsdebugx(1, "INA219 Output Current: %0.3fA", (int16_t) data * OUTPUT_CURRENT_LSB_MAGIC);
  dstate_setinfo("output.current", "%0.3f", (int16_t) data * OUTPUT_CURRENT_LSB_MAGIC);
  
  close(ina219_fd);
  ina219_fd = 0;
}

static void get_realtime_battery_state(void)
{
  int16_t data = 0;
  int ina219_fd;
  int attempt;

  upsdebugx(3, __func__);

  if (firmware_version >= FIRMWARE_VERSION_INA219_IN_FW) {
    uint8_t valid_flags;
    int16_t current_mA;

    valid_flags = get_memory_byte(CURRENT_VALID_FLAGS_CMD_V29 - UPSPLUS_MEMORY_START);
    if (!(valid_flags & BATTERY_CURRENT_VALID_FLAG)) {
      upsdebugx(2, "Battery current not valid (stale), skipping");
      return;
    }

    current_mA = (int16_t) get_memory_word(BATTERY_CURRENT_CMD_V29 - UPSPLUS_MEMORY_START);
    battery_current = current_mA / 1000.0;

    upsdebugx(1, "Battery Current: %0.3fA", battery_current);
    dstate_setinfo("battery.current", "%0.3f", battery_current);

    {
      float power_W = fabs((battery_voltage / 1000.0) * battery_current);
      upsdebugx(1, "Battery Power: %0.3fW", power_W);
      dstate_setinfo("battery.power", "%0.3f", power_W);

      /* If discharging, estimate time based on battery power */
      if (power_state == POWER_NOT_CONNECTED) {
        estimate_battery_runtime(power_W);
      }
    }
    return;
  }

  ina219_fd = open_i2c_bus(i2c_bus_path, INA219_BATTERY_I2C_ADDRESS);
  
  /* Configure/Calibrate INA219 only if needed */
  {
    __s32 cfg, cal;
    cfg = i2c_smbus_read_word_data(ina219_fd, INA219_CONFIGURATION_CMD);
    if (!reg16_matches(cfg, INA219_CONFIGURATION_VALUE)) {
      I2C_WRITE_WORD_INA219(ina219_fd, INA219_CONFIGURATION_CMD, INA219_CONFIGURATION_VALUE, __func__)
    }
    cal = i2c_smbus_read_word_data(ina219_fd, INA219_CALIBRATION_CMD);
    if (!reg16_matches(cal, INA219_CALIBRATION_VALUE_MAGIC)) {
      I2C_WRITE_WORD_INA219(ina219_fd, INA219_CALIBRATION_CMD, INA219_CALIBRATION_VALUE_MAGIC, __func__)
    }
  }

  /* Read INA219 values */
  attempt = 3;
  do {
    I2C_READ_WORD_INA219(ina219_fd, INA219_BUSVOLTAGE_CMD, __func__)
  } while (!(data & INA219_CONVERSION_READY) && attempt--);

  if (attempt == 0) {
    upsdebugx(1, "INA219 Battery Voltage value not ready");
    close(ina219_fd);
    ina219_fd = 0;
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
  
  I2C_READ_WORD_INA219(ina219_fd, INA219_POWER_CMD, __func__)
  upsdebugx(1, "INA219 Battery Power: %0.3fW", data * BATTERY_POWER_LSB_MAGIC);
  dstate_setinfo("battery.power", "%0.3f", data * BATTERY_POWER_LSB_MAGIC);

  // If discharging, estimate time based on battery power
  if (power_state == POWER_NOT_CONNECTED) {
    estimate_battery_runtime(data * BATTERY_POWER_LSB_MAGIC);
  }
  
  I2C_READ_WORD_INA219(ina219_fd, INA219_CURRENT_CMD, __func__)
  /* Current is a signed 16bit number */
  upsdebugx(1, "INA219 Battery Current: %0.3fA", (int16_t) data * BATTERY_CURRENT_LSB_MAGIC);
  dstate_setinfo("battery.current", "%0.3f", (int16_t) data * BATTERY_CURRENT_LSB_MAGIC);
  battery_current = (int16_t) data * BATTERY_CURRENT_LSB_MAGIC;
  
  close(ina219_fd);
  ina219_fd = 0;
}

static void get_firmware_version(void)
{
  uint16_t data = 0;
  
  upsdebugx(3, __func__);
  
  /* Read from memory buffer instead of I2C */
  data = get_memory_word(FIRMWARE_VERSION_CMD - UPSPLUS_MEMORY_START);
  if (data == 0 && !memory_initialized) {
    upsdebugx(2, "Memory buffer not available for firmware version, skipping");
    return;
  }
  
  firmware_version = data;
  
  upsdebugx(1, "UPS Firmware Version: %d", data);
  dstate_setinfo("ups.firmware", "%d", data);
}

static void get_serial_number(void)
{
  __u8 block[12];
  int fd;
  int has_data;

  upsdebugx(3, __func__);

  /* If we've already read it once, just publish the cached value */
  if (serial_initialized) {
    dstate_setinfo("device.serial", "%s", serial_cached);
    dstate_setinfo("ups.serial", "%s", serial_cached);
    upsdebugx(2, "Serial Number (cached): %s", serial_cached);
    return;
  }

  /* Read directly from the UPSPlus (0xF0..0xFB). SN does not change at runtime. */
  fd = open_i2c_bus(i2c_bus_path, UPSPLUS_I2C_ADDRESS);
  if (fd < 0) {
    upsdebugx(2, "Unable to open I2C bus to read serial number");
    return;
  }

  has_data = 0;
  for (int i = 0; i < 12; i++) {
    __s32 b = i2c_smbus_read_byte_data(fd, SERIAL_NUMBER_CMD + i);
    if (b < 0) {
      upsdebugx(2, "Failed reading serial byte %d", i);
      close(fd);
      return;
    }
    block[i] = (__u8)(b & 0xFF);
    if (block[i] != 0) has_data = 1;
  }
  close(fd);

  if (!has_data) {
    upsdebugx(2, "Serial number appears empty (all zeros), skipping");
    return;
  }

  /* Format and cache */
  snprintf(serial_cached, sizeof(serial_cached), "%08X-%08X-%08X",
           (block[3]  << 24) | (block[2]  << 16) | (block[1]  << 8) | block[0],
           (block[7]  << 24) | (block[6]  << 16) | (block[5]  << 8) | block[4],
           (block[11] << 24) | (block[10] << 16) | (block[9]  << 8) | block[8]);

  serial_initialized = 1;
  upsdebugx(1, "Serial Number: %s", serial_cached);
  dstate_setinfo("device.serial", "%s", serial_cached);
  dstate_setinfo("ups.serial", "%s", serial_cached);
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
  if (data == 0 && !memory_initialized) {
    upsdebugx(2, "Memory buffer not available for MCU voltage, skipping");
    return;
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
  
  /* Read 32-bit value from memory buffer using get_memory_dword() */
  running_time = get_memory_dword(RUNNING_TIME_CMD - UPSPLUS_MEMORY_START);
  
  if (running_time == 0 && !memory_initialized) {
    upsdebugx(2, "Memory buffer not available for running time, skipping");
    return;
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
  
  /* Read 32-bit value from memory buffer using get_memory_dword() */
  charging_time = get_memory_dword(CHARGING_TIME_CMD - UPSPLUS_MEMORY_START);
  
  if (charging_time == 0 && !memory_initialized) {
    upsdebugx(2, "Memory buffer not available for charging time, skipping");
    return;
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
  if (data == 0 && !memory_initialized) {
    upsdebugx(2, "Memory buffer not available for battery sample period, skipping");
    return;
  }
  
  upsdebugx(1, "Battery sample period: %d minutes", data);
  if (data >= BATTERY_SAMPLE_PERIOD_MINIMUM && data <= BATTERY_SAMPLE_PERIOD_MAXIMUM) {
    dstate_setinfo("battery.sample.period", "%d", data);

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
  if (data == 0 && !memory_initialized) {
    upsdebugx(2, "Memory buffer not available for battery empty voltage, skipping");
    return;
  }
  
  upsdebugx(1, "Battery Voltage Empty: %0.3fV", data / 1000.0);
  if (data >= BATTERY_VOLTAGE_MINIMUM && data <= BATTERY_VOLTAGE_MAXIMUM) {
    battery_empty = data;
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
  if (data == 0 && !memory_initialized) {
    upsdebugx(2, "Memory buffer not available for battery param custom, skipping");
    return;
  }
  
  battery_param_custom = data;

  upsdebugx(1, "Battery parameter custom: %s",
            data == BATTERY_PARAM_CUSTOM_ENABLE ? "enabled" : "disabled");
  dstate_setinfo("battery.param.custom", "%s",
                 data == BATTERY_PARAM_CUSTOM_ENABLE ? "enabled" : "disabled");
}

/*
 * Read the reserved battery low charge register separately;
 * location varies by firmware version.
 */
static void get_reserved_battery_low_charge(void)
{
  uint16_t data;

  upsdebugx(3, __func__);

  if (firmware_version >= 20) {
    /* fw >= 20: dedicated register at 0x2B holds the percentage directly */
    I2C_READ_WORD(upsfd, BATTERY_LOW_CHARGE_CMD_V20, __func__)
    if (data <= 100) {
      battery_charge_low = (uint8_t)data;
    } else {
      upsdebugx(2, "Low Charge Threshold out of range (%d%%), using default %d%%",
                data, DEFAULT_CHARGE_LOW);
      battery_charge_low = DEFAULT_CHARGE_LOW;
    }
  } else {
    /* fw < 20: read from legacy 0xEF register; use configured value if the
     * BATTERY_LOW_CHARGE_CONFIGURED flag is set, otherwise default to 10%.
     */
    I2C_READ_WORD(upsfd, RESERVED_BATTERY_LOW_CHARGE_CMD_LEGACY, __func__)
    if (data & BATTERY_LOW_CHARGE_CONFIGURED) {
      uint8_t threshold = data & BATTERY_LOW_CHARGE_MASK;
      if (threshold <= 100) {
        battery_charge_low = threshold;
      } else {
        battery_charge_low = DEFAULT_CHARGE_LOW;
      }
    } else {
      battery_charge_low = DEFAULT_CHARGE_LOW;
    }
  }

  upsdebugx(1, "Low Charge Threshold: %d%%", battery_charge_low);
  dstate_setinfo("battery.charge.low", "%d", battery_charge_low);
}

static void reset_shutdown_restart_timers(void)
{
  upsdebugx(3, __func__);
  
  I2C_WRITE_BYTE(upsfd, RESTART_TIMER_CMD, 0x0, __func__)
  I2C_WRITE_BYTE(upsfd, SHUTDOWN_TIMER_CMD, 0x0, __func__)
  
  /* Invalidate live memory buffer since we wrote to live registers */
  memory_initialized = 0;
  upsdebugx(2, "Invalidated memory buffer after resetting timer registers");
}

static void get_power_off_timer(void)
{
  uint8_t data;
  
  upsdebugx(3, __func__);
  
  /* Read from memory buffer instead of I2C */
  data = get_memory_byte(SHUTDOWN_TIMER_CMD - UPSPLUS_MEMORY_START);
  if (data == 0 && !memory_initialized) {
    upsdebugx(2, "Memory buffer not available for shutdown timer, skipping");
    return;
  }
  
  /* Validate timer value according to spec: 0 (not running) or 10-255 (running) on
  corrected firmware (>= 20); original vendor firmware has no enforced minimum, so any
  nonzero value below 10 seen there is a legitimate mid-countdown reading, not garbage. */
  /* See spirous 255 values appear, if so assume they are bad */
  if ((data != 0 && firmware_version >= 20 && data < TIMER_MINIMUM_FIXED_FW) || data == 255) {
    upsdebugx(1, "Invalid shutdown timer value: %ds (spec: 0 or 10-255), resetting to 0", data);
    data = 0;
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
  
  /* Invalidate live memory buffer since we wrote to a live register */
  memory_initialized = 0;
  upsdebugx(2, "Invalidated memory buffer after writing to register 0x%02X", cmd);
  
  upsdebugx(1, "Set Shutdown Timer: %ds", data);
  dstate_setinfo("ups.timer.shutdown", "%d", data);
}

static void get_reboot_timer(void)
{
  uint8_t data;
  
  upsdebugx(3, __func__);
  
  /* Read from memory buffer instead of I2C */
  data = get_memory_byte(RESTART_TIMER_CMD - UPSPLUS_MEMORY_START);
  if (data == 0 && !memory_initialized) {
    upsdebugx(2, "Memory buffer not available for restart timer, skipping");
    return;
  }
  
  /* Validate timer value according to spec: 0 (not running) or 10-255 (running) on
  corrected firmware (>= 20); original vendor firmware's reboot timer only ever actually
  fires at exactly 5, so a mid-countdown reading below 10 there is legitimate, not garbage. */
  /* See spirous 255 values appear, if so assume they are bad */
  if ((data != 0 && firmware_version >= 20 && data < TIMER_MINIMUM_FIXED_FW) || data == 255) {
    upsdebugx(1, "Invalid reboot timer value: %ds (spec: 0 or 10-255)", data);
    data = 0;
  }
  
  upsdebugx(1, "Reboot Timer: %ds", data);
  dstate_setinfo("ups.timer.reboot", "%d", data);
}

static void get_load_on_delay_v20(void)
{
  uint8_t data;
  
  upsdebugx(3, __func__);
  
  /* Read from memory buffer instead of I2C */
  data = get_memory_byte(LOAD_ON_DELAY_CMD_V20 - UPSPLUS_MEMORY_START);
  if (data == 0 && !memory_initialized) {
    upsdebugx(2, "Memory buffer not available for load on delay, skipping");
    return;
  }
  
  upsdebugx(1, "Load On Delay: %ds", data);
  dstate_setinfo("ups.delay.start", "%d", data);
}

static void set_reboot_timer(const short data)
{
  uint8_t cmd = RESTART_TIMER_CMD;
  
  upsdebugx(3, __func__);
  
  reset_shutdown_restart_timers();
  I2C_WRITE_BYTE(upsfd, cmd, data, __func__)
  
  /* Invalidate live memory buffer since we wrote to a live register */
  memory_initialized = 0;
  upsdebugx(2, "Invalidated memory buffer after writing to register 0x%02X", cmd);
  
  upsdebugx(1, "Set Reboot Timer: %ds", data);
  dstate_setinfo("ups.timer.reboot", "%d", data);
}

static void set_load_on_delay_v20(const short data)
{
  uint8_t cmd = LOAD_ON_DELAY_CMD_V20;
  
  upsdebugx(3, __func__);
  
  I2C_WRITE_BYTE(upsfd, cmd, data, __func__)
  
  /* Invalidate live memory buffer since we wrote to a live register */
  memory_initialized = 0;
  upsdebugx(2, "Invalidated memory buffer after writing to register 0x%02X", cmd);
  
  upsdebugx(1, "Set Load On Delay: %ds", data);
}

static void get_ups_auto_restart(void)
{
  uint8_t data = ups_auto_restart;
  
  if (ups_auto_restart < 0) {
    upsdebugx(3, __func__);
    /* Read from memory buffer instead of I2C */
    data = get_memory_byte(WAKEUP_ON_CHARGE_CMD - UPSPLUS_MEMORY_START);
    if (data == 0 && !memory_initialized) {
      upsdebugx(2, "Memory buffer not available for wakeup on charge, skipping");
      return;
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
  
  /* Invalidate live memory buffer since we wrote to a live register */
  memory_initialized = 0;
  upsdebugx(2, "Invalidated memory buffer after writing to register 0x%02X", cmd);
  
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
  
  /* Read 32-bit value from memory buffer using get_memory_dword() */
  uptime = get_memory_dword(UPTIME_CMD - UPSPLUS_MEMORY_START);
  
  if (uptime == 0 && !memory_initialized) {
    upsdebugx(2, "Memory buffer not available for uptime, skipping");
    return;
  }
  
  upsdebugx(1, "Device uptime: %ds", uptime);
  dstate_setinfo("device.uptime", "%d", uptime);
}

static void check_operating_state(void)
{
  /* Use last-sampled MCU values from memory buffer */
  uint16_t usbc_mv     = get_memory_word(USBC_VOLTAGE_CMD - UPSPLUS_MEMORY_START);
  uint16_t microusb_mv = get_memory_word(MICROUSB_VOLTAGE_CMD - UPSPLUS_MEMORY_START);
  
  upsdebugx(3, "Checking operating state with sampled USB-C: %0.3fV, MicroUSB: %0.3fV",
            usbc_mv / 1000.0, microusb_mv / 1000.0);

  /* Non-standard fields: both input rails are sampled by the MCU regardless of which
   * one is actually powering the UPS, so publish both independent of input.voltage
   * (which only reflects whichever source is active). */
  dstate_setinfo("input.voltage.usbc", "%0.3f", usbc_mv / 1000.0);
  dstate_setinfo("input.voltage.microusb", "%0.3f", microusb_mv / 1000.0);

  if (usbc_mv > USB_VOLTAGE_MINIMUM) {
    power_state = USBC_POWER_CONNECTED;
    upsdebugx(1, "USB-C Input Voltage: %0.3fV", usbc_mv / 1000.0);
    if (usbc_mv <= USB_VOLTAGE_MAXIMUM) {
      dstate_setinfo("input.voltage.nominal", "%0.3f", USBC_NOMINAL_VOLTAGE);
      dstate_setinfo("input.voltage", "%0.3f", usbc_mv / 1000.0);
    } else {
      upsdebugx(2, "Input Voltage out of range, skipping.");
    }
  } else if (microusb_mv > USB_VOLTAGE_MINIMUM) {
    power_state = MICROUSB_POWER_CONNECTED;
    upsdebugx(1, "MicroUSB Input Voltage: %0.3fV", microusb_mv / 1000.0);
    if (microusb_mv <= USB_VOLTAGE_MAXIMUM) {
      dstate_setinfo("input.voltage.nominal", "%0.3f", MICROUSB_NOMINAL_VOLTAGE);
      dstate_setinfo("input.voltage", "%0.3f", microusb_mv / 1000.0);
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
  /* Status = sample-timed MCU registers (via memory buffer) + real-time INA219 current */
  
  I2C_WRITE_BYTE(upsfd, cmd, 0x1, __func__)
  
  /* Invalidate live memory buffer since we wrote to a live register */
  memory_initialized = 0;
  upsdebugx(2, "Invalidated memory buffer after writing to register 0x%02X", cmd);
}

static void reset_battery(void)
{
  uint8_t cmd = BATTERY_PARAM_CUSTOM_CMD;
  
  upsdebugx(3, __func__);
  
  I2C_WRITE_BYTE(upsfd, cmd, BATTERY_PARAM_CUSTOM_DISABLE, __func__)
  
  /* Invalidate live memory buffer since we wrote to a live register */
  memory_initialized = 0;
  upsdebugx(2, "Invalidated memory buffer after writing to register 0x%02X", cmd);
}

/*
 Always reset timers back to zero before trying to set them. Others
 have reported issues with setting a timer that is already running with
 a value other than zero.
 */
static int upsplus_setvar(const char *key, const char *value)
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
    short min_timer = (firmware_version < 20) ? TIMER_MINIMUM_LEGACY_FW : TIMER_MINIMUM_FIXED_FW;
    if (str_to_short(value, &data, 10) && data >= min_timer) {
      set_power_off_timer(data);
      return STAT_SET_HANDLED;
    }
    upsdebugx(1, "Unknown value for ups.timer.shutdown: %s", value);
    return STAT_SET_UNKNOWN;
  }
  
  if (!strcasecmp(key, "ups.timer.reboot")) {
    short min_timer = (firmware_version < 20) ? TIMER_MINIMUM_LEGACY_FW : TIMER_MINIMUM_FIXED_FW;
    if (str_to_short(value, &data, 10) && data >= min_timer) {
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
static int upsplus_instcmd(const char *cmd, const char *reserved)
{
  short data;
  
  upsdebugx(2, "In %s with %s and extra %s.", __func__, cmd, reserved);
  
  if (!strcasecmp(cmd, "load.off.delay")) {
    set_power_off_timer((firmware_version < 20) ? TIMER_MINIMUM_LEGACY_FW : TIMER_MINIMUM_FIXED_FW);
    return STAT_INSTCMD_HANDLED;
  }

  if (!strcasecmp(cmd, "load.on.delay")) {
    if (firmware_version < 20) {
      upsdebugx(1, "load.on.delay not supported on firmware %d", firmware_version);
      return STAT_INSTCMD_UNKNOWN;
    }
    if (reserved == NULL) {
      upsdebugx(1, "load.on.delay requires a delay value");
      return STAT_INSTCMD_UNKNOWN;
    }
    if (!str_to_short(reserved, &data, 10) || data < 0 || data > 255) {
      upsdebugx(1, "Unknown value for load.on.delay: %s", reserved);
      return STAT_INSTCMD_UNKNOWN;
    }
    set_load_on_delay_v20(data);
    return STAT_INSTCMD_HANDLED;
  }
  
  if (!strcasecmp(cmd, "shutdown.return")) {
    /* Reboot timer ignores auto restart setting, and always restarts */
    if (firmware_version < 20) {
      /* Original vendor firmware: reboot timer is broken and only
      works if the timer is set to 5 seconds */
      set_reboot_timer(TIMER_MINIMUM_LEGACY_FW);
    } else {
      /* Corrected firmware: 5 seconds is below the valid 10-255s
      range and is silently rejected, so use the minimum instead */
      set_reboot_timer(TIMER_MINIMUM_FIXED_FW);
    }
    return STAT_INSTCMD_HANDLED;
  }
  
  if (!strcasecmp(cmd, "shutdown.stayoff")) {
    set_ups_auto_restart(WAKEUP_ON_CHARGE_DISABLE);
    set_power_off_timer((firmware_version < 20) ? TIMER_MINIMUM_LEGACY_FW : TIMER_MINIMUM_FIXED_FW);
    return STAT_INSTCMD_HANDLED;
  }
  
  if (!strcasecmp(cmd, "shutdown.stop")) {
    reset_shutdown_restart_timers();
    return STAT_INSTCMD_HANDLED;
  }
  
  if (!strcasecmp(cmd, "shutdown.reboot.graceful")) {
    if (firmware_version < 20) {
      /* Original vendor firmware: reboot timer is broken, it will
      only work if the timer is set to 5 seconds or less */
      set_reboot_timer(TIMER_MINIMUM_LEGACY_FW);
    } else {
      /* Corrected firmware: 5 seconds is below the valid 10-255s
      range and is silently rejected, so use the minimum instead */
      set_reboot_timer(TIMER_MINIMUM_FIXED_FW);
    }
    return STAT_INSTCMD_HANDLED;
  }
  
  return STAT_INSTCMD_UNKNOWN;
}

void upsdrv_initinfo(void)
{
  int memory_result;

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
  /* version. First read memory to populate the buffers */
  upsdebugx(2, "Reading initial memory to detect device");
  memory_result = read_upsplus_memory();
  upsdebugx(2, "Initial memory read result: %d", memory_result);
  
  get_firmware_version();
  if (firmware_version == 0) {
    fatal_with_errno(-1, "Failed to find UPSPlus on i2c bus, exiting.\n");
  }
  get_serial_number();
  
  /* Memory is updated on every upsdrv_updateinfo() call for maximum freshness */
  upsdebugx(1, "Memory will be updated on every upsdrv_updateinfo() call");
  
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
  if (firmware_version >= 20) {
    dstate_addcmd("load.on.delay");/* Startup countdown (60) */
  }
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
  int critical_result;
  int memory_result;

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
  critical_result = read_critical_data();
  upsdebugx(2, "read_critical_data() returned: %d", critical_result);

  /* Read entire memory buffer with double-read validation - updated on every call */
  /* Note: Critical data (power status, battery voltage/current, input/output voltage) */
  /* is also updated every 2 seconds, so status determination uses fresh data */
  memory_result = read_upsplus_memory();
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
  if (firmware_version >= 20) {
    get_load_on_delay_v20();
  }
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
  printf("Unified bulk caching: all regs 0x01-0xFB are bulk-read with double-read validation to handle device update cycle corruption.\n");
  printf("All MCU values are served from validated cache. On firmware < 29 the INA219 current/power\n");
  printf("sensors are polled directly for real-time electrical readings; firmware >= 29 publishes\n");
  printf("calibrated current via its own registers, so this driver no longer talks to the INA219 chips.\n");
  printf("This minimizes I2C chatter while ensuring data integrity through validation.\n");
  printf("Tune with: criticalinterval (seconds).\n");
  printf("\n");
}

void upsdrv_makevartable(void)
{
  addvar(VAR_FLAG, "factoryreset", "Reset UPSPlus to factory settings and exit.");
  
  addvar(VAR_FLAG, "batteryreset", "Reset Battery min/max/low to automatic settings and exit.");
  
  addvar(VAR_VALUE, "sampleperiod", "Set number of minutes between sampling battery state (2 - 1440, default: 2)");
  
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
      upsdebugx(1, "Updated sample period to: %d minutes", sampleperiod);
      

    } else {
      upsdebugx(1, "Ignoring sampleperiod, out of range: %s", getval("sampleperiod"));
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
}

void upsdrv_cleanup(void)
{
  close(upsfd);
}

/* optionally tweak prognames[] entries */
void upsdrv_tweak_prognames(void)
{
}


