#pragma once

#define HAL_BOARD_NAME "URUS"

/* INTERNAL SHAL CORE TYPES DEFINES */
#define SHAL_CORE_CYGWIN    1
#define SHAL_CORE_ANDROID   2
#define SHAL_CORE_APM       3

#if CONFIG_SHAL_CORE == SHAL_CORE_CYGWIN
#define HAL_BOARD_LOG_DIRECTORY "logs"
#define HAL_BOARD_TERRAIN_DIRECTORY "terrain"
#define HAL_PARAM_DEFAULTS_PATH "etc/defaults.parm"
#define HAL_CPU_CLASS HAL_CPU_CLASS_1000
#define HAL_STORAGE_SIZE            16384
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE
#define HAL_OS_POSIX_IO         1
#define HAL_HAVE_BOARD_VOLTAGE  1
#define HAL_GPIO_LED_ON           LOW
#define HAL_GPIO_LED_OFF          HIGH
#define HAL_INS_DEFAULT HAL_INS_HIL
#define HAL_BARO_DEFAULT HAL_BARO_URUS
#define HAL_COMPASS_DEFAULT HAL_COMPASS_URUS
#elif (CONFIG_SHAL_CORE == SHAL_CORE_APM) && defined(SHAL_CORE_APM2)
#define HAL_CPU_CLASS HAL_CPU_CLASS_16
#define HAL_STORAGE_SIZE            4096
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE
#define HAL_INS_DEFAULT HAL_INS_MPU60XX_SPI
#define HAL_INS_MPU60x0_NAME "MPU60XX_SPI0"
#define HAL_BARO_DEFAULT HAL_BARO_MS5611_SPI
#define HAL_BARO_MS5611_NAME "MS5611_SPI0"
#define HAL_COMPASS_DEFAULT HAL_COMPASS_HMC5843
#define HAL_COMPASS_HMC5843_I2C_ADDR 0x1E
#define HAL_COMPASS_HMC5843_I2C_BUS 0
#define HAL_HAVE_BOARD_VOLTAGE  0
#define DATAFLASH_NO_CLI 1
#define EXTERNAL_LED_ARMED      61    // Armed LED - AN7
#define EXTERNAL_LED_GPS        60    // GPS LED - AN6
#define EXTERNAL_LED_MOTOR1     58    // Motor1 LED - AN4
#define EXTERNAL_LED_MOTOR2     62    // Motor2 LED - AN8
#define HAL_GPIO_A_LED_PIN      27
#define HAL_GPIO_B_LED_PIN      26
#define HAL_GPIO_C_LED_PIN      25
#define HAL_GPIO_LED_ON         LOW
#define HAL_GPIO_LED_OFF        HIGH
#define HAL_GPIO_USB_MUX_PIN 23
#elif (CONFIG_SHAL_CORE == SHAL_CORE_APM) && defined(SHAL_CORE_APM2_HIL)
#define HAL_CPU_CLASS HAL_CPU_CLASS_16
#define HAL_STORAGE_SIZE            4096
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE
#define HAL_INS_DEFAULT HAL_INS_HIL
#define HAL_BARO_DEFAULT HAL_BARO_URUS
#define HAL_COMPASS_DEFAULT HAL_COMPASS_URUS
#define HAL_HAVE_BOARD_VOLTAGE  0
#define DATAFLASH_NO_CLI 1
#define HAL_GPIO_LED_ON           HIGH
#define HAL_GPIO_LED_OFF          LOW
#elif (CONFIG_SHAL_CORE == SHAL_CORE_APM) && defined(SHAL_CORE_MEGA01)
#define HAL_CPU_CLASS HAL_CPU_CLASS_16
#define HAL_STORAGE_SIZE            4096
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE
#define HAL_INS_DEFAULT HAL_INS_MPU60XX_I2C
#define HAL_INS_MPU60x0_I2C_BUS 0
#define HAL_INS_MPU60x0_I2C_ADDR 0x68
#define HAL_BARO_DEFAULT HAL_BARO_MS5611_I2C
#define HAL_BARO_MS5611_I2C_BUS 0
#define HAL_BARO_MS5611_I2C_ADDR 0x77
#define HAL_COMPASS_DEFAULT HAL_COMPASS_HMC5843
#define HAL_COMPASS_HMC5843_I2C_ADDR 0x1E
#define HAL_COMPASS_HMC5843_I2C_BUS 0
#define HAL_HAVE_BOARD_VOLTAGE  0
#define DATAFLASH_NO_CLI 1
#define EXTERNAL_LED_ARMED      61    // Armed LED - AN7
#define EXTERNAL_LED_GPS        60    // GPS LED - AN6
#define EXTERNAL_LED_MOTOR1     58    // Motor1 LED - AN4
#define EXTERNAL_LED_MOTOR2     62    // Motor2 LED - AN8
#define HAL_GPIO_A_LED_PIN      13
#define HAL_GPIO_B_LED_PIN      31
#define HAL_GPIO_C_LED_PIN      30
#define HAL_GPIO_LED_ON         HIGH
#define HAL_GPIO_LED_OFF        LOW
#define HAL_GPIO_USB_MUX_PIN -1
#elif (CONFIG_SHAL_CORE == SHAL_CORE_APM) && defined(SHAL_CORE_MEGA02)
#define HAL_CPU_CLASS HAL_CPU_CLASS_16
#define HAL_STORAGE_SIZE            4096
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE
#define HAL_INS_DEFAULT HAL_INS_HIL
#define HAL_BARO_DEFAULT HAL_BARO_BMP085
#define HAL_BARO_BMP085_BUS 0
#define HAL_BARO_BMP085_I2C_ADDR 0x77
#define HAL_COMPASS_DEFAULT HAL_COMPASS_HMC5843
#define HAL_COMPASS_HMC5843_I2C_ADDR 0x1E
#define HAL_COMPASS_HMC5843_I2C_BUS 0
#define HAL_HAVE_BOARD_VOLTAGE  0
#define DATAFLASH_NO_CLI 1
#define HAL_GPIO_A_LED_PIN      30
#define HAL_GPIO_B_LED_PIN      31
#define HAL_GPIO_C_LED_PIN      32
#define HAL_GPIO_LED_ON         LOW
#define HAL_GPIO_LED_OFF        HIGH
#define HAL_GPIO_USB_MUX_PIN -1
#define HIL_SUPPORT 1
#elif (CONFIG_SHAL_CORE == SHAL_CORE_APM) && defined(SHAL_CORE_APM328)
#define HAL_CPU_CLASS HAL_CPU_CLASS_16
#define HAL_STORAGE_SIZE            1024
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE
#define HAL_INS_DEFAULT HAL_INS_HIL
#define HAL_BARO_DEFAULT HAL_BARO_URUS
#define HAL_COMPASS_DEFAULT HAL_COMPASS_URUS
#define HAL_HAVE_BOARD_VOLTAGE  0
#define DATAFLASH_NO_CLI 1
#define HAL_GPIO_LED_ON           HIGH
#define HAL_GPIO_LED_OFF          LOW
#define HIL_SUPPORT 1
#define HAL_GPIO_USB_MUX_PIN -1
#elif (CONFIG_SHAL_CORE == SHAL_CORE_APM) && defined(SHAL_CORE_APM16U)
#define HAL_CPU_CLASS HAL_CPU_CLASS_16
#define HAL_STORAGE_SIZE            1024
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE
#define HAL_INS_DEFAULT HAL_INS_HIL
#define HAL_BARO_DEFAULT HAL_BARO_URUS
#define HAL_COMPASS_DEFAULT HAL_COMPASS_URUS
#define HAL_HAVE_BOARD_VOLTAGE  0
#define DATAFLASH_NO_CLI 1
#define HAL_GPIO_LED_ON           HIGH
#define HAL_GPIO_LED_OFF          LOW
#define HIL_SUPPORT 1
#define HAL_GPIO_USB_MUX_PIN -1
#else
#error "no URUS SHAL CORE set"
#endif
