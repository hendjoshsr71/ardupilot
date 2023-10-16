# Modal AI FC v2

# FIXME
# Does ModalAI have its own USB Vendor ID?
# USB setup
# USB_VENDOR  # 0x2DAE # ONLY FOR USE BY HEX! NOBODY ELSE
# USB_PRODUCT # 0x1016
USB_STRING_MANUFACTURER "ModalAI_FCV2"

# crystal frequency
OSCILLATOR_HZ 16000000  # PHO & PH1, OSC_IN & OSC_OUT


# FIXME CHECK THIS 
# ChibiOS system timer
STM32_ST_USE_TIMER 2

# board ID for firmware load
APJ_BOARD_ID 41776

# default to all pins low to avoid ESD issues
DEFAULTGPIO OUTPUT LOW PULLDOWN


FLASH_RESERVE_START_KB 128

# flash size
FLASH_SIZE_KB 2048

# with 2M flash we can afford to optimize for speed
env OPTIMIZE -O2

# enable RAMTROM parameter storage
define HAL_STORAGE_SIZE 32768
define HAL_WITH_RAMTRON 1


########### UARTS ##########

# order of UARTs (and USB)
#            USB1_1  TELEM1  TELEM2  GPS1    RCIN    Debug   USB1_2
SERIAL_ORDER OTG1    UART7   UART5   USART1  USART6  USART3  OTG2

# USB
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
PA9 VBUS INPUT OPENDRAIN    # This input pin is used to detect that power is valid on USB.

# UART7 Telem 1, TELEM_UART_4W
PF6 UART7_RX UART7
PE8 UART7_TX UART7
PF8 UART7_RTS UART7
PE10 UART7_CTS UART7

# UART5 Telem 2, MSS_UART_4W
PD2 UART5_RX UART5
PC12 UART5_TX UART5
PC8 UART5_RTS UART5
PC9 UART5_CTS UART5

# USART1 GPS1, UART_2W
PB7 USART1_RX USART1 NODMA
PB6 USART1_TX USART1

# USART6, spektrum
# PC7 PIN# G15
PC7 USART6_RX USART6 NODMA GPIO(74)
PC6 USART6_TX USART6

# USART3 Debug, UART_2W_DEBUG
PD9 USART3_RX USART3 NODMA
PD8 USART3_TX USART3

########## END UARTS ##########

# Debug Pins
# Note Pin 8 of J2 connector is Boot0 and is pulled down on the FCV2
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD


########## I2C Ports ##########

# order of I2C buses
#         Internal | Modal APM | EXPANSION_I2C  | EXT_GPS_I2C
I2C_ORDER I2C4       I2C3        I2C2             I2C1

# I2C4, BMP388 BARO (ADDR 0x76), ICP-20100 BARO (ADDR 0x63), & BMM150 MAG (0x10 through 0x13 possible)
PF14 I2C4_SCL I2C4
PF15 I2C4_SDA I2C4

# I2C3, MODAL_APM_I2C
PA8 I2C3_SCL I2C3
PH8 I2C3_SDA I2C3

# I2C2, EXPANSION_I2C
PF1 I2C2_SCL I2C2
PF0 I2C2_SDA I2C2

# I2C1, EXT_GPS_I2C
PB8 I2C1_SCL I2C1
PB9 I2C1_SDA I2C1

define HAL_I2C_INTERNAL_MASK 1

### I2C Sensors: I2C4
BARO BMP388 I2C:0:0x76
BARO2 ICP201XX I2C:0:0x63
COMPASS BMM150 I2C:0:0x10 false ROTATION_NONE
PF3 MAG_DRDY INPUT PULLDOWN

### Enable external compasses
define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define HAL_COMPASS_AUTO_ROT_DEFAULT 2

########## END I2C Ports ##########


########## SPI Ports ##########
/* SPI
 * SPI1 : ICM_42688_1
 * SPI2 : ICM_42688_2
 * SPI3 : Not Used
 * SPI4 : Not Used
 * SPI5 : FRAM 256 Kb
 * SPI6 : External1 Reserved (DISABLED)
 *
 */

# SPI1 - ICM_42688_1
PA5 SPI1_SCK SPI1
PG9 SPI1_MISO SPI1
PB5 SPI1_MOSI SPI1

PI9 ICM_42688_1_CS CS
PF2 IMU_1_DRDY INPUT PULLDOWN

# SPI2 - ICM42688_2
PI1 SPI2_SCK SPI2
PI2 SPI2_MISO SPI2
PI3 SPI2_MOSI SPI2

PH5 ICM_42688_2_CS CS
PA10 IMU_2_DRDY INPUT PULLDOWN

# SPI5 - FRAM
PF7 SPI5_SCK SPI5
PH7 SPI5_MISO SPI5
PF11 SPI5_MOSI SPI5

# The CS pin for FRAM (ramtron). This one is marked as using
# SPEED_VERYLOW, which matches the HAL_PX4 setup.
PG7 FRAM_CS CS SPEED_VERYLOW

# Disable for now
# SPI6 External
# PB3 SPI6_SCK SPI6
# PA6 SPI6_MISO SPI6
# PG14 SPI6_MOSI SPI6
# PI10 GYRO_EXT_CS CS

# FIXME
# FSYNC SPI1 & SPI2 for ICM42688p
# TIM14_CH1 out option for setting a 32K CLK from the timer.
# To use, Drive FSYNC signal LOW (or leave tri-state). PX4 uses this as a Piezo Buzzer.
# PE9 TIM1_CH1 TIM1 IMU1_2_FSYNC PULLDOWN LOW
# PF9 TIM14_CH1 TIM14

#define GPIO_SPI4_MAG_INT           /* PD12 */   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN12)

#define GPIO_SYNC                   /* PE9  */   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_100MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN9)

## SPI Devices
SPIDEV icm42688_1     SPI1 DEVID1  ICM_42688_1_CS  MODE3  2*MHZ  8*MHZ
SPIDEV icm42688_2     SPI2 DEVID1  ICM_42688_2_CS  MODE3  2*MHZ  8*MHZ
SPIDEV ramtron        SPI5 DEVID1  FRAM_CS         MODE3  8*MHZ  8*MHZ

# SPI6 disabled due to lack of DMA channels
#SPIDEV bmi088_g      SPI6 DEVID3  BMI088_G_CS     MODE3 10*MHZ 10*MHZ

# All Internal IMUs Enabled
IMU Invensensev3 SPI:icm42688_1 ROTATION_NONE
IMU Invensensev3 SPI:icm42688_2 ROTATION_NONE

########## END SPI Ports ##########


########## Interrupt Pins ##########
PG5  BARO_INT INPUT PULLDOWN # BMP388 DRDY

########## END Interrupt Pins ##########


########## CAN BUS ##########

# CAN bus
# No FD CAN, 1Mbps Transceiver
PD0 CAN1_RX CAN1
PD1 CAN1_TX CAN1
# PD15 GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW GPIO(70)

########## END CAN BUS ##########


########## RC Input ##########

# setup for spektrum satellite bind
define HAL_GPIO_SPEKTRUM_RC 74
PH2 SPEKTRUM_PWR OUTPUT HIGH GPIO(73)
define HAL_GPIO_SPEKTRUM_PWR 73
define HAL_SPEKTRUM_PWR_ENABLED 1

########## END RC Input ##########


########## PWM Channels ##########
# Now we start defining some PWM pins. We also map these pins to GPIO
# values, so users can set SERVOx_FUNCTION=-1 to determine which
# PWM outputs on the primary MCU are set up as GPIOs.
# To match HAL_PX4 we number the GPIOs for the PWM outputs
# starting at 50.
PI0  TIM5_CH4  TIM5  PWM(1) GPIO(50)
PH12 TIM5_CH3  TIM5  PWM(2) GPIO(51)
PH11 TIM5_CH2  TIM5  PWM(3) GPIO(52)
PH10 TIM5_CH1  TIM5  PWM(4) GPIO(53)
PD13 TIM4_CH2  TIM4  PWM(5) GPIO(54)
PD14 TIM4_CH3  TIM4  PWM(6) GPIO(55)

# FIXME : taken from ARKV6X
# we need to disable DMA on the last 2 FMU channels
# as timer 12 doesn't have a TIMn_UP DMA option
PH6  TIM12_CH1 TIM12 PWM(7) GPIO(56) NODMA
PH9  TIM12_CH2 TIM12 PWM(8) GPIO(57) NODMA

########## END PWM Channels ##########


########## LED Channels ##########

# RGB LED
PE3 LED_RED OUTPUT OPENDRAIN GPIO(90) HIGH
PE4 LED_GREEN OUTPUT OPENDRAIN GPIO(91) HIGH
PE5 LED_BLUE OUTPUT OPENDRAIN GPIO(92) HIGH

# setup for "pixracer" RGB LEDs
define AP_NOTIFY_GPIO_LED_RGB_RED_PIN 90
define AP_NOTIFY_GPIO_LED_RGB_GREEN_PIN 91
define AP_NOTIFY_GPIO_LED_RGB_BLUE_PIN 92

define AP_NOTIFY_GPIO_LED_RGB_ENABLED 1

########## END LED Channels ##########


########## UNUSED PINS ##########

###### Port A
# AVIATOR Expansion Pins Port
PA0 AVIATOR_ADC1_CH16 INPUT FLOATING
PA4 AVIATOR_ADC1_CH18 INPUT FLOATING

###### Port B & C
PB0 AVIATOR_ADC1_CH9 INPUT FLOATING

###### Port D & E

###### Port F & G
PF12 AVIATOR_ADC1_CH6 INPUT FLOATING

# PX4 HWID pins, unused
PH3 HW_VER_SENS INPUT FLOATING          # ADC3_INP14
PH4 HW_REV_SENS INPUT FLOATING          # ADC3_INP15
PG0 HW_VER_REV_DRIVE INPUT FLOATING

# LSE crystal, 32.768kHz unused
PC14 LSE1 INPUT FLOATING
PC15 LSE2 INPUT FLOATING

########## END UNUSED PINS ##########


########## Storage: SDCard ###########

# Setup the pins for the microSD card
include sdcard.inc

# Enable FAT filesystem support (needs a microSD defined via SDMMC).
define HAL_OS_FATFS_IO 1
define FATFS_HAL_DEVICE SDCD2

########## END Storage: SDCard ###########


######### Power Input Detection & Measurement #########

# The VDD sense pin. This is used to sense primary board voltage.
PB1 VDD_5V_SENS ADC1 SCALE(2)       # ADC1_CH(5)

# FIXME PC3_C not handled in script
#PC3_C SCALED_V3V3 ADC3 SCALE(2)      # ADC3_INP1

define ANALOG_VCC_5V_PIN 5

# Power flag pins: these tell the MCU the status of the various power
# supplies that are available. The pin names need to exactly match the
# names used in AnalogIn.cpp. 
# FIXME High when powered by DC power connector, low when powered by USB
PG1 VDD_BRICK_VALID INPUT PULLUP

### Battery

# from px4
#define VOXLPM_INA231_ADDR_P5_12VDC		0x45

#  * VOXLPM v2 - Note that these are unshifted addresses.
#define VOXLPM_LTC2946_ADDR_P5VD		0x6b // 0x6b  = 0xd6 >> 1

### FIX ME Enable 5V monitor

# INA231 battery monitor
define HAL_BATTMON_INA2XX_BUS 1
define HAL_BATTMON_INA2XX_ADDR 0x44
define HAL_BATT_MONITOR_DEFAULT 21

# on some units a LTC2946 is used
define HAL_BATTMON_LTC2946_BUS 1
define HAL_BATTMON_LTC2946_ADDR 0x6a

######### END Power Input Detection & Measurement #########

######### Analog to Digital Conversion #########

define HAL_USE_ADC FALSE

# FIXME Not sure about this
# define STM32_ADC_USE_ADC1 FALSE
define HAL_DISABLE_ADC_DRIVER TRUE

######### END Analog to Digital Conversion #########


########## Settings ##########

### IMU Settings
# Enable all IMUs to be used, two active EKF Lanes
# Enable fast sampling on both IMUs
define HAL_EKF_IMU_MASK_DEFAULT 3
define HAL_DEFAULT_INS_FAST_SAMPLE 3

########## END Settings ##########


########## Board Validation Checks ##########

# Sensor Check alias for validating board type
# CHECK_ICM20649          spi_check_register_inv2("icm20948", INV2REG_WHOAMI, INV2_WHOAMI_ICM20649)
# CHECK_ICM20602_EXT      spi_check_register("icm20602_ext", MPUREG_WHOAMI, MPU_WHOAMI_ICM20602)
# CHECK_ICM20948_EXT      spi_check_register_inv2("icm20948_ext", INV2REG_WHOAMI, INV2_WHOAMI_ICM20948)
# CHECK_MS5611            check_ms5611("ms5611")
# CHECK_MS5611_EXT        check_ms5611("ms5611_ext")

# Sensor Check Macros to be used for validating board type
# CHECK_IMU0_PRESENT   $CHECK_ICM20602_EXT
# CHECK_IMU1_PRESENT   $CHECK_ICM20948_EXT
# CHECK_IMU2_PRESENT   $CHECK_ICM20649
# CHECK_BARO0_PRESENT  $CHECK_MS5611
# CHECK_BARO1_PRESENT  $CHECK_MS5611_EXT

# BOARD_VALIDATE $CHECK_IMU0_PRESENT $CHECK_IMU1_PRESENT $CHECK_IMU2_PRESENT $CHECK_BARO0_PRESENT $CHECK_BARO1_PRESENT

########## END Board Validation Checks ##########


########## DMA Priority ##########

DMA_PRIORITY SPI1* SPI2* SDMMC* USART6* ADC* UART* USART*
DMA_NOSHARE SPI1* SPI2*

########## END DMA Priority ##########


########## DFU Reboot ##########
# enable DFU reboot for installing bootloader
# note that if firmware is build with --secure-bl then DFU is
# disabled
ENABLE_DFU_BOOT 1

########## END DFU Reboot ##########

#### Unconnected Pins
# ###### Port A
# PA1
# PA2
# PA3
# PA7
# PA10
# PA15

# ###### Port B & C
# PB2
# PB10
# PB11
# PB12
# PB13

# PC0
# PC1
# PC2_C
# PC4
# PC5
# PC10
# PC11
# PC13 INPUT FLOATING

# ###### Port D & E
# PD3
# PD4
# PD5
# PD10
# PD11
# PD12

# PE0
# PE1
# PE2
# PE6
# PE7
# PE11
# PE12
# PE13
# PE14
# PE15

# ###### Port F & G
# PF4
# PF5
# PF10
# PF13
# PG2
# PG3
# PG4
# PG6
# PG8
# PG10
# PG12
# PG13
# PG15

# ###### Port H & I
# PH13
# PH14
# PH15

# PI4
# PI5
# PI6
# PI7
# PI8
# PI11
