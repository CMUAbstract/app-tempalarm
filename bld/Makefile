EXEC = tempalarm

OBJECTS = \
	main.o \

DEPS += \
	libchain \
	libcapybara \
	libfxl \
	libmspware \
	libmcppot \
	libmspuartlink \
	libio \
	libmsp \

# Local config

# Build version for reference board (continuously-powered) for experiements
CONFIG_REF = 0

# Capacity mode:
#   * reconfig: reconfigurable, manually reconfigured in tasks
#   * small: fixed capacity set to cap needed for smallest task
#   * big: fixed capacity set to cap needed for largest task
CONFIG_CAP = reconf

ifeq ($(CONFIG_CAP),reconf)
override CFLAGS += -DCONFIG_CAP_RECONF
else ifeq ($(CONFIG_CAP),small)
override CFLAGS += -DCONFIG_CAP_SMALL
else ifeq ($(CONFIG_CAP),big)
override CFLAGS += -DCONFIG_CAP_BIG
else # CONFIG_CAP
$(error Invalid CONFIG_CAP value)
endif # CONFIG_CAP

ifeq ($(CONFIG_REF),1)
TIMESTAMPS = 1
override CFLAGS += -DCONFIG_REF
else # !CONFIG_REF
TIMESTAMPS = 0
endif # !CONFIG_REF

#TEMP_SENSOR = internal
TEMP_SENSOR = external

export MAIN_CLOCK_FREQ = 8000000

export CLOCK_FREQ_ACLK = 32768
export CLOCK_FREQ_SMCLK = $(MAIN_CLOCK_FREQ)
export CLOCK_FREQ_MCLK = $(MAIN_CLOCK_FREQ)

export LIBMSP_CLOCK_SOURCE = DCO
export LIBMSP_DCO_FREQ = $(MAIN_CLOCK_FREQ)

export LIBMSP_SLEEP_TIMER = B.0.0
export LIBMSP_SLEEP_TIMER_CLK = ACLK
export LIBMSP_SLEEP_TIMER_DIV = 8*1

ifeq ($(TIMESTAMPS),1)
override CFLAGS += -DTIMESTAMPS
export LIBMSP_TICK_TIMER = A.0.0
export LIBMSP_TICK_TIMER_CLK = ACLK
export LIBMSP_TICK_TIMER_DIV = 8*8
endif # TIMESTAMPS

ifeq ($(TEMP_SENSOR),internal)
override CFLAGS += -DTEMP_SENSOR_INTERNAL
export LIBMSP_TEMP = 1
else ifeq ($(TEMP_SENSOR),external)
override CFLAGS += -DTEMP_SENSOR_EXTERNAL
DEPS += libtemp
export LIBMSP_TEMP = 0
export LIBTEMP_SENSOR = tmp36
export LIBTEMP_ADC_CHAN = 5 # pin 1.5
else # TEMP_SENSOR
$(error Invalid choice for TEMP_SENSOR)
endif # TEMP_SENSOR

# To use add: libmspsoftuart to DEPS
#export LIBIO_BACKEND = swuart
#export LIBMSPSOFTUART_PIN_TX = 3.6
#export LIBMSPSOFTUART_BAUDRATE = 9600
#export LIBMSPSOFTUART_CLOCK_FREQ = $(MAIN_CLOCK_FREQ)
#export LIBMSPSOFTUART_TIMER = B.0.5

export LIBIO_BACKEND = hwuart
export LIBMSP_UART_IDX = 0
export LIBMSP_UART_PIN_TX = 2.0
export LIBMSP_UART_BAUDRATE = 115200
export LIBMSP_UART_CLOCK = SMCLK

## Datalink to radio MCU
#export LIBMSP_UART_IDX = 1
#export LIBMSP_UART_PIN_TX = 2.5
#export LIBMSP_UART_BAUDRATE = 115200
#export LIBMSP_UART_CLOCK = SMCLK

export LIBMSPUARTLINK_UART_IDX = 1
export LIBMSPUARTLINK_PIN_TX = 2.5
export LIBMSPUARTLINK_BAUDRATE = 115200
export LIBMSPUARTLINK_CLK = SMCLK

ifeq ($(BOARD_MAJOR).$(BOARD_MINOR),1.0)
export LIBCAPYBARA_PIN_VBOOST_OK = 2.3
else ifeq ($(BOARD_MAJOR).$(BOARD_MINOR),1.1)
export LIBCAPYBARA_PIN_VBOOST_OK = 3.4
endif # BOARD_MAJOR.BOARD_MINOR

export LIBCAPYBARA_PIN_VBANK_OK = 2.2
export LIBCAPYBARA_PIN_BOOST_SW = 2.7
export LIBCAPYBARA_VBANK_DIV = 10:20
export LIBCAPYBARA_VBANK_COMP_REF = 1.2 # V
export LIBCAPYBARA_VBANK_COMP_SETTLE_MS = 2
export LIBCAPYBARA_VBANK_COMP_CHAN = E.11 # comparator type and channel for Vbank voltage
export LIBCAPYBARA_VBANK_COMP_PIN = 2.4
export LIBCAPYBARA_DEEP_DISCHARGE = 1.6 # V

export LIBCAPYBARA_NUM_BANKS = 4

export LIBCAPYBARA_SWITCH_DESIGN = NC
export LIBCAPYBARA_SWITCH_CONTROL = ONE_PIN

ifeq ($(LIBCAPYBARA_SWITCH_CONTROL),ONE_PIN)
export LIBCAPYBARA_BANK_PORT_0 = J.0
export LIBCAPYBARA_BANK_PORT_1 = J.2
export LIBCAPYBARA_BANK_PORT_2 = J.4
export LIBCAPYBARA_BANK_PORT_3 = 4.0
else ifeq ($(LIBCAPYBARA_SWITCH_CONTROL),TWO_PIN)
export LIBCAPYBARA_BANK_PORT_0_OPEN  ?= J.0
export LIBCAPYBARA_BANK_PORT_0_CLOSE ?= J.1
export LIBCAPYBARA_BANK_PORT_1_OPEN  ?= J.2
export LIBCAPYBARA_BANK_PORT_1_CLOSE ?= J.3
export LIBCAPYBARA_BANK_PORT_2_OPEN  ?= J.4
export LIBCAPYBARA_BANK_PORT_2_CLOSE ?= J.5
export LIBCAPYBARA_BANK_PORT_3_OPEN  ?= 4.0
export LIBCAPYBARA_BANK_PORT_3_CLOSE ?= 4.1
endif # LIBCAPYBARA_SWITCH_CONTROL

override VOLTAGE = 2400 # programming voltage in mV
export VOLTAGE
