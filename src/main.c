#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

#include <libmspware/driverlib.h> // incl before libmsp/periph.h for OUT macro workaround

#include <libmsp/watchdog.h>
#include <libmsp/clock.h>
#include <libmsp/gpio.h>
#include <libmsp/periph.h>
#include <libmsp/sleep.h>
#include <libmsp/temp.h>
#include <libmsp/uart.h>
#include <libmspuartlink/uartlink.h>
#include <libio/console.h>
#include <libchain/chain.h>

#if BOARD_MAJOR == 1 && BOARD_MINOR == 1
#include <libfxl/fxl6408.h>
#endif // BOARD_{MAJOR,MINOR}

#include <libcapybara/capybara.h>

#ifdef TIMESTAMPS
#include <libmsp/tick.h>
#endif // TIMESTAMPS

#ifdef TEMP_SENSOR_EXTERNAL
#include <libtemp/temp.h>
#endif // TEMP_SENSOR_EXTERNAL

#include "pins.h"

#define NUM_TEMP_SAMPLES 1 // TODO: increase back to 8 or 16
#define MIN_ALARM_AGE  100 // minimum samples between alarms

#define SERIES_LEN       8
#define SERIES_LEN_MASK  0x07

#define TEMP_NORMAL_MIN 21
#define TEMP_NORMAL_MAX 29

// Units of temp in the log output are 1/FIXEDPOINT_FACTOR degrees C
#define TEMP_FIXEDPOINT_FACTOR 10

TASK(1, task_init)
TASK(2, task_sample)
TASK(3, task_append)
TASK(4, task_alarm)
TASK(5, task_delay)

struct msg_idx {
    CHAN_FIELD(int, idx);
};

struct msg_sample {
    CHAN_FIELD(float, sample);
};

struct msg_series {
    CHAN_FIELD_ARRAY(float, series, SERIES_LEN);
    CHAN_FIELD(int, len);
};

struct msg_series_idx {
    CHAN_FIELD_ARRAY(float, series, SERIES_LEN);
    CHAN_FIELD(int, idx);
    CHAN_FIELD(int, alarm_age);
};

struct msg_self_series {
    SELF_CHAN_FIELD_ARRAY(float, series, SERIES_LEN);
    SELF_CHAN_FIELD(int, idx);
    SELF_CHAN_FIELD(int, alarm_age);
};
#define FIELD_INIT_msg_self_series {\
    SELF_FIELD_ARRAY_INITIALIZER(SERIES_LEN), \
    SELF_FIELD_INITIALIZER, \
    SELF_FIELD_INITIALIZER \
}

CHANNEL(task_init, task_append, msg_series_idx);
CHANNEL(task_sample, task_append, msg_sample);
SELF_CHANNEL(task_append, msg_self_series);
CHANNEL(task_append, task_alarm, msg_series);

typedef enum __attribute__((packed)) {
    RADIO_CMD_SET_ADV_PAYLOAD = 0,
} radio_cmd_t;

typedef struct __attribute__((packed)) {
    radio_cmd_t cmd;
    uint8_t series[SERIES_LEN];
} radio_pkt_t;

static radio_pkt_t radio_pkt;

static inline void radio_on()
{
#if BOARD_MAJOR == 1 && BOARD_MINOR == 0

#if PORT_RADIO_SW != PORT_RADIO_RST // we assume this below
#error Unexpected pin config: RAD_SW and RAD_RST not on same port
#endif // PORT_RADIO_SW != PORT_RADIO_RST

    GPIO(PORT_RADIO_SW, OUT) |= BIT(PIN_RADIO_SW) | BIT(PIN_RADIO_RST);
    GPIO(PORT_RADIO_RST, OUT) &= ~BIT(PIN_RADIO_RST);

#elif BOARD_MAJOR == 1 && BOARD_MINOR == 1
    // Assert reset and give it time before turning on power, to make sure that
    // radio doesn't power on while reset is (not yet) asserted and starts.
    fxl_set(BIT_RADIO_RST);
    msp_sleep(8); // 10ms
    fxl_set(BIT_RADIO_SW);
    msp_sleep(8); // ~10ms
    fxl_clear(BIT_RADIO_RST);

#else // BOARD_{MAJOR,MINOR}
#error Unsupported board: do not know how to turn off radio (see BOARD var)
#endif // BOARD_{MAJOR,MINOR}
}

static inline void radio_off()
{
#if BOARD_MAJOR == 1 && BOARD_MINOR == 0
    GPIO(PORT_RADIO_SW, OUT) &= ~BIT(PIN_RADIO_SW);
#elif BOARD_MAJOR == 1 && BOARD_MINOR == 1
    fxl_clear(BIT_RADIO_SW);
#else // BOARD_{MAJOR,MINOR}
#error Unsupported board: do not know how to turn on radio (see BOARD var)
#endif // BOARD_{MAJOR,MINOR}
}

#if BOARD_MAJOR == 1 && BOARD_MINOR == 1 // in this app, I2C is only needed on v1.1
void i2c_setup(void) {
  /*
  * Select Port 1
  * Set Pin 6, 7 to input Secondary Module Function:
  *   (UCB0SIMO/UCB0SDA, UCB0SOMI/UCB0SCL)
  */

  GPIO_setAsPeripheralModuleFunctionInputPin(
    GPIO_PORT_P1,
    GPIO_PIN6 + GPIO_PIN7,
    GPIO_SECONDARY_MODULE_FUNCTION
  );


  EUSCI_B_I2C_initMasterParam param = {0};
  param.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
  param.i2cClk = CS_getSMCLK();
  param.dataRate = EUSCI_B_I2C_SET_DATA_RATE_100KBPS;
  param.byteCounterThreshold = 0;
  param.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;

  EUSCI_B_I2C_initMaster(EUSCI_B0_BASE, &param);
}
#endif // BOARD_{MAJOR,MINOR}

void init_hw() {
    msp_watchdog_disable();
    msp_gpio_unlock();

    // TEMPORARY: debug pin
    P3OUT &= ~(BIT5 | BIT6 | BIT7);
    P3DIR |= BIT5 | BIT6 | BIT7;

    __enable_interrupt();

#ifndef CONFIG_REF // TEMP: don't wait when testing on continuous power only!
    capybara_wait_for_supply();
    capybara_wait_for_vcap();
#endif // !CONFIG_REF

    capybara_config_pins();

    msp_clock_setup();
#ifndef CONFIG_REF // TEMP: don't wait when testing on continuous power only!

    // TODO: do it here?
#if defined(CONFIG_CAP_RECONF)
    capybara_config_banks(0x0);
#elif defined(CONFIG_CAP_BIG)
    capybara_config_banks(0x1);
#elif defined(CONFIG_CAP_SMALL)
    capybara_config_banks(0x0);
#endif // CONFIG_*

    capybara_wait_for_supply();
    if (capybara_shutdown_on_deep_discharge() == CB_ERROR_ALREADY_DEEPLY_DISCHARGED) {
        capybara_shutdown();
    }

#endif // !CONFIG_REF

    INIT_CONSOLE();
    LOG2("TempAlarm v1.0\r\n");

#ifdef TIMESTAMPS
    LOG("tick,temp_c\r\n");
    msp_tick_start();
#endif // TIMESTAMPS

#if BOARD_MAJOR == 1 && BOARD_MINOR == 0
    GPIO(PORT_SENSE_SW, OUT) &= ~BIT(PIN_SENSE_SW);
    GPIO(PORT_SENSE_SW, DIR) |= BIT(PIN_SENSE_SW);

    GPIO(PORT_RADIO_SW, OUT) &= ~BIT(PIN_RADIO_SW);
    GPIO(PORT_RADIO_SW, DIR) |= BIT(PIN_RADIO_SW);
#elif BOARD_MAJOR == 1 && BOARD_MINOR == 1
    LOG2("i2c init\r\n");
    i2c_setup();

    LOG2("fxl init\r\n");
    fxl_init();

    fxl_out(BIT_RADIO_SW);
    fxl_out(BIT_RADIO_RST);
    fxl_pull_up(BIT_CCS_WAKE);
    // SENSE_SW is present but is not electrically correct: do not use.
#else // BOARD_{MAJOR,MINOR}
#error Unsupported board: do not know what pins to configure (see BOARD var)
#endif // BOARD_{MAJOR,MINOR}

    LOG2(".%u.\r\n", curctx->task->idx);
}

void task_init()
{
    int zero = 0;
    CHAN_OUT1(int, idx, zero, CH(task_init, task_append));
    CHAN_OUT1(int, alarm_age, zero, CH(task_init, task_append));

    for (int i = 0; i < SERIES_LEN; ++i) {
        CHAN_OUT1(int, series[i], zero, CH(task_init, task_append));
    }

    TRANSITION_TO(task_sample);
}

void task_sample()
{
    float temp = 0;
    for (int i = 0; i < NUM_TEMP_SAMPLES; ++i) {
        P3OUT |= BIT6;
#if defined(TEMP_SENSOR_INTERNAL)
        float tmp_sample = msp_sample_temperature();
#elif defined(TEMP_SENSOR_EXTERNAL)
        float tmp_sample = temp_sample();
#endif // TEMP_SENSOR_*
        temp += tmp_sample;
        LOG2("temp %i\r\n", (int)(tmp_sample * TEMP_FIXEDPOINT_FACTOR));
        P3OUT &= ~BIT6;
    }
    temp /= NUM_TEMP_SAMPLES;
    LOG2("temp avg: %i\r\n", (int)(temp * TEMP_FIXEDPOINT_FACTOR));
#ifdef TIMESTAMPS
    uint32_t timestamp = msp_ticks();
    LOG("%u:%u,%i\r\n", (uint16_t)(timestamp >> 16), (uint16_t)(timestamp & 0xFFFF), (int)(temp * TEMP_FIXEDPOINT_FACTOR));
#else
    LOG("%i\r\n", (int)(temp * TEMP_FIXEDPOINT_FACTOR));
#endif // TIMESTAMPS

    float temp_sample = temp;
    CHAN_OUT1(float, sample, temp_sample, CH(task_sample, task_append));

    TRANSITION_TO(task_append);
}

void task_append()
{

    float temp_sample = *CHAN_IN1(float, sample, CH(task_sample, task_append));
    LOG2("temp %i\r\n", (int)(temp_sample * TEMP_FIXEDPOINT_FACTOR));

    int idx = *CHAN_IN2(int, idx, CH(task_init, task_append),
                                 SELF_IN_CH(task_append));
    int alarm_age = *CHAN_IN2(int, alarm_age, CH(task_init, task_append),
                                 SELF_IN_CH(task_append));

    CHAN_OUT1(float, series[idx], temp_sample , SELF_OUT_CH(task_append));
    LOG2("series[%u] <- %i\r\n", idx, temp_sample);

    idx = (idx + 1) & SERIES_LEN_MASK; // assumes power of 2 len
    CHAN_OUT1(int, idx, idx, SELF_OUT_CH(task_append));

    if (!(TEMP_NORMAL_MIN <= temp_sample && temp_sample <= TEMP_NORMAL_MAX) &&
        (alarm_age == -1 || alarm_age > MIN_ALARM_AGE)) {

        LOG2("ALARM!\r\n");

        // iterate from oldest to newest sample
        int start_idx = (idx - 1) & SERIES_LEN_MASK; // index into circ buffer (-1 cause already inc'ed)
        int i = start_idx;
        int j = 0; // output index
        do {
            float sample = *CHAN_IN2(float, series[i], SELF_IN_CH(task_append),
                                                   CH(task_init, task_append));
            CHAN_OUT1(float, series[j], sample, CH(task_append, task_alarm));
            ++j;
            i = (i + 1) & SERIES_LEN_MASK; // assumes power of 2 len
        } while (i != start_idx);
        // for now, this is fixed length, but send anyway for future
        CHAN_OUT1(int, len, j, CH(task_append, task_alarm));

        int zero = 0;
        CHAN_OUT1(int, alarm_age, zero, SELF_OUT_CH(task_append));

        TRANSITION_TO(task_alarm);
    } else {
        if (alarm_age <= MIN_ALARM_AGE) // cap it, this way we don't have to deal with overflow
            ++alarm_age;
        CHAN_OUT1(int, alarm_age, alarm_age, SELF_OUT_CH(task_append));

        // TODO: would be good to have a chain primitive of transition with delay,
        // that updates the task pointer, and then goes to sleep; in order to
        // not sleep if you powered off (assuming the charge time is always greater
        // than the desired delay).
        TRANSITION_TO(task_delay);
    }

}

void task_delay() {
    //P3OUT |= BIT6;
#ifdef CONFIG_REF
    msp_sleep(4000); // 1000ms
#else // !CONFIG_REF
    //msp_sleep(16); // 4ms
#endif // !CONFIG_REF
    //P3OUT &= BIT6;
    TRANSITION_TO(task_sample);
}

void task_alarm()
{
#if defined(CONFIG_CAP_RECONF)
    capybara_config_banks(0x1);
#elif defined(CONFIG_CAP_BIG)
    // already configed to 0x1
#elif defined(CONFIG_CAP_SMALL)
    // already configed to 0x0
#endif // CONFIG_CAP_RECONF

    radio_pkt.cmd = RADIO_CMD_SET_ADV_PAYLOAD;

    int len = *CHAN_IN1(int, len, CH(task_append, task_alarm));
    for (int j = 0; j < len; ++j) {
        float sample = *CHAN_IN1(float, series[j], CH(task_append, task_alarm));
        radio_pkt.series[j] = (int)sample;
    }

    // TODO: send radio_pkt to radio IC over UART link
    LOG2("TX PKT (len %u):\r\n", len);
    int j;
    for (j = 0; j < len; j += 16) {
        for (int c = 0; c < 16 && j + c < len; ++c)
            LOG2("%02i ", (int)radio_pkt.series[j + c]);
        LOG2("\r\n");
    }
    for (; j < len % 16; ++j)
        LOG2("%i ", (int)radio_pkt.series[j]);
    LOG2("\r\n");

    P3OUT |= BIT5;
    radio_on();
    msp_sleep(64); // ~15ms @ ACLK/8

    uartlink_open_tx();
    uartlink_send((uint8_t *)&radio_pkt.cmd, sizeof(radio_pkt.cmd) + len);
    uartlink_close();

    // TODO: wait until radio is finished; for now, wait for 0.25sec
    msp_sleep(1024); // 0.125sec

    radio_off();
    P3OUT &= ~BIT5;

#ifdef CONFIG_CAP_RECONF
    capybara_config_banks(0x0);
#elif defined(CONFIG_CAP_BIG)
    // already configed to 0x2
#elif defined(CONFIG_CAP_SMALL)
    // already configed to 0x0
#endif // CONFIG_CAP_RECONF

    TRANSITION_TO(task_sample);
}

#define _THIS_PORT 2
__attribute__ ((interrupt(GPIO_VECTOR(_THIS_PORT))))
void  GPIO_ISR(_THIS_PORT) (void)
{
    switch (__even_in_range(INTVEC(_THIS_PORT), INTVEC_RANGE(_THIS_PORT))) {
#if BOARD_MAJOR == 1 && BOARD_MINOR == 0
#if LIBCAPYBARA_PORT_VBOOST_OK == _THIS_PORT
        case INTFLAG(LIBCAPYBARA_PORT_VBOOST_OK, LIBCAPYBARA_PIN_VBOOST_OK):
            capybara_vboost_ok_isr();
            break;
#else
#error Handler in wrong ISR: capybara_vboost_ok_isr
#endif // LIBCAPYBARA_PORT_VBOOST_OK
#endif // BOARD_{MAJOR,MINOR}
    }
}
#undef _THIS_PORT

#define _THIS_PORT 3
__attribute__ ((interrupt(GPIO_VECTOR(_THIS_PORT))))
void  GPIO_ISR(_THIS_PORT) (void)
{
    switch (__even_in_range(INTVEC(_THIS_PORT), INTVEC_RANGE(_THIS_PORT))) {
#if BOARD_MAJOR == 1 && BOARD_MINOR == 1
#if LIBCAPYBARA_PORT_VBOOST_OK == _THIS_PORT
        case INTFLAG(LIBCAPYBARA_PORT_VBOOST_OK, LIBCAPYBARA_PIN_VBOOST_OK):
            capybara_vboost_ok_isr();
            break;
#else
#error Handler in wrong ISR: capybara_vboost_ok_isr
#endif // LIBCAPYBARA_PORT_VBOOST_OK
#endif // BOARD_{MAJOR,MINOR}
    }
}
#undef _THIS_PORT

ENTRY_TASK(task_init)
INIT_FUNC(init_hw)
