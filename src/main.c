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
#include <libio/console.h>
#include <libchain/chain.h>

#include <libcapybara/capybara.h>

#include "pins.h"

#define NUM_TEMP_SAMPLES 16

#define SERIES_LEN       128
#define SERIES_LEN_MASK  0x7F

#define TEMP_NORMAL_MIN 20
#define TEMP_NORMAL_MAX 38

TASK(1, task_init)
TASK(2, task_sample)
TASK(3, task_append)
TASK(4, task_alarm)
TASK(5, task_delay)

struct msg_idx {
    CHAN_FIELD(int, idx);
};

struct msg_sample {
    CHAN_FIELD(int, sample);
};

struct msg_series {
    CHAN_FIELD_ARRAY(int, series, SERIES_LEN);
    CHAN_FIELD(int, len);
};

struct msg_series_idx {
    CHAN_FIELD_ARRAY(int, series, SERIES_LEN);
    CHAN_FIELD(int, idx);
};

struct msg_self_series {
    SELF_CHAN_FIELD_ARRAY(int, series, SERIES_LEN);
    SELF_CHAN_FIELD(int, idx);
};
#define FIELD_INIT_msg_self_series {\
    SELF_FIELD_ARRAY_INITIALIZER(SERIES_LEN), \
    SELF_FIELD_INITIALIZER \
}

CHANNEL(task_init, task_append, msg_series_idx);
CHANNEL(task_sample, task_append, msg_sample);
SELF_CHANNEL(task_append, msg_self_series);
CHANNEL(task_append, task_alarm, msg_series);

typedef enum {
    PKT_TYPE_NONE = 0,
    PKT_TYPE_ALARM = 1,
} pkt_type_t;

typedef struct {
    pkt_type_t type;
    uint8_t series[SERIES_LEN];
} radio_pkt_t;

static radio_pkt_t radio_pkt;

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
  param.dataRate = EUSCI_B_I2C_SET_DATA_RATE_400KBPS;
  param.byteCounterThreshold = 0;
  param.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;

  EUSCI_B_I2C_initMaster(EUSCI_B0_BASE, &param);
}

void init_hw() {
    msp_watchdog_disable();
    msp_gpio_unlock();

    // TEMPORARY: debug pin
    P3OUT &= ~(BIT5 | BIT6);
    P3DIR |= BIT5 | BIT6;

    __enable_interrupt();

#if 1 // TEMP: don't wait when testing on continuous power only!
    capybara_wait_for_supply();
    capybara_wait_for_vcap();
#endif

    capybara_config_pins();

    GPIO(PORT_SENSE_SW, OUT) &= ~BIT(PIN_SENSE_SW);
    GPIO(PORT_SENSE_SW, DIR) |= BIT(PIN_SENSE_SW);

    GPIO(PORT_RADIO_SW, OUT) &= ~BIT(PIN_RADIO_SW);
    GPIO(PORT_RADIO_SW, DIR) |= BIT(PIN_RADIO_SW);

    // TODO: do it here?
    capybara_config_banks(0x0);
    //capybara_config_banks(0x1);

#if 1 // TEMP: don't wait when testing on continuous power only!
    capybara_wait_for_supply();
    capybara_shutdown_on_deep_discharge(); // noop if already below threshold
#endif

    msp_clock_setup();

    INIT_CONSOLE();
    LOG("TempAlarm v1.0\r\n");

    LOG(".%u.\r\n", curctx->task->idx);
}

void task_init()
{
    int idx = 0;
    CHAN_OUT1(int, idx, idx, CH(task_init, task_append));

    int zero = 0;
    for (int i = 0; i < SERIES_LEN; ++i) {
        CHAN_OUT1(int, series[i], zero, CH(task_init, task_append));
    }

    TRANSITION_TO(task_sample);
}

void task_sample()
{
    P3OUT |= BIT6;
    int temp = 0;
    for (int i = 0; i < NUM_TEMP_SAMPLES; ++i) {
        int temp_sample = msp_sample_temperature();
        temp += temp_sample;
        LOG2("temp %i\r\n", temp_sample);
    }
    temp /= NUM_TEMP_SAMPLES;
    LOG("temp avg: %i\r\n", temp);
    P3OUT &= ~BIT6;

    CHAN_OUT1(int, sample, temp, CH(task_sample, task_append));

    TRANSITION_TO(task_append);
}

void task_append()
{
    int temp_sample = *CHAN_IN1(int, sample, CH(task_sample, task_append));
    LOG("temp %i\r\n", temp_sample);

    int idx = *CHAN_IN2(int, idx, CH(task_init, task_append),
                                 SELF_IN_CH(task_append));

    CHAN_OUT1(int, series[idx], temp_sample , SELF_OUT_CH(task_append));
    LOG("series[%u] <- %i\r\n", idx, temp_sample);

    idx = (idx + 1) & SERIES_LEN_MASK; // assumes power of 2 len
    CHAN_OUT1(int, idx, idx, SELF_OUT_CH(task_append));

    if (!(TEMP_NORMAL_MIN <= temp_sample && temp_sample <= TEMP_NORMAL_MAX)) {

        LOG("ALARM!\r\n");

        // iterate from oldest to newest sample
        int start_idx = (idx - 1) & SERIES_LEN_MASK; // index into circ buffer (-1 cause already inc'ed)
        int i = start_idx;
        int j = 0; // output index
        do {
            int sample = *CHAN_IN2(int, series[i], SELF_IN_CH(task_append),
                                                   CH(task_init, task_append));
            CHAN_OUT1(int, series[j], sample, CH(task_append, task_alarm));
            ++j;
            i = (i + 1) & SERIES_LEN_MASK; // assumes power of 2 len
        } while (i != start_idx);
        // for now, this is fixed length, but send anyway for future
        CHAN_OUT1(int, len, j, CH(task_append, task_alarm));

        TRANSITION_TO(task_alarm);
    }

    // TODO: would be good to have a chain primitive of transition with delay,
    // that updates the task pointer, and then goes to sleep; in order to
    // not sleep if you powered off (assuming the charge time is always greater
    // than the desired delay).
    TRANSITION_TO(task_delay);
}

void task_delay() {
    msp_sleep(1024); // 0.250 sec
    TRANSITION_TO(task_sample);
}

void task_alarm()
{
    capybara_config_banks(0x1);

    P3OUT |= BIT5;
    radio_pkt.type = PKT_TYPE_ALARM;
    int len = *CHAN_IN1(int, len, CH(task_append, task_alarm));
    for (int j = 0; j < len; ++j) {
        int sample = *CHAN_IN1(int, series[j], CH(task_append, task_alarm));
        radio_pkt.series[j] = sample;
    }
    P3OUT &= ~BIT5;

    // TODO: send radio_pkt to radio IC over UART link
    LOG("TX PKT (len %u):\r\n", len);
    int j;
    for (j = 0; j < len; j += 16) {
        for (int c = 0; c < 16 && j + c < len; ++c)
            LOG("%02i ", (int)radio_pkt.series[j + c]);
        LOG("\r\n");
    }
    for (; j < len % 16; ++j)
        LOG("%i ", (int)radio_pkt.series[j]);
    LOG("\r\n");

    P3OUT |= BIT5;
    GPIO(PORT_RADIO_SW, OUT) |= BIT(PIN_RADIO_SW);

    // TODO: wait until radio is finished; for now, wait for 0.25sec
    msp_sleep(1024);

    GPIO(PORT_RADIO_SW, OUT) &= ~BIT(PIN_RADIO_SW);
    P3OUT &= ~BIT5;

    P3OUT |= BIT5;
    capybara_config_banks(0x0);
    P3OUT &= ~BIT5;

    TRANSITION_TO(task_sample);
}

#define _THIS_PORT 2
__attribute__ ((interrupt(GPIO_VECTOR(_THIS_PORT))))
void  GPIO_ISR(_THIS_PORT) (void)
{
    switch (__even_in_range(INTVEC(_THIS_PORT), INTVEC_RANGE(_THIS_PORT))) {
#if LIBCAPYBARA_PORT_VBOOST_OK == _THIS_PORT
        case INTFLAG(LIBCAPYBARA_PORT_VBOOST_OK, LIBCAPYBARA_PIN_VBOOST_OK):
            capybara_vboost_ok_isr();
            break;
#else
#error Handler in wrong ISR: capybara_vboost_ok_isr
#endif // LIBCAPYBARA_PORT_VBOOST_OK
    }
}
#undef _THIS_PORT

ENTRY_TASK(task_init)
INIT_FUNC(init_hw)
