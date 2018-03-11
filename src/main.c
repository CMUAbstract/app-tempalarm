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

#define NUM_TEMP_SAMPLES 1
#if CONFIG_REF
#define MIN_ALARM_AGE  125 // minimum samples between alarms
#else // !CONFIG_REF
//#define MIN_ALARM_AGE  75 // minimum samples between alarms
#define MIN_ALARM_AGE  2 // minimum samples between alarms
#endif // !CONFIG_REF

// 25 is max BLE adv payload size
#define SERIES_LEN       23

// needed for libchain's array field initializer
#define REPEAT23(x)     REPEAT16(x),REPEAT4(x),REPEAT2(x),x

#if CONFIG_REF
#define TEMP_NORMAL_MIN 180 // degC * 10
#define TEMP_NORMAL_MAX 260 // degC * 10
#else // !CONFIG_REF
#define TEMP_NORMAL_MIN 185 // degC * 10
#define TEMP_NORMAL_MAX 255 // degC * 10
#endif

#define RADIO_ON_CYCLES   60 // ~12ms (a bundle of 2-3 pkts 25 payload bytes each on Pin=0)
#define RADIO_BOOT_CYCLES 60
#define RADIO_RST_CYCLES   1

#define TEMP_FIXEDPOINT_FACTOR 10

#if defined(CONFIG_REF) || !defined(CONFIG_CAP_PRECHARGE)
TASK(1, task_init)
TASK(2, task_sample)
TASK(3, task_output)
TASK(4, task_append)
TASK(5, task_alarm)
TASK(6, task_delay)
#else // !CONFIG_REF && CONFIG_CAP_PRECHARGE
TASK(1, task_init, DEFAULT)
TASK(2, task_sample, LOWP)
TASK(3, task_output)
TASK(4, task_append, PREBURST, MEDLOWP, LOWP)
TASK(5, task_alarm, BURST)
TASK(6, task_delay, DEFAULT)
#endif // !CONFIG_REF && CONFIG_CAP_PRECHARGE

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

struct msg_alarm {
    CHAN_FIELD_ARRAY(int, series, SERIES_LEN);
    CHAN_FIELD(int, len);
    CHAN_FIELD(int, alarm_type);
    CHAN_FIELD(int, alarm_seq);
};

struct msg_series_idx {
    CHAN_FIELD_ARRAY(int, series, SERIES_LEN);
    CHAN_FIELD(int, idx);
    CHAN_FIELD(int, alarm_age);
    CHAN_FIELD(int, alarm_seq);
};

struct msg_self_series {
    SELF_CHAN_FIELD_ARRAY(int, series, SERIES_LEN);
    SELF_CHAN_FIELD(int, idx);
    SELF_CHAN_FIELD(int, alarm_age);
    SELF_CHAN_FIELD(int, alarm_seq);
};
#define FIELD_INIT_msg_self_series {\
    SELF_FIELD_ARRAY_INITIALIZER(SERIES_LEN), \
    SELF_FIELD_INITIALIZER, \
    SELF_FIELD_INITIALIZER, \
    SELF_FIELD_INITIALIZER \
}

CHANNEL(task_init, task_append, msg_series_idx);
CHANNEL(task_sample, task_append, msg_sample);
CHANNEL(task_sample, task_output, msg_sample);
SELF_CHANNEL(task_append, msg_self_series);
CHANNEL(task_append, task_alarm, msg_alarm);

typedef enum {
    ALARM_TYPE_COLD = 0,
    ALARM_TYPE_HOT,
} alarm_type_t;

typedef enum __attribute__((packed)) {
    RADIO_CMD_SET_ADV_PAYLOAD = 0,
} radio_cmd_t;

typedef struct __attribute__((packed)) {
    radio_cmd_t cmd;
    uint8_t alarm_type;
    uint8_t alarm_seq;
    uint8_t series[SERIES_LEN];
} radio_pkt_t;

static radio_pkt_t radio_pkt;

static bool alarm_sent = false;

static inline void radio_on()
{
#if (BOARD_MAJOR == 1 && BOARD_MINOR == 0) || (BOARD_MAJOR == 2 && BOARD_MINOR == 0)

#if PORT_RADIO_SW != PORT_RADIO_RST // we assume this below
#error Unexpected pin config: RAD_SW and RAD_RST not on same port
#endif // PORT_RADIO_SW != PORT_RADIO_RST

    GPIO(PORT_RADIO_SW, OUT) |= BIT(PIN_RADIO_SW) | BIT(PIN_RADIO_RST);
    GPIO(PORT_RADIO_RST, OUT) &= ~BIT(PIN_RADIO_RST);

#elif BOARD_MAJOR == 1 && BOARD_MINOR == 1
    // Assert reset and give it time before turning on power, to make sure that
    // radio doesn't power on while reset is (not yet) asserted and starts.
    fxl_set(BIT_RADIO_RST);
    msp_sleep(RADIO_RST_CYCLES);
    fxl_set(BIT_RADIO_SW);
    msp_sleep(RADIO_RST_CYCLES);
    fxl_clear(BIT_RADIO_RST);

#else // BOARD_{MAJOR,MINOR}
#error Unsupported board: do not know how to turn off radio (see BOARD var)
#endif // BOARD_{MAJOR,MINOR}
}

static inline void radio_off()
{
#if (BOARD_MAJOR == 1 && BOARD_MINOR == 0) || (BOARD_MAJOR == 2 && BOARD_MINOR == 0)
    GPIO(PORT_RADIO_RST, OUT) |= BIT(PIN_RADIO_RST); // reset for clean(er) shutdown
    msp_sleep(RADIO_RST_CYCLES);
    GPIO(PORT_RADIO_SW, OUT) &= ~BIT(PIN_RADIO_SW);
#elif BOARD_MAJOR == 1 && BOARD_MINOR == 1
    fxl_set(BIT_RADIO_RST); // reset for clean(er) shutdown
    msp_sleep(RADIO_RST_CYCLES);
    fxl_clear(BIT_RADIO_SW);
#else // BOARD_{MAJOR,MINOR}
#error Unsupported board: do not know how to turn on radio (see BOARD var)
#endif // BOARD_{MAJOR,MINOR}
}

void capybara_transition()
{
    // need to explore exactly how we want BURST tasks to be followed --> should
    // we ever shutdown to reconfigure? Or should we always ride the burst wave
    // until we're out of energy?
    // TODO no really, a case statement isn't going to kill you!

    // Check previous burst state and register a finished burst
    if(burst_status){
        burst_status = 2;
    }
    task_cfg_spec_t curpwrcfg = curctx->task->spec_cfg;
    switch(curpwrcfg){
        case BURST:
            prechg_status = 0;
            capybara_config_banks(prechg_config.banks);
            burst_status = 1;
            break;

        case PREBURST:
            if(!prechg_status){
                prechg_config.banks = curctx->task->precfg->banks;
                capybara_config_banks(prechg_config.banks);
                // Mark that we finished the config_banks_command
                prechg_status = 1;
                capybara_shutdown();
                capybara_wait_for_supply();
            }
            //intentional fall through

        case CONFIGD:
            if(base_config.banks != curctx->task->opcfg->banks){
                base_config.banks = curctx->task->opcfg->banks;
                capybara_config_banks(base_config.banks);
                capybara_wait_for_supply();
            }
            //Another intentional fall through

        default:
            //capybara_config_banks(base_config.banks);
            //capybara_wait_for_supply();
            break;
    }

#if 0
    // HaNDLe a burst
    if(curpwrcfg == BURST){
        prechg_status = 0;
        capybara_config_banks(prechg_config.banks);
        burst_status = 1;
        //capybara_wait_for_supply();
    }
    else{
        burst_status = 0;
        // Set up a precharge in response to a preburst task if we haven't
        // precharged already.
        if( curpwrcfg == PREBURST && !prechg_status){
            prechg_config.banks = curctx->task->precfg->banks;
            capybara_config_banks(prechg_config.banks);
            // Mark that we finished the config_banks_command
            prechg_status = 1;
            capybara_shutdown();
            capybara_wait_for_supply();
        }
        // Handle a new config, either from a CONFIGD task a preburst task
        if(curpwrcfg == CONFIGD || curpwrcfg == PREBURST){
            base_config.banks = curctx->task->opcfg->banks;
            capybara_config_banks(base_config.banks);
            capybara_wait_for_supply();
        }
    }
#endif

    //LOG("Running task %u \r\n",curctx->task->idx);
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
  param.dataRate = EUSCI_B_I2C_SET_DATA_RATE_400KBPS;
  param.byteCounterThreshold = 0;
  param.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;

  EUSCI_B_I2C_initMaster(EUSCI_B0_BASE, &param);
}
#endif // BOARD_{MAJOR,MINOR}

int main() {
    msp_watchdog_disable();
    msp_gpio_unlock();

    // TEMPORARY: debug pin
    GPIO(PORT_DBG, OUT) &= ~(BIT(PIN_DBG0) | BIT(PIN_DBG1) | BIT(PIN_DBG2));
    GPIO(PORT_DBG, DIR) |= BIT(PIN_DBG0) | BIT(PIN_DBG1) | BIT(PIN_DBG2);

    GPIO(PORT_DBG, OUT) |= BIT(PIN_DBG0);

    __enable_interrupt();

#ifndef CONFIG_REF // TEMP: don't wait when testing on continuous power only!
    GPIO(PORT_DBG, OUT) |= BIT(PIN_DBG0);
    capybara_wait_for_supply();
    GPIO(PORT_DBG, OUT) &= ~BIT(PIN_DBG0);
#endif // !CONFIG_REF

    GPIO(PORT_DBG, OUT) |= BIT(PIN_DBG0);
    capybara_config_pins();
    msp_clock_setup();
    GPIO(PORT_DBG, OUT) &= ~BIT(PIN_DBG0);

#ifndef CONFIG_REF // TEMP: don't wait when testing on continuous power only!

    GPIO(PORT_DBG, OUT) |= BIT(PIN_DBG0);
    // Setup deep discharge shutdown interrupt before reconfiguring banks,
    // so that if we deep discharge during reconfiguration, we still at
    // least try to avoid cold start. NOTE: this is controversial.
    cb_rc_t deep_discharge_status = capybara_shutdown_on_deep_discharge();
    GPIO(PORT_DBG, OUT) &= ~BIT(PIN_DBG0);

    // TODO: do it here?
    GPIO(PORT_DBG, OUT) |= BIT(PIN_DBG0);
#if defined(CONFIG_CAP_RECONF)
    capybara_config_banks(0x0);
#elif defined(CONFIG_CAP_BIG)
    capybara_config_banks(0x1);
#elif defined(CONFIG_CAP_SMALL)
    capybara_config_banks(0x0);
#endif // CONFIG_*
    GPIO(PORT_DBG, OUT) &= ~BIT(PIN_DBG0);

    // We still want to attempt reconfiguration, no matter how low Vcap
    // is, because otherwise we'd never be able to reconfigure. But, we
    // definitely do not want to run Vcap into the ground if we know
    // it is already below deep discharge threshold. NOTE: this doesn't
    // seem to matter much, because Vcap eventually discharges anyway
    // (regardless of whether we shutdown or not), if deep discharge threshold
    // was crossed.
    if (deep_discharge_status == CB_ERROR_ALREADY_DEEPLY_DISCHARGED)
        capybara_shutdown();

    GPIO(PORT_DBG, OUT) |= BIT(PIN_DBG0);
    capybara_wait_for_supply();
    GPIO(PORT_DBG, OUT) &= ~BIT(PIN_DBG0);

#endif // !CONFIG_REF

    GPIO(PORT_DBG, OUT) |= BIT(PIN_DBG0);
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
    GPIO(PORT_DBG, OUT) &= ~BIT(PIN_DBG0);

    GPIO(PORT_DBG, OUT) |= BIT(PIN_DBG0);

    LOG2("i2c init\r\n");
    i2c_setup();

    LOG2("fxl init\r\n");
    fxl_init();

    fxl_out(BIT_RADIO_SW | BIT_RADIO_RST);
    fxl_pull_up(BIT_CCS_WAKE);
    // SENSE_SW is present but is not electrically correct: do not use.
#elif BOARD_MAJOR == 2 && BOARD_MINOR == 0
    GPIO(PORT_RADIO_SW, OUT) &= ~BIT(PIN_RADIO_SW);
    GPIO(PORT_RADIO_SW, DIR) |= BIT(PIN_RADIO_SW);
#else // BOARD_{MAJOR,MINOR}
#error Unsupported board: do not know what pins to configure (see BOARD var)
#endif // BOARD_{MAJOR,MINOR}

    GPIO(PORT_DBG, OUT) &= ~BIT(PIN_DBG0);

    LOG2(".%u.\r\n", curctx->task->idx);

    return chain_main();
}

void task_init()
{
    int zero = 0;
    CHAN_OUT1(int, idx, zero, CH(task_init, task_append));
    CHAN_OUT1(int, alarm_age, zero, CH(task_init, task_append));
    CHAN_OUT1(int, alarm_seq, zero, CH(task_init, task_append));

    for (int i = 0; i < SERIES_LEN; ++i) {
        CHAN_OUT1(int, series[i], zero, CH(task_init, task_append));
    }

    TRANSITION_TO(task_sample);
}

void task_sample()
{
#if !defined(CONFIG_REF) && defined(CONFIG_CAP_PRECHARGE)
    capybara_transition();
#endif // CONFIG_CAP_RECONF

    int tmp_sample;
    int temp = 0;
    for (int i = 0; i < NUM_TEMP_SAMPLES; ++i) {
        GPIO(PORT_DBG, OUT) |= BIT(PIN_DBG1);
#if defined(TEMP_SENSOR_INTERNAL)
        tmp_sample = msp_sample_temperature();
#elif defined(TEMP_SENSOR_EXTERNAL)
        tmp_sample = temp_sample();
#endif // TEMP_SENSOR_*
        GPIO(PORT_DBG, OUT) &= ~BIT(PIN_DBG1);
        GPIO(PORT_DBG, OUT) |= BIT(PIN_DBG1);
        temp += tmp_sample;
        LOG2("temp %i\r\n", tmp_sample);
        GPIO(PORT_DBG, OUT) &= ~BIT(PIN_DBG1);
    }
    GPIO(PORT_DBG, OUT) |= BIT(PIN_DBG1);
    temp /= NUM_TEMP_SAMPLES;
    CHAN_OUT1(int, sample, temp, CH(task_sample, task_append));
    GPIO(PORT_DBG, OUT) &= ~BIT(PIN_DBG1);

#ifdef TIMESTAMPS // have to timestamp and output in this task to correlate with sample
    uint32_t timestamp = msp_ticks();
    LOG("%u:%u,%i\r\n", (uint16_t)(timestamp >> 16), (uint16_t)(timestamp & 0xFFFF), temp);

    TRANSITION_TO(task_append);
#else
    CHAN_OUT1(int, sample, temp, CH(task_sample, task_output));
    TRANSITION_TO(task_output);
#endif // TIMESTAMPS
}

void task_output()
{
    int temp = *CHAN_IN1(int, sample, CH(task_sample, task_output));

#if VERBOSE > 2
    LOG2("temp avg: %i\r\n", temp);
#else
    LOG("%i\r\n", temp);
#endif // VERBOSE

    TRANSITION_TO(task_append);
}

void task_append()
{
#if !defined(CONFIG_REF) && defined(CONFIG_CAP_PRECHARGE)
    capybara_transition();
#endif // CONFIG_CAP_RECONF

    int tmp_sample = *CHAN_IN1(int, sample, CH(task_sample, task_append));
    LOG2("temp %i\r\n", tmp_sample);

    int idx = *CHAN_IN2(int, idx, CH(task_init, task_append),
                                 SELF_IN_CH(task_append));
    int alarm_age = *CHAN_IN2(int, alarm_age, CH(task_init, task_append),
                                 SELF_IN_CH(task_append));

    CHAN_OUT1(int, series[idx], tmp_sample , SELF_OUT_CH(task_append));
    LOG2("series[%u] <- %i\r\n", idx, tmp_sample);

    idx = (idx + 1) % SERIES_LEN;
    CHAN_OUT1(int, idx, idx, SELF_OUT_CH(task_append));

#ifdef CONFIG_REF
    if (!(TEMP_NORMAL_MIN <= tmp_sample && tmp_sample <= TEMP_NORMAL_MAX) &&
        (alarm_age == 0 || alarm_age > MIN_ALARM_AGE)) {
#else
    //if (!(TEMP_NORMAL_MIN <= tmp_sample && tmp_sample <= TEMP_NORMAL_MAX)) {
    if (!(TEMP_NORMAL_MIN <= tmp_sample && tmp_sample <= TEMP_NORMAL_MAX) &&
        (!alarm_sent) && 
        (alarm_age == 0 || alarm_age > MIN_ALARM_AGE)) {
#endif

        int alarm_type;
        if (tmp_sample < TEMP_NORMAL_MIN)
            alarm_type = ALARM_TYPE_COLD;
        else if (tmp_sample > TEMP_NORMAL_MAX)
            alarm_type = ALARM_TYPE_HOT;
        CHAN_OUT1(int, alarm_type, alarm_type, CH(task_append, task_alarm));

        int alarm_seq = *CHAN_IN2(int, alarm_seq, CH(task_init, task_append),
                                                  SELF_IN_CH(task_append));
        CHAN_OUT1(int, alarm_seq, alarm_seq, CH(task_append, task_alarm));

        ++alarm_seq;
        CHAN_OUT1(int, alarm_seq, alarm_seq, SELF_OUT_CH(task_append));

        LOG2("# t %u s %u\r\n", alarm_type, alarm_seq);

        // iterate from oldest to newest sample
        int start_idx = idx > 0 ? idx - 1 : SERIES_LEN - 1; // index into circ buffer (-1 cause already inc'ed)
        int i = start_idx;
        int j = 0; // output index
        do {
            int sample = *CHAN_IN2(int, series[i], SELF_IN_CH(task_append),
                                                   CH(task_init, task_append));
            CHAN_OUT1(int, series[j], sample, CH(task_append, task_alarm));
            ++j;
            i = (i + 1) % SERIES_LEN;
        } while (i != start_idx);
        // for now, this is fixed length, but send anyway for future
        CHAN_OUT1(int, len, j, CH(task_append, task_alarm));

        int one = 1;
        CHAN_OUT1(int, alarm_age, one, SELF_OUT_CH(task_append));

        alarm_sent = true;

#if !defined(CONFIG_REF) && defined(CONFIG_CAP_RECONF)
        capybara_config_banks(0x1);
        TRANSITION_TO_AND_SHUTDOWN(task_alarm);
#else // CONFIG_CAP_*
        TRANSITION_TO(task_alarm);
#endif // CONFIG_CAP_*
    } else {

        if (tmp_sample < TEMP_NORMAL_MIN)
            LOG2("# t 0 age: %u\r\n", alarm_age);
        else if (tmp_sample > TEMP_NORMAL_MAX)
            LOG2("# t 1 age: %u\r\n", alarm_age);

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
#if !defined(CONFIG_REF) && defined(CONFIG_CAP_PRECHARGE)
    capybara_transition();
#endif // CONFIG_CAP_RECONF

    //GPIO(PORT_DBG, OUT) |= BIT(PIN_DBG1);
#ifdef CONFIG_REF
    msp_sleep(500); // 125ms
#else // !CONFIG_REF
    //msp_sleep(16); // 4ms
#endif // !CONFIG_REF
    //GPIO(PORT_DBG, OUT) &= BIT(PIN_DBG1);
    TRANSITION_TO(task_sample);
}

void task_alarm()
{
#if !defined(CONFIG_REF)
#if defined(CONFIG_CAP_RECONF)
    capybara_config_banks(0x1);
#elif defined(CONFIG_CAP_BIG)
    // already configed to 0x1
#elif defined(CONFIG_CAP_SMALL)
    // already configed to 0x0
#elif defined(CONFIG_CAP_PRECHARGE)
    capybara_transition();
#endif // CONFIG_CAP_RECONF
#endif // CONFIG_REF

    int alarm_type = *CHAN_IN1(int, alarm_type, CH(task_append, task_alarm));
    int alarm_seq = *CHAN_IN1(int, alarm_seq, CH(task_append, task_alarm));
    LOG2("alarm: type %d seq %d\r\n", alarm_type, alarm_seq);

    radio_pkt.cmd = RADIO_CMD_SET_ADV_PAYLOAD;

    radio_pkt.alarm_type = alarm_type;
    radio_pkt.alarm_seq = alarm_seq;
    int len = *CHAN_IN1(int, len, CH(task_append, task_alarm));
    for (int j = 0; j < len; ++j) {
        int sample = *CHAN_IN1(int, series[j], CH(task_append, task_alarm));
        radio_pkt.series[j] = sample / TEMP_FIXEDPOINT_FACTOR;
    }

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

    GPIO(PORT_DBG, OUT) |= BIT(PIN_DBG2);
    radio_on();

    GPIO(PORT_DBG, OUT) &= ~BIT(PIN_DBG2);
    GPIO(PORT_DBG, OUT) |= BIT(PIN_DBG2);

    msp_sleep(RADIO_BOOT_CYCLES); // ~15ms @ ACLK/8

    GPIO(PORT_DBG, OUT) &= ~BIT(PIN_DBG2);
    GPIO(PORT_DBG, OUT) |= BIT(PIN_DBG2);

    uartlink_open_tx();
    uartlink_send((uint8_t *)&radio_pkt.cmd,
                   sizeof(radio_pkt.cmd) + 1 /* alarm type */ + 1 /* alarm seq */ + len);
    uartlink_close();

    GPIO(PORT_DBG, OUT) &= ~BIT(PIN_DBG2);
    GPIO(PORT_DBG, OUT) |= BIT(PIN_DBG2);

    // TODO: wait until radio is finished; for now, wait blindly
    msp_sleep(RADIO_ON_CYCLES);

    GPIO(PORT_DBG, OUT) &= ~BIT(PIN_DBG2);
    GPIO(PORT_DBG, OUT) |= BIT(PIN_DBG2);

    radio_off();
    GPIO(PORT_DBG, OUT) &= ~BIT(PIN_DBG2);

#ifdef CONFIG_CAP_RECONF
    capybara_config_banks(0x0);
#elif defined(CONFIG_CAP_BIG)
    // already configed to 0x2
#elif defined(CONFIG_CAP_SMALL)
    // already configed to 0x0
#endif // CONFIG_CAP_RECONF

#if !defined(CONFIG_REF) && defined(CONFIG_CAP_RECONF)
    TRANSITION_TO_AND_SHUTDOWN(task_sample);
#else // CONFIG_CAP_*
    TRANSITION_TO(task_sample);
#endif // CONFIG_CAP_*
}

#define _THIS_PORT 2
ISR(GPIO_VECTOR(_THIS_PORT))
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
ISR(GPIO_VECTOR(_THIS_PORT))
{
    switch (__even_in_range(INTVEC(_THIS_PORT), INTVEC_RANGE(_THIS_PORT))) {
#if (BOARD_MAJOR == 1 && BOARD_MINOR == 1) || (BOARD_MAJOR == 2 && BOARD_MINOR == 0)
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
