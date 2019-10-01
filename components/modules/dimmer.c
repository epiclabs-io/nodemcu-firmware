// Module for interfacing with an MQTT broker
#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp_log.h"
#include "lauxlib.h"
#include "lmem.h"
#include "lnodeaux.h"
#include "module.h"
#include "platform.h"
#include "task/task.h"

#include <string.h>

#define DIMMER_METATABLE "dimmer.mt"
#define TAG "DIMMER"

typedef struct {
    int pin;
    uint8_t mode;
    int level;
} dim_t;

#define TIMER_RESOLUTION 10  // us;
#define MAINS_FREQUENCY 50   // Hz
#define TIMER_DIVIDER (TIMER_BASE_CLK / 1000 / 1000 * TIMER_RESOLUTION)
#define TICKS_PER_CYCLE (1000 * 1000 / (MAINS_FREQUENCY * 2) / TIMER_RESOLUTION)

static intr_handle_t s_timer_handle;
static int timerGroup = -1;
static int timerNum = -1;
static int zcPin = -1;
static int t = 0;
static int p = TICKS_PER_CYCLE;
static bool reset = false;
static dim_t* dims = NULL;
static int dimCount = 0;

static timg_dev_t* timerGroupDev;

static int check_err(lua_State* L, esp_err_t err) {
    switch (err) {
        case ESP_ERR_INVALID_ARG:
            luaL_error(L, "invalid argument");
        case ESP_ERR_INVALID_STATE:
            luaL_error(L, "internal logic error");
        case ESP_OK:
            break;
    }
    return 0;
}

static void timer_isr(void* arg) {
    if (reset) {
        reset = false;
        for (int i = 0; i < dimCount; i++) {
            if (dims[i].level == 0) {
                gpio_set_level(dims[i].pin, 1);
            } else {
                gpio_set_level(dims[i].pin, 0);
            }
        }
        p = t;
        t = 0;
    } else {
        t++;

        for (int i = 0; i < dimCount; i++) {
            int level = dims[i].level;
            if (t == level && level > 0) {
                gpio_set_level(dims[i].pin, 1);
            }
        }
    }
    timerGroupDev->int_clr_timers.t0 = 1;
    timerGroupDev->hw_timer[timerNum].config.alarm_en = 1;
}
static int zCount = 0;
static void zc_isr(void* arg) {
    zCount++;

    if (t > TICKS_PER_CYCLE / 2) {  // debounce the zc pin
        reset = true;
    }

    //gpio_intr_enable(zcPin);
}

static void enable_interrupts(lua_State* L) {
    if (timerGroup != -1)
        check_err(L, timer_enable_intr(timerGroup, timerNum));

    if (zcPin != -1)
        check_err(L, gpio_intr_enable(zcPin));
}

static void disable_interrupts(lua_State* L) {
    if (timerGroup != -1)
        check_err(L, timer_disable_intr(timerGroup, timerNum));

    if (zcPin != -1)
        check_err(L, gpio_intr_disable(zcPin));
}

// pin
static int dimmer_add(lua_State* L) {
    int pin = luaL_checkint(L, 1);
    for (int i = 0; i < dimCount; i++) {
        if (dims[i].pin == pin) {
            return 0;
        }
    }
    check_err(L, gpio_set_direction(pin, GPIO_MODE_OUTPUT));
    disable_interrupts(L);
    dims = luaM_realloc_(L, dims, dimCount * sizeof(dim_t), (dimCount + 1) * sizeof(dim_t));
    dims[dimCount].pin = pin;
    dims[dimCount].level = 0;
    dims[dimCount].mode = 0;
    dimCount++;
    gpio_set_level(pin, 0);
    enable_interrupts(L);
    return 0;
}

static int dimmer_remove(lua_State* L) {
    int pin = luaL_checkint(L, 1);
    disable_interrupts(L);
    for (int i = 0; i < dimCount; i++) {
        if (dims[i].pin == pin) {
            for (int j = i; j < dimCount - 1; j++) {
                dims[j] = dims[j + 1];
            }
            dims = luaM_realloc_(L, dims, dimCount * sizeof(dim_t), (dimCount - 1) * sizeof(dim_t));
            dimCount--;
            enable_interrupts(L);
            return 0;
        }
    }
    enable_interrupts(L);
    luaL_error(L, "Error: pin %d is not dimmed.", pin);
    return 0;
}

static int dimmer_list_debug(lua_State* L) {
    disable_interrupts(L);
    ESP_LOGD(TAG, "p=%d, zcount=%d, t=%d", p, zCount, t);
    for (int i = 0; i < dimCount; i++) {
        ESP_LOGD(TAG, "pin=%d, mode=%d, level=%d", dims[i].pin, dims[i].mode, dims[i].level);
    }
    enable_interrupts(L);
    return 0;
}

static int dimmer_set_level(lua_State* L) {
    int pin = luaL_checkint(L, 1);
    int level = 1000 - luaL_checkint(L, 2);
    if (level > 1000) {
        level = 1000;
    } else if (level < 0) {
        level = 0;
    }

    for (int i = 0; i < dimCount; i++) {
        if (dims[i].pin == pin) {
            int newLevel = (int)(((double)level) / 1000.0 * ((double)p));
            dims[i].level = newLevel;
            if (newLevel <= t) {  // if the new level has been set at a time in which the pin should've been turned on already, turn it on.
                gpio_set_level(dims[i].pin, 1);
            }
            return 0;
        }
    }

    luaL_error(L, "Cannot set dim level of unconfigured pin %d", pin);
    return 0;
}

// hw timer group, hw_timer num, zc pin
static int dimmer_setup(lua_State* L) {
    disable_interrupts(L);
    timerGroup = luaL_checkint(L, 1);
    timerNum = luaL_checkint(L, 2);
    zcPin = luaL_checkint(L, 3);
    ESP_LOGD(TAG, "Dimmer setup. TG=%d, TN=%d, ZC=%d", timerGroup, timerNum, zcPin);

    if (timerGroup == 0) {
        timerGroupDev = &TIMERG0;
    } else {
        timerGroupDev = &TIMERG1;
    }

    timer_config_t config = {
        .alarm_en = true,
        .counter_en = false,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = true,
        .divider = TIMER_DIVIDER};

    timer_init(timerGroup, timerNum, &config);

    timer_set_counter_value(timerGroup, timerNum, 0);
    printf("timer_set_counter_value\n");

    timer_set_alarm_value(timerGroup, timerNum, 1);
    printf("timer_set_alarm_value\n");
    //  timer_enable_intr(timerGroup, timerNum);
    //    printf("timer_enable_intr\n");
    timer_isr_register(timerGroup, timerNum, &timer_isr, NULL, 0, &s_timer_handle);
    printf("timer_isr_register\n");

    timer_start(timerGroup, timerNum);
    printf("timer_start\n");

    check_err(L, gpio_set_direction(zcPin, GPIO_MODE_INPUT));
    check_err(L, gpio_set_pull_mode(zcPin, GPIO_PULLUP_ONLY));
    //    check_err(L, gpio_intr_disable(zcPin));
    check_err(L, gpio_set_intr_type(zcPin, GPIO_INTR_POSEDGE));
    check_err(L, gpio_isr_handler_add(zcPin, zc_isr, NULL));

    ESP_LOGD(TAG, "zc interrupt set");
    enable_interrupts(L);
    return 0;
}

// map client methods to functions:
LROT_BEGIN(dimmer_metatable)
/*
        {LSTRKEY("connect"), LFUNCVAL(mqtt_connect)},
        {LSTRKEY("close"), LFUNCVAL(mqtt_close)},
        {LSTRKEY("lwt"), LFUNCVAL(mqtt_lwt)},
        {LSTRKEY("publish"), LFUNCVAL(mqtt_publish)},
        {LSTRKEY("subscribe"), LFUNCVAL(mqtt_subscribe)},
        {LSTRKEY("unsubscribe"), LFUNCVAL(mqtt_unsubscribe)},
        {LSTRKEY("on"), LFUNCVAL(mqtt_on)},
        {LSTRKEY("__gc"), LFUNCVAL(mqtt_delete)},
        {LSTRKEY("__index"), LROVAL(mqtt_metatable_map)},
        */
LROT_END(dimmer_metatable, NULL, 0)

// Module function map
LROT_BEGIN(dimmer)
LROT_FUNCENTRY(setup, dimmer_setup)
LROT_FUNCENTRY(add, dimmer_add)
LROT_FUNCENTRY(remove, dimmer_remove)
LROT_FUNCENTRY(setLevel, dimmer_set_level)
LROT_FUNCENTRY(list, dimmer_list_debug)
LROT_END(dimmer, NULL, 0)

int luaopen_dimmer(lua_State* L) {
    luaL_rometatable(L, DIMMER_METATABLE, (void*)dimmer_metatable_map);  // create metatable for dimmer
    return 0;
}

NODEMCU_MODULE(DIMMER, "dimmer", dimmer, luaopen_dimmer);
