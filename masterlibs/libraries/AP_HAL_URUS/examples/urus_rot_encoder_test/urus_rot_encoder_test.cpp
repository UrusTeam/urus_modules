#include <AP_HAL/AP_HAL.h>
#include <UR_Rotary_Encoder/UR_Rotary_Encoder.h>

#include "urus_rot_encoder_test.h"

#define TEST_INTERRUPT ENABLED
#define LED_PIN        13

static UR_Rotary_Encoder rot_enc;

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static CLurus_rot_encoder_test urus_rot_encoder_test_app;

void CLurus_rot_encoder_test::update(void)
{
    if (rot_enc.get_step_status()) {
        const char *enc_dir = rot_enc.get_enc_dir() ? "CW" : "CCW";
        uint16_t tick_val = rot_enc.get_step_val();
        uint32_t rpm = rot_enc.get_rpm();

        hal.console->printf("%03d %ld DIR: %s\n", tick_val, rpm, enc_dir);
        hal.gpio->toggle(LED_PIN);
    }
}

void CLurus_rot_encoder_test::setup(void)
{
    hal.gpio->pinMode(LED_PIN, HAL_GPIO_OUTPUT);

    hal.console->printf("\nStarting urus_rot_encoder_test_app...\n");

    UR_Rotary_Encoder::rot_enc_conf_t rot_conf;

    rot_conf.pin_a = 2;
    rot_conf.pin_b = 4;

    rot_enc.set_encoder_state_conf(rot_conf);

    rot_enc.set_step_val(200);
    rot_enc.inverted_dir(false);

    rot_enc.configure();

#if TEST_INTERRUPT == ENABLED
    hal.gpio->attach_interrupt(2, rot_enc.interrupt_flipflop, 0);
 #endif // TEST_INTERRUPT
}

void CLurus_rot_encoder_test::loop(void)
{
    update();
#if TEST_INTERRUPT == DISABLED
    rot_enc.update();
#endif // TEST_INTERRUPT
}

void setup(void);
void loop(void);

void setup(void)
{
    urus_rot_encoder_test_app.setup();
}

void loop(void)
{
    urus_rot_encoder_test_app.loop();
}

AP_HAL_MAIN();
