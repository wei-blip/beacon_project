#ifndef ZEPHYR_INCLUDE_LED_UTILS_LED_UTILS_H_
#define ZEPHYR_INCLUDE_LED_UTILS_LED_UTILS_H_

#include <drivers/led_strip.h>

#ifdef __cplusplus
extern "C" {
#endif

struct led_hsv {
    /** Hue channel: 0 ... 360 */
    int16_t h;
    /** Saturation channel */
    uint8_t s;
    /** Value channel */
    uint8_t v;
};

void led_hsv2rgb(const struct led_hsv *hsv, struct led_rgb *rgb);
uint8_t led_gamma_correction(uint8_t v);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_LED_UTILS_LED_UTILS_H_ */
