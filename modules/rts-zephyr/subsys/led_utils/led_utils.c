#include <led_utils/led_utils.h>

void led_hsv2rgb(const struct led_hsv *hsv, struct led_rgb *rgb)
{
    if (!hsv->v)
    {
        rgb->r = rgb->g = rgb->b = 0;
    }
    else if (!hsv->s)
    {
        rgb->r = rgb->g = rgb->b = hsv->v;
    }
    else
    {
        int hue = hsv->h % 360;
        hue = hue < 0 ? 360 + hue : hue;

        int sector = hue / 60;
        int angle = sector & 1 ? 60 - hue % 60 : hue % 60;

        int high = hsv->v;
        int low = (255 - hsv->s) * high / 255;
        int middle = low + (high - low) * angle / 60;

        switch (sector)
        {
            case 0: // red -> yellow
                rgb->r = high;
                rgb->g = middle;
                rgb->b = low;

                break;

            case 1: // yellow -> green
                rgb->r = middle;
                rgb->g = high;
                rgb->b = low;

                break;

            case 2: // green -> cyan
                rgb->r = low;
                rgb->g = high;
                rgb->b = middle;

                break;

            case 3: // cyan -> blue
                rgb->r = low;
                rgb->g = middle;
                rgb->b = high;

                break;

            case 4: // blue -> magenta
                rgb->r = middle;
                rgb->g = low;
                rgb->b = high;

                break;

            case 5: // magenta -> red
                rgb->r = high;
                rgb->g = low;
                rgb->b = middle;

            default:
                break;
        }
    }
}

#ifdef CONFIG_LED_UTILS_USE_PRECALCULATED_GAMMA_TABLE
static const uint8_t led_gamma_table[] = {
    0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   1,   1,   1,   1,
    1,   1,   1,   1,   2,   2,   2,   2,
    2,   2,   3,   3,   3,   3,   4,   4,
    4,   4,   5,   5,   5,   5,   6,   6,
    6,   7,   7,   7,   8,   8,   8,   9,
    9,   9,   10,  10,  11,  11,  11,  12,
    12,  13,  13,  14,  14,  15,  15,  16,
    16,  17,  17,  18,  18,  19,  19,  20,
    20,  21,  21,  22,  23,  23,  24,  24,
    25,  26,  26,  27,  28,  28,  29,  30,
    30,  31,  32,  32,  33,  34,  35,  35,
    36,  37,  38,  38,  39,  40,  41,  42,
    42,  43,  44,  45,  46,  47,  47,  48,
    49,  50,  51,  52,  53,  54,  55,  56,
    56,  57,  58,  59,  60,  61,  62,  63,
    64,  65,  66,  67,  68,  69,  70,  71,
    73,  74,  75,  76,  77,  78,  79,  80,
    81,  82,  84,  85,  86,  87,  88,  89,
    91,  92,  93,  94,  95,  97,  98,  99,
    100, 102, 103, 104, 105, 107, 108, 109,
    111, 112, 113, 115, 116, 117, 119, 120,
    121, 123, 124, 126, 127, 128, 130, 131,
    133, 134, 136, 137, 139, 140, 142, 143,
    145, 146, 148, 149, 151, 152, 154, 155,
    157, 158, 160, 162, 163, 165, 166, 168,
    170, 171, 173, 175, 176, 178, 180, 181,
    183, 185, 186, 188, 190, 192, 193, 195,
    197, 199, 200, 202, 204, 206, 207, 209,
    211, 213, 215, 217, 218, 220, 222, 224,
    226, 228, 230, 232, 233, 235, 237, 239,
    241, 243, 245, 247, 249, 251, 253, 255 };
#endif

uint8_t led_gamma_correction(uint8_t v)
{
#ifdef CONFIG_LED_UTILS_USE_PRECALCULATED_GAMMA_TABLE
    return led_gamma_table[v];
#else
    return (v * v + v) >> 8;
#endif
}
