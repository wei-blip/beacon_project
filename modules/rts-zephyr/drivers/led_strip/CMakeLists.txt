# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_WS2812_STRIP_PWM ws2812_pwm_stm32.c)
