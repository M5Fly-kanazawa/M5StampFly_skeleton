/*
 * MIT License
 *
 * Copyright (c) 2024 Kouhei Ito
 * Copyright (c) 2024 M5Stack
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "led.hpp"

uint32_t Led_color       = 0x000000;
uint32_t Led_color2      = 255;
uint32_t Led_color3      = 0x000000;
uint16_t LedBlinkCounter = 0;
CRGB led_esp[1];
CRGB led_onboard[2];

void led_drive(void);
void onboard_led1(CRGB p, uint8_t state);
void onboard_led2(CRGB p, uint8_t state);
void esp_led(CRGB p, uint8_t state);

void led_init(void) {
    FastLED.addLeds<WS2812, PIN_LED_ONBORD, GRB>(led_onboard, 2);
    FastLED.addLeds<WS2812, PIN_LED_ESP, GRB>(led_esp, 1);
    FastLED.setBrightness(15);
    esp_led(WHITE, 1);
    onboard_led1(WHITE, 1);
    onboard_led2(WHITE, 1);
    led_show();
    led_show();
    led_show();
}

void led_show(void) {
    uint8_t bright = 8;
    if (  StampFly.sensor.voltage > 0.7 && StampFly.sensor.voltage < POWER_LIMIT) {
        onboard_led1(POWEROFFCOLOR, 1);
        onboard_led2(POWEROFFCOLOR, 1);
    }
    FastLED.show(bright);
}

void led_drive(void) {
    //onboard_led1(PERPLE, 1);
    //onboard_led2(YELLOW, 1);
    //esp_led(WHITE,1);
    // LED show
    led_show();
}

void onboard_led1(CRGB p, uint8_t state) {
    if (state == 1) {
        led_onboard[0] = p;
    } else {
        led_onboard[0] = 0;
    }
    return;
}

void onboard_led2(CRGB p, uint8_t state) {
    if (state == 1) {
        led_onboard[1] = p;
    } else {
        led_onboard[1] = 0;
    }
    return;
}

void esp_led(CRGB p, uint8_t state) {
    if (state == 1)
        led_esp[0] = p;
    else
        led_esp[0] = 0;
    return;
}
