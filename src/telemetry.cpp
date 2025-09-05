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

#include "telemetry.hpp"
#include "rc.hpp"
#include "led.hpp"
#include "sensor.hpp"
#include "main_loop.hpp"
#include "stampfly.hpp"

uint8_t Telem_mode     = 0;
uint8_t Telem_cnt      = 0;
const uint8_t MAXINDEX = 14*4+2;
const uint8_t MININDEX = 30;

void telemetry_sequence(void);
void telemetry_sequence_fast(void);
void make_telemetry_header_data(uint8_t* senddata);
void make_telemetry_data(uint8_t* senddata);
void make_telemetry_data_fast(uint8_t* senddata);
void data2log(uint8_t* data_list, float add_data, uint8_t index);
void float2byte(float x, uint8_t* dst);
void append_data(uint8_t* data, uint8_t* newdata, uint8_t index, uint8_t len);
void data_set(uint8_t* datalist, float value, uint8_t* index);
void data_set_uint32(uint8_t* datalist, uint32_t value, uint8_t* index);
void data_set_uint16(uint8_t* datalist, uint16_t value, uint8_t* index);
void data_set_uint8(uint8_t* datalist, uint8_t value, uint8_t* index);

void telemetry(void) {
    //uint8_t senddata[MAXINDEX];
    
    if (Telem_mode == 0) {
        // Send header data
        Telem_mode = 1;
        //make_telemetry_header_data(senddata);

        // Send !
        //telemetry_send(senddata, sizeof(senddata));
    } else if (StampFly.flag.mode > AVERAGE_MODE) {
        const uint8_t N = 10;
        // N回に一度送信
        if (Telem_cnt == 0) telemetry_sequence();
        Telem_cnt++;
        if (Telem_cnt > N - 1) Telem_cnt = 0;
        // telemetry_sequence();
    }
}

void telemetry_sequence(void) {
    uint8_t senddata[MAXINDEX];

    switch (Telem_mode) {
        case 1:
            make_telemetry_data(senddata);
            // Send !
            if (telemetry_send(&peerInfo[TELEM], senddata, sizeof(senddata)) == 1){
                esp_led(0x330000, 1);  // Telemetory Reciver OFF
                //USBSerial.printf("NG Mode=%d\n\r", StampFly.flag.mode);
            }
            else{
                esp_led(0x003300, 1);  // Telemetory Reciver ON
                //USBSerial.printf("OK Mode=%d\n\r", StampFly.flag.mode);
            }
            // Telem_mode = 2;
            break;
    }
}

void make_telemetry_header_data(uint8_t* senddata) {
    float d_float;
    uint8_t d_int[4];
    uint8_t index = 0;

    index = 2;
    for (uint8_t i = 0; i < (MAXINDEX - 2) / 4; i++) {
        data2log(senddata, 0.0f, index);
        index = index + 4;
    }
    // Telemetry Header
    senddata[0] = 99;
    senddata[1] = 99;
    index       = 2;
    #if 0
    data_set(senddata, Roll_rate_kp, &index);
    data_set(senddata, Roll_rate_ti, &index);
    data_set(senddata, Roll_rate_td, &index);
    data_set(senddata, Roll_rate_eta, &index);
    data_set(senddata, Pitch_rate_kp, &index);
    data_set(senddata, Pitch_rate_ti, &index);
    data_set(senddata, Pitch_rate_td, &index);
    data_set(senddata, Pitch_rate_eta, &index);
    data_set(senddata, Yaw_rate_kp, &index);
    data_set(senddata, Yaw_rate_ti, &index);
    data_set(senddata, Yaw_rate_td, &index);
    data_set(senddata, Yaw_rate_eta, &index);
    data_set(senddata, Rall_angle_kp, &index);
    data_set(senddata, Rall_angle_ti, &index);
    data_set(senddata, Rall_angle_td, &index);
    data_set(senddata, Rall_angle_eta, &index);
    data_set(senddata, Pitch_angle_kp, &index);
    data_set(senddata, Pitch_angle_ti, &index);
    data_set(senddata, Pitch_angle_td, &index);
    data_set(senddata, Pitch_angle_eta, &index);
    #endif
}

void make_telemetry_data(uint8_t* senddata) {
    float d_float;
    uint8_t d_int[4];
    uint8_t index = 0;

    // Telemetry Header
    senddata[0] = 88;
    senddata[1] = 88;
    index       = 2;
    data_set(senddata, StampFly.times.elapsed_time, &index);            // 1 Time
    data_set(senddata, StampFly.times.interval_time, &index);           // 2 Interval Time
    data_set(senddata, StampFly.sensor.roll_angel * 180 / PI, &index);  // 3 Roll_angle
    data_set(senddata, StampFly.sensor.pitch_angle * 180 / PI, &index); // 4 Pitch_angle
    data_set(senddata, StampFly.sensor.yaw_angle * 180 / PI, &index);   // 5 Yaw_angle
    data_set(senddata, StampFly.sensor.roll_rate * 180 / PI, &index);   // 6 Roll_rate
    data_set(senddata, StampFly.sensor.pitch_rate * 180 / PI, &index);  // 7 Pitch_rate
    data_set(senddata, StampFly.sensor.yaw_rate * 180 / PI, &index);    // 8 Pitch_rate
    data_set(senddata, StampFly.sensor.accx, &index);                   // 9 accx
    data_set(senddata, StampFly.sensor.accy, &index);                   // 10 accy
    data_set(senddata, StampFly.sensor.accz, &index);                   // 11 accz
    data_set(senddata, StampFly.ref.roll, &index);                      // 12 Roll_ref
    data_set(senddata, StampFly.ref.pitch, &index);                     // 13 Pitch_ref
    data_set(senddata, StampFly.ref.throttle, &index);                  // 14 Yaw_ref
}

void data_set(uint8_t* datalist, float value, uint8_t* index) {
    data2log(datalist, value, *index);
    *index = *index + 4;
}

void data_set_uint32(uint8_t* datalist, uint32_t value, uint8_t* index) {
    append_data(datalist, (uint8_t*)&value, *index, 4);
    *index = *index + 4;
}

void data_set_uint16(uint8_t* datalist, uint16_t value, uint8_t* index) {
    append_data(datalist, (uint8_t*)&value, *index, 2);
    *index = *index + 2;
}

void data_set_uint8(uint8_t* datalist, uint8_t value, uint8_t* index) {
    append_data(datalist, (uint8_t*)&value, *index, 1);
    *index = *index + 1;
}

void data2log(uint8_t* data_list, float add_data, uint8_t index) {
    uint8_t d_int[4];
    float d_float = add_data;
    float2byte(d_float, d_int);
    append_data(data_list, d_int, index, 4);
}

void float2byte(float x, uint8_t* dst) {
    uint8_t* dummy;
    dummy  = (uint8_t*)&x;
    dst[0] = dummy[0];
    dst[1] = dummy[1];
    dst[2] = dummy[2];
    dst[3] = dummy[3];
}

void append_data(uint8_t* data, uint8_t* newdata, uint8_t index, uint8_t len) {
    for (uint8_t i = index; i < index + len; i++) {
        data[i] = newdata[i - index];
    }
}
