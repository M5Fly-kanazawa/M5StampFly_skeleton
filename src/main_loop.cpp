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

//
// StampFly Flight Control Main Module
//
// Desigend by Kouhei Ito 2023~2024
//
// 2024-08-11 StampFly 自己開発用のスケルトンプログラム制作開始

#include "main_loop.hpp"
#include "motor.hpp"
#include "rc.hpp"
#include "pid.hpp"
#include "sensor.hpp"
#include "led.hpp"
#include "telemetry.hpp"
#include "button.hpp"
#include "buzzer.h"
#include "stampfly.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


volatile uint8_t armButtonState = 0;
volatile uint8_t armButtonPressedAndRerleased = 0;
volatile uint8_t previousArmButtonState = 0; 

void IRAM_ATTR onTimer(void);
void init_copter(void);
void update_loop400Hz(void);
void init_mode(void);
void average_mode(void);
void flight_mode(void);
void parking_mode(void);
void loop_400Hz(void);
float limit(float value, float min, float max);
float deadband(float value, float db);

// Main loop
void loop_400Hz(void) {
    // 400Hzで以降のコードが実行

    update_loop400Hz();
    
    // Mode select
    if (StampFly.flag.mode == INIT_MODE) 
        init_mode();
    else if (StampFly.flag.mode == AVERAGE_MODE)
        average_mode();
    else if (StampFly.flag.mode == FLIGHT_MODE)
        flight_mode();
    else if (StampFly.flag.mode == PARKING_MODE)
        parking_mode();

    //// Telemetry
    telemetry();
    StampFly.flag.oldmode = StampFly.flag.mode;  // Memory now mode
    
    // End of Loop_400Hz function    
}

// 割り込み関数
// Intrupt function
hw_timer_t* timer = NULL;
void IRAM_ATTR onTimer(void) {
    StampFly.flag.loop = 1;
    //loop_400Hz();
}

// Initialize StampFly
void init_copter(void) {
    //disableCore1WDT();
    // Initialize Mode
    StampFly.flag.mode = INIT_MODE;
    StampFly.flag.loop = 0;
    // Initialaze LED function
    led_init();
    // Initialize Serial communication
    USBSerial.begin(115200);
    USBSerial.setTxTimeoutMs(0);
    delay(1500);
    USBSerial.printf("Start StampFly! Skeleton\r\n");
    motor_init();
    sensor_init();
    rc_init();

    // init button G0
    init_button();
    setup_pwm_buzzer();
    USBSerial.printf("Finish StampFly init!\r\n");
    start_tone();

    // 割り込み設定
    // Initialize intrupt
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 2500, true);
    timerAlarmEnable(timer);
}

//loop400Hzの更新関数
void update_loop400Hz(void) {
    uint32_t now_time;

    while (StampFly.flag.loop == 0);
    StampFly.flag.loop = 0;

    #if 0
    USBSerial.printf("%9.4f %9.4f %04d\n\r", 
        StampFly.times.elapsed_time, 
        StampFly.times.interval_time,
        StampFly.sensor.bottom_tof_range);
    #endif

    //Clock
    now_time = micros();
    StampFly.times.old_elapsed_time = StampFly.times.elapsed_time;
    StampFly.times.elapsed_time = 1e-6 * (now_time - StampFly.times.start_time);
    StampFly.times.interval_time = StampFly.times.elapsed_time - StampFly.times.old_elapsed_time;
    
    // Read Sensor Value
    sensor_read(&StampFly.sensor);
    
    // LED Drive
    led_drive();

    // Read Button Value
    armButtonState = Stick[BUTTON_ARM];
    if (armButtonState != previousArmButtonState) {
        if (armButtonState == 0) {
            armButtonPressedAndRerleased = 1;
        }
        previousArmButtonState = armButtonState;
    }
}

void init_mode(void) {
    motor_stop();
    StampFly.counter.offset = 0;
    //Mode change
    StampFly.flag.mode = AVERAGE_MODE;
    return;

}

void average_mode(void) {
    // Gyro offset Estimate 角速度のオフセットを取得
    // Set LED Color
    onboard_led1(PERPLE, 1);
    onboard_led2(PERPLE, 1);

    if (StampFly.counter.offset < AVERAGENUM) {
        sensor_calc_offset_avarage();
        StampFly.counter.offset++;
        return;
    }
    // Mode change
    StampFly.flag.mode   = PARKING_MODE;
    StampFly.times.start_time = micros();
    StampFly.times.old_elapsed_time = 0.0f;
    return;
}

void flight_mode(void) {
    //飛行するためのコードを以下に記述する
    // Set LED Color
    onboard_led1(YELLOW, 1);
    onboard_led2(YELLOW, 1);

    //スロットルの値を取得
    StampFly.ref.throttle = limit(Stick[THROTTLE], 0.0, 0.9);
    
    //制御目標を送信機のStickの倒し量から取得
    StampFly.ref.roll = limit(Stick[AILERON], -0.9, 0.9);
    StampFly.ref.pitch = limit(Stick[ELEVATOR], -0.9, 0.9);
    StampFly.ref.yaw = 5*limit(Stick[RUDDER], -0.9, 0.9);

    //不感帯を適用（ファイルの最後にdeadband関数追加）
    StampFly.ref.roll = deadband(StampFly.ref.roll, 0.03);
    StampFly.ref.pitch   = deadband(StampFly.ref.pitch, 0.03);
    StampFly.ref.yaw  = deadband(StampFly.ref.yaw,  0.03);

    //角速度誤差を計算
    float roll_rate_error  = StampFly.ref.roll  - StampFly.sensor.roll_rate;
    float pitch_rate_error = StampFly.ref.pitch - StampFly.sensor.pitch_rate;
    float yaw_rate_error   = StampFly.ref.yaw   - StampFly.sensor.yaw_rate;

    //PID制御則
    float delta_roll  = StampFly.pid.roll.update(  
        roll_rate_error, 
        StampFly.times.interval_time);

    float delta_pitch = StampFly.pid.pitch.update(
        pitch_rate_error, 
        StampFly.times.interval_time);
    
    float delta_yaw   = StampFly.pid.yaw.update(
        yaw_rate_error, 
        StampFly.times.interval_time);

    //トリム調整（機体のアンバランスをキャンセルするためトリム値を加算）
    float trim_roll  = 0.01;
    float trim_pitch = 0.00;
    float trim_yaw   =  0.0;
    delta_roll  += trim_roll;
    delta_pitch += trim_pitch;
    delta_yaw   += trim_yaw;

    //ミキシング
    float front_left_duty  = StampFly.ref.throttle + 0.25*(  delta_roll + delta_pitch - delta_yaw)/BATTERY_VOLTAGE;
    float front_right_duty = StampFly.ref.throttle + 0.25*(- delta_roll + delta_pitch + delta_yaw)/BATTERY_VOLTAGE;
    float rear_left_duty   = StampFly.ref.throttle + 0.25*(  delta_roll - delta_pitch + delta_yaw)/BATTERY_VOLTAGE;
    float rear_right_duty  = StampFly.ref.throttle + 0.25*(- delta_roll - delta_pitch - delta_yaw)/BATTERY_VOLTAGE;

    //Duty比を0.0~0.95に制限
    front_left_duty  = limit(front_left_duty,  0.0, 0.95);
    front_right_duty = limit(front_right_duty, 0.0, 0.95);
    rear_left_duty   = limit(rear_left_duty,   0.0, 0.95);
    rear_right_duty  = limit(rear_right_duty,  0.0, 0.95);

    //PWMのDutyをセット
    motor_set_duty_fl(front_left_duty);
    motor_set_duty_fr(front_right_duty);
    motor_set_duty_rl(rear_left_duty);
    motor_set_duty_rr(rear_right_duty);

    //Arm（スロットル）ボタンを監視して押されたらParkingモードに復帰するためのコード
    if (armButtonPressedAndRerleased)StampFly.flag.mode = PARKING_MODE;
    armButtonPressedAndRerleased = 0;

}

void parking_mode(void) {
    //着陸している時に行う処理を記述する
    // Set LED Color
    onboard_led1(GREEN, 1);
    onboard_led2(GREEN, 1);

    StampFly.counter.loop = 0;
    
    //PID Gain set
    const float kp_roll  = 0.65f;
    const float ti_roll  = 0.7f;
    const float td_roll  = 0.01f;
    const float eta_roll  = 0.125f;

    const float kp_pitch = 0.95f;
    const float ti_pitch = 0.7f;
    const float td_pitch = 0.01f;
    const float eta_pitch = 0.125f;

    const float kp_yaw   = 3.0f;
    const float ti_yaw   = 0.7f;
    const float td_yaw   = 0.01f;
    const float eta_yaw   = 0.125f;
    const float h = 0.0025f;

    StampFly.pid.roll.set_parameter(  kp_roll,  ti_roll,  td_roll,  eta_roll,  h);
    StampFly.pid.pitch.set_parameter( kp_pitch, ti_pitch, td_pitch, eta_pitch, h);
    StampFly.pid.yaw.set_parameter(   kp_yaw,   ti_yaw,   td_yaw,   eta_yaw,   h);

    //PID Integral Reset
    StampFly.pid.roll.reset();
    StampFly.pid.pitch.reset();
    StampFly.pid.yaw.reset();

    motor_stop();
    if (armButtonPressedAndRerleased)StampFly.flag.mode = FLIGHT_MODE;
    armButtonPressedAndRerleased = 0;

    //電源電圧をシリアルモニタで表示
    USBSerial.printf("Battery Voltage: %5.2f V\r\n", StampFly.sensor.voltage);
}

float limit(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

float deadband(float value, float db) {
    if (value > db) return (value - db) / (1.0 - db);
    if (value < -db) return (value + db) / (1.0 - db);
    return 0.0;
}