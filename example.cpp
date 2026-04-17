#include "SpeedPID.h"
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <cmath>

constexpr int8_t PIN_DIR_1      = 23;
constexpr int8_t PIN_PWM_1      = 22;
constexpr int8_t PIN_ROTARY_A_1 = 27;
constexpr int8_t PIN_ROTARY_B_1 = 14;
constexpr int8_t Motor1_CH      = 0;
constexpr int8_t W1_SIGN        = -1;

constexpr double target_rpm = 420;

constexpr int16_t PWM_LIMIT = 255;

ESP32Encoder enc1;

SpeedPID speed_pid_1(1.5, 3., 0., -PWM_LIMIT, PWM_LIMIT);

constexpr double ENC_COUNTS_PER_REV = 4096.0;    // エンコーダ1回転カウント
constexpr double GEAR_RATIO         = 25. / 22.; // 減速比 (円盤) : (エンコーダ側のギヤ)

// signは、モータの配線や取り付け向きの補正
void setMotor(int8_t dirPin, int pwmCh, int sign, int pwm_signed) {
    bool is_dir = HIGH;
    int  duty   = sign * pwm_signed;
    if (duty > 0) {
        is_dir = HIGH;
    } else {
        is_dir = LOW;
    }
    digitalWrite(dirPin, is_dir);
    ledcWrite(pwmCh, abs(duty)); // pwmCh は LEDC channel (PWM_CH_1 など)
}

unsigned long last = micros();

void setup() {
    Serial.begin(115200);

    pinMode(PIN_DIR_1, OUTPUT);
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    enc1.attachHalfQuad(PIN_ROTARY_A_1, PIN_ROTARY_B_1);

    enc1.clearCount();

    ledcSetup(Motor1_CH, 12800, 8);

    ledcAttachPin(PIN_PWM_1, Motor1_CH);

    last = micros();
}

constexpr long CONTROL_CYCLE = 5000;
long           prev_count_1  = 0;

void loop() {
    unsigned long now = micros();
    if (now - last < CONTROL_CYCLE) return;
    last += CONTROL_CYCLE;
    double dt = CONTROL_CYCLE * 1e-6;

    long c1 = enc1.getCount();

    const long dc1 = c1 - prev_count_1;

    prev_count_1 = c1;

    double rpm = dc1 * GEAR_RATIO / ENC_COUNTS_PER_REV / dt * 60;

    double output_1 = speed_pid_1.update(target_rpm, rpm, dt);

    setMotor(PIN_DIR_1, Motor1_CH, W1_SIGN, (int)(lround(output_1)));

    Serial.printf("rpm:%f target:%f", rpm, target_rpm);
}
