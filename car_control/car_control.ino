#include <Arduino.h>
#include "sbus.h"
#include <ESP32Servo.h>
#include <driver/pulse_cnt.h>
#include "rtc_wdt.h"

#define MOTOR_A_PWM 21
#define MOTOR_A_IN1 23
#define MOTOR_A_IN2 22
#define MOTOR_A_EA  36
#define MOTOR_A_EB  39

#define MOTOR_B_PWM 32
#define MOTOR_B_IN1 25
#define MOTOR_B_IN2 33
#define MOTOR_B_EA  34
#define MOTOR_B_EB  35

#define MOTOR_C_PWM 13
#define MOTOR_C_IN1 12
#define MOTOR_C_IN2 14
#define MOTOR_C_EA  27
#define MOTOR_C_EB  26

#define MOTOR_D_PWM 5
#define MOTOR_D_IN1 2
#define MOTOR_D_IN2 4
#define MOTOR_D_EA  19
#define MOTOR_D_EB  18

#define SERVO_PIN   15   //舵机引脚

#define VERTREFRESH 50    //计数器刷新频率
#define MAXPWM      255  
#define DIFF_TURN_GAIN 1

#define VISION_UART_BAUD 115200
#define VISION_UART_RX_PIN 37   // TODO: change to the actual ESP32 RX pin wired to K230 TX.
#define VISION_UART_TX_PIN -1   // K230 only sends detections, so TX is unused by default.

#define VISION_START_HIGH 0xAA
#define VISION_START_LOW  0xBB
#define VISION_PACKET_DETECTIONS 0x01
#define VISION_TEAM_RED 0x01
#define VISION_MAX_DETECTIONS 12
#define VISION_DETECTION_SIZE 10
#define VISION_MAX_PAYLOAD (2 + VISION_MAX_DETECTIONS * VISION_DETECTION_SIZE)
#define VISION_TIMEOUT_MS 500

#define FRAME_UP_ANGLE 10       // Calibrate on hardware.
#define FRAME_DOWN_ANGLE 170    // Calibrate on hardware.

#define AUTO_CONTROL_PERIOD_MS 50
#define SEARCH_RZ_SPEED 70
#define SEARCH_SWEEP_MS 2500
#define APPROACH_Y_SPEED 75
#define APPROACH_X_GAIN 0.18f
#define APPROACH_RZ_GAIN 0.08f
#define CENTER_DEADZONE 45
#define CAPTURE_CY_THRESHOLD 820
#define CAPTURE_H_THRESHOLD 220
#define CAPTURE_HOLD_MS 700
#define SAFE_APPROACH_Y_SPEED 65
#define SAFE_APPROACH_X_GAIN 0.16f
#define SAFE_APPROACH_RZ_GAIN 0.06f
#define SAFE_CY_THRESHOLD 760
#define SAFE_H_THRESHOLD 520
#define RELEASE_HOLD_MS 700
#define RELEASE_BACKUP_MS 900
#define RELEASE_BACKUP_SPEED -45

// 两轮差速底盘：A/B为左右驱动轮，前方为万向轮。
pcnt_unit_handle_t pcnt_unit_1 = NULL;
pcnt_unit_handle_t pcnt_unit_2 = NULL;

Servo servo;
int minUs = 500;
int maxUs = 2500;

bfs::SbusRx sbus_rx(&Serial2, 16, 17, true, false);   //sbus接收机
bfs::SbusData sbus_data;
int sbusMiddle[3] = {1000,1038,1000};
HardwareSerial VisionSerial(1);

/* 定义控制模式 */
enum ControlMode { AUTO, MANUAL };
ControlMode controlMode = AUTO;  // 默认模式

/*-------------------霍尔编码器计数器--------------------*/
//MG370每圈13个脉冲，1:34减速比，轮子转动一圈共有442个脉冲,一秒的时间里，每个duty（0-255）对应约16.56个脉冲，约2.24289rpm
volatile int Velocity1 = 0;   //左轮，换算为0-255的duty
volatile int Velocity2 = 0;   //右轮，换算为0-255的duty

// 观察点回调函数
static bool pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *event_data, void *user_ctx) {
    // 当计数器达到观察点时，清零计数器
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
    return false;  // 返回 false 表示不停止计数器
}

// 初始化 PCNT 模块
void setupEncoderPCNT(pcnt_unit_handle_t* unit, int pulse_pin, int ctrl_pin) {
    // 配置 PCNT 单元
    pcnt_unit_config_t unit_config = {
        .low_limit = -32768,  // 计数器下限
        .high_limit = 32767,  // 计数器上限
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, unit));

    // 配置通道
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = pulse_pin,
        .level_gpio_num = ctrl_pin,
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(*unit, &chan_config, &pcnt_chan));

    // 设置通道行为
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // 设置毛刺滤波器（阈值单位为 APB 时钟周期，1 个周期 = 12.5ns）
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(*unit, &filter_config));

    // 设置观察点
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(*unit, unit_config.high_limit));  // 上限观察点
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(*unit, unit_config.low_limit));   // 下限观察点

    // 注册事件回调函数
    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_on_reach,
    };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(*unit, &cbs, NULL));

    // 启用 PCNT 单元
    ESP_ERROR_CHECK(pcnt_unit_enable(*unit));

    // 启动 PCNT 单元
    ESP_ERROR_CHECK(pcnt_unit_start(*unit));
}

// 读取编码器计数值
int readEncoderPCNT(pcnt_unit_handle_t unit) {
    int pulse_count = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(unit, &pulse_count));
    return pulse_count;
}

// 定时器回调函数，用于定期读取编码器值并计算速度
void calculateSpeed(TimerHandle_t xTimer) {
    static int lastCount1 = 0, lastCount2 = 0;

    // 读取当前计数值
    int currentCount1 = readEncoderPCNT(pcnt_unit_1);
    int currentCount2 = readEncoderPCNT(pcnt_unit_2);

    // 计算脉冲变化量，并处理计数器回绕
    int delta1 = currentCount1 - lastCount1;
    if (delta1 > 3276) {
        delta1 -= 32768;  // 正向回绕
    } else if (delta1 < -3276) {
        delta1 += 32768;  // 反向回绕
    }

    int delta2 = currentCount2 - lastCount2;
    if (delta2 > 3276) {
        delta2 -= 32768;
    } else if (delta2 < -3276) {
        delta2 += 32768;
    }

    // 更新上一次的计数值
    lastCount1 = currentCount1;
    lastCount2 = currentCount2;

    // 计算速度（RPM）
    int speed1 = (delta1 * 60) / (374 * 0.05);    // 11*34=374脉冲一圈
    int speed2 = (delta2 * 60) / (374 * 0.05);

    // 计算速度（duty）
    Velocity1 = (delta1 * 2000) / (34 * VERTREFRESH);
    Velocity2 = (delta2 * 2000) / (34 * VERTREFRESH);
  
    // 打印速度和计数值
    // Serial.printf("Encoder 1: Count=%d, Speed=%d, Velocity1=%d\n", currentCount1, speed1, Velocity1);
    // Serial.printf("Encoder 2: Count=%d, Speed=%d, Velocity2=%d\n", currentCount2, speed2, Velocity2);
    // Serial.println("-----------------------------");
    // esp_task_wdt_reset();
}
/*-------------------霍尔编码器计数器--------------------*/

class MotorController {
private:
    int pwmPin, dirPinA, dirPinB;
    int targetSpeed;        // 目标速度 (duty)
    volatile int* currentSpeed;      // 当前速度 (指针)
    float kp, ki, kd;       // PID 参数
    int integral;           // 积分项
    int lastError;          // 上一次误差
    int maxOutput;          // 最大 PWM 输出

public:
    MotorController(int pwm, int dirA, int dirB, int maxPWM = MAXPWM)
        : pwmPin(pwm), dirPinA(dirA), dirPinB(dirB), targetSpeed(0), currentSpeed(nullptr),
          kp(0.1), ki(0), kd(0.001), integral(0), lastError(0), maxOutput(maxPWM) {
        begin();
    }

    void begin() {
        ledcAttach(pwmPin, 12000, 8);      //使用12kHz和8位分辨率
        pinMode(dirPinA, OUTPUT);
        pinMode(dirPinB, OUTPUT);
    }

    int getCurrentSpeed() {
        return currentSpeed ? *currentSpeed : 0;  // 如果指针为空，则返回 0
    }

    void setPID(float p, float i, float d) {
        kp = p; ki = i; kd = d;
    }

    void setTargetSpeed(int speed) {
        targetSpeed = speed;
    }

    void setCurrentSpeed(volatile int* speed) {  // 接受指针
        currentSpeed = speed;
    }

    void update() {
        if (!currentSpeed) return;  // 如果未设置 currentSpeed，则直接返回

        int error = targetSpeed - *currentSpeed;
        integral += error;
        int derivative = error - lastError;
        lastError = error;

        int output = targetSpeed + kp * error + ki * integral + kd * derivative;
        output = constrain(output, -maxOutput, maxOutput);

        if (output > 0) {
            digitalWrite(dirPinA, HIGH);
            digitalWrite(dirPinB, LOW);
        } else if (output < 0) {
            digitalWrite(dirPinA, LOW);
            digitalWrite(dirPinB, HIGH);
            output = -output;
        } else {
            digitalWrite(dirPinA, LOW);
            digitalWrite(dirPinB, LOW);
        }
        ledcWrite(pwmPin, output);
    }
};

void initSbusMiddle(int16_t sbus_ch0, int16_t sbus_ch1, int16_t sbus_ch3){          //sbus中值校准
    sbusMiddle[0] = sbus_ch0;
    sbusMiddle[1] = sbus_ch1;
    sbusMiddle[2] = sbus_ch3;
    // Serial.println("sbus middle init");
}

/*------------------车轮初始化------------------------------*/
MotorController leftMotor(MOTOR_A_PWM, MOTOR_A_IN1, MOTOR_A_IN2);
MotorController rightMotor(MOTOR_B_PWM, MOTOR_B_IN1, MOTOR_B_IN2);
/*------------------车轮初始化------------------------------*/

// 两轮差速底盘无法横移，target_x由上层转向控制吸收。
void calc_velocity(int target_x, int target_y, int target_zr, int* left, int* right) {
  (void)target_x;
  int turn = target_zr * DIFF_TURN_GAIN;
  *left = constrain(target_y + turn, -MAXPWM, MAXPWM);
  *right = constrain(target_y - turn, -MAXPWM, MAXPWM);
}

// 设置两侧驱动轮速度：y为前后，rz为原地/弧线转向。
void handleMotor(int x, int y, int rz) {
    int left_vel = 0, right_vel = 0;
    calc_velocity(x, y, rz, &left_vel, &right_vel);
    leftMotor.setTargetSpeed(left_vel);
    rightMotor.setTargetSpeed(right_vel);
}

int sbusMap(int input, int middle) {
  int temp = (input - middle)/4;
  return temp*MAXPWM/300;
}

// 手动控制定时器回调函数
uint8_t last_sbus = 0;
void manualControl_TimerCallback(TimerHandle_t xTimer) {
    // esp_task_wdt_reset();
    if (sbus_rx.Read()) {                                   // 设置模式
      sbus_data = sbus_rx.data();
      if (sbus_data.ch[5] < 1000 && controlMode == AUTO) {
          controlMode = MANUAL;
      } else if (sbus_data.ch[5] > 1100 && controlMode == MANUAL) {
          controlMode = AUTO;
      }
      if (sbus_data.ch[6] > 1200) {
          initSbusMiddle(sbus_data.ch[0], sbus_data.ch[1], sbus_data.ch[3]);     // 中值校准
      }
      if (sbus_data.lost_frame || sbus_data.failsafe) {
          last_sbus++;
          if (last_sbus > 62) {   //62x80=4960ms没信号切AUTO模式
              controlMode = AUTO;
              last_sbus = 0;
          }
      } else last_sbus = 0;
    
      if (controlMode == MANUAL) {
          Serial.println("MANUAL MODE");
          
          // 解析并映射遥控器通道值
          int x = sbusMap(sbus_data.ch[0], sbusMiddle[0]);
          int y = -sbusMap(sbus_data.ch[1], sbusMiddle[1]);
          int rz = -sbusMap(sbus_data.ch[3], sbusMiddle[2]);
          if (sbus_data.ch[6] > 1200) {           //中值校准时停止运动
              handleMotor(0, 0, 0);
          } else if (sbus_data.lost_frame || sbus_data.failsafe) {
              handleMotor(0, 0, 0);
          } else {
              handleMotor(x, y, rz);
              Serial.printf("x:%d, y:%d, rz:%d \n", x, y, rz);
              // Serial.printf("sbusMiddle[0]:%d, sbusMiddle[1]:%d, sbusMiddle[2]:%d \n", sbusMiddle[0], sbusMiddle[1], sbusMiddle[2]);
          }
          
          int servo_angle = map(sbus_data.ch[4], 352, 1696, 0, 180);
          servo.write(servo_angle);
          Serial.printf("servo servo angle:%d \n", servo_angle);
      }
    } else {
        Serial.println("SBUS Read failed");
        if (controlMode == MANUAL)  handleMotor(0, 0, 0);
    }
}
/*------------------K230视觉检测协议与自动状态机------------------*/
enum VisionClassId {
    CLASS_CROSS_MARKER = 0,
    CLASS_RED_BALL = 1,
    CLASS_YELLOW_BALL = 2,
    CLASS_BLUE_BALL = 3,
    CLASS_BLACK_BALL = 4,
    CLASS_RED_SAFE_ZONE = 5,
    CLASS_BLUE_SAFE_ZONE = 6,
    CLASS_PURPLE_BOUNDARY = 7,
    CLASS_BLUE_START_ZONE = 8,
    CLASS_RED_START_ZONE = 9
};

struct VisionDetection {
    uint8_t class_id;
    uint8_t score;
    uint16_t cx;
    uint16_t cy;
    uint16_t w;
    uint16_t h;
};

struct VisionFrame {
    uint8_t frame_id;
    uint8_t count;
    bool valid;
    uint32_t received_ms;
    VisionDetection detections[VISION_MAX_DETECTIONS];
} latestVision = {0, 0, false, 0, {}};

enum AutoState {
    SEARCH_BALL,
    APPROACH_BALL,
    CAPTURE,
    FIND_SAFE_ZONE,
    APPROACH_SAFE_ZONE,
    RELEASE
};

AutoState autoState = SEARCH_BALL;
uint32_t autoStateStartedMs = 0;
int searchDirection = 1;
bool carryingTarget = false;

enum VisionParseState {
    WAIT_START_HIGH,
    WAIT_START_LOW,
    READ_TYPE,
    READ_FRAME_ID,
    READ_PAYLOAD_LEN,
    READ_PAYLOAD,
    READ_CRC_HIGH,
    READ_CRC_LOW
};

VisionParseState visionParseState = WAIT_START_HIGH;
uint8_t parserType = 0;
uint8_t parserFrameId = 0;
uint8_t parserPayloadLen = 0;
uint8_t parserPayload[VISION_MAX_PAYLOAD];
uint8_t parserPayloadIndex = 0;
uint8_t parserCrcHigh = 0;
uint16_t parserCrc = 0xFFFF;

uint16_t crc16CcittUpdate(uint16_t crc, uint8_t data) {
    crc ^= (uint16_t)data << 8;
    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 0x8000) {
            crc = (crc << 1) ^ 0x1021;
        } else {
            crc <<= 1;
        }
    }
    return crc;
}

void resetVisionParser() {
    visionParseState = WAIT_START_HIGH;
    parserPayloadIndex = 0;
    parserPayloadLen = 0;
    parserCrc = 0xFFFF;
}

uint16_t readU16Le(const uint8_t *data) {
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

bool parseDetectionPayload() {
    if (parserType != VISION_PACKET_DETECTIONS || parserPayloadLen < 2) {
        return false;
    }

    uint8_t team_color = parserPayload[0];
    uint8_t reported_count = parserPayload[1];
    if (team_color != VISION_TEAM_RED) {
        return false;
    }

    uint8_t available_count = (parserPayloadLen - 2) / VISION_DETECTION_SIZE;
    uint8_t count = min(reported_count, available_count);
    count = min(count, (uint8_t)VISION_MAX_DETECTIONS);

    latestVision.frame_id = parserFrameId;
    latestVision.count = count;
    latestVision.valid = true;
    latestVision.received_ms = millis();

    for (uint8_t i = 0; i < count; i++) {
        uint8_t *item = &parserPayload[2 + i * VISION_DETECTION_SIZE];
        latestVision.detections[i].class_id = item[0];
        latestVision.detections[i].score = item[1];
        latestVision.detections[i].cx = readU16Le(&item[2]);
        latestVision.detections[i].cy = readU16Le(&item[4]);
        latestVision.detections[i].w = readU16Le(&item[6]);
        latestVision.detections[i].h = readU16Le(&item[8]);
    }

    return true;
}

void processVisionByte(uint8_t incoming) {
    switch (visionParseState) {
        case WAIT_START_HIGH:
            if (incoming == VISION_START_HIGH) {
                visionParseState = WAIT_START_LOW;
            }
            break;

        case WAIT_START_LOW:
            if (incoming == VISION_START_LOW) {
                parserCrc = 0xFFFF;
                parserPayloadIndex = 0;
                visionParseState = READ_TYPE;
            } else {
                visionParseState = WAIT_START_HIGH;
            }
            break;

        case READ_TYPE:
            parserType = incoming;
            parserCrc = crc16CcittUpdate(parserCrc, incoming);
            visionParseState = READ_FRAME_ID;
            break;

        case READ_FRAME_ID:
            parserFrameId = incoming;
            parserCrc = crc16CcittUpdate(parserCrc, incoming);
            visionParseState = READ_PAYLOAD_LEN;
            break;

        case READ_PAYLOAD_LEN:
            parserPayloadLen = incoming;
            parserCrc = crc16CcittUpdate(parserCrc, incoming);
            if (parserPayloadLen > VISION_MAX_PAYLOAD) {
                resetVisionParser();
            } else if (parserPayloadLen == 0) {
                visionParseState = READ_CRC_HIGH;
            } else {
                parserPayloadIndex = 0;
                visionParseState = READ_PAYLOAD;
            }
            break;

        case READ_PAYLOAD:
            parserPayload[parserPayloadIndex++] = incoming;
            parserCrc = crc16CcittUpdate(parserCrc, incoming);
            if (parserPayloadIndex >= parserPayloadLen) {
                visionParseState = READ_CRC_HIGH;
            }
            break;

        case READ_CRC_HIGH:
            parserCrcHigh = incoming;
            visionParseState = READ_CRC_LOW;
            break;

        case READ_CRC_LOW: {
            uint16_t received_crc = ((uint16_t)parserCrcHigh << 8) | incoming;
            if (received_crc == parserCrc) {
                parseDetectionPayload();
            } else {
                Serial.println("Vision CRC error");
            }
            resetVisionParser();
            break;
        }
    }
}

const char *autoStateName(AutoState state) {
    switch (state) {
        case SEARCH_BALL: return "SEARCH_BALL";
        case APPROACH_BALL: return "APPROACH_BALL";
        case CAPTURE: return "CAPTURE";
        case FIND_SAFE_ZONE: return "FIND_SAFE_ZONE";
        case APPROACH_SAFE_ZONE: return "APPROACH_SAFE_ZONE";
        case RELEASE: return "RELEASE";
    }
    return "UNKNOWN";
}

void enterAutoState(AutoState nextState) {
    if (autoState != nextState) {
        Serial.printf("AUTO %s -> %s\n", autoStateName(autoState), autoStateName(nextState));
    }
    autoState = nextState;
    autoStateStartedMs = millis();
}

int clampSpeed(int value, int limit) {
    return constrain(value, -limit, limit);
}

int centerControl(uint16_t cx, float gain, int limit) {
    int error = (int)cx - 500;
    if (abs(error) < CENTER_DEADZONE) {
        return 0;
    }
    return clampSpeed((int)(error * gain), limit);
}

int targetPoints(uint8_t class_id) {
    if (class_id == CLASS_YELLOW_BALL) return 15;
    if (class_id == CLASS_BLACK_BALL) return 10;
    if (class_id == CLASS_RED_BALL) return 5;
    return 0;
}

bool isTargetBall(uint8_t class_id) {
    return class_id == CLASS_RED_BALL || class_id == CLASS_BLACK_BALL || class_id == CLASS_YELLOW_BALL;
}

bool findBestTarget(VisionDetection *bestTarget) {
    bool found = false;
    long bestScore = -999999;

    for (uint8_t i = 0; i < latestVision.count; i++) {
        VisionDetection det = latestVision.detections[i];
        if (!isTargetBall(det.class_id) || det.score < 35) {
            continue;
        }

        long priority = (long)targetPoints(det.class_id) * 1000L;
        priority += det.cy;
        priority += det.h / 2;
        priority -= abs((int)det.cx - 500);

        if (!found || priority > bestScore) {
            *bestTarget = det;
            bestScore = priority;
            found = true;
        }
    }

    return found;
}

bool findRedSafeZone(VisionDetection *safeZone) {
    bool found = false;
    long bestArea = 0;

    for (uint8_t i = 0; i < latestVision.count; i++) {
        VisionDetection det = latestVision.detections[i];
        if (det.class_id != CLASS_RED_SAFE_ZONE || det.score < 35) {
            continue;
        }

        long area = (long)det.w * det.h;
        if (!found || area > bestArea) {
            *safeZone = det;
            bestArea = area;
            found = true;
        }
    }

    return found;
}

bool visionFrameFresh(uint32_t now) {
    return latestVision.valid && (now - latestVision.received_ms <= VISION_TIMEOUT_MS);
}

void stopAutoForVisionLoss() {
    bool hadFreshFrame = latestVision.valid;
    handleMotor(0, 0, 0);
    latestVision.valid = false;
    if (autoState != SEARCH_BALL) {
        enterAutoState(SEARCH_BALL);
    }
    if (hadFreshFrame) {
        Serial.println("Vision timeout: motors stopped");
    }
}

void updateAutoControl(uint32_t now) {
    if (!visionFrameFresh(now)) {
        stopAutoForVisionLoss();
        return;
    }

    VisionDetection target;
    VisionDetection safeZone;

    switch (autoState) {
        case SEARCH_BALL:
            carryingTarget = false;
            servo.write(FRAME_UP_ANGLE);
            if (findBestTarget(&target)) {
                enterAutoState(APPROACH_BALL);
                handleMotor(0, 0, 0);
            } else {
                if (now - autoStateStartedMs > SEARCH_SWEEP_MS) {
                    searchDirection = -searchDirection;
                    autoStateStartedMs = now;
                }
                handleMotor(0, 0, SEARCH_RZ_SPEED * searchDirection);
            }
            break;

        case APPROACH_BALL:
            servo.write(FRAME_UP_ANGLE);
            if (!findBestTarget(&target)) {
                enterAutoState(SEARCH_BALL);
                handleMotor(0, 0, 0);
                break;
            }
            if (target.cy >= CAPTURE_CY_THRESHOLD || target.h >= CAPTURE_H_THRESHOLD) {
                enterAutoState(CAPTURE);
                handleMotor(0, 0, 0);
                break;
            }
            handleMotor(
                centerControl(target.cx, APPROACH_X_GAIN, 90),
                APPROACH_Y_SPEED,
                centerControl(target.cx, APPROACH_RZ_GAIN, 60)
            );
            break;

        case CAPTURE:
            handleMotor(0, 0, 0);
            servo.write(FRAME_DOWN_ANGLE);
            if (now - autoStateStartedMs >= CAPTURE_HOLD_MS) {
                carryingTarget = true;
                enterAutoState(FIND_SAFE_ZONE);
            }
            break;

        case FIND_SAFE_ZONE:
            servo.write(FRAME_DOWN_ANGLE);
            if (findRedSafeZone(&safeZone)) {
                enterAutoState(APPROACH_SAFE_ZONE);
                handleMotor(0, 0, 0);
            } else {
                if (now - autoStateStartedMs > SEARCH_SWEEP_MS) {
                    searchDirection = -searchDirection;
                    autoStateStartedMs = now;
                }
                handleMotor(0, 0, SEARCH_RZ_SPEED * searchDirection);
            }
            break;

        case APPROACH_SAFE_ZONE:
            servo.write(FRAME_DOWN_ANGLE);
            if (!findRedSafeZone(&safeZone)) {
                enterAutoState(FIND_SAFE_ZONE);
                handleMotor(0, 0, 0);
                break;
            }
            if (safeZone.cy >= SAFE_CY_THRESHOLD || safeZone.h >= SAFE_H_THRESHOLD) {
                enterAutoState(RELEASE);
                handleMotor(0, 0, 0);
                break;
            }
            handleMotor(
                centerControl(safeZone.cx, SAFE_APPROACH_X_GAIN, 80),
                SAFE_APPROACH_Y_SPEED,
                centerControl(safeZone.cx, SAFE_APPROACH_RZ_GAIN, 50)
            );
            break;

        case RELEASE:
            servo.write(FRAME_UP_ANGLE);
            if (now - autoStateStartedMs < RELEASE_HOLD_MS) {
                handleMotor(0, 0, 0);
            } else if (now - autoStateStartedMs < RELEASE_HOLD_MS + RELEASE_BACKUP_MS) {
                handleMotor(0, RELEASE_BACKUP_SPEED, 0);
            } else {
                carryingTarget = false;
                enterAutoState(SEARCH_BALL);
                handleMotor(0, 0, 0);
            }
            break;
    }
}

// 自动控制任务：读取K230检测帧并在ESP32侧执行搬运状态机。
void autoControlTask(void *pvParameters) {
    uint32_t lastControlMs = 0;

    while (true) {
        if (controlMode == AUTO) {
            while (VisionSerial.available()) {
                processVisionByte((uint8_t)VisionSerial.read());
            }

            uint32_t now = millis();
            if (now - lastControlMs >= AUTO_CONTROL_PERIOD_MS) {
                updateAutoControl(now);
                lastControlMs = now;
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
/*------------------K230视觉检测协议与自动状态机------------------*/

void motorUpdateTimerCallback(TimerHandle_t xTimer) {
    leftMotor.update();
    rightMotor.update();
}
void setup() {
    // 配置看门狗
    rtc_wdt_protect_off();
    rtc_wdt_enable();          //启用看门狗
    rtc_wdt_set_time(RTC_WDT_STAGE0, 2000); // 设置看门狗超时 2000ms.则reset重启

/*------------------夹取舵机初始化------------------*/
    ESP32PWM::allocateTimer(0);
    servo.setPeriodHertz(50);
    servo.attach(SERVO_PIN, minUs, maxUs);
    delay(50);
    servo.write(FRAME_UP_ANGLE);    //舵机初始位置，实车需要校准
/*------------------夹取舵机初始化------------------*/
    
    Serial.begin(115200);
    VisionSerial.begin(VISION_UART_BAUD, SERIAL_8N1, VISION_UART_RX_PIN, VISION_UART_TX_PIN);
    Serial.printf("Vision UART started: baud=%d rx=%d tx=%d\n",
                  VISION_UART_BAUD, VISION_UART_RX_PIN, VISION_UART_TX_PIN);

/*-------------------SBUS接收机--------------------*/ 
    Serial2.begin(100000, SERIAL_8N2, 16, 17);
    sbus_rx.Begin();
    delay(10);
/*-------------------SBUS接收机--------------------*/ 

/*-------------------霍尔编码器定义--------------------*/
    // 初始化左右驱动轮编码器
    setupEncoderPCNT(&pcnt_unit_1, MOTOR_A_EA, MOTOR_A_EB);  // 左轮编码器
    setupEncoderPCNT(&pcnt_unit_2, MOTOR_B_EA, MOTOR_B_EB);  // 右轮编码器

    // 创建定时器，每 50ms 调用一次 calculateSpeed 函数
    TimerHandle_t speedTimer = xTimerCreate(
        "SpeedTimer",               // 定时器名称
        pdMS_TO_TICKS(VERTREFRESH),          // 定时器周期（50ms）
        pdTRUE,                     // 自动重载
        (void*)0,                   // 定时器 ID
        calculateSpeed              // 回调函数
    );

    // 启动定时器
    if (speedTimer != NULL) {
        xTimerStart(speedTimer, 0);
    }
/*-------------------霍尔编码器定义--------------------*/

/*------------------设置motor速度指针-------------------*/ 
    leftMotor.setCurrentSpeed(&Velocity1);
    rightMotor.setCurrentSpeed(&Velocity2);
/*------------------设置motor速度指针-------------------*/ 

/*------------------创建手动控制定时器（80ms）------------------*/
    TimerHandle_t manualControlTimer = xTimerCreate(
        "manualControlTimer",
        pdMS_TO_TICKS(80),
        pdTRUE,
        (void*)4,
        manualControl_TimerCallback
    );
    // 启动定时器
    if (manualControlTimer != NULL) {
        xTimerStart(manualControlTimer, 0);
    }
/*------------------创建手动控制定时器（80ms）------------------*/

/*------------------创建自动控制任务------------------*/
    xTaskCreate(
        autoControlTask,
        "autoControlTask",
        4096,
        (void*)NULL,
        2,
        NULL
    );
/*------------------创建自动控制任务------------------*/
   
    handleMotor(0,0,0);

/*----------------创建电机速度刷新定时器（50ms）----------------*/
    TimerHandle_t motorUpdateTimer = xTimerCreate(
        "motorUpdateTimer",
        pdMS_TO_TICKS(50),
        pdTRUE,
        NULL,
        motorUpdateTimerCallback
    );
    if (motorUpdateTimer != NULL) {
        xTimerStart(motorUpdateTimer, 0);
    }
/*----------------创建电机速度刷新定时器（50ms）----------------*/
}

void loop() {
    rtc_wdt_feed();
    delay(500);   
}
