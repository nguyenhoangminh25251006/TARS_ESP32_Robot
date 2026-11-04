#include "movements.h"

#include <algorithm>
#include <cstring>

#include "oscillator.h"

Otto::Otto() {
    is_otto_resting_ = false;
    for (int i = 0; i < SERVO_COUNT; i++) {
        servo_pins_[i] = -1;
        servo_trim_[i] = 0;
    }
}

Otto::~Otto() {
    DetachServos();
}

unsigned long IRAM_ATTR millis() {
    return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

void Otto::Init(int right_pitch, int right_roll, int left_pitch, int left_roll, int body,
                int head) {
    servo_pins_[RIGHT_PITCH] = right_pitch;
    servo_pins_[RIGHT_ROLL] = right_roll;
    servo_pins_[LEFT_PITCH] = left_pitch;
    servo_pins_[LEFT_ROLL] = left_roll;
    servo_pins_[BODY] = body;
    servo_pins_[HEAD] = head;

    AttachServos();
    is_otto_resting_ = false;
}

///////////////////////////////////////////////////////////////////
//-- ATTACH & DETACH FUNCTIONS ----------------------------------//
///////////////////////////////////////////////////////////////////
void Otto::AttachServos() {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].Attach(servo_pins_[i]);
        }
    }
}

void Otto::DetachServos() {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].Detach();
        }
    }
}

///////////////////////////////////////////////////////////////////
//-- OSCILLATORS TRIMS ------------------------------------------//
///////////////////////////////////////////////////////////////////
void Otto::SetTrims(int right_pitch, int right_roll, int left_pitch, int left_roll, int body,
                    int head) {
    servo_trim_[RIGHT_PITCH] = right_pitch;
    servo_trim_[RIGHT_ROLL] = right_roll;
    servo_trim_[LEFT_PITCH] = left_pitch;
    servo_trim_[LEFT_ROLL] = left_roll;
    servo_trim_[BODY] = body;
    servo_trim_[HEAD] = head;

    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].SetTrim(servo_trim_[i]);
        }
    }
}

///////////////////////////////////////////////////////////////////
//-- BASIC MOTION FUNCTIONS -------------------------------------//
///////////////////////////////////////////////////////////////////
void Otto::MoveServos(int time, int servo_target[]) {
    if (GetRestState() == true) {
        SetRestState(false);
    }

    final_time_ = millis() + time;
    if (time > 10) {
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (servo_pins_[i] != -1) {
                increment_[i] = (servo_target[i] - servo_[i].GetPosition()) / (time / 10.0);
            }
        }

        for (int iteration = 1; millis() < final_time_; iteration++) {
            partial_time_ = millis() + 10;
            for (int i = 0; i < SERVO_COUNT; i++) {
                if (servo_pins_[i] != -1) {
                    servo_[i].SetPosition(servo_[i].GetPosition() + increment_[i]);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    } else {
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (servo_pins_[i] != -1) {
                servo_[i].SetPosition(servo_target[i]);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(time));
    }

    // final adjustment to the target.
    bool f = true;
    int adjustment_count = 0;
    while (f && adjustment_count < 10) {
        f = false;
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (servo_pins_[i] != -1 && servo_target[i] != servo_[i].GetPosition()) {
                f = true;
                break;
            }
        }
        if (f) {
            for (int i = 0; i < SERVO_COUNT; i++) {
                if (servo_pins_[i] != -1) {
                    servo_[i].SetPosition(servo_target[i]);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            adjustment_count++;
        }
    };
}

void Otto::MoveSingle(int position, int servo_number) {
    if (position > 180)
        position = 90;
    if (position < 0)
        position = 90;

    if (GetRestState() == true) {
        SetRestState(false);
    }

    if (servo_number >= 0 && servo_number < SERVO_COUNT && servo_pins_[servo_number] != -1) {
        servo_[servo_number].SetPosition(position);
    }
}

void Otto::OscillateServos(int amplitude[SERVO_COUNT], int offset[SERVO_COUNT], int period,
                           double phase_diff[SERVO_COUNT], float cycle = 1) {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].SetO(offset[i]);
            servo_[i].SetA(amplitude[i]);
            servo_[i].SetT(period);
            servo_[i].SetPh(phase_diff[i]);
        }
    }

    double ref = millis();
    double end_time = period * cycle + ref;

    while (millis() < end_time) {
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (servo_pins_[i] != -1) {
                servo_[i].Refresh();
            }
        }
        vTaskDelay(5);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
}

void Otto::Execute(int amplitude[SERVO_COUNT], int offset[SERVO_COUNT], int period,
                   double phase_diff[SERVO_COUNT], float steps = 1.0) {
    if (GetRestState() == true) {
        SetRestState(false);
    }

    int cycles = (int)steps;

    //-- Execute complete cycles
    if (cycles >= 1)
        for (int i = 0; i < cycles; i++)
            OscillateServos(amplitude, offset, period, phase_diff);

    //-- Execute the final not complete cycle
    OscillateServos(amplitude, offset, period, phase_diff, (float)steps - cycles);
    vTaskDelay(pdMS_TO_TICKS(10));
}

///////////////////////////////////////////////////////////////////
//-- HOME = Otto at rest position -------------------------------//
///////////////////////////////////////////////////////////////////
void Otto::Home(bool hands_down) {
    if (is_otto_resting_ == false) {  // Go to rest position only if necessary
        MoveServos(1000, servo_initial_);
        is_otto_resting_ = true;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
}

bool Otto::GetRestState() {
    return is_otto_resting_;
}

void Otto::SetRestState(bool state) {
    is_otto_resting_ = state;
}

///////////////////////////////////////////////////////////////////
//-- PREDETERMINED MOTION SEQUENCES -----------------------------//
///////////////////////////////////////////////////////////////////

//---------------------------------------------------------
//-- ç»Ÿä¸€æ‰‹éƒ¨åŠ¨ä½œå‡½æ•°
//--  Parameters:
//--    action: åŠ¨ä½œç±»åž‹ 1=ä¸¾å·¦æ‰‹, 2=ä¸¾å³æ‰‹, 3=ä¸¾åŒæ‰‹, 4=æ”¾å·¦æ‰‹, 5=æ”¾å³æ‰‹, 6=æ”¾åŒæ‰‹,
//--            7=æŒ¥å·¦æ‰‹, 8=æŒ¥å³æ‰‹, 9=æŒ¥åŒæ‰‹, 10=æ‹æ‰“å·¦æ‰‹, 11=æ‹æ‰“å³æ‰‹, 12=æ‹æ‰“åŒæ‰‹
//--    times: é‡å¤æ¬¡æ•°
//--    amount: åŠ¨ä½œå¹…åº¦ (10-50)
//--    period: åŠ¨ä½œæ—¶é—´
//---------------------------------------------------------
void Otto::HandAction(int action, int times, int amount, int period) {
    // é™åˆ¶å‚æ•°èŒƒå›´
    times = 2 * std::max(3, std::min(100, times));
    amount = std::max(10, std::min(50, amount));
    period = std::max(100, std::min(1000, period));

    int current_positions[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) {
        current_positions[i] = (servo_pins_[i] != -1) ? servo_[i].GetPosition() : servo_initial_[i];
    }

    switch (action) {
        case 1:  // ä¸¾å·¦æ‰‹
            current_positions[LEFT_PITCH] = 180;
            MoveServos(period, current_positions);
            break;

        case 2:  // ä¸¾å³æ‰‹
            current_positions[RIGHT_PITCH] = 0;
            MoveServos(period, current_positions);
            break;

        case 3:  // ä¸¾åŒæ‰‹
            current_positions[LEFT_PITCH] = 180;
            current_positions[RIGHT_PITCH] = 0;
            MoveServos(period, current_positions);
            break;

        case 4:  // æ”¾å·¦æ‰‹
        case 5:  // æ”¾å³æ‰‹
        case 6:  // æ”¾åŒæ‰‹
            // å›žåˆ°åˆå§‹ä½ç½®
            memcpy(current_positions, servo_initial_, sizeof(current_positions));
            MoveServos(period, current_positions);
            break;

        case 7:  // æŒ¥å·¦æ‰‹
            current_positions[LEFT_PITCH] = 150;
            MoveServos(period, current_positions);
            for (int i = 0; i < times; i++) {
                current_positions[LEFT_PITCH] = 150 + (i % 2 == 0 ? -30 : 30);
                MoveServos(period / 10, current_positions);
                vTaskDelay(pdMS_TO_TICKS(period / 10));
            }
            memcpy(current_positions, servo_initial_, sizeof(current_positions));
            MoveServos(period, current_positions);
            break;

        case 8:  // æŒ¥å³æ‰‹
            current_positions[RIGHT_PITCH] = 30;
            MoveServos(period, current_positions);
            for (int i = 0; i < times; i++) {
                current_positions[RIGHT_PITCH] = 30 + (i % 2 == 0 ? 30 : -30);
                MoveServos(period / 10, current_positions);
                vTaskDelay(pdMS_TO_TICKS(period / 10));
            }
            memcpy(current_positions, servo_initial_, sizeof(current_positions));
            MoveServos(period, current_positions);
            break;

        case 9:  // æŒ¥åŒæ‰‹
            current_positions[LEFT_PITCH] = 150;
            current_positions[RIGHT_PITCH] = 30;
            MoveServos(period, current_positions);
            for (int i = 0; i < times; i++) {
                current_positions[LEFT_PITCH] = 150 + (i % 2 == 0 ? -30 : 30);
                current_positions[RIGHT_PITCH] = 30 + (i % 2 == 0 ? 30 : -30);
                MoveServos(period / 10, current_positions);
                vTaskDelay(pdMS_TO_TICKS(period / 10));
            }
            memcpy(current_positions, servo_initial_, sizeof(current_positions));
            MoveServos(period, current_positions);
            break;

        case 10:  // æ‹æ‰“å·¦æ‰‹
            current_positions[LEFT_ROLL] = 20;
            MoveServos(period, current_positions);
            for (int i = 0; i < times; i++) {
                current_positions[LEFT_ROLL] = 20 - amount;
                MoveServos(period / 10, current_positions);
                current_positions[LEFT_ROLL] = 20 + amount;
                MoveServos(period / 10, current_positions);
            }
            current_positions[LEFT_ROLL] = 0;
            MoveServos(period, current_positions);
            break;

        case 11:  // æ‹æ‰“å³æ‰‹
            current_positions[RIGHT_ROLL] = 160;
            MoveServos(period, current_positions);
            for (int i = 0; i < times; i++) {
                current_positions[RIGHT_ROLL] = 160 + amount;
                MoveServos(period / 10, current_positions);
                current_positions[RIGHT_ROLL] = 160 - amount;
                MoveServos(period / 10, current_positions);
            }
            current_positions[RIGHT_ROLL] = 180;
            MoveServos(period, current_positions);
            break;

        case 12:  // æ‹æ‰“åŒæ‰‹
            current_positions[LEFT_ROLL] = 20;
            current_positions[RIGHT_ROLL] = 160;
            MoveServos(period, current_positions);
            for (int i = 0; i < times; i++) {
                current_positions[LEFT_ROLL] = 20 - amount;
                current_positions[RIGHT_ROLL] = 160 + amount;
                MoveServos(period / 10, current_positions);
                current_positions[LEFT_ROLL] = 20 + amount;
                current_positions[RIGHT_ROLL] = 160 - amount;
                MoveServos(period / 10, current_positions);
            }
            current_positions[LEFT_ROLL] = 0;
            current_positions[RIGHT_ROLL] = 180;
            MoveServos(period, current_positions);
            break;
    }
}

//---------------------------------------------------------
//-- ç»Ÿä¸€èº«ä½“åŠ¨ä½œå‡½æ•°
//--  Parameters:
//--    action: åŠ¨ä½œç±»åž‹ 1=å·¦è½¬, 2=å³è½¬ï¼Œ3=å›žä¸­å¿ƒ
//--    times: è½¬åŠ¨æ¬¡æ•°
//--    amount: æ—‹è½¬è§’åº¦ (0-90åº¦ï¼Œä»¥90åº¦ä¸ºä¸­å¿ƒå·¦å³æ—‹è½¬)
//--    period: åŠ¨ä½œæ—¶é—´
//---------------------------------------------------------
void Otto::BodyAction(int action, int times, int amount, int period) {
    // é™åˆ¶å‚æ•°èŒƒå›´
    times = std::max(1, std::min(10, times));
    amount = std::max(0, std::min(90, amount));
    period = std::max(500, std::min(3000, period));

    int current_positions[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            current_positions[i] = servo_[i].GetPosition();
        } else {
            current_positions[i] = servo_initial_[i];
        }
    }

    int body_center = servo_initial_[BODY];
    int target_angle = body_center;

    switch (action) {
        case 1:  // å·¦è½¬
            target_angle = body_center + amount;
            target_angle = std::min(180, target_angle);
            break;
        case 2:  // å³è½¬
            target_angle = body_center - amount;
            target_angle = std::max(0, target_angle);
            break;
        case 3:  // å›žä¸­å¿ƒ
            target_angle = body_center;
            break;
        default:
            return;  // æ— æ•ˆåŠ¨ä½œ
    }

    current_positions[BODY] = target_angle;
    MoveServos(period, current_positions);
    vTaskDelay(pdMS_TO_TICKS(100));
}

//---------------------------------------------------------
//-- ç»Ÿä¸€å¤´éƒ¨åŠ¨ä½œå‡½æ•°
//--  Parameters:
//--    action: åŠ¨ä½œç±»åž‹ 1=æŠ¬å¤´, 2=ä½Žå¤´, 3=ç‚¹å¤´, 4=å›žä¸­å¿ƒ, 5=è¿žç»­ç‚¹å¤´
//--    times: é‡å¤æ¬¡æ•° (ä»…å¯¹è¿žç»­ç‚¹å¤´æœ‰æ•ˆ)
//--    amount: è§’åº¦åç§» (1-15åº¦èŒƒå›´å†…)
//--    period: åŠ¨ä½œæ—¶é—´
//---------------------------------------------------------
void Otto::HeadAction(int action, int times, int amount, int period) {
    // é™åˆ¶å‚æ•°èŒƒå›´
    times = std::max(1, std::min(10, times));
    amount = std::max(1, std::min(15, abs(amount)));
    period = std::max(300, std::min(3000, period));

    int current_positions[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            current_positions[i] = servo_[i].GetPosition();
        } else {
            current_positions[i] = servo_initial_[i];
        }
    }

    int head_center = 90;  // å¤´éƒ¨ä¸­å¿ƒä½ç½®

    switch (action) {
        case 1:                                              // æŠ¬å¤´
            current_positions[HEAD] = head_center + amount;  // æŠ¬å¤´æ˜¯å¢žåŠ è§’åº¦
            MoveServos(period, current_positions);
            break;

        case 2:                                              // ä½Žå¤´
            current_positions[HEAD] = head_center - amount;  // ä½Žå¤´æ˜¯å‡å°‘è§’åº¦
            MoveServos(period, current_positions);
            break;

        case 3:  // ç‚¹å¤´ (ä¸Šä¸‹è¿åŠ¨)
            // å…ˆæŠ¬å¤´
            current_positions[HEAD] = head_center + amount;
            MoveServos(period / 3, current_positions);
            vTaskDelay(pdMS_TO_TICKS(period / 6));

            // å†ä½Žå¤´
            current_positions[HEAD] = head_center - amount;
            MoveServos(period / 3, current_positions);
            vTaskDelay(pdMS_TO_TICKS(period / 6));

            // å›žåˆ°ä¸­å¿ƒ
            current_positions[HEAD] = head_center;
            MoveServos(period / 3, current_positions);
            break;

        case 4:  // å›žåˆ°ä¸­å¿ƒä½ç½®
            current_positions[HEAD] = head_center;
            MoveServos(period, current_positions);
            break;

        case 5:  // è¿žç»­ç‚¹å¤´
            for (int i = 0; i < times; i++) {
                // æŠ¬å¤´
                current_positions[HEAD] = head_center + amount;
                MoveServos(period / 2, current_positions);

                // ä½Žå¤´
                current_positions[HEAD] = head_center - amount;
                MoveServos(period / 2, current_positions);

                vTaskDelay(pdMS_TO_TICKS(50));  // çŸ­æš‚åœé¡¿
            }

            // å›žåˆ°ä¸­å¿ƒ
            current_positions[HEAD] = head_center;
            MoveServos(period / 2, current_positions);
            break;

        default:
            // æ— æ•ˆåŠ¨ä½œï¼Œå›žåˆ°ä¸­å¿ƒ
            current_positions[HEAD] = head_center;
            MoveServos(period, current_positions);
            break;
    }
}
