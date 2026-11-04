/*
    Ottoæœºå™¨äººæŽ§åˆ¶å™¨ - MCPåè®®ç‰ˆæœ¬
*/

#include <cJSON.h>
#include <esp_log.h>

#include <cstdlib> 
#include <cstring>

#include "application.h"
#include "board.h"
#include "config.h"
#include "mcp_server.h"
#include "otto_movements.h"
#include "power_manager.h"
#include "sdkconfig.h"
#include "settings.h"
#include <wifi_manager.h>

#define TAG "OttoController"

class OttoController {
private:
    Otto otto_;
    TaskHandle_t action_task_handle_ = nullptr;
    QueueHandle_t action_queue_;
    bool has_hands_ = false;
    bool is_action_in_progress_ = false;

    struct OttoActionParams {
        int action_type;
        int steps;
        int speed;
        int direction;
        int amount;
        char servo_sequence_json[512];  // ç”¨äºŽå­˜å‚¨èˆµæœºåºåˆ—çš„JSONå­—ç¬¦ä¸²
    };

    enum ActionType {
        ACTION_WALK = 1,
        ACTION_TURN = 2,
        ACTION_JUMP = 3,
        ACTION_SWING = 4,
        ACTION_MOONWALK = 5,
        ACTION_BEND = 6,
        ACTION_SHAKE_LEG = 7,
        ACTION_SIT = 25,  // åä¸‹
        ACTION_RADIO_CALISTHENICS = 26,  // å¹¿æ’­ä½“æ“
        ACTION_MAGIC_CIRCLE = 27,  // çˆ±çš„é­”åŠ›è½¬åœˆåœˆ
        ACTION_UPDOWN = 8,
        ACTION_TIPTOE_SWING = 9,
        ACTION_JITTER = 10,
        ACTION_ASCENDING_TURN = 11,
        ACTION_CRUSAITO = 12,
        ACTION_FLAPPING = 13,
        ACTION_HANDS_UP = 14,
        ACTION_HANDS_DOWN = 15,
        ACTION_HAND_WAVE = 16,
        ACTION_WINDMILL = 20,  // å¤§é£Žè½¦
        ACTION_TAKEOFF = 21,   // èµ·é£ž
        ACTION_FITNESS = 22,   // å¥èº«
        ACTION_GREETING = 23,  // æ‰“æ‹›å‘¼
        ACTION_SHY = 24,        // å®³ç¾ž
        ACTION_SHOWCASE = 28,   // å±•ç¤ºåŠ¨ä½œ
        ACTION_HOME = 17,
        ACTION_SERVO_SEQUENCE = 18,  // èˆµæœºåºåˆ—ï¼ˆè‡ªç¼–ç¨‹ï¼‰
        ACTION_WHIRLWIND_LEG = 19    // æ—‹é£Žè…¿
    };

    static void ActionTask(void* arg) {
        OttoController* controller = static_cast<OttoController*>(arg);
        OttoActionParams params;
        controller->otto_.AttachServos();

        while (true) {
            if (xQueueReceive(controller->action_queue_, &params, pdMS_TO_TICKS(1000)) == pdTRUE) {
                ESP_LOGI(TAG, "æ‰§è¡ŒåŠ¨ä½œ: %d", params.action_type);
                PowerManager::PauseBatteryUpdate();  // åŠ¨ä½œå¼€å§‹æ—¶æš‚åœç”µé‡æ›´æ–°
                controller->is_action_in_progress_ = true;
                if (params.action_type == ACTION_SERVO_SEQUENCE) {
                    // æ‰§è¡Œèˆµæœºåºåˆ—ï¼ˆè‡ªç¼–ç¨‹ï¼‰- ä»…æ”¯æŒçŸ­é”®åæ ¼å¼
                    cJSON* json = cJSON_Parse(params.servo_sequence_json);
                    if (json != nullptr) {
                        ESP_LOGD(TAG, "JSONè§£æžæˆåŠŸï¼Œé•¿åº¦=%d", strlen(params.servo_sequence_json));
                        // ä½¿ç”¨çŸ­é”®å "a" è¡¨ç¤ºåŠ¨ä½œæ•°ç»„
                        cJSON* actions = cJSON_GetObjectItem(json, "a");
                        if (cJSON_IsArray(actions)) {
                            int array_size = cJSON_GetArraySize(actions);
                            ESP_LOGI(TAG, "æ‰§è¡Œèˆµæœºåºåˆ—ï¼Œå…±%dä¸ªåŠ¨ä½œ", array_size);
                            
                            // èŽ·å–åºåˆ—æ‰§è¡Œå®ŒæˆåŽçš„å»¶è¿Ÿï¼ˆçŸ­é”®å "d"ï¼Œé¡¶å±‚å‚æ•°ï¼‰
                            int sequence_delay = 0;
                            cJSON* delay_item = cJSON_GetObjectItem(json, "d");
                            if (cJSON_IsNumber(delay_item)) {
                                sequence_delay = delay_item->valueint;
                                if (sequence_delay < 0) sequence_delay = 0;
                            }
                            
                            // åˆå§‹åŒ–å½“å‰èˆµæœºä½ç½®ï¼ˆç”¨äºŽä¿æŒæœªæŒ‡å®šçš„èˆµæœºä½ç½®ï¼‰
                            int current_positions[SERVO_COUNT];
                            for (int j = 0; j < SERVO_COUNT; j++) {
                                current_positions[j] = 90;  // é»˜è®¤ä¸­é—´ä½ç½®
                            }
                            // æ‰‹éƒ¨èˆµæœºé»˜è®¤ä½ç½®
                            current_positions[LEFT_HAND] = 45;
                            current_positions[RIGHT_HAND] = 180 - 45;
                            
                            for (int i = 0; i < array_size; i++) {
                                cJSON* action_item = cJSON_GetArrayItem(actions, i);
                                if (cJSON_IsObject(action_item)) {
                                    // æ£€æŸ¥æ˜¯å¦ä¸ºæŒ¯è¡å™¨æ¨¡å¼ï¼ˆçŸ­é”®å "osc"ï¼‰
                                    cJSON* osc_item = cJSON_GetObjectItem(action_item, "osc");
                                    if (cJSON_IsObject(osc_item)) {
                                        // æŒ¯è¡å™¨æ¨¡å¼ - ä½¿ç”¨Execute2ï¼Œä»¥ç»å¯¹è§’åº¦ä¸ºä¸­å¿ƒæŒ¯è¡
                                        int amplitude[SERVO_COUNT] = {0};
                                        int center_angle[SERVO_COUNT] = {0};
                                        double phase_diff[SERVO_COUNT] = {0};
                                        int period = 300;  // é»˜è®¤å‘¨æœŸ300æ¯«ç§’
                                        float steps = 8.0;  // é»˜è®¤æ­¥æ•°8.0
                                        
                                        const char* servo_names[] = {"ll", "rl", "lf", "rf", "lh", "rh"};
                                        
                                        // è¯»å–æŒ¯å¹…ï¼ˆçŸ­é”®å "a"ï¼‰ï¼Œé»˜è®¤0åº¦
                                        for (int j = 0; j < SERVO_COUNT; j++) {
                                            amplitude[j] = 0;  // é»˜è®¤æŒ¯å¹…0åº¦
                                        }
                                        cJSON* amp_item = cJSON_GetObjectItem(osc_item, "a");
                                        if (cJSON_IsObject(amp_item)) {
                                            for (int j = 0; j < SERVO_COUNT; j++) {
                                                cJSON* amp_value = cJSON_GetObjectItem(amp_item, servo_names[j]);
                                                if (cJSON_IsNumber(amp_value)) {
                                                    int amp = amp_value->valueint;
                                                    if (amp >= 10 && amp <= 90) {
                                                        amplitude[j] = amp;
                                                    }
                                                }
                                            }
                                        }
                                        
                                        // è¯»å–ä¸­å¿ƒè§’åº¦ï¼ˆçŸ­é”®å "o"ï¼‰ï¼Œé»˜è®¤90åº¦ï¼ˆç»å¯¹è§’åº¦0-180åº¦ï¼‰
                                        for (int j = 0; j < SERVO_COUNT; j++) {
                                            center_angle[j] = 90;  // é»˜è®¤ä¸­å¿ƒè§’åº¦90åº¦ï¼ˆä¸­é—´ä½ç½®ï¼‰
                                        }
                                        cJSON* center_item = cJSON_GetObjectItem(osc_item, "o");
                                        if (cJSON_IsObject(center_item)) {
                                            for (int j = 0; j < SERVO_COUNT; j++) {
                                                cJSON* center_value = cJSON_GetObjectItem(center_item, servo_names[j]);
                                                if (cJSON_IsNumber(center_value)) {
                                                    int center = center_value->valueint;
                                                    if (center >= 0 && center <= 180) {
                                                        center_angle[j] = center;
                                                    }
                                                }
                                            }
                                        }
                                        
                                        // å®‰å…¨æ£€æŸ¥ï¼šé˜²æ­¢å·¦å³è…¿è„šåŒæ—¶åšå¤§å¹…åº¦æŒ¯è¡ï¼ˆæŒ¯å¹…æ£€æŸ¥ï¼‰
                                        const int LARGE_AMPLITUDE_THRESHOLD = 40;  // å¤§å¹…åº¦æŒ¯å¹…é˜ˆå€¼ï¼š40åº¦
                                        bool left_leg_large = amplitude[LEFT_LEG] >= LARGE_AMPLITUDE_THRESHOLD;
                                        bool right_leg_large = amplitude[RIGHT_LEG] >= LARGE_AMPLITUDE_THRESHOLD;
                                        bool left_foot_large = amplitude[LEFT_FOOT] >= LARGE_AMPLITUDE_THRESHOLD;
                                        bool right_foot_large = amplitude[RIGHT_FOOT] >= LARGE_AMPLITUDE_THRESHOLD;
                                        
                                        if (left_leg_large && right_leg_large) {
                                            ESP_LOGW(TAG, "æ£€æµ‹åˆ°å·¦å³è…¿åŒæ—¶å¤§å¹…åº¦æŒ¯è¡ï¼Œé™åˆ¶å³è…¿æŒ¯å¹…");
                                            amplitude[RIGHT_LEG] = 0;  // ç¦æ­¢å³è…¿æŒ¯è¡
                                        }
                                        if (left_foot_large && right_foot_large) {
                                            ESP_LOGW(TAG, "æ£€æµ‹åˆ°å·¦å³è„šåŒæ—¶å¤§å¹…åº¦æŒ¯è¡ï¼Œé™åˆ¶å³è„šæŒ¯å¹…");
                                            amplitude[RIGHT_FOOT] = 0;  // ç¦æ­¢å³è„šæŒ¯è¡
                                        }
                                        
                                        // è¯»å–ç›¸ä½å·®ï¼ˆçŸ­é”®å "ph"ï¼Œå•ä½ä¸ºåº¦ï¼Œè½¬æ¢ä¸ºå¼§åº¦ï¼‰
                                        cJSON* phase_item = cJSON_GetObjectItem(osc_item, "ph");
                                        if (cJSON_IsObject(phase_item)) {
                                            for (int j = 0; j < SERVO_COUNT; j++) {
                                                cJSON* phase_value = cJSON_GetObjectItem(phase_item, servo_names[j]);
                                                if (cJSON_IsNumber(phase_value)) {
                                                    // å°†åº¦æ•°è½¬æ¢ä¸ºå¼§åº¦
                                                    phase_diff[j] = phase_value->valuedouble * 3.141592653589793 / 180.0;
                                                }
                                            }
                                        }
                                        
                                        // è¯»å–å‘¨æœŸï¼ˆçŸ­é”®å "p"ï¼‰ï¼ŒèŒƒå›´100-3000æ¯«ç§’
                                        cJSON* period_item = cJSON_GetObjectItem(osc_item, "p");
                                        if (cJSON_IsNumber(period_item)) {
                                            period = period_item->valueint;
                                            if (period < 100) period = 100;
                                            if (period > 3000) period = 3000;  // ä¸Žæè¿°ä¸€è‡´ï¼Œé™åˆ¶3000æ¯«ç§’
                                        }
                                        
                                        // è¯»å–å‘¨æœŸæ•°ï¼ˆçŸ­é”®å "c"ï¼‰ï¼ŒèŒƒå›´0.1-20.0
                                        cJSON* steps_item = cJSON_GetObjectItem(osc_item, "c");
                                        if (cJSON_IsNumber(steps_item)) {
                                            steps = (float)steps_item->valuedouble;
                                            if (steps < 0.1) steps = 0.1;
                                            if (steps > 20.0) steps = 20.0;  // ä¸Žæè¿°ä¸€è‡´ï¼Œé™åˆ¶20.0
                                        }
                                        
                                        // æ‰§è¡ŒæŒ¯è¡ - ä½¿ç”¨Execute2ï¼Œä»¥ç»å¯¹è§’åº¦ä¸ºä¸­å¿ƒ
                                        ESP_LOGI(TAG, "æ‰§è¡ŒæŒ¯è¡åŠ¨ä½œ%d: period=%d, steps=%.1f", i, period, steps);
                                        controller->otto_.Execute2(amplitude, center_angle, period, phase_diff, steps);
                                        
                                        // æŒ¯è¡åŽæ›´æ–°ä½ç½®ï¼ˆä½¿ç”¨center_angleä½œä¸ºæœ€ç»ˆä½ç½®ï¼‰
                                        for (int j = 0; j < SERVO_COUNT; j++) {
                                            current_positions[j] = center_angle[j];
                                        }
                                    } else {
                                        // æ™®é€šç§»åŠ¨æ¨¡å¼
                                        // ä»Žå½“å‰ä½ç½®æ•°ç»„å¤åˆ¶ï¼Œä¿æŒæœªæŒ‡å®šçš„èˆµæœºä½ç½®
                                        int servo_target[SERVO_COUNT];
                                        for (int j = 0; j < SERVO_COUNT; j++) {
                                            servo_target[j] = current_positions[j];
                                        }
                                        
                                        // ä»ŽJSONä¸­è¯»å–èˆµæœºä½ç½®ï¼ˆçŸ­é”®å "s"ï¼‰
                                        cJSON* servos_item = cJSON_GetObjectItem(action_item, "s");
                                        if (cJSON_IsObject(servos_item)) {
                                            // çŸ­é”®åï¼šll/rl/lf/rf/lh/rh
                                            const char* servo_names[] = {"ll", "rl", "lf", "rf", "lh", "rh"};
                                            
                                            for (int j = 0; j < SERVO_COUNT; j++) {
                                                cJSON* servo_value = cJSON_GetObjectItem(servos_item, servo_names[j]);
                                                if (cJSON_IsNumber(servo_value)) {
                                                    int position = servo_value->valueint;
                                                    // é™åˆ¶ä½ç½®èŒƒå›´åœ¨0-180åº¦
                                                    if (position >= 0 && position <= 180) {
                                                        servo_target[j] = position;
                                                    }
                                                }
                                            }
                                        }
                                                                                                                    
                                        // èŽ·å–ç§»åŠ¨é€Ÿåº¦ï¼ˆçŸ­é”®å "v"ï¼Œé»˜è®¤1000æ¯«ç§’ï¼‰
                                        int speed = 1000;
                                        cJSON* speed_item = cJSON_GetObjectItem(action_item, "v");
                                        if (cJSON_IsNumber(speed_item)) {
                                            speed = speed_item->valueint;
                                            if (speed < 100) speed = 100;  // æœ€å°100æ¯«ç§’
                                            if (speed > 3000) speed = 3000;  // æœ€å¤§3000æ¯«ç§’
                                        }
                                        
                                        // æ‰§è¡Œèˆµæœºç§»åŠ¨
                                        ESP_LOGI(TAG, "æ‰§è¡ŒåŠ¨ä½œ%d: ll=%d, rl=%d, lf=%d, rf=%d, v=%d",
                                                 i, servo_target[LEFT_LEG], servo_target[RIGHT_LEG],
                                                 servo_target[LEFT_FOOT], servo_target[RIGHT_FOOT], speed);
                                        controller->otto_.MoveServos(speed, servo_target);
                                        
                                        // æ›´æ–°å½“å‰ä½ç½®æ•°ç»„ï¼Œç”¨äºŽä¸‹ä¸€ä¸ªåŠ¨ä½œ
                                        for (int j = 0; j < SERVO_COUNT; j++) {
                                            current_positions[j] = servo_target[j];
                                        }
                                    }
                                    
                                    // èŽ·å–åŠ¨ä½œåŽçš„å»¶è¿Ÿæ—¶é—´ï¼ˆçŸ­é”®å "d"ï¼‰
                                    int delay_after = 0;
                                    cJSON* delay_item = cJSON_GetObjectItem(action_item, "d");
                                    if (cJSON_IsNumber(delay_item)) {
                                        delay_after = delay_item->valueint;
                                        if (delay_after < 0) delay_after = 0;
                                    }
                                    
                                    // åŠ¨ä½œåŽçš„å»¶è¿Ÿï¼ˆæœ€åŽä¸€ä¸ªåŠ¨ä½œåŽä¸å»¶è¿Ÿï¼‰
                                    if (delay_after > 0 && i < array_size - 1) {
                                        ESP_LOGI(TAG, "åŠ¨ä½œ%dæ‰§è¡Œå®Œæˆï¼Œå»¶è¿Ÿ%dæ¯«ç§’", i, delay_after);
                                        vTaskDelay(pdMS_TO_TICKS(delay_after));
                                    }
                                }
                            }
                            
                            // åºåˆ—æ‰§è¡Œå®ŒæˆåŽçš„å»¶è¿Ÿï¼ˆç”¨äºŽåºåˆ—ä¹‹é—´çš„åœé¡¿ï¼‰
                            if (sequence_delay > 0) {
                                // æ£€æŸ¥é˜Ÿåˆ—ä¸­æ˜¯å¦è¿˜æœ‰å¾…æ‰§è¡Œçš„åºåˆ—
                                UBaseType_t queue_count = uxQueueMessagesWaiting(controller->action_queue_);
                                if (queue_count > 0) {
                                    ESP_LOGI(TAG, "åºåˆ—æ‰§è¡Œå®Œæˆï¼Œå»¶è¿Ÿ%dæ¯«ç§’åŽæ‰§è¡Œä¸‹ä¸€ä¸ªåºåˆ—ï¼ˆé˜Ÿåˆ—ä¸­è¿˜æœ‰%dä¸ªåºåˆ—ï¼‰", 
                                             sequence_delay, queue_count);
                                    vTaskDelay(pdMS_TO_TICKS(sequence_delay));
                                }
                            }
                            // é‡Šæ”¾JSONå†…å­˜
                            cJSON_Delete(json);
                        } else {
                            ESP_LOGE(TAG, "èˆµæœºåºåˆ—æ ¼å¼é”™è¯¯: 'a'ä¸æ˜¯æ•°ç»„");
                            cJSON_Delete(json);
                        }
                    } else {
                        // èŽ·å–cJSONçš„é”™è¯¯ä¿¡æ¯
                        const char* error_ptr = cJSON_GetErrorPtr();
                        int json_len = strlen(params.servo_sequence_json);
                        ESP_LOGE(TAG, "è§£æžèˆµæœºåºåˆ—JSONå¤±è´¥ï¼Œé•¿åº¦=%dï¼Œé”™è¯¯ä½ç½®: %s", json_len, 
                                 error_ptr ? error_ptr : "æœªçŸ¥");
                        ESP_LOGE(TAG, "JSONå†…å®¹: %s", params.servo_sequence_json);
                    }
                } else {
                    // æ‰§è¡Œé¢„å®šä¹‰åŠ¨ä½œ
                    switch (params.action_type) {
                        case ACTION_WALK:
                            controller->otto_.Walk(params.steps, params.speed, params.direction,
                                                   params.amount);
                            break;
                        case ACTION_TURN:
                            controller->otto_.Turn(params.steps, params.speed, params.direction,
                                                   params.amount);
                            break;
                        case ACTION_JUMP:
                            controller->otto_.Jump(params.steps, params.speed);
                            break;
                        case ACTION_SWING:
                            controller->otto_.Swing(params.steps, params.speed, params.amount);
                            break;
                        case ACTION_MOONWALK:
                            controller->otto_.Moonwalker(params.steps, params.speed, params.amount,
                                                         params.direction);
                            break;
                        case ACTION_BEND:
                            controller->otto_.Bend(params.steps, params.speed, params.direction);
                            break;
                        case ACTION_SHAKE_LEG:
                            controller->otto_.ShakeLeg(params.steps, params.speed, params.direction);
                            break;
                        case ACTION_SIT:
                            controller->otto_.Sit();
                            break;
                        case ACTION_RADIO_CALISTHENICS:
                            if (controller->has_hands_) {
                                controller->otto_.RadioCalisthenics();
                            }
                            break;
                        case ACTION_MAGIC_CIRCLE:
                            if (controller->has_hands_) {
                                controller->otto_.MagicCircle();
                            }
                            break;
                        case ACTION_SHOWCASE:
                            controller->otto_.Showcase();
                            break;
                        case ACTION_UPDOWN:
                            controller->otto_.UpDown(params.steps, params.speed, params.amount);
                            break;
                        case ACTION_TIPTOE_SWING:
                            controller->otto_.TiptoeSwing(params.steps, params.speed, params.amount);
                            break;
                        case ACTION_JITTER:
                            controller->otto_.Jitter(params.steps, params.speed, params.amount);
                            break;
                        case ACTION_ASCENDING_TURN:
                            controller->otto_.AscendingTurn(params.steps, params.speed, params.amount);
                            break;
                        case ACTION_CRUSAITO:
                            controller->otto_.Crusaito(params.steps, params.speed, params.amount,
                                                       params.direction);
                            break;
                        case ACTION_FLAPPING:
                            controller->otto_.Flapping(params.steps, params.speed, params.amount,
                                                       params.direction);
                            break;
                        case ACTION_WHIRLWIND_LEG:
                            controller->otto_.WhirlwindLeg(params.steps, params.speed, params.amount);
                            break;
                        case ACTION_HANDS_UP:
                            if (controller->has_hands_) {
                                controller->otto_.HandsUp(params.speed, params.direction);
                            }
                            break;
                        case ACTION_HANDS_DOWN:
                            if (controller->has_hands_) {
                                controller->otto_.HandsDown(params.speed, params.direction);
                            }
                            break;
                        case ACTION_HAND_WAVE:
                            if (controller->has_hands_) {
                                controller->otto_.HandWave( params.direction);
                            }
                            break;
                        case ACTION_WINDMILL:
                            if (controller->has_hands_) {
                                controller->otto_.Windmill(params.steps, params.speed, params.amount);
                            }
                            break;
                        case ACTION_TAKEOFF:
                            if (controller->has_hands_) {
                                controller->otto_.Takeoff(params.steps, params.speed, params.amount);
                            }
                            break;
                        case ACTION_FITNESS:
                            if (controller->has_hands_) {
                                controller->otto_.Fitness(params.steps, params.speed, params.amount);
                            }
                            break;
                        case ACTION_GREETING:
                            if (controller->has_hands_) {
                                controller->otto_.Greeting(params.direction, params.steps);
                            }
                            break;
                        case ACTION_SHY:
                            if (controller->has_hands_) {
                                controller->otto_.Shy(params.direction, params.steps);
                            }
                            break;
                        case ACTION_HOME:
                            controller->otto_.Home(true);
                            break;
                    }
                    if(params.action_type != ACTION_SIT){
                        if (params.action_type != ACTION_HOME && params.action_type != ACTION_SERVO_SEQUENCE) {
                            controller->otto_.Home(params.action_type != ACTION_HANDS_UP);
                        }
                    }
                }
                controller->is_action_in_progress_ = false;
                PowerManager::ResumeBatteryUpdate();  // åŠ¨ä½œç»“æŸæ—¶æ¢å¤ç”µé‡æ›´æ–°
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
    }

    void StartActionTaskIfNeeded() {
        if (action_task_handle_ == nullptr) {
            xTaskCreate(ActionTask, "otto_action", 1024 * 3, this, configMAX_PRIORITIES - 1,
                        &action_task_handle_);
        }
    }

    void QueueAction(int action_type, int steps, int speed, int direction, int amount) {
        // æ£€æŸ¥æ‰‹éƒ¨åŠ¨ä½œ
        if ((action_type >= ACTION_HANDS_UP && action_type <= ACTION_HAND_WAVE) || 
            (action_type == ACTION_WINDMILL) || (action_type == ACTION_TAKEOFF) || 
            (action_type == ACTION_FITNESS) || (action_type == ACTION_GREETING) ||
            (action_type == ACTION_SHY) || (action_type == ACTION_RADIO_CALISTHENICS) ||
            (action_type == ACTION_MAGIC_CIRCLE)) {
            if (!has_hands_) {
                ESP_LOGW(TAG, "å°è¯•æ‰§è¡Œæ‰‹éƒ¨åŠ¨ä½œï¼Œä½†æœºå™¨äººæ²¡æœ‰é…ç½®æ‰‹éƒ¨èˆµæœº");
                return;
            }
        }

        ESP_LOGI(TAG, "åŠ¨ä½œæŽ§åˆ¶: ç±»åž‹=%d, æ­¥æ•°=%d, é€Ÿåº¦=%d, æ–¹å‘=%d, å¹…åº¦=%d", action_type, steps,
                 speed, direction, amount);

        OttoActionParams params = {action_type, steps, speed, direction, amount, ""};
        xQueueSend(action_queue_, &params, portMAX_DELAY);
        StartActionTaskIfNeeded();
    }

    void QueueServoSequence(const char* servo_sequence_json) {
        if (servo_sequence_json == nullptr) {
            ESP_LOGE(TAG, "åºåˆ—JSONä¸ºç©º");
            return;
        }
        
        int input_len = strlen(servo_sequence_json);
        const int buffer_size = 512;  // servo_sequence_jsonæ•°ç»„å¤§å°
        ESP_LOGI(TAG, "é˜Ÿåˆ—èˆµæœºåºåˆ—ï¼Œè¾“å…¥é•¿åº¦=%dï¼Œç¼“å†²åŒºå¤§å°=%d", input_len, buffer_size);
        
        if (input_len >= buffer_size) {
            ESP_LOGE(TAG, "JSONå­—ç¬¦ä¸²å¤ªé•¿ï¼è¾“å…¥é•¿åº¦=%dï¼Œæœ€å¤§å…è®¸=%d", input_len, buffer_size - 1);
            return;
        }
        
        if (input_len == 0) {
            ESP_LOGW(TAG, "åºåˆ—JSONä¸ºç©ºå­—ç¬¦ä¸²");
            return;
        }
        
        OttoActionParams params = {ACTION_SERVO_SEQUENCE, 0, 0, 0, 0, ""};
        // å¤åˆ¶JSONå­—ç¬¦ä¸²åˆ°ç»“æž„ä½“ä¸­ï¼ˆé™åˆ¶é•¿åº¦ï¼‰
        strncpy(params.servo_sequence_json, servo_sequence_json, sizeof(params.servo_sequence_json) - 1);
        params.servo_sequence_json[sizeof(params.servo_sequence_json) - 1] = '\0';
        
        ESP_LOGD(TAG, "åºåˆ—å·²åŠ å…¥é˜Ÿåˆ—: %s", params.servo_sequence_json);
        
        xQueueSend(action_queue_, &params, portMAX_DELAY);
        StartActionTaskIfNeeded();
    }

    void LoadTrimsFromNVS() {
        Settings settings("otto_trims", false);

        int left_leg = settings.GetInt("left_leg", 0);
        int right_leg = settings.GetInt("right_leg", 0);
        int left_foot = settings.GetInt("left_foot", 0);
        int right_foot = settings.GetInt("right_foot", 0);
        int left_hand = settings.GetInt("left_hand", 0);
        int right_hand = settings.GetInt("right_hand", 0);

        ESP_LOGI(TAG, "ä»ŽNVSåŠ è½½å¾®è°ƒè®¾ç½®: å·¦è…¿=%d, å³è…¿=%d, å·¦è„š=%d, å³è„š=%d, å·¦æ‰‹=%d, å³æ‰‹=%d",
                 left_leg, right_leg, left_foot, right_foot, left_hand, right_hand);

        otto_.SetTrims(left_leg, right_leg, left_foot, right_foot, left_hand, right_hand);
    }

public:
    OttoController(const HardwareConfig& hw_config) {
        otto_.Init(
            hw_config.left_leg_pin, 
            hw_config.right_leg_pin, 
            hw_config.left_foot_pin, 
            hw_config.right_foot_pin, 
            hw_config.left_hand_pin,
            hw_config.right_hand_pin
        );

        has_hands_ = (hw_config.left_hand_pin != GPIO_NUM_NC && hw_config.right_hand_pin != GPIO_NUM_NC);
        ESP_LOGI(TAG, "Ottoæœºå™¨äººåˆå§‹åŒ–%sæ‰‹éƒ¨èˆµæœº", has_hands_ ? "å¸¦" : "ä¸å¸¦");
        ESP_LOGI(TAG, "èˆµæœºå¼•è„šé…ç½®: LL=%d, RL=%d, LF=%d, RF=%d, LH=%d, RH=%d",
                 hw_config.left_leg_pin, hw_config.right_leg_pin,
                 hw_config.left_foot_pin, hw_config.right_foot_pin,
                 hw_config.left_hand_pin, hw_config.right_hand_pin);

        LoadTrimsFromNVS();

        action_queue_ = xQueueCreate(10, sizeof(OttoActionParams));

        QueueAction(ACTION_HOME, 1, 1000, 1, 0);  // direction=1è¡¨ç¤ºå¤ä½æ‰‹éƒ¨

        RegisterMcpTools();
    }

    void RegisterMcpTools() {
        auto& mcp_server = McpServer::GetInstance();

        ESP_LOGI(TAG, "å¼€å§‹æ³¨å†ŒMCPå·¥å…·...");

        // ç»Ÿä¸€åŠ¨ä½œå·¥å…·ï¼ˆé™¤äº†èˆµæœºåºåˆ—å¤–çš„æ‰€æœ‰åŠ¨ä½œï¼‰
        mcp_server.AddTool("self.otto.action",
                           "æ‰§è¡Œæœºå™¨äººåŠ¨ä½œã€‚action: åŠ¨ä½œåç§°ï¼›æ ¹æ®åŠ¨ä½œç±»åž‹æä¾›ç›¸åº”å‚æ•°ï¼šdirection: æ–¹å‘ï¼Œ1=å‰è¿›/å·¦è½¬ï¼Œ-1=åŽé€€/å³è½¬ï¼›0=å·¦å³åŒæ—¶"
                           "steps: åŠ¨ä½œæ­¥æ•°ï¼Œ1-100ï¼›speed: åŠ¨ä½œé€Ÿåº¦ï¼Œ100-3000ï¼Œæ•°å€¼è¶Šå°è¶Šå¿«ï¼›amount: åŠ¨ä½œå¹…åº¦ï¼Œ0-170ï¼›arm_swing: æ‰‹è‡‚æ‘†åŠ¨å¹…åº¦ï¼Œ0-170ï¼›"
                           "åŸºç¡€åŠ¨ä½œï¼šwalk(è¡Œèµ°ï¼Œéœ€steps/speed/direction/arm_swing)ã€turn(è½¬èº«ï¼Œéœ€steps/speed/direction/arm_swing)ã€jump(è·³è·ƒï¼Œéœ€steps/speed)ã€"
                           "swing(æ‘‡æ‘†ï¼Œéœ€steps/speed/amount)ã€moonwalk(å¤ªç©ºæ­¥ï¼Œéœ€steps/speed/direction/amount)ã€bend(å¼¯æ›²ï¼Œéœ€steps/speed/direction)ã€"
                           "shake_leg(æ‘‡è…¿ï¼Œéœ€steps/speed/direction)ã€updown(ä¸Šä¸‹è¿åŠ¨ï¼Œéœ€steps/speed/amount)ã€whirlwind_leg(æ—‹é£Žè…¿ï¼Œéœ€steps/speed/amount)ï¼›"
                           "å›ºå®šåŠ¨ä½œï¼šsit(åä¸‹)ã€showcase(å±•ç¤ºåŠ¨ä½œ)ã€home(å¤ä½)ï¼›"
                           "æ‰‹éƒ¨åŠ¨ä½œ(éœ€æ‰‹éƒ¨èˆµæœº)ï¼šhands_up(ä¸¾æ‰‹ï¼Œéœ€speed/direction)ã€hands_down(æ”¾æ‰‹ï¼Œéœ€speed/direction)ã€hand_wave(æŒ¥æ‰‹ï¼Œéœ€direction)ã€"
                           "windmill(å¤§é£Žè½¦ï¼Œéœ€steps/speed/amount)ã€takeoff(èµ·é£žï¼Œéœ€steps/speed/amount)ã€fitness(å¥èº«ï¼Œéœ€steps/speed/amount)ã€"
                           "greeting(æ‰“æ‹›å‘¼ï¼Œéœ€direction/steps)ã€shy(å®³ç¾žï¼Œéœ€direction/steps)ã€radio_calisthenics(å¹¿æ’­ä½“æ“)ã€magic_circle(çˆ±çš„é­”åŠ›è½¬åœˆåœˆ)",
                           PropertyList({
                               Property("action", kPropertyTypeString, "sit"),
                               Property("steps", kPropertyTypeInteger, 3, 1, 100),
                               Property("speed", kPropertyTypeInteger, 700, 100, 3000),
                               Property("direction", kPropertyTypeInteger, 1, -1, 1),
                               Property("amount", kPropertyTypeInteger, 30, 0, 170),
                               Property("arm_swing", kPropertyTypeInteger, 50, 0, 170)
                           }),
                           [this](const PropertyList& properties) -> ReturnValue {
                               std::string action = properties["action"].value<std::string>();
                               // æ‰€æœ‰å‚æ•°éƒ½æœ‰é»˜è®¤å€¼ï¼Œç›´æŽ¥è®¿é—®å³å¯
                               int steps = properties["steps"].value<int>();
                               int speed = properties["speed"].value<int>();
                               int direction = properties["direction"].value<int>();
                               int amount = properties["amount"].value<int>();
                               int arm_swing = properties["arm_swing"].value<int>();

                               // åŸºç¡€ç§»åŠ¨åŠ¨ä½œ
                               if (action == "walk") {
                                   QueueAction(ACTION_WALK, steps, speed, direction, arm_swing);
                                   return true;
                               } else if (action == "turn") {
                                   QueueAction(ACTION_TURN, steps, speed, direction, arm_swing);
                                   return true;
                               } else if (action == "jump") {
                                   QueueAction(ACTION_JUMP, steps, speed, 0, 0);
                                   return true;
                               } else if (action == "swing") {
                                   QueueAction(ACTION_SWING, steps, speed, 0, amount);
                                   return true;
                               } else if (action == "moonwalk") {
                                   QueueAction(ACTION_MOONWALK, steps, speed, direction, amount);
                                   return true;
                               } else if (action == "bend") {
                                   QueueAction(ACTION_BEND, steps, speed, direction, 0);
                                   return true;
                               } else if (action == "shake_leg") {
                                   QueueAction(ACTION_SHAKE_LEG, steps, speed, direction, 0);
                                   return true;
                               } else if (action == "updown") {
                                   QueueAction(ACTION_UPDOWN, steps, speed, 0, amount);
                                   return true;
                               } else if (action == "whirlwind_leg") {
                                   QueueAction(ACTION_WHIRLWIND_LEG, steps, speed, 0, amount);
                                   return true;
                               }
                               // å›ºå®šåŠ¨ä½œ
                               else if (action == "sit") {
                                   QueueAction(ACTION_SIT, 1, 0, 0, 0);
                                   return true;
                               } else if (action == "showcase") {
                                   QueueAction(ACTION_SHOWCASE, 1, 0, 0, 0);
                                   return true;
                               } else if (action == "home") {
                                   QueueAction(ACTION_HOME, 1, 1000, 1, 0);
                                   return true;
                               }
                               // æ‰‹éƒ¨åŠ¨ä½œ
                               else if (action == "hands_up") {
                                   if (!has_hands_) {
                                       return "é”™è¯¯ï¼šæ­¤åŠ¨ä½œéœ€è¦æ‰‹éƒ¨èˆµæœºæ”¯æŒ";
                                   }
                                   QueueAction(ACTION_HANDS_UP, 1, speed, direction, 0);
                                   return true;
                               } else if (action == "hands_down") {
                                   if (!has_hands_) {
                                       return "é”™è¯¯ï¼šæ­¤åŠ¨ä½œéœ€è¦æ‰‹éƒ¨èˆµæœºæ”¯æŒ";
                                   }
                                   QueueAction(ACTION_HANDS_DOWN, 1, speed, direction, 0);
                                   return true;
                               } else if (action == "hand_wave") {
                                   if (!has_hands_) {
                                       return "é”™è¯¯ï¼šæ­¤åŠ¨ä½œéœ€è¦æ‰‹éƒ¨èˆµæœºæ”¯æŒ";
                                   }
                                   QueueAction(ACTION_HAND_WAVE, 1, 0, 0, direction);
                                   return true;
                               } else if (action == "windmill") {
                                   if (!has_hands_) {
                                       return "é”™è¯¯ï¼šæ­¤åŠ¨ä½œéœ€è¦æ‰‹éƒ¨èˆµæœºæ”¯æŒ";
                                   }
                                   QueueAction(ACTION_WINDMILL, steps, speed, 0, amount);
                                   return true;
                               } else if (action == "takeoff") {
                                   if (!has_hands_) {
                                       return "é”™è¯¯ï¼šæ­¤åŠ¨ä½œéœ€è¦æ‰‹éƒ¨èˆµæœºæ”¯æŒ";
                                   }
                                   QueueAction(ACTION_TAKEOFF, steps, speed, 0, amount);
                                   return true;
                               } else if (action == "fitness") {
                                   if (!has_hands_) {
                                       return "é”™è¯¯ï¼šæ­¤åŠ¨ä½œéœ€è¦æ‰‹éƒ¨èˆµæœºæ”¯æŒ";
                                   }
                                   QueueAction(ACTION_FITNESS, steps, speed, 0, amount);
                                   return true;
                               } else if (action == "greeting") {
                                   if (!has_hands_) {
                                       return "é”™è¯¯ï¼šæ­¤åŠ¨ä½œéœ€è¦æ‰‹éƒ¨èˆµæœºæ”¯æŒ";
                                   }
                                   QueueAction(ACTION_GREETING, steps, 0, direction, 0);
                                   return true;
                               } else if (action == "shy") {
                                   if (!has_hands_) {
                                       return "é”™è¯¯ï¼šæ­¤åŠ¨ä½œéœ€è¦æ‰‹éƒ¨èˆµæœºæ”¯æŒ";
                                   }
                                   QueueAction(ACTION_SHY, steps, 0, direction, 0);
                                   return true;
                               } else if (action == "radio_calisthenics") {
                                   if (!has_hands_) {
                                       return "é”™è¯¯ï¼šæ­¤åŠ¨ä½œéœ€è¦æ‰‹éƒ¨èˆµæœºæ”¯æŒ";
                                   }
                                   QueueAction(ACTION_RADIO_CALISTHENICS, 1, 0, 0, 0);
                                   return true;
                               } else if (action == "magic_circle") {
                                   if (!has_hands_) {
                                       return "é”™è¯¯ï¼šæ­¤åŠ¨ä½œéœ€è¦æ‰‹éƒ¨èˆµæœºæ”¯æŒ";
                                   }
                                   QueueAction(ACTION_MAGIC_CIRCLE, 1, 0, 0, 0);
                                   return true;
                               } else {
                                   return "é”™è¯¯ï¼šæ— æ•ˆçš„åŠ¨ä½œåç§°ã€‚å¯ç”¨åŠ¨ä½œï¼šwalk, turn, jump, swing, moonwalk, bend, shake_leg, updown, whirlwind_leg, sit, showcase, home, hands_up, hands_down, hand_wave, windmill, takeoff, fitness, greeting, shy, radio_calisthenics, magic_circle";
                               }
                           });


        // èˆµæœºåºåˆ—å·¥å…·ï¼ˆæ”¯æŒåˆ†æ®µå‘é€ï¼Œæ¯æ¬¡å‘é€ä¸€ä¸ªåºåˆ—ï¼Œè‡ªåŠ¨æŽ’é˜Ÿæ‰§è¡Œï¼‰
        mcp_server.AddTool(
            "self.otto.servo_sequences",
            "AIè‡ªå®šä¹‰åŠ¨ä½œç¼–ç¨‹ï¼ˆå³å…´åŠ¨ä½œï¼‰ã€‚æ”¯æŒåˆ†æ®µå‘é€åºåˆ—ï¼šè¶…è¿‡5ä¸ªåºåˆ—å»ºè®®AIå¯ä»¥è¿žç»­å¤šæ¬¡è°ƒç”¨æ­¤å·¥å…·ï¼Œæ¯æ¬¡å‘é€ä¸€ä¸ªçŸ­åºåˆ—ï¼Œç³»ç»Ÿä¼šè‡ªåŠ¨æŽ’é˜ŸæŒ‰é¡ºåºæ‰§è¡Œã€‚æ”¯æŒæ™®é€šç§»åŠ¨å’ŒæŒ¯è¡å™¨ä¸¤ç§æ¨¡å¼ã€‚"
            "æœºå™¨äººç»“æž„ï¼šåŒæ‰‹å¯ä¸Šä¸‹æ‘†åŠ¨ï¼ŒåŒè…¿å¯å†…æ”¶å¤–å±•ï¼ŒåŒè„šå¯ä¸Šä¸‹ç¿»è½¬ã€‚"
            "èˆµæœºè¯´æ˜Žï¼š"
            "ll(å·¦è…¿)ï¼šå†…æ”¶å¤–å±•ï¼Œ0åº¦=å®Œå…¨å¤–å±•ï¼Œ90åº¦=ä¸­ç«‹ï¼Œ180åº¦=å®Œå…¨å†…æ”¶ï¼›"
            "rl(å³è…¿)ï¼šå†…æ”¶å¤–å±•ï¼Œ0åº¦=å®Œå…¨å†…æ”¶ï¼Œ90åº¦=ä¸­ç«‹ï¼Œ180åº¦=å®Œå…¨å¤–å±•ï¼›"
            "lf(å·¦è„š)ï¼šä¸Šä¸‹ç¿»è½¬ï¼Œ0åº¦=å®Œå…¨å‘ä¸Šï¼Œ90åº¦=æ°´å¹³ï¼Œ180åº¦=å®Œå…¨å‘ä¸‹ï¼›"
            "rf(å³è„š)ï¼šä¸Šä¸‹ç¿»è½¬ï¼Œ0åº¦=å®Œå…¨å‘ä¸‹ï¼Œ90åº¦=æ°´å¹³ï¼Œ180åº¦=å®Œå…¨å‘ä¸Šï¼›"
            "lh(å·¦æ‰‹)ï¼šä¸Šä¸‹æ‘†åŠ¨ï¼Œ0åº¦=å®Œå…¨å‘ä¸‹ï¼Œ90åº¦=æ°´å¹³ï¼Œ180åº¦=å®Œå…¨å‘ä¸Šï¼›"
            "rh(å³æ‰‹)ï¼šä¸Šä¸‹æ‘†åŠ¨ï¼Œ0åº¦=å®Œå…¨å‘ä¸Šï¼Œ90åº¦=æ°´å¹³ï¼Œ180åº¦=å®Œå…¨å‘ä¸‹ï¼›"
            "sequence: å•ä¸ªåºåˆ—å¯¹è±¡ï¼ŒåŒ…å«'a'åŠ¨ä½œæ•°ç»„ï¼Œé¡¶å±‚å¯é€‰å‚æ•°ï¼š"
            "'d'(åºåˆ—æ‰§è¡Œå®ŒæˆåŽå»¶è¿Ÿæ¯«ç§’æ•°ï¼Œç”¨äºŽåºåˆ—ä¹‹é—´çš„åœé¡¿)ã€‚"
            "æ¯ä¸ªåŠ¨ä½œå¯¹è±¡åŒ…å«ï¼š"
            "æ™®é€šæ¨¡å¼ï¼š's'èˆµæœºä½ç½®å¯¹è±¡(é”®åï¼šll/rl/lf/rf/lh/rhï¼Œå€¼ï¼š0-180åº¦)ï¼Œ'v'ç§»åŠ¨é€Ÿåº¦100-3000æ¯«ç§’(é»˜è®¤1000)ï¼Œ'd'åŠ¨ä½œåŽå»¶è¿Ÿæ¯«ç§’æ•°(é»˜è®¤0)ï¼›"
            "æŒ¯è¡æ¨¡å¼ï¼š'osc'æŒ¯è¡å™¨å¯¹è±¡ï¼ŒåŒ…å«'a'æŒ¯å¹…å¯¹è±¡(å„èˆµæœºæŒ¯å¹…10-90åº¦ï¼Œé»˜è®¤20åº¦)ï¼Œ'o'ä¸­å¿ƒè§’åº¦å¯¹è±¡(å„èˆµæœºæŒ¯è¡ä¸­å¿ƒç»å¯¹è§’åº¦0-180åº¦ï¼Œé»˜è®¤90åº¦)ï¼Œ'ph'ç›¸ä½å·®å¯¹è±¡(å„èˆµæœºç›¸ä½å·®ï¼Œåº¦ï¼Œ0-360åº¦ï¼Œé»˜è®¤0åº¦)ï¼Œ'p'å‘¨æœŸ100-3000æ¯«ç§’(é»˜è®¤500)ï¼Œ'c'å‘¨æœŸæ•°0.1-20.0(é»˜è®¤5.0)ï¼›"
            "ä½¿ç”¨æ–¹å¼ï¼šAIå¯ä»¥è¿žç»­å¤šæ¬¡è°ƒç”¨æ­¤å·¥å…·ï¼Œæ¯æ¬¡å‘é€ä¸€ä¸ªåºåˆ—ï¼Œç³»ç»Ÿä¼šè‡ªåŠ¨æŽ’é˜ŸæŒ‰é¡ºåºæ‰§è¡Œã€‚"
            "é‡è¦è¯´æ˜Žï¼šå·¦å³è…¿è„šéœ‡è¡çš„æ—¶å€™ï¼Œæœ‰ä¸€åªè„šå¿…é¡»åœ¨90åº¦ï¼Œå¦åˆ™ä¼šæŸåæœºå™¨äººï¼Œå¦‚æžœå‘é€å¤šä¸ªåºåˆ—ï¼ˆåºåˆ—æ•°>1ï¼‰ï¼Œå®Œæˆæ‰€æœ‰åºåˆ—åŽéœ€è¦å¤ä½æ—¶ï¼ŒAIåº”è¯¥æœ€åŽå•ç‹¬è°ƒç”¨self.otto.homeå·¥å…·è¿›è¡Œå¤ä½ï¼Œä¸è¦åœ¨åºåˆ—ä¸­è®¾ç½®å¤ä½å‚æ•°ã€‚"
            "æ™®é€šæ¨¡å¼ç¤ºä¾‹ï¼šå‘é€3ä¸ªåºåˆ—ï¼Œæœ€åŽè°ƒç”¨å¤ä½ï¼š"
            "ç¬¬1æ¬¡è°ƒç”¨{\"sequence\":\"{\\\"a\\\":[{\\\"s\\\":{\\\"ll\\\":100},\\\"v\\\":1000}],\\\"d\\\":500}\"}ï¼Œ"
            "ç¬¬2æ¬¡è°ƒç”¨{\"sequence\":\"{\\\"a\\\":[{\\\"s\\\":{\\\"ll\\\":90},\\\"v\\\":800}],\\\"d\\\":500}\"}ï¼Œ"
            "ç¬¬3æ¬¡è°ƒç”¨{\"sequence\":\"{\\\"a\\\":[{\\\"s\\\":{\\\"ll\\\":80},\\\"v\\\":800}]}\"}ï¼Œ"
            "æœ€åŽè°ƒç”¨self.otto.homeå·¥å…·è¿›è¡Œå¤ä½ã€‚"
            "æŒ¯è¡å™¨æ¨¡å¼ç¤ºä¾‹ï¼š"
            "ç¤ºä¾‹1-åŒè‡‚åŒæ­¥æ‘†åŠ¨ï¼š{\"sequence\":\"{\\\"a\\\":[{\\\"osc\\\":{\\\"a\\\":{\\\"lh\\\":30,\\\"rh\\\":30},\\\"o\\\":{\\\"lh\\\":90,\\\"rh\\\":-90},\\\"p\\\":500,\\\"c\\\":5.0}}],\\\"d\\\":0}\"}ï¼›"
            "ç¤ºä¾‹2-åŒè…¿äº¤æ›¿æŒ¯è¡ï¼ˆæ³¢æµªæ•ˆæžœï¼‰ï¼š{\"sequence\":\"{\\\"a\\\":[{\\\"osc\\\":{\\\"a\\\":{\\\"ll\\\":20,\\\"rl\\\":20},\\\"o\\\":{\\\"ll\\\":90,\\\"rl\\\":-90},\\\"ph\\\":{\\\"rl\\\":180},\\\"p\\\":600,\\\"c\\\":3.0}}],\\\"d\\\":0}\"}ï¼›"
            "ç¤ºä¾‹3-å•è…¿æŒ¯è¡é…åˆå›ºå®šè„šï¼ˆå®‰å…¨ï¼‰ï¼š{\"sequence\":\"{\\\"a\\\":[{\\\"osc\\\":{\\\"a\\\":{\\\"ll\\\":45},\\\"o\\\":{\\\"ll\\\":90,\\\"lf\\\":90},\\\"p\\\":400,\\\"c\\\":4.0}}],\\\"d\\\":0}\"}ï¼›"
            "ç¤ºä¾‹4-å¤æ‚å¤šèˆµæœºæŒ¯è¡ï¼ˆæ‰‹å’Œè…¿ï¼‰ï¼š{\"sequence\":\"{\\\"a\\\":[{\\\"osc\\\":{\\\"a\\\":{\\\"lh\\\":25,\\\"rh\\\":25,\\\"ll\\\":15},\\\"o\\\":{\\\"lh\\\":90,\\\"rh\\\":90,\\\"ll\\\":90,\\\"lf\\\":90},\\\"ph\\\":{\\\"rh\\\":180},\\\"p\\\":800,\\\"c\\\":6.0}}],\\\"d\\\":500}\"}ï¼›"
            "ç¤ºä¾‹5-å¿«é€Ÿæ‘‡æ‘†ï¼š{\"sequence\":\"{\\\"a\\\":[{\\\"osc\\\":{\\\"a\\\":{\\\"ll\\\":30,\\\"rl\\\":30},\\\"o\\\":{\\\"ll\\\":90,\\\"rl\\\":90},\\\"ph\\\":{\\\"rl\\\":180},\\\"p\\\":300,\\\"c\\\":10.0}}],\\\"d\\\":0}\"}ã€‚",
            PropertyList({Property("sequence", kPropertyTypeString,
                                   "{\"a\":[{\"s\":{\"ll\":90,\"rl\":90},\"v\":1000}]}")}),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string sequence = properties["sequence"].value<std::string>();
                // æ£€æŸ¥æ˜¯å¦æ˜¯JSONå¯¹è±¡ï¼ˆå¯èƒ½æ˜¯å­—ç¬¦ä¸²æ ¼å¼æˆ–å·²è§£æžçš„å¯¹è±¡ï¼‰
                // å¦‚æžœsequenceæ˜¯JSONå­—ç¬¦ä¸²ï¼Œç›´æŽ¥ä½¿ç”¨ï¼›å¦‚æžœæ˜¯å¯¹è±¡å­—ç¬¦ä¸²ï¼Œä¹Ÿéœ€è¦ä½¿ç”¨
                QueueServoSequence(sequence.c_str());
                return true;
            });


        mcp_server.AddTool("self.otto.stop", "ç«‹å³åœæ­¢æ‰€æœ‰åŠ¨ä½œå¹¶å¤ä½", PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               if (action_task_handle_ != nullptr) {
                                   vTaskDelete(action_task_handle_);
                                   action_task_handle_ = nullptr;
                               }
                               is_action_in_progress_ = false;
                               PowerManager::ResumeBatteryUpdate();  // åœæ­¢åŠ¨ä½œæ—¶æ¢å¤ç”µé‡æ›´æ–°
                               xQueueReset(action_queue_);

                               QueueAction(ACTION_HOME, 1, 1000, 1, 0);
                               return true;
                           });

        mcp_server.AddTool(
            "self.otto.set_trim",
            "æ ¡å‡†å•ä¸ªèˆµæœºä½ç½®ã€‚è®¾ç½®æŒ‡å®šèˆµæœºçš„å¾®è°ƒå‚æ•°ä»¥è°ƒæ•´æœºå™¨äººçš„åˆå§‹ç«™ç«‹å§¿æ€ï¼Œè®¾ç½®å°†æ°¸ä¹…ä¿å­˜ã€‚"
            "servo_type: èˆµæœºç±»åž‹(left_leg/right_leg/left_foot/right_foot/left_hand/right_hand); "
            "trim_value: å¾®è°ƒå€¼(-50åˆ°50åº¦)",
            PropertyList({Property("servo_type", kPropertyTypeString, "left_leg"),
                          Property("trim_value", kPropertyTypeInteger, 0, -50, 50)}),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string servo_type = properties["servo_type"].value<std::string>();
                int trim_value = properties["trim_value"].value<int>();

                ESP_LOGI(TAG, "è®¾ç½®èˆµæœºå¾®è°ƒ: %s = %dåº¦", servo_type.c_str(), trim_value);

                // èŽ·å–å½“å‰æ‰€æœ‰å¾®è°ƒå€¼
                Settings settings("otto_trims", true);
                int left_leg = settings.GetInt("left_leg", 0);
                int right_leg = settings.GetInt("right_leg", 0);
                int left_foot = settings.GetInt("left_foot", 0);
                int right_foot = settings.GetInt("right_foot", 0);
                int left_hand = settings.GetInt("left_hand", 0);
                int right_hand = settings.GetInt("right_hand", 0);

                // æ›´æ–°æŒ‡å®šèˆµæœºçš„å¾®è°ƒå€¼
                if (servo_type == "left_leg") {
                    left_leg = trim_value;
                    settings.SetInt("left_leg", left_leg);
                } else if (servo_type == "right_leg") {
                    right_leg = trim_value;
                    settings.SetInt("right_leg", right_leg);
                } else if (servo_type == "left_foot") {
                    left_foot = trim_value;
                    settings.SetInt("left_foot", left_foot);
                } else if (servo_type == "right_foot") {
                    right_foot = trim_value;
                    settings.SetInt("right_foot", right_foot);
                } else if (servo_type == "left_hand") {
                    if (!has_hands_) {
                        return "é”™è¯¯ï¼šæœºå™¨äººæ²¡æœ‰é…ç½®æ‰‹éƒ¨èˆµæœº";
                    }
                    left_hand = trim_value;
                    settings.SetInt("left_hand", left_hand);
                } else if (servo_type == "right_hand") {
                    if (!has_hands_) {
                        return "é”™è¯¯ï¼šæœºå™¨äººæ²¡æœ‰é…ç½®æ‰‹éƒ¨èˆµæœº";
                    }
                    right_hand = trim_value;
                    settings.SetInt("right_hand", right_hand);
                } else {
                    return "é”™è¯¯ï¼šæ— æ•ˆçš„èˆµæœºç±»åž‹ï¼Œè¯·ä½¿ç”¨: left_leg, right_leg, left_foot, "
                           "right_foot, left_hand, right_hand";
                }

                otto_.SetTrims(left_leg, right_leg, left_foot, right_foot, left_hand, right_hand);

                QueueAction(ACTION_JUMP, 1, 500, 0, 0);

                return "èˆµæœº " + servo_type + " å¾®è°ƒè®¾ç½®ä¸º " + std::to_string(trim_value) +
                       " åº¦ï¼Œå·²æ°¸ä¹…ä¿å­˜";
            });

        mcp_server.AddTool("self.otto.get_trims", "èŽ·å–å½“å‰çš„èˆµæœºå¾®è°ƒè®¾ç½®", PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               Settings settings("otto_trims", false);

                               int left_leg = settings.GetInt("left_leg", 0);
                               int right_leg = settings.GetInt("right_leg", 0);
                               int left_foot = settings.GetInt("left_foot", 0);
                               int right_foot = settings.GetInt("right_foot", 0);
                               int left_hand = settings.GetInt("left_hand", 0);
                               int right_hand = settings.GetInt("right_hand", 0);

                               std::string result =
                                   "{\"left_leg\":" + std::to_string(left_leg) +
                                   ",\"right_leg\":" + std::to_string(right_leg) +
                                   ",\"left_foot\":" + std::to_string(left_foot) +
                                   ",\"right_foot\":" + std::to_string(right_foot) +
                                   ",\"left_hand\":" + std::to_string(left_hand) +
                                   ",\"right_hand\":" + std::to_string(right_hand) + "}";

                               ESP_LOGI(TAG, "èŽ·å–å¾®è°ƒè®¾ç½®: %s", result.c_str());
                               return result;
                           });

        mcp_server.AddTool("self.otto.get_status", "èŽ·å–æœºå™¨äººçŠ¶æ€ï¼Œè¿”å›ž moving æˆ– idle",
                           PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                               return is_action_in_progress_ ? "moving" : "idle";
                           });

        mcp_server.AddTool("self.battery.get_level", "èŽ·å–æœºå™¨äººç”µæ± ç”µé‡å’Œå……ç”µçŠ¶æ€", PropertyList(),
                           [](const PropertyList& properties) -> ReturnValue {
                               auto& board = Board::GetInstance();
                               int level = 0;
                               bool charging = false;
                               bool discharging = false;
                               board.GetBatteryLevel(level, charging, discharging);

                               std::string status =
                                   "{\"level\":" + std::to_string(level) +
                                   ",\"charging\":" + (charging ? "true" : "false") + "}";
                               return status;
                           });
                           
        mcp_server.AddTool("self.otto.get_ip", "èŽ·å–æœºå™¨äººWiFi IPåœ°å€", PropertyList(),
                           [](const PropertyList& properties) -> ReturnValue {
                               auto& wifi = WifiManager::GetInstance();
                               std::string ip = wifi.GetIpAddress();
                               if (ip.empty()) {
                                   return "{\"ip\":\"\",\"connected\":false}";
                               }
                               std::string status = "{\"ip\":\"" + ip + "\",\"connected\":true}";
                               return status;
                           });                           

        ESP_LOGI(TAG, "MCPå·¥å…·æ³¨å†Œå®Œæˆ");
    }

    ~OttoController() {
        if (action_task_handle_ != nullptr) {
            vTaskDelete(action_task_handle_);
            action_task_handle_ = nullptr;
        }
        vQueueDelete(action_queue_);
    }
};

static OttoController* g_otto_controller = nullptr;

void InitializeOttoController(const HardwareConfig& hw_config) {
    if (g_otto_controller == nullptr) {
        g_otto_controller = new OttoController(hw_config);
        ESP_LOGI(TAG, "OttoæŽ§åˆ¶å™¨å·²åˆå§‹åŒ–å¹¶æ³¨å†ŒMCPå·¥å…·");
    }
}
