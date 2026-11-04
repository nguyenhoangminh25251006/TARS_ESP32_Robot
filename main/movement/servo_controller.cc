#include "movement/servo_controller.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <algorithm>
#include <math.h>
#include "nvs_flash.h"
#include "nvs.h"

static const char* TAG = "ServoController";

struct ServoMoveData {
    uint8_t channel;
    int current;
    int target;
    int step;
    int distance;
    int steps_taken;
};

ServoController& ServoController::GetInstance() {
    static ServoController inst;
    return inst;
}

ServoController::ServoController() : inited_(false), is_moving_(false) {
    current_positions_[CH_LEFT_HEIGHT]  = kLH_Idle();
    current_positions_[CH_RIGHT_HEIGHT] = kRH_Idle();
    current_positions_[CH_LEFT_LEG]     = kLL_Neutral;
    current_positions_[CH_RIGHT_LEG]    = kRL_Neutral;
}

ServoController::~ServoController() { Shutdown(); }

// ================================================================
// Initialize: Set góc mặc định khi khởi động (Đứng nghỉ)
// ================================================================
void ServoController::Initialize() {
    if (inited_) return;
    if (!PCA9685::GetInstance().Initialize(21, 22)) {
        ESP_LOGE(TAG, "Failed to initialize PCA9685");
        return;
    }
    PCA9685::GetInstance().SetPWMFreq(50.0f);
    
    // Load saved calibration values from NVS before setting default positions
    LoadCalibration();
    StartCalibrationTask();

    ESP_LOGI(TAG, "Init: setting calibrated Stand position");

    // 1. Khởi động đặt ngay góc Calib để giữ đúng setup của người dùng
    PCA9685::GetInstance().SetPWM(CH_LEFT_HEIGHT,  0, kLH_Stand);
    PCA9685::GetInstance().SetPWM(CH_RIGHT_HEIGHT, 0, kRH_Stand);
    PCA9685::GetInstance().SetPWM(CH_LEFT_LEG,     0, kLL_Neutral);
    PCA9685::GetInstance().SetPWM(CH_RIGHT_LEG,    0, kRL_Neutral);

    current_positions_[CH_LEFT_HEIGHT]  = kLH_Stand;
    current_positions_[CH_RIGHT_HEIGHT] = kRH_Stand;
    current_positions_[CH_LEFT_LEG]     = kLL_Neutral;
    current_positions_[CH_RIGHT_LEG]    = kRL_Neutral;

    vTaskDelay(pdMS_TO_TICKS(500));
    // Tắt PWM để relax, nhưng đã set góc Calib trước đó
    DisableAllServos();

    inited_ = true;
    ESP_LOGI(TAG, "Ready — holding Stand posture LH=%d RH=%d LL=%d RL=%d",
             kLH_Stand, kRH_Stand, kLL_Neutral, kRL_Neutral);
}

void ServoController::Shutdown() { DisableAllServos(); }

void ServoController::DisableAllServos() {
    PCA9685::GetInstance().SetPWM(CH_LEFT_HEIGHT,  0, 0);
    PCA9685::GetInstance().SetPWM(CH_RIGHT_HEIGHT, 0, 0);
    PCA9685::GetInstance().SetPWM(CH_LEFT_LEG,     0, 0);
    PCA9685::GetInstance().SetPWM(CH_RIGHT_LEG,    0, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
}

void ServoController::MoveServosSynchronized(int tLH, int tRH, int tLL, int tRL, float speed) {
    int targets[4] = {tLH, tRH, tLL, tRL};
    ServoMoveData servos[4];
    int count = 0;

    for (int i = 0; i < 4; i++) {
        if (targets[i] < 0 || targets[i] == current_positions_[i]) continue;
        int dist = abs(targets[i] - current_positions_[i]);
        servos[count++] = {(uint8_t)i, current_positions_[i], targets[i],
                           (targets[i] > current_positions_[i]) ? 1 : -1, dist, 0};
    }
    if (count == 0) return;

    int max_dist = 0;
    for (int i = 0; i < count; i++) max_dist = std::max(max_dist, servos[i].distance);

    float eff = std::min(std::max(speed, 0.01f), 1.0f);
    
    // Logic chuẩn từ TARS Python: base_delay = 0.02 * (1.0 - speed) -> ra giây
    // Đổi sang microseconds để dùng esp_rom_delay_us (độ trễ mịn)
    float base_delay_us = 20000.0f * (1.0f - eff);

    bool done = false;
    while (!done) {
        done = true;
        for (int i = 0; i < count; i++) {
            if (servos[i].current != servos[i].target) {
                servos[i].current += servos[i].step;
                PCA9685::GetInstance().SetPWM(servos[i].channel, 0, servos[i].current);
                servos[i].steps_taken++;
                done = false;
            }
        }
        
        if (!done) {
            // Tính toán tiến độ và Easing (Chậm đầu - nhanh giữa - chậm cuối)
            float progress = 1.0f;
            if (max_dist > 0) {
                int min_steps = max_dist;
                for (int i = 0; i < count; i++) {
                    if (servos[i].current != servos[i].target || servos[i].steps_taken > 0) {
                        min_steps = std::min(min_steps, servos[i].steps_taken);
                    }
                }
                progress = (float)min_steps / (float)max_dist;
            }

            float eased = 0.0f;
            if (progress < 0.5f) {
                eased = 2.0f * progress * progress;
            } else {
                eased = 1.0f - 2.0f * (1.0f - progress) * (1.0f - progress);
            }
            
            // Hệ số easing multiplier giống Python
            float delay_multiplier = 1.0f + 1.0f * (1.0f - 4.0f * (eased - 0.5f) * (eased - 0.5f));
            uint32_t delay_us = (uint32_t)(base_delay_us * delay_multiplier);
            
            // Nếu độ trễ > 10ms (1 tick của 100Hz) thì ưu tiên dùng vTaskDelay để nhường CPU
            if (delay_us >= 10000) {
                vTaskDelay(pdMS_TO_TICKS(delay_us / 1000));
            } else if (delay_us > 0) {
                esp_rom_delay_us(delay_us);
            }
        }
    }
    
    for (int i = 0; i < count; i++) {
        current_positions_[servos[i].channel] = servos[i].target;
    }
}

// ================================================================
// SEQUENCES - LÀM CHUẨN THEO LOGIC CRUTCH GAIT (HAI CHÂN NHƯ NHAU)
// ================================================================

void ServoController::SequenceReset() {
    MoveServosSynchronized(kLH_Idle(), kRH_Idle(), kLL_Neutral, kRL_Neutral, 0.7f);
    vTaskDelay(pdMS_TO_TICKS(200));
    DisableAllServos();
}

void ServoController::SequenceWalkForward() {
    float fast_sp = 0.7f;
    float slow_sp = 0.3f;

    // Chuẩn bị: Thân hạ xuống, đưa tay đòn về phía trước (Fwd) để lấy đà
    int lh_lift_more = kLH_Lift() + 25;
    int rh_lift_more = kRH_Lift() - 25;
    
    // Tay phải vươn ra sau nhiều hơn để cân bằng lực đẩy với tay trái
    int rl_back_thrust = kRL_Back() + 20;

    MoveServosSynchronized(kLH_Lift(), kRH_Lift(), kLL_Neutral, kRL_Neutral, slow_sp);
    MoveServosSynchronized(kLH_Lift(), kRH_Lift(), kLL_Fwd(), kRL_Fwd(), slow_sp);

    for (int cycle = 0; cycle < 3; cycle++) {
        // 1. Đập tay đòn xuống (Stand) -> Nâng bổng thân lên
        MoveServosSynchronized(kLH_Stand, kRH_Stand, kLL_Fwd(), kRL_Fwd(), fast_sp);
        vTaskDelay(pdMS_TO_TICKS(150)); 
        
        // 2. Ưỡn ra đằng trước mượn lực: Đánh chân ra sau cực nhanh để ném thân tới
        // Sử dụng rl_back_thrust cho tay phải để vươn ra sau sâu hơn
        MoveServosSynchronized(kLH_Stand, kRH_Stand, kLL_Back(), rl_back_thrust, fast_sp);
        vTaskDelay(pdMS_TO_TICKS(150));
        
        // 3. Nâng tay đòn lên (Lift) -> Hạ thân chạm đất
        MoveServosSynchronized(kLH_Lift(), kRH_Lift(), kLL_Back(), rl_back_thrust, fast_sp);
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // 4. Từ từ thu chân ra đằng trước đến một nửa (Neutral)
        MoveServosSynchronized(kLH_Lift(), kRH_Lift(), kLL_Neutral, kRL_Neutral, slow_sp);
        vTaskDelay(pdMS_TO_TICKS(50));

        // 5. Thu về một nửa -> Đòn kéo lên để nâng chân tránh quệt đất
        // Lúc này rh_lift_more (-40) sẽ phát huy tác dụng nâng cao đòn phải
        MoveServosSynchronized(lh_lift_more, rh_lift_more, kLL_Neutral, kRL_Neutral, slow_sp);
        vTaskDelay(pdMS_TO_TICKS(50));

        // 6. Thu nốt chân ra đằng trước (Fwd)
        MoveServosSynchronized(lh_lift_more, rh_lift_more, kLL_Fwd(), kRL_Fwd(), slow_sp);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    // Trở về Stand (Góc Calib) để lấy dáng chuẩn, sau đó ngắt điện để tiết kiệm
    MoveServosSynchronized(kLH_Stand, kRH_Stand, kLL_Neutral, kRL_Neutral, slow_sp);
    vTaskDelay(pdMS_TO_TICKS(500));
    DisableAllServos();
}

void ServoController::SequenceStepForward() {
    float sp = 0.25f;
    MoveServosSynchronized(kLH_Stand, kRH_Stand, kLL_Neutral, kRL_Neutral, sp);
    vTaskDelay(pdMS_TO_TICKS(150));
    
    // Vươn tay phải ra sau sâu hơn một chút (tương tự WalkForward)
    int rl_back_thrust = kRL_Back() + 15;
    MoveServosSynchronized(kLH_Stand, kRH_Stand, kLL_Back(), rl_back_thrust, sp);
    vTaskDelay(pdMS_TO_TICKS(150));
    
    int ll_half = (kLL_Back() + kLL_Neutral) / 2;
    int rl_half = (rl_back_thrust + kRL_Neutral) / 2;
    MoveServosSynchronized(kLH_Lift(), kRH_Lift(), ll_half, rl_half, sp);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    int lh_lift_more = kLH_Lift() + 15;
    int rh_lift_more = kRH_Lift() - 15;
    MoveServosSynchronized(lh_lift_more, rh_lift_more, kLL_Neutral, kRL_Neutral, sp);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    MoveServosSynchronized(kLH_Stand, kRH_Stand, kLL_Neutral, kRL_Neutral, 0.8f);
    vTaskDelay(pdMS_TO_TICKS(500));
    DisableAllServos();
}

void ServoController::SequenceWalkBackward() {
    float fast_sp = 0.7f;
    float slow_sp = 0.3f;

    int lh_lift_more = kLH_Lift() + 25;
    int rh_lift_more = kRH_Lift() - 25;

    MoveServosSynchronized(kLH_Lift(), kRH_Lift(), kLL_Neutral, kRL_Neutral, slow_sp);
    MoveServosSynchronized(kLH_Lift(), kRH_Lift(), kLL_Back(), kRL_Back(), slow_sp);

    for (int cycle = 0; cycle < 3; cycle++) {
        // 1. Đập tay đòn xuống (Stand) -> Nâng bổng thân lên
        MoveServosSynchronized(kLH_Stand, kRH_Stand, kLL_Back(), kRL_Back(), fast_sp);
        vTaskDelay(pdMS_TO_TICKS(100));

        // 2. Ném thân về sau bằng cách gạt mạnh chân ra trước
        MoveServosSynchronized(kLH_Stand, kRH_Stand, kLL_Fwd(), kRL_Fwd(), fast_sp);
        vTaskDelay(pdMS_TO_TICKS(150));
        
        // 3. Hạ thân chạm đất
        MoveServosSynchronized(kLH_Lift(), kRH_Lift(), kLL_Fwd(), kRL_Fwd(), fast_sp);
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // 4. Từ từ thu chân ra sau đến một nửa (Neutral)
        MoveServosSynchronized(kLH_Lift(), kRH_Lift(), kLL_Neutral, kRL_Neutral, slow_sp);
        vTaskDelay(pdMS_TO_TICKS(50));

        // 5. Nâng đòn lên cao để nhấc chân tránh quệt đất
        MoveServosSynchronized(lh_lift_more, rh_lift_more, kLL_Neutral, kRL_Neutral, slow_sp);
        vTaskDelay(pdMS_TO_TICKS(50));

        // 6. Thu nốt chân ra sau (Back)
        MoveServosSynchronized(lh_lift_more, rh_lift_more, kLL_Back(), kRL_Back(), slow_sp);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    // Trở về Stand (Góc Calib) để lấy dáng chuẩn, sau đó ngắt điện để tiết kiệm
    MoveServosSynchronized(kLH_Stand, kRH_Stand, kLL_Neutral, kRL_Neutral, slow_sp);
    vTaskDelay(pdMS_TO_TICKS(500));
    DisableAllServos();
}

void ServoController::SequenceStepBackward() {
    float sp = 0.25f;
    MoveServosSynchronized(kLH_Stand, kRH_Stand, kLL_Neutral, kRL_Neutral, sp);
    vTaskDelay(pdMS_TO_TICKS(150));
    MoveServosSynchronized(kLH_Stand, kRH_Stand, kLL_Fwd(),     kRL_Fwd(),     sp);
    vTaskDelay(pdMS_TO_TICKS(150));
    
    int ll_half = (kLL_Fwd() + kLL_Neutral) / 2;
    int rl_half = (kRL_Fwd() + kRL_Neutral) / 2;
    MoveServosSynchronized(kLH_Lift(), kRH_Lift(), ll_half, rl_half, sp);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    int lh_lift_more = kLH_Lift() + 15;
    int rh_lift_more = kRH_Lift() - 15;
    MoveServosSynchronized(lh_lift_more, rh_lift_more, kLL_Neutral, kRL_Neutral, sp);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    MoveServosSynchronized(kLH_Stand, kRH_Stand, kLL_Neutral, kRL_Neutral, 0.8f);
    vTaskDelay(pdMS_TO_TICKS(500));
    DisableAllServos();
}

void ServoController::SequenceTurnRight() {
    float sp = 0.25f;

    MoveServosSynchronized(kLH_Stand, kRH_Stand, kLL_Neutral, kRL_Neutral, sp);
    vTaskDelay(pdMS_TO_TICKS(150));
    int ll_half = (kLL_Fwd() + kLL_Neutral) / 2;
    int rl_half = (kRL_Back() + kRL_Neutral) / 2;
    int lh_lift_more = kLH_Lift() + 15;
    int rh_lift_more = kRH_Lift() - 15;

    for (int i = 0; i < 3; i++) {
        // 1. Hạ tay đòn (Stand)
        MoveServosSynchronized(kLH_Stand, kRH_Stand, kLL_Neutral, kRL_Neutral, sp);
        vTaskDelay(pdMS_TO_TICKS(100));

        // 2. Chân trái tiến, chân phải lùi
        MoveServosSynchronized(kLH_Stand, kRH_Stand, kLL_Fwd(), kRL_Back(), sp);
        vTaskDelay(pdMS_TO_TICKS(150));
        
        // 3. Nâng tay đòn lên (Lift) -> Hạ thân chạm đất
        MoveServosSynchronized(kLH_Lift(), kRH_Lift(), kLL_Fwd(), kRL_Back(), sp);
        vTaskDelay(pdMS_TO_TICKS(150));
        
        // 4. Thu chân về một nửa khi bụng chạm đất
        MoveServosSynchronized(kLH_Lift(), kRH_Lift(), ll_half, rl_half, sp);
        vTaskDelay(pdMS_TO_TICKS(50));

        // 5. Nâng đòn lên thêm nữa
        MoveServosSynchronized(lh_lift_more, rh_lift_more, ll_half, rl_half, sp);
        vTaskDelay(pdMS_TO_TICKS(50));

        // 6. Thu nốt chân về Neutral
        MoveServosSynchronized(lh_lift_more, rh_lift_more, kLL_Neutral, kRL_Neutral, sp);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    MoveServosSynchronized(kLH_Stand, kRH_Stand, kLL_Neutral, kRL_Neutral, sp);
    MoveServosSynchronized(kLH_Idle(), kRH_Idle(), kLL_Neutral, kRL_Neutral, 0.8f);
    vTaskDelay(pdMS_TO_TICKS(500));
    DisableAllServos();
}

void ServoController::SequenceTurnLeft() {
    float sp = 0.25f;

    MoveServosSynchronized(kLH_Stand, kRH_Stand, kLL_Neutral, kRL_Neutral, sp);
    vTaskDelay(pdMS_TO_TICKS(150));
    int ll_half = (kLL_Back() + kLL_Neutral) / 2;
    int rl_half = (kRL_Fwd() + kRL_Neutral) / 2;
    int lh_lift_more = kLH_Lift() + 15;
    int rh_lift_more = kRH_Lift() - 15;

    for (int i = 0; i < 3; i++) {
        // 1. Hạ tay đòn (Stand)
        MoveServosSynchronized(kLH_Stand, kRH_Stand, kLL_Neutral, kRL_Neutral, sp);
        vTaskDelay(pdMS_TO_TICKS(100));

        // 2. Chân trái lùi, chân phải tiến
        MoveServosSynchronized(kLH_Stand, kRH_Stand, kLL_Back(), kRL_Fwd(), sp);
        vTaskDelay(pdMS_TO_TICKS(150));
        
        // 3. Nâng tay đòn lên (Lift) -> Hạ thân chạm đất
        MoveServosSynchronized(kLH_Lift(), kRH_Lift(), kLL_Back(), kRL_Fwd(), sp);
        vTaskDelay(pdMS_TO_TICKS(150));
        
        // 4. Thu chân về một nửa khi bụng chạm đất
        MoveServosSynchronized(kLH_Lift(), kRH_Lift(), ll_half, rl_half, sp);
        vTaskDelay(pdMS_TO_TICKS(50));

        // 5. Nâng đòn lên thêm nữa
        MoveServosSynchronized(lh_lift_more, rh_lift_more, ll_half, rl_half, sp);
        vTaskDelay(pdMS_TO_TICKS(50));

        // 6. Thu nốt chân về Neutral
        MoveServosSynchronized(lh_lift_more, rh_lift_more, kLL_Neutral, kRL_Neutral, sp);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    MoveServosSynchronized(kLH_Stand, kRH_Stand, kLL_Neutral, kRL_Neutral, sp);
    MoveServosSynchronized(kLH_Idle(), kRH_Idle(), kLL_Neutral, kRL_Neutral, 0.8f);
    vTaskDelay(pdMS_TO_TICKS(500));
    DisableAllServos();
}

void ServoController::SequenceLaugh() {
    MoveServosSynchronized(kLH_Stand, kRH_Stand, kLL_Neutral, kRL_Neutral, 0.8f);
    for (int i = 0; i < 5; i++) {
        MoveServosSynchronized(kLH_Lift(), kRH_Lift(), -1, -1, 0.8f);
        vTaskDelay(pdMS_TO_TICKS(80));
        MoveServosSynchronized(kLH_Stand, kRH_Stand, -1, -1, 0.8f);
        vTaskDelay(pdMS_TO_TICKS(80));
    }
    MoveServosSynchronized(kLH_Idle(), kRH_Idle(), kLL_Neutral, kRL_Neutral, 0.8f);
    DisableAllServos();
}

void ServoController::SequencePose() {
    MoveServosSynchronized(kLH_Stand, kRH_Stand, kLL_Neutral, kRL_Neutral, 0.8f);
    MoveServosSynchronized(kLH_Stand, kRH_Stand, kLL_Fwd(), kRL_Fwd(), 0.4f);
    vTaskDelay(pdMS_TO_TICKS(3000));
    MoveServosSynchronized(kLH_Stand, kRH_Stand, kLL_Neutral, kRL_Neutral, 0.4f);
    MoveServosSynchronized(kLH_Idle(), kRH_Idle(), kLL_Neutral, kRL_Neutral, 0.8f);
    DisableAllServos();
}

// ================================================================
// Dispatch
// ================================================================
void ServoController::ExecuteSequenceTask(MovementCommand cmd) {
    switch (cmd) {
        case MovementCommand::MOVE_FORWARD:
        case MovementCommand::ADVANCE:
        case MovementCommand::FOLLOW_ME:
        case MovementCommand::COME_HERE:   SequenceWalkForward();  break;
        case MovementCommand::MOVE_BACKWARD:
        case MovementCommand::RETREAT:     SequenceWalkBackward(); break;
        case MovementCommand::TURN_LEFT:
        case MovementCommand::ROTATE_LEFT: SequenceTurnLeft();     break;
        case MovementCommand::TURN_RIGHT:
        case MovementCommand::ROTATE_RIGHT:SequenceTurnRight();    break;
        case MovementCommand::MOVE_CM_10:  SequenceStepForward();  break;
        case MovementCommand::CIRCLE:
            { for (int i = 0; i < 4; i++) SequenceTurnRight(); }  break;
        case MovementCommand::LAUGH:       SequenceLaugh();        break;
        case MovementCommand::POSE:        SequencePose();         break;
        default:                           SequenceReset();        break;
    }
}

void ServoController::MovementTaskWrapper(void* arg) {
    MovementCommand cmd = *(MovementCommand*)arg;
    free(arg);
    ServoController::GetInstance().ExecuteSequenceTask(cmd);
    ServoController::GetInstance().is_moving_.store(false);
    vTaskDelete(NULL);
}

bool ServoController::PerformCommand(MovementCommand cmd) {
    Initialize();
    if (is_moving_.load()) {
        ESP_LOGW(TAG, "Still moving, skipping: %d", (int)cmd);
        return false;
    }
    MovementCommand* arg = (MovementCommand*)malloc(sizeof(MovementCommand));
    if (!arg) return false;
    *arg = cmd;
    is_moving_.store(true);
    if (xTaskCreate(MovementTaskWrapper, "movement_task", 4096, arg, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate failed");
        is_moving_.store(false);
        free(arg);
        return false;
    }
    return true;
}

bool ServoController::PerformCommandBlocking(MovementCommand cmd) {
    Initialize();
    if (is_moving_.load()) {
        ESP_LOGW(TAG, "Still moving, skipping: %d", (int)cmd);
        return false;
    }
    is_moving_.store(true);
    ExecuteSequenceTask(cmd);
    is_moving_.store(false);
    return true;
}
// ================================================================
// Calibration System
// ================================================================

void ServoController::LoadCalibration() {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("servo_calib", NVS_READONLY, &my_handle);
    if (err == ESP_OK) {
        int32_t val;
        if (nvs_get_i32(my_handle, "lh_stand", &val) == ESP_OK) kLH_Stand = val;
        if (nvs_get_i32(my_handle, "rh_stand", &val) == ESP_OK) kRH_Stand = val;
        if (nvs_get_i32(my_handle, "ll_neutral", &val) == ESP_OK) kLL_Neutral = val;
        if (nvs_get_i32(my_handle, "rl_neutral", &val) == ESP_OK) kRL_Neutral = val;
        nvs_close(my_handle);
        ESP_LOGI(TAG, "Calibration loaded: LH=%d, RH=%d, LL=%d, RL=%d", kLH_Stand, kRH_Stand, kLL_Neutral, kRL_Neutral);
    } else {
        ESP_LOGW(TAG, "No calibration found (err=%s). Using defaults.", esp_err_to_name(err));
    }
}

void ServoController::SaveCalibration() {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("servo_calib", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        nvs_set_i32(my_handle, "lh_stand", kLH_Stand);
        nvs_set_i32(my_handle, "rh_stand", kRH_Stand);
        nvs_set_i32(my_handle, "ll_neutral", kLL_Neutral);
        nvs_set_i32(my_handle, "rl_neutral", kRL_Neutral);
        esp_err_t ec = nvs_commit(my_handle);
        nvs_close(my_handle);
        ESP_LOGI(TAG, "Calibration SAVED (commit=%s)", esp_err_to_name(ec));
    } else {
        ESP_LOGE(TAG, "Error opening NVS! %s", esp_err_to_name(err));
    }
}

void ServoController::CalibrationConsoleTask(void* arg) {
    ServoController* ctl = (ServoController*)arg;
    ESP_LOGI("Calib", "--- CALIBRATION MODE ACTIVATED ---");
    ESP_LOGI("Calib", "Keys: q/a(LH), w/s(RH), e/d(LL), r/f(RL), SPACE(reset), S(save)");
    while(1) {
        int c = fgetc(stdin);
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        bool changed = false;
        if (c == 'q') { ctl->kLH_Stand++; changed = true; }
        else if (c == 'a') { ctl->kLH_Stand--; changed = true; }
        else if (c == 'w') { ctl->kRH_Stand++; changed = true; }
        else if (c == 's') { ctl->kRH_Stand--; changed = true; }
        else if (c == 'e') { ctl->kLL_Neutral++; changed = true; }
        else if (c == 'd') { ctl->kLL_Neutral--; changed = true; }
        else if (c == 'r') { ctl->kRL_Neutral++; changed = true; }
        else if (c == 'f') { ctl->kRL_Neutral--; changed = true; }
        else if (c == 'S') { 
            ctl->SaveCalibration(); 
            printf("\r\n[Calib] Saved! LH=%d RH=%d LL=%d RL=%d\r\n", 
                ctl->kLH_Stand, ctl->kRH_Stand, ctl->kLL_Neutral, ctl->kRL_Neutral);
        }
        else if (c == ' ') { 
            ctl->LoadCalibration();
            printf("\r\n[Calib] Reverted to saved! LH=%d RH=%d LL=%d RL=%d\r\n", 
                ctl->kLH_Stand, ctl->kRH_Stand, ctl->kLL_Neutral, ctl->kRL_Neutral);
            ctl->MoveServosSynchronized(ctl->kLH_Stand, ctl->kRH_Stand, ctl->kLL_Neutral, ctl->kRL_Neutral, 0.5f);
            continue;
        }
        else if (c == 'i') { printf("\r\n[Calib] Test Walk Fwd\r\n"); ctl->SequenceWalkForward(); continue; }
        else if (c == 'k') { printf("\r\n[Calib] Test Walk Back\r\n"); ctl->SequenceWalkBackward(); continue; }
        else if (c == 'j') { printf("\r\n[Calib] Test Turn Left\r\n"); ctl->SequenceTurnLeft(); continue; }
        else if (c == 'l') { printf("\r\n[Calib] Test Turn Right\r\n"); ctl->SequenceTurnRight(); continue; }
        else if (c == 'm') { printf("\r\n[Calib] Test Small Step Fwd\r\n"); ctl->SequenceStepForward(); continue; }
        else if (c == 'n') { printf("\r\n[Calib] Test Small Step Back\r\n"); ctl->SequenceStepBackward(); continue; }
        else if (c == 'x') { printf("\r\n[Calib] Relax Servos\r\n"); ctl->DisableAllServos(); continue; }

        if (changed) {
            printf("\r\n[Calib] LIVE: LH=%d RH=%d LL=%d RL=%d\r\n", 
                ctl->kLH_Stand, ctl->kRH_Stand, ctl->kLL_Neutral, ctl->kRL_Neutral);
            // Use MoveServosSynchronized to prevent violent snapping when entering calibration mode
            ctl->MoveServosSynchronized(ctl->kLH_Stand, ctl->kRH_Stand, ctl->kLL_Neutral, ctl->kRL_Neutral, 0.9f);
        }
    }
}

void ServoController::StartCalibrationTask() {
    xTaskCreate(CalibrationConsoleTask, "calib_task", 4096, this, 5, NULL);
}
