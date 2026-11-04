#pragma once

#include "movement/movement_commands.h"
#include "movement/pca9685.h"
#include <atomic>

// ================================================================
// BẢNG CÀI ĐẶT GÓC SERVO (USER SETTINGS)
// Anh có thể thay đổi các giá trị ở đây để test.
// Giá trị PWM (ticks) từ khoảng 100 (~0 độ) đến 540 (~180 độ), giữa là 320 (~90 độ).
// ================================================================

// --- SERVO THÂN (HEIGHT) ---
// Không dùng góc sát biên kiểu "khóa cơ". Servo bị ép ở 0/180 rất dễ rung,
// nóng và làm chân phải mất lực khi bắt đầu nâng.
// Các hằng số mặc định sẽ được định nghĩa bên trong class để có thể thay đổi được (Calibration)


class ServoController {
public:
    static ServoController& GetInstance();
    void Initialize();
    bool PerformCommand(MovementCommand cmd);
    bool PerformCommandBlocking(MovementCommand cmd);
    void Shutdown();

    // Calibration settings
    int kLH_Stand = 440;
    int kRH_Stand = 170;
    int kLL_Neutral = 260;
    int kRL_Neutral = 310;

    void LoadCalibration();
    void SaveCalibration();
    void StartCalibrationTask();

    // Derived settings
    int kLH_Idle() const { return kLH_Stand + 80; }
    int kRH_Idle() const { return kRH_Stand - 85; }
    int kLH_Lift() const { return kLH_Stand + 35; }
    int kRH_Lift() const { return kRH_Stand - 45; }
    
    int kLL_Fwd() const { return kLL_Neutral + 70; }
    int kLL_Back() const { return kLL_Neutral - 70; }
    
    int kRL_Fwd() const { return kRL_Neutral - 70; }
    int kRL_Back() const { return kRL_Neutral + 70; }

private:
    ServoController();
    ~ServoController();

    bool inited_;
    std::atomic<bool> is_moving_;

    void MoveServosSynchronized(int tLH, int tRH, int tLL, int tRL, float speed);
    void DisableAllServos();

    void SequenceWalkForward();
    void SequenceWalkBackward();
    void SequenceStepForward();
    void SequenceStepBackward();
    void SequenceTurnRight();
    void SequenceTurnLeft();
    void SequenceLaugh();
    void SequencePose();
    void SequenceReset();

    int current_positions_[4];

    // ================================================================
    // KÊNH SERVO (PCA9685 CHANNELS)
    // PHẢI CHUẨN THEO LOGIC CODE TARS GỐC!
    // Kênh 0, 1: Servo tay đòn trên cùng (Height)
    // Kênh 2, 3: Servo trục giữa (Swing/Leg)
    // ================================================================
    static const uint8_t CH_LEFT_HEIGHT  = 0;
    static const uint8_t CH_RIGHT_HEIGHT = 1;
    static const uint8_t CH_LEFT_LEG     = 2;
    static const uint8_t CH_RIGHT_LEG    = 3;

    static void MovementTaskWrapper(void* arg);
    static void CalibrationConsoleTask(void* arg);
    void ExecuteSequenceTask(MovementCommand cmd);
};
