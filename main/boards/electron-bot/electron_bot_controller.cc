/*
    Electron Botæœºå™¨äººæŽ§åˆ¶å™¨ - MCPåè®®ç‰ˆæœ¬
*/

#include <cJSON.h>
#include <esp_log.h>

#include <cstring>

#include "application.h"
#include "board.h"
#include "config.h"
#include "mcp_server.h"
#include "movements.h"
#include "sdkconfig.h"
#include "settings.h"

#define TAG "ElectronBotController"

struct ElectronBotActionParams {
    int action_type;
    int steps;
    int speed;
    int direction;
    int amount;
};

class ElectronBotController {
private:
    Otto electron_bot_;
    TaskHandle_t action_task_handle_ = nullptr;
    QueueHandle_t action_queue_;
    bool is_action_in_progress_ = false;

    enum ActionType {
        // æ‰‹éƒ¨åŠ¨ä½œ 1-12
        ACTION_HAND_LEFT_UP = 1,      // ä¸¾å·¦æ‰‹
        ACTION_HAND_RIGHT_UP = 2,     // ä¸¾å³æ‰‹
        ACTION_HAND_BOTH_UP = 3,      // ä¸¾åŒæ‰‹
        ACTION_HAND_LEFT_DOWN = 4,    // æ”¾å·¦æ‰‹
        ACTION_HAND_RIGHT_DOWN = 5,   // æ”¾å³æ‰‹
        ACTION_HAND_BOTH_DOWN = 6,    // æ”¾åŒæ‰‹
        ACTION_HAND_LEFT_WAVE = 7,    // æŒ¥å·¦æ‰‹
        ACTION_HAND_RIGHT_WAVE = 8,   // æŒ¥å³æ‰‹
        ACTION_HAND_BOTH_WAVE = 9,    // æŒ¥åŒæ‰‹
        ACTION_HAND_LEFT_FLAP = 10,   // æ‹æ‰“å·¦æ‰‹
        ACTION_HAND_RIGHT_FLAP = 11,  // æ‹æ‰“å³æ‰‹
        ACTION_HAND_BOTH_FLAP = 12,   // æ‹æ‰“åŒæ‰‹

        // èº«ä½“åŠ¨ä½œ 13-14
        ACTION_BODY_TURN_LEFT = 13,    // å·¦è½¬
        ACTION_BODY_TURN_RIGHT = 14,   // å³è½¬
        ACTION_BODY_TURN_CENTER = 15,  // å›žä¸­å¿ƒ

        // å¤´éƒ¨åŠ¨ä½œ 16-20
        ACTION_HEAD_UP = 16,          // æŠ¬å¤´
        ACTION_HEAD_DOWN = 17,        // ä½Žå¤´
        ACTION_HEAD_NOD_ONCE = 18,    // ç‚¹å¤´ä¸€æ¬¡
        ACTION_HEAD_CENTER = 19,      // å›žä¸­å¿ƒ
        ACTION_HEAD_NOD_REPEAT = 20,  // è¿žç»­ç‚¹å¤´

        // ç³»ç»ŸåŠ¨ä½œ 21
        ACTION_HOME = 21  // å¤ä½åˆ°åˆå§‹ä½ç½®
    };

    static void ActionTask(void* arg) {
        ElectronBotController* controller = static_cast<ElectronBotController*>(arg);
        ElectronBotActionParams params;
        controller->electron_bot_.AttachServos();

        while (true) {
            if (xQueueReceive(controller->action_queue_, &params, pdMS_TO_TICKS(1000)) == pdTRUE) {
                ESP_LOGI(TAG, "æ‰§è¡ŒåŠ¨ä½œ: %d", params.action_type);
                controller->is_action_in_progress_ = true;  // å¼€å§‹æ‰§è¡ŒåŠ¨ä½œ

                // æ‰§è¡Œç›¸åº”çš„åŠ¨ä½œ
                if (params.action_type >= ACTION_HAND_LEFT_UP &&
                    params.action_type <= ACTION_HAND_BOTH_FLAP) {
                    // æ‰‹éƒ¨åŠ¨ä½œ
                    controller->electron_bot_.HandAction(params.action_type, params.steps,
                                                         params.amount, params.speed);
                } else if (params.action_type >= ACTION_BODY_TURN_LEFT &&
                           params.action_type <= ACTION_BODY_TURN_CENTER) {
                    // èº«ä½“åŠ¨ä½œ
                    int body_direction = params.action_type - ACTION_BODY_TURN_LEFT + 1;
                    controller->electron_bot_.BodyAction(body_direction, params.steps,
                                                         params.amount, params.speed);
                } else if (params.action_type >= ACTION_HEAD_UP &&
                           params.action_type <= ACTION_HEAD_NOD_REPEAT) {
                    // å¤´éƒ¨åŠ¨ä½œ
                    int head_action = params.action_type - ACTION_HEAD_UP + 1;
                    controller->electron_bot_.HeadAction(head_action, params.steps, params.amount,
                                                         params.speed);
                } else if (params.action_type == ACTION_HOME) {
                    // å¤ä½åŠ¨ä½œ
                    controller->electron_bot_.Home(true);
                }
                controller->is_action_in_progress_ = false;  // åŠ¨ä½œæ‰§è¡Œå®Œæ¯•
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    void QueueAction(int action_type, int steps, int speed, int direction, int amount) {
        ESP_LOGI(TAG, "åŠ¨ä½œæŽ§åˆ¶: ç±»åž‹=%d, æ­¥æ•°=%d, é€Ÿåº¦=%d, æ–¹å‘=%d, å¹…åº¦=%d", action_type, steps,
                 speed, direction, amount);

        ElectronBotActionParams params = {action_type, steps, speed, direction, amount};
        xQueueSend(action_queue_, &params, portMAX_DELAY);
        StartActionTaskIfNeeded();
    }

    void StartActionTaskIfNeeded() {
        if (action_task_handle_ == nullptr) {
            xTaskCreate(ActionTask, "electron_bot_action", 1024 * 4, this, configMAX_PRIORITIES - 1,
                        &action_task_handle_);
        }
    }

    void LoadTrimsFromNVS() {
        Settings settings("electron_trims", false);

        int right_pitch = settings.GetInt("right_pitch", 0);
        int right_roll = settings.GetInt("right_roll", 0);
        int left_pitch = settings.GetInt("left_pitch", 0);
        int left_roll = settings.GetInt("left_roll", 0);
        int body = settings.GetInt("body", 0);
        int head = settings.GetInt("head", 0);
        electron_bot_.SetTrims(right_pitch, right_roll, left_pitch, left_roll, body, head);
    }

public:
    ElectronBotController() {
        electron_bot_.Init(Right_Pitch_Pin, Right_Roll_Pin, Left_Pitch_Pin, Left_Roll_Pin, Body_Pin,
                           Head_Pin);

        LoadTrimsFromNVS();
        action_queue_ = xQueueCreate(10, sizeof(ElectronBotActionParams));

        QueueAction(ACTION_HOME, 1, 1000, 0, 0);

        RegisterMcpTools();
        ESP_LOGI(TAG, "Electron BotæŽ§åˆ¶å™¨å·²åˆå§‹åŒ–å¹¶æ³¨å†ŒMCPå·¥å…·");
    }

    void RegisterMcpTools() {
        auto& mcp_server = McpServer::GetInstance();

        ESP_LOGI(TAG, "å¼€å§‹æ³¨å†ŒElectron Bot MCPå·¥å…·...");

        // æ‰‹éƒ¨åŠ¨ä½œç»Ÿä¸€å·¥å…·
        mcp_server.AddTool(
            "self.electron.hand_action",
            "æ‰‹éƒ¨åŠ¨ä½œæŽ§åˆ¶ã€‚action: 1=ä¸¾æ‰‹, 2=æ”¾æ‰‹, 3=æŒ¥æ‰‹, 4=æ‹æ‰“; hand: 1=å·¦æ‰‹, 2=å³æ‰‹, 3=åŒæ‰‹; "
            "steps: åŠ¨ä½œé‡å¤æ¬¡æ•°(1-10); speed: åŠ¨ä½œé€Ÿåº¦(500-1500ï¼Œæ•°å€¼è¶Šå°è¶Šå¿«); amount: "
            "åŠ¨ä½œå¹…åº¦(10-50ï¼Œä»…ä¸¾æ‰‹åŠ¨ä½œä½¿ç”¨)",
            PropertyList({Property("action", kPropertyTypeInteger, 1, 1, 4),
                          Property("hand", kPropertyTypeInteger, 3, 1, 3),
                          Property("steps", kPropertyTypeInteger, 1, 1, 10),
                          Property("speed", kPropertyTypeInteger, 1000, 500, 1500),
                          Property("amount", kPropertyTypeInteger, 30, 10, 50)}),
            [this](const PropertyList& properties) -> ReturnValue {
                int action_type = properties["action"].value<int>();
                int hand_type = properties["hand"].value<int>();
                int steps = properties["steps"].value<int>();
                int speed = properties["speed"].value<int>();
                int amount = properties["amount"].value<int>();

                // æ ¹æ®åŠ¨ä½œç±»åž‹å’Œæ‰‹éƒ¨ç±»åž‹è®¡ç®—å…·ä½“åŠ¨ä½œ
                int base_action;
                switch (action_type) {
                    case 1:
                        base_action = ACTION_HAND_LEFT_UP;
                        break;  // ä¸¾æ‰‹
                    case 2:
                        base_action = ACTION_HAND_LEFT_DOWN;
                        amount = 0;
                        break;  // æ”¾æ‰‹
                    case 3:
                        base_action = ACTION_HAND_LEFT_WAVE;
                        amount = 0;
                        break;  // æŒ¥æ‰‹
                    case 4:
                        base_action = ACTION_HAND_LEFT_FLAP;
                        amount = 0;
                        break;  // æ‹æ‰“
                    default:
                        base_action = ACTION_HAND_LEFT_UP;
                }
                int action_id = base_action + (hand_type - 1);

                QueueAction(action_id, steps, speed, 0, amount);
                return true;
            });

        // èº«ä½“åŠ¨ä½œ
        mcp_server.AddTool(
            "self.electron.body_turn",
            "èº«ä½“è½¬å‘ã€‚steps: è½¬å‘æ­¥æ•°(1-10); speed: è½¬å‘é€Ÿåº¦(500-1500ï¼Œæ•°å€¼è¶Šå°è¶Šå¿«); direction: "
            "è½¬å‘æ–¹å‘(1=å·¦è½¬, 2=å³è½¬, 3=å›žä¸­å¿ƒ); angle: è½¬å‘è§’åº¦(0-90åº¦)",
            PropertyList({Property("steps", kPropertyTypeInteger, 1, 1, 10),
                          Property("speed", kPropertyTypeInteger, 1000, 500, 1500),
                          Property("direction", kPropertyTypeInteger, 1, 1, 3),
                          Property("angle", kPropertyTypeInteger, 45, 0, 90)}),
            [this](const PropertyList& properties) -> ReturnValue {
                int steps = properties["steps"].value<int>();
                int speed = properties["speed"].value<int>();
                int direction = properties["direction"].value<int>();
                int amount = properties["angle"].value<int>();

                int action;
                switch (direction) {
                    case 1:
                        action = ACTION_BODY_TURN_LEFT;
                        break;
                    case 2:
                        action = ACTION_BODY_TURN_RIGHT;
                        break;
                    case 3:
                        action = ACTION_BODY_TURN_CENTER;
                        break;
                    default:
                        action = ACTION_BODY_TURN_LEFT;
                }

                QueueAction(action, steps, speed, 0, amount);
                return true;
            });

        // å¤´éƒ¨åŠ¨ä½œ
        mcp_server.AddTool("self.electron.head_move",
                           "å¤´éƒ¨è¿åŠ¨ã€‚action: 1=æŠ¬å¤´, 2=ä½Žå¤´, 3=ç‚¹å¤´, 4=å›žä¸­å¿ƒ, 5=è¿žç»­ç‚¹å¤´; steps: "
                           "åŠ¨ä½œé‡å¤æ¬¡æ•°(1-10); speed: åŠ¨ä½œé€Ÿåº¦(500-1500ï¼Œæ•°å€¼è¶Šå°è¶Šå¿«); angle: "
                           "å¤´éƒ¨è½¬åŠ¨è§’åº¦(1-15åº¦)",
                           PropertyList({Property("action", kPropertyTypeInteger, 3, 1, 5),
                                         Property("steps", kPropertyTypeInteger, 1, 1, 10),
                                         Property("speed", kPropertyTypeInteger, 1000, 500, 1500),
                                         Property("angle", kPropertyTypeInteger, 5, 1, 15)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int action_num = properties["action"].value<int>();
                               int steps = properties["steps"].value<int>();
                               int speed = properties["speed"].value<int>();
                               int amount = properties["angle"].value<int>();
                               int action = ACTION_HEAD_UP + (action_num - 1);
                               QueueAction(action, steps, speed, 0, amount);
                               return true;
                           });

        // ç³»ç»Ÿå·¥å…·
        mcp_server.AddTool("self.electron.stop", "ç«‹å³åœæ­¢", PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               // æ¸…ç©ºé˜Ÿåˆ—ä½†ä¿æŒä»»åŠ¡å¸¸é©»
                               xQueueReset(action_queue_);
                               is_action_in_progress_ = false;
                               QueueAction(ACTION_HOME, 1, 1000, 0, 0);
                               return true;
                           });

        mcp_server.AddTool("self.electron.get_status", "èŽ·å–æœºå™¨äººçŠ¶æ€ï¼Œè¿”å›ž moving æˆ– idle",
                           PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                               return is_action_in_progress_ ? "moving" : "idle";
                           });

        // å•ä¸ªèˆµæœºæ ¡å‡†å·¥å…·
        mcp_server.AddTool(
            "self.electron.set_trim",
            "æ ¡å‡†å•ä¸ªèˆµæœºä½ç½®ã€‚è®¾ç½®æŒ‡å®šèˆµæœºçš„å¾®è°ƒå‚æ•°ä»¥è°ƒæ•´ElectronBotçš„åˆå§‹å§¿æ€ï¼Œè®¾ç½®å°†æ°¸ä¹…ä¿å­˜ã€‚"
            "servo_type: èˆµæœºç±»åž‹(right_pitch:å³è‡‚æ—‹è½¬, right_roll:å³è‡‚æŽ¨æ‹‰, left_pitch:å·¦è‡‚æ—‹è½¬, "
            "left_roll:å·¦è‡‚æŽ¨æ‹‰, body:èº«ä½“, head:å¤´éƒ¨); "
            "trim_value: å¾®è°ƒå€¼(-30åˆ°30åº¦)",
            PropertyList({Property("servo_type", kPropertyTypeString, "right_pitch"),
                          Property("trim_value", kPropertyTypeInteger, 0, -30, 30)}),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string servo_type = properties["servo_type"].value<std::string>();
                int trim_value = properties["trim_value"].value<int>();

                ESP_LOGI(TAG, "è®¾ç½®èˆµæœºå¾®è°ƒ: %s = %dåº¦", servo_type.c_str(), trim_value);

                // èŽ·å–å½“å‰æ‰€æœ‰å¾®è°ƒå€¼
                Settings settings("electron_trims", true);
                int right_pitch = settings.GetInt("right_pitch", 0);
                int right_roll = settings.GetInt("right_roll", 0);
                int left_pitch = settings.GetInt("left_pitch", 0);
                int left_roll = settings.GetInt("left_roll", 0);
                int body = settings.GetInt("body", 0);
                int head = settings.GetInt("head", 0);

                // æ›´æ–°æŒ‡å®šèˆµæœºçš„å¾®è°ƒå€¼
                if (servo_type == "right_pitch") {
                    right_pitch = trim_value;
                    settings.SetInt("right_pitch", right_pitch);
                } else if (servo_type == "right_roll") {
                    right_roll = trim_value;
                    settings.SetInt("right_roll", right_roll);
                } else if (servo_type == "left_pitch") {
                    left_pitch = trim_value;
                    settings.SetInt("left_pitch", left_pitch);
                } else if (servo_type == "left_roll") {
                    left_roll = trim_value;
                    settings.SetInt("left_roll", left_roll);
                } else if (servo_type == "body") {
                    body = trim_value;
                    settings.SetInt("body", body);
                } else if (servo_type == "head") {
                    head = trim_value;
                    settings.SetInt("head", head);
                } else {
                    return "é”™è¯¯ï¼šæ— æ•ˆçš„èˆµæœºç±»åž‹ï¼Œè¯·ä½¿ç”¨: right_pitch, right_roll, left_pitch, "
                           "left_roll, body, head";
                }

                electron_bot_.SetTrims(right_pitch, right_roll, left_pitch, left_roll, body, head);

                QueueAction(ACTION_HOME, 1, 500, 0, 0);

                return "èˆµæœº " + servo_type + " å¾®è°ƒè®¾ç½®ä¸º " + std::to_string(trim_value) +
                       " åº¦ï¼Œå·²æ°¸ä¹…ä¿å­˜";
            });

        mcp_server.AddTool("self.electron.get_trims", "èŽ·å–å½“å‰çš„èˆµæœºå¾®è°ƒè®¾ç½®", PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               Settings settings("electron_trims", false);

                               int right_pitch = settings.GetInt("right_pitch", 0);
                               int right_roll = settings.GetInt("right_roll", 0);
                               int left_pitch = settings.GetInt("left_pitch", 0);
                               int left_roll = settings.GetInt("left_roll", 0);
                               int body = settings.GetInt("body", 0);
                               int head = settings.GetInt("head", 0);

                               std::string result =
                                   "{\"right_pitch\":" + std::to_string(right_pitch) +
                                   ",\"right_roll\":" + std::to_string(right_roll) +
                                   ",\"left_pitch\":" + std::to_string(left_pitch) +
                                   ",\"left_roll\":" + std::to_string(left_roll) +
                                   ",\"body\":" + std::to_string(body) +
                                   ",\"head\":" + std::to_string(head) + "}";

                               ESP_LOGI(TAG, "èŽ·å–å¾®è°ƒè®¾ç½®: %s", result.c_str());
                               return result;
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

        ESP_LOGI(TAG, "Electron Bot MCPå·¥å…·æ³¨å†Œå®Œæˆ");
    }

    ~ElectronBotController() {
        if (action_task_handle_ != nullptr) {
            vTaskDelete(action_task_handle_);
            action_task_handle_ = nullptr;
        }
        vQueueDelete(action_queue_);
    }
};

static ElectronBotController* g_electron_controller = nullptr;

void InitializeElectronBotController() {
    if (g_electron_controller == nullptr) {
        g_electron_controller = new ElectronBotController();
        ESP_LOGI(TAG, "Electron BotæŽ§åˆ¶å™¨å·²åˆå§‹åŒ–å¹¶æ³¨å†ŒMCPå·¥å…·");
    }
}
