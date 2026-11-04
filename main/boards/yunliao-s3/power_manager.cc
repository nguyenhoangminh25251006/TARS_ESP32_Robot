#include "power_manager.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "esp_log.h"
#include "config.h"
#include <esp_sleep.h>
#include "esp_log.h"
#include "settings.h"

#define TAG "PowerManager"

static QueueHandle_t gpio_evt_queue = NULL;
uint16_t battCnt;//é—ªç¯æ¬¡æ•°
int battLife = 70; //ç”µé‡

// ä¸­æ–­æœåŠ¡ç¨‹åº
static void IRAM_ATTR batt_mon_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// æ·»åŠ ä»»åŠ¡å¤„ç†å‡½æ•°
static void batt_mon_task(void* arg) {
    uint32_t io_num;
    while(1) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            battCnt++;
        }
    }
}

static void calBattLife() {
    // è®¡ç®—ç”µé‡
    battLife = battCnt;

    if (battLife > 100){
        battLife = 100;
    }
    // ESP_LOGI(TAG, "Battery life:%d", (int)battLife);
    // é‡ç½®è®¡æ•°å™¨
    battCnt = 0;
}

PowerManager::PowerManager(){
}

void PowerManager::Initialize(){
    // åˆå§‹åŒ–5VæŽ§åˆ¶å¼•è„š
    gpio_config_t io_conf_5v = {
        .pin_bit_mask = 1<<BOOT_5V_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf_5v));

    // åˆå§‹åŒ–4GæŽ§åˆ¶å¼•è„š
    gpio_config_t io_conf_4g = {
        .pin_bit_mask = 1<<BOOT_4G_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf_4g));

    // ç”µæ± ç”µé‡ç›‘æµ‹å¼•è„šé…ç½®
    gpio_config_t io_conf_batt_mon = {
        .pin_bit_mask = 1ull<<MON_BATT_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf_batt_mon));
    // åˆ›å»ºç”µé‡GPIOäº‹ä»¶é˜Ÿåˆ—
    gpio_evt_queue = xQueueCreate(2, sizeof(uint32_t));
    // å®‰è£…ç”µé‡GPIO ISRæœåŠ¡
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    // æ·»åŠ ä¸­æ–­å¤„ç†
    ESP_ERROR_CHECK(gpio_isr_handler_add(MON_BATT_PIN, batt_mon_isr_handler, (void*)MON_BATT_PIN));
     // åˆ›å»ºç›‘æŽ§ä»»åŠ¡
    xTaskCreate(&batt_mon_task, "batt_mon_task", 1024, NULL, 10, NULL);

    // åˆå§‹åŒ–ç›‘æµ‹å¼•è„š
    gpio_config_t mon_conf = {};
    mon_conf.pin_bit_mask = 1ULL << MON_USB_PIN;
    mon_conf.mode = GPIO_MODE_INPUT;
    mon_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    mon_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&mon_conf);

    // åˆ›å»ºç”µæ± ç”µé‡æ£€æŸ¥å®šæ—¶å™¨
    esp_timer_create_args_t timer_args = {
        .callback = [](void* arg) {
            PowerManager* self = static_cast<PowerManager*>(arg);
            self->CheckBatteryStatus();
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "battery_check_timer",
        .skip_unhandled_events = true,
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle_));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handle_, 1000000));
}

void PowerManager::CheckBatteryStatus(){
    call_count_++;
    if(call_count_ >= MON_BATT_CNT) {
        calBattLife();
        call_count_ = 0;
    }

    bool new_charging_status = IsCharging();
    if (new_charging_status != is_charging_) {
        is_charging_ = new_charging_status;
        if (charging_callback_) {
            charging_callback_(is_charging_);
        }
    }

    bool new_discharging_status = IsDischarging();
    if (new_discharging_status != is_discharging_) {
        is_discharging_ = new_discharging_status;
        if (discharging_callback_) {
            discharging_callback_(is_discharging_);
        }
    }
}

bool PowerManager::IsCharging() {
    return gpio_get_level(MON_USB_PIN) == 1 && !IsChargingDone();
}

bool PowerManager::IsDischarging() {
    return gpio_get_level(MON_USB_PIN) == 0;
}

bool PowerManager::IsChargingDone() {
    return battLife >= 95;
}

int PowerManager::GetBatteryLevel() {
    return battLife; 
}

void PowerManager::OnChargingStatusChanged(std::function<void(bool)> callback) {
    charging_callback_ = callback;
}

void PowerManager::OnChargingStatusDisChanged(std::function<void(bool)> callback) {
    discharging_callback_ = callback;
}

void PowerManager::CheckStartup() {
    Settings settings1("board", true);
    if(settings1.GetInt("sleep_flag", 0) > 0){
        vTaskDelay(pdMS_TO_TICKS(1000));
        if( gpio_get_level(BOOT_BUTTON_PIN) == 1) {
            Sleep(); //è¿›å…¥ä¼‘çœ æ¨¡å¼
        }else{
            settings1.SetInt("sleep_flag", 0);
        }
    }
}

void PowerManager::Start5V() {
    gpio_set_level(BOOT_5V_PIN, 1);
}

void PowerManager::Shutdown5V() {
    gpio_set_level(BOOT_5V_PIN, 0);
}

void PowerManager::Start4G() {
    gpio_set_level(BOOT_4G_PIN, 1);
}

void PowerManager::Shutdown4G() {
    gpio_set_level(BOOT_4G_PIN, 0);
    gpio_set_level(ML307_RX_PIN,1);
    gpio_set_level(ML307_TX_PIN,1);
}

void PowerManager::Sleep() {
    ESP_LOGI(TAG, "Entering deep sleep");
    Settings settings("board", true);
    settings.SetInt("sleep_flag", 1);
    Shutdown4G();
    Shutdown5V();

    if(gpio_evt_queue) {
        vQueueDelete(gpio_evt_queue);
        gpio_evt_queue = NULL;
    }
    ESP_ERROR_CHECK(gpio_isr_handler_remove(BOOT_BUTTON_PIN));
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(BOOT_BUTTON_PIN, 0));
    ESP_ERROR_CHECK(rtc_gpio_pulldown_dis(BOOT_BUTTON_PIN));
    ESP_ERROR_CHECK(rtc_gpio_pullup_en(BOOT_BUTTON_PIN));
    esp_deep_sleep_start();
} 
