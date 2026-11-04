#include "sscma_camera.h"
#include "mcp_server.h"
#include "lvgl_display.h"
#include "lvgl_image.h"
#include "board.h"
#include "system_info.h"
#include "config.h"
#include "settings.h"

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <cstring>
#include "application.h"
#include "sscma_client_commands.h"

#define TAG "SscmaCamera"

#define IMG_JPEG_BUF_SIZE   48 * 1024

static bool __himax_keepalive_check(sscma_client_handle_t client)
{
    esp_err_t ret = ESP_OK;
    sscma_client_reply_t reply = {0};
    int retry = 3;
    while(retry--) {
        ret = sscma_client_request(client, CMD_PREFIX CMD_AT_ID CMD_QUERY CMD_SUFFIX, &reply, true, pdMS_TO_TICKS(2000));
        if (reply.payload != NULL) {
            sscma_client_reply_clear(&reply);
        }
        if( ret != ESP_OK ) {
            ESP_LOGE(TAG, "Himax keepalive check failed: %d", ret);
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            return true;
        }
    }
    return false;
}

SscmaCamera::SscmaCamera(esp_io_expander_handle_t io_exp_handle) {
    sscma_client_io_spi_config_t spi_io_config = {0};
    spi_io_config.sync_gpio_num = BSP_SSCMA_CLIENT_SPI_SYNC;
    spi_io_config.cs_gpio_num = BSP_SSCMA_CLIENT_SPI_CS;
    spi_io_config.pclk_hz = BSP_SSCMA_CLIENT_SPI_CLK;
    spi_io_config.spi_mode = 0;
    spi_io_config.wait_delay = 10; //ä¸¤ä¸ªtransferä¹‹é—´è‡³å°‘å»¶æ—¶4ms,ä½†å½“å‰ FREERTOS_HZ=100, å»¶æ—¶ç²¾åº¦åªèƒ½è¾¾åˆ°10ms, 
    spi_io_config.user_ctx = NULL;
    spi_io_config.io_expander = io_exp_handle;
    spi_io_config.flags.sync_use_expander = BSP_SSCMA_CLIENT_RST_USE_EXPANDER;

    sscma_client_new_io_spi_bus((sscma_client_spi_bus_handle_t)BSP_SSCMA_CLIENT_SPI_NUM, &spi_io_config, &sscma_client_io_handle_);

    sscma_client_config_t sscma_client_config = SSCMA_CLIENT_CONFIG_DEFAULT();
    sscma_client_config.event_queue_size = CONFIG_SSCMA_EVENT_QUEUE_SIZE;
    sscma_client_config.tx_buffer_size = CONFIG_SSCMA_TX_BUFFER_SIZE;
    sscma_client_config.rx_buffer_size = CONFIG_SSCMA_RX_BUFFER_SIZE;
    sscma_client_config.process_task_stack = CONFIG_SSCMA_PROCESS_TASK_STACK_SIZE;
    sscma_client_config.process_task_affinity = CONFIG_SSCMA_PROCESS_TASK_AFFINITY;
    sscma_client_config.process_task_priority = CONFIG_SSCMA_PROCESS_TASK_PRIORITY;
    sscma_client_config.monitor_task_stack = CONFIG_SSCMA_MONITOR_TASK_STACK_SIZE;
    sscma_client_config.monitor_task_affinity = CONFIG_SSCMA_MONITOR_TASK_AFFINITY;
    sscma_client_config.monitor_task_priority = CONFIG_SSCMA_MONITOR_TASK_PRIORITY;
    sscma_client_config.reset_gpio_num = BSP_SSCMA_CLIENT_RST;
    sscma_client_config.io_expander = io_exp_handle;
    sscma_client_config.flags.reset_use_expander = BSP_SSCMA_CLIENT_RST_USE_EXPANDER;

    sscma_client_new(sscma_client_io_handle_, &sscma_client_config, &sscma_client_handle_);

    sscma_data_queue_ = xQueueCreate(1, sizeof(SscmaData));

    sscma_client_callback_t callback = {0};

    detection_state = SscmaCamera::IDLE;
    state_start_time = 0;
    need_start_cooldown = false;
    callback.on_event = [](sscma_client_handle_t client, const sscma_client_reply_t *reply, void *user_ctx) {
        SscmaCamera* self = static_cast<SscmaCamera*>(user_ctx);
        if (!self) return;
        
        char *img = NULL;
        int img_size = 0;
        int box_count = 0;
        sscma_client_box_t     *boxes = NULL;
        int class_count = 0;
        sscma_client_class_t *classes = NULL;
        int point_count = 0;
        sscma_client_point_t  *points = NULL;
        int model_type = 0;
        int obj_cnt = 0;

        int width = 0, height = 0;
        cJSON *data = cJSON_GetObjectItem(reply->payload, "data");
        if (data != NULL && cJSON_IsObject(data)) {
            cJSON *resolution = cJSON_GetObjectItem(data, "resolution");
            if (data != NULL && cJSON_IsArray(resolution) && cJSON_GetArraySize(resolution) == 2) {
                width = cJSON_GetArrayItem(resolution, 0)->valueint;
                height = cJSON_GetArrayItem(resolution, 1)->valueint;
            }
        }
        
        switch ((width+height)) {
            case (416+416): 
            {
                bool is_object_detected = false;
                bool is_need_wake = false;
                
                // å®šæœŸæ›´æ–°æ£€æµ‹é…ç½®å‚æ•°ï¼Œé¿å…é¢‘ç¹NVSè®¿é—®
                int64_t cur_tm = esp_timer_get_time();

                // å°è¯•èŽ·å–æ£€æµ‹æ¡†æ•°æ®ï¼ˆç›®æ ‡æ£€æµ‹æ¨¡åž‹ï¼‰
                if (sscma_utils_fetch_boxes_from_reply(reply, &boxes, &box_count) == ESP_OK && box_count > 0) {
                    for (int i = 0; i < box_count; i++) {
                        ESP_LOGI(TAG, "[box %d]: x=%d, y=%d, w=%d, h=%d, score=%d, target=%d", i,  \
                                boxes[i].x, boxes[i].y, boxes[i].w, boxes[i].h, boxes[i].score, boxes[i].target);
                        if (boxes[i].target == self->detect_target && boxes[i].score > self->detect_threshold) {
                           is_object_detected = true;
                           model_type = 0;
                           obj_cnt++;
                           break;
                        }
                    }
                    free(boxes);
                } else if (sscma_utils_fetch_classes_from_reply(reply, &classes, &class_count) == ESP_OK && class_count > 0) {
                    // å°è¯•èŽ·å–åˆ†ç±»æ•°æ®ï¼ˆåˆ†ç±»æ¨¡åž‹ï¼‰
                    for (int i = 0; i < class_count; i++) {
                        ESP_LOGI(TAG, "[class %d]: target=%d, score=%d", i,
                                classes[i].target, classes[i].score);
                        if (classes[i].target == self->detect_target && classes[i].score > self->detect_threshold) {
                           is_object_detected = true;
                           model_type = 1;
                           obj_cnt++;
                        }
                    }
                    free(classes);
                } else if (sscma_utils_fetch_points_from_reply(reply, &points, &point_count) == ESP_OK && point_count > 0) {
                     // å°è¯•èŽ·å–å…³é”®ç‚¹æ•°æ®ï¼ˆå§¿æ€ä¼°è®¡æ¨¡åž‹ï¼‰
                    for (int i = 0; i < point_count; i++) {
                        ESP_LOGI(TAG, "[point %d]: x=%d, y=%d, z=%d, score=%d, target=%d", i, 
                                points[i].x, points[i].y, points[i].z, points[i].score, points[i].target);
                        if (points[i].target == self->detect_target && points[i].score > self->detect_threshold) {
                           is_object_detected = true;
                           model_type = 2;
                           obj_cnt++;
                        }
                    }
                    free(points);
                }

                // å¦‚æžœéœ€è¦å¼€å§‹å†·å´æœŸï¼ŒçŽ°åœ¨å¼€å§‹è®¡æ—¶
                if (self->need_start_cooldown) { // å›žè°ƒæš‚åœï¼Œæ ‡å¿—ä¿æŒï¼Œç­‰å¾…å›žè°ƒæ¢å¤åŽå¼€å§‹è®¡æ—¶
                    self->state_start_time = cur_tm;
                    self->need_start_cooldown = false;
                    ESP_LOGI(TAG, "Starting cooldown timer");
                }
                
                // çŠ¶æ€æœºé©±åŠ¨çš„æ£€æµ‹é€»è¾‘ - åªåœ¨äººå‘˜å‡ºçŽ°æ—¶è§¦å‘
                switch (self->detection_state) {
                    case SscmaCamera::IDLE:
                        if (is_object_detected) {
                            // äººå‘˜å‡ºçŽ°ï¼Œå¼€å§‹éªŒè¯ï¼ˆè¿™æ˜¯ä»Žæ— åˆ°æœ‰çš„è½¬æ¢ï¼‰
                            self->detection_state = SscmaCamera::VALIDATING;
                            self->state_start_time = cur_tm; // è®°å½•ç‰©ä½“å‡ºçŽ°æ—¶é—´
                            self->last_detected_time = cur_tm; // åˆå§‹åŒ–æœ€åŽæ£€æµ‹æ—¶é—´
                            ESP_LOGI(TAG, "object appeared, starting validation");
                        }
                        break;
                        
                    case SscmaCamera::VALIDATING:
                        if (is_object_detected) {
                            // æ›´æ–°æœ€åŽæ£€æµ‹åˆ°çš„æ—¶é—´
                            self->last_detected_time = cur_tm;
                            // æ£€æŸ¥æ˜¯å¦éªŒè¯è¶³å¤Ÿæ—¶é—´
                            if ((cur_tm - self->state_start_time) >= (self->detect_duration_sec * 1000000)) {
                                is_need_wake = true;
                            }
                        } else {
                            // éªŒè¯æœŸé—´äººå‘˜ç¦»å¼€ï¼Œæ£€æŸ¥åŽ»æŠ–åŠ¨æ—¶é—´
                            if (self->last_detected_time > 0 && 
                                (cur_tm - self->last_detected_time) >= self->detect_debounce_sec * 1000000LL) {
                                // åŽ»æŠ–åŠ¨æ—¶é—´å·²è¿‡ï¼Œç¡®è®¤äººå‘˜å·²ç¦»å¼€ï¼Œå›žåˆ°ç©ºé—²
                                self->detection_state = SscmaCamera::IDLE;
                                self->last_detected_time = 0;
                                ESP_LOGI(TAG, "object left during validation (debounced), back to idle");
                            }
                        }
                        break;
                        
                    case SscmaCamera::COOLDOWN:
                        // å†·å´æœŸï¼Œéœ€è¦æ»¡è¶³ä¸¤ä¸ªæ¡ä»¶ï¼š1)objectç¦»å¼€ 2)è¿‡äº†15ç§’
                        if (!is_object_detected && 
                            (cur_tm - self->state_start_time) >= (self->detect_invoke_interval_sec * 1000000LL)) {
                            // objectç¦»å¼€ä¸”å†·å´æ—¶é—´åˆ°ï¼Œå›žåˆ°ç©ºé—²çŠ¶æ€
                            self->detection_state = SscmaCamera::IDLE;
                            ESP_LOGI(TAG, "Cooldown complete and object left, back to idle - ready for next appearance");
                        }
                        // å…¶ä»–æƒ…å†µç»§ç»­ä¿æŒå†·å´çŠ¶æ€
                        break;
                }


                if( is_need_wake ) {
                    ESP_LOGI(TAG, "Validation complete, triggering conversation (type=%d, res=%dx%d)", 
                             self->detect_target, width, height);
                    
                    // è§¦å‘å¯¹è¯
                    std::string wake_word;
                    if ( model_type  == 0 ) {
                        std::string cached_target_name = "object";
                        if( self->model != NULL && self->model->classes[self->detect_target] != NULL ) {
                            cached_target_name = self->model->classes[self->detect_target];
                        }
                        wake_word = "<detect>" + std::to_string(obj_cnt) + " " + cached_target_name + " detected </detect>";
                    } else if ( model_type  == 1 ) {
                        std::string cached_target_name = "object";
                        if( self->model != NULL && self->model->classes[self->detect_target] != NULL ) {
                            cached_target_name = self->model->classes[self->detect_target];
                        }
                        wake_word = "<detect>" + std::to_string(obj_cnt) + " " + cached_target_name + " detected </detect>";
                    } else if ( model_type  == 2 ) {
                        std::string cached_target_name = "object";
                        if( self->model != NULL && self->model->classes[self->detect_target] != NULL ) {
                            cached_target_name = self->model->classes[self->detect_target];
                        }
                        wake_word = "<detect>" + std::to_string(obj_cnt) + " " + cached_target_name + " detected </detect>";
                    }
                    printf("wake_word:%s\n", wake_word.c_str());
                    Application::GetInstance().WakeWordInvoke(wake_word);
                    
                    // è¿›å…¥å†·å´çŠ¶æ€ï¼Œæ ‡è®°éœ€è¦å¼€å§‹å†·å´æœŸï¼›å¦‚ä¸‹å˜é‡å°†åœ¨ä¼šè¯ç»“æŸåŽè¢«ä½¿ç”¨ï¼Œç­‰å¾…å›žè°ƒæ¢å¤åŽå¼€å§‹è®¡æ—¶
                    self->detection_state = SscmaCamera::COOLDOWN;
                    self->need_start_cooldown = true;
                }
            }
                break;
            case (640+480):

                if (sscma_utils_fetch_image_from_reply(reply, &img, &img_size) == ESP_OK)
                {
                    ESP_LOGI(TAG, "image_size: %d\n", img_size);
                    // å°†æ•°æ®é€šè¿‡é˜Ÿåˆ—å‘é€å‡ºåŽ»
                    SscmaData data;
                    data.img = (uint8_t*)img;
                    data.len = img_size;

                    // æ¸…ç©ºé˜Ÿåˆ—ï¼Œä¿è¯åªä¿å­˜æœ€æ–°çš„æ•°æ®
                    SscmaData dummy;
                    while (xQueueReceive(self->sscma_data_queue_, &dummy, 0) == pdPASS) {
                        if (dummy.img) {
                            heap_caps_free(dummy.img);
                        }
                    }
                    xQueueSend(self->sscma_data_queue_, &data, 0);
                    // æ³¨æ„ï¼šimg çš„é‡Šæ”¾ç”±æŽ¥æ”¶æ–¹è´Ÿè´£
                }
                break;
            default:
                ESP_LOGI(TAG, "unknown resolution");
                break;
        }
    };
    callback.on_connect = [](sscma_client_handle_t client, const sscma_client_reply_t *reply, void *user_ctx) {
        ESP_LOGI(TAG, "SSCMA client connected");
        SscmaCamera* self = static_cast<SscmaCamera*>(user_ctx);
        if (self) {
            self->sscma_restarted_ = true;
        }
    };

    callback.on_log = [](sscma_client_handle_t client, const sscma_client_reply_t *reply, void *user_ctx) {
        ESP_LOGI(TAG, "log: %s\n", reply->data);
    };

    sscma_client_register_callback(sscma_client_handle_, &callback, this);

    sscma_client_init(sscma_client_handle_);

    ESP_LOGI(TAG, "SSCMA client initialized");
    // è®¾ç½®åˆ†è¾¨çŽ‡
    // 3 = 640x480
    if (sscma_client_set_sensor(sscma_client_handle_, 1, 3, true)) {
        ESP_LOGE(TAG, "Failed to set sensor");
        sscma_client_del(sscma_client_handle_);
        sscma_client_handle_ = NULL;
        return;
    }

    // èŽ·å–è®¾å¤‡ä¿¡æ¯
    sscma_client_info_t *info;
    if (sscma_client_get_info(sscma_client_handle_, &info, true) == ESP_OK) {
        ESP_LOGI(TAG, "Device Info - ID: %s, Name: %s", 
            info->id ? info->id : "NULL", 
            info->name ? info->name : "NULL");
    }
    // åˆå§‹åŒ–JPEGæ•°æ®çš„å†…å­˜
    jpeg_data_.len = 0;
    jpeg_data_.buf = (uint8_t*)heap_caps_malloc(IMG_JPEG_BUF_SIZE, MALLOC_CAP_SPIRAM);;
    if ( jpeg_data_.buf == nullptr ) {
        ESP_LOGE(TAG, "Failed to allocate memory for JPEG buffer");
        return;
    }

    //åˆå§‹åŒ–JPEGè§£ç 
    jpeg_error_t err;
    jpeg_dec_config_t config = { .output_type = JPEG_PIXEL_FORMAT_RGB565_LE, .rotate = JPEG_ROTATE_0D };
    err = jpeg_dec_open(&config, &jpeg_dec_);
    if ( err != JPEG_ERR_OK ) {
        ESP_LOGE(TAG, "Failed to open JPEG decoder");
        return;
    }
    jpeg_io_ = (jpeg_dec_io_t*)heap_caps_malloc(sizeof(jpeg_dec_io_t), MALLOC_CAP_SPIRAM);
    if (!jpeg_io_) {
        ESP_LOGE(TAG, "Failed to allocate memory for JPEG IO");
        jpeg_dec_close(jpeg_dec_);
        return;
    }
    memset(jpeg_io_, 0, sizeof(jpeg_dec_io_t));

    jpeg_out_ = (jpeg_dec_header_info_t*)heap_caps_aligned_alloc(16, sizeof(jpeg_dec_header_info_t), MALLOC_CAP_SPIRAM);
    if (!jpeg_out_) {
        ESP_LOGE(TAG, "Failed to allocate memory for JPEG output header");
        heap_caps_free(jpeg_io_);
        jpeg_dec_close(jpeg_dec_);
        return;
    }
    memset(jpeg_out_, 0, sizeof(jpeg_dec_header_info_t));

    // åˆå§‹åŒ–é¢„è§ˆå›¾ç‰‡çš„å†…å­˜
    memset(&preview_image_, 0, sizeof(preview_image_));
    preview_image_.header.magic = LV_IMAGE_HEADER_MAGIC;
    preview_image_.header.cf = LV_COLOR_FORMAT_RGB565;
    preview_image_.header.flags = LV_IMAGE_FLAGS_ALLOCATED | LV_IMAGE_FLAGS_MODIFIABLE;
    preview_image_.header.w = 640;
    preview_image_.header.h = 480;

    preview_image_.header.stride = preview_image_.header.w * 2;
    preview_image_.data_size = preview_image_.header.w * preview_image_.header.h * 2;
    preview_image_.data =(uint8_t*)jpeg_calloc_align(preview_image_.data_size, 16);
    if (preview_image_.data == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate memory for preview image");
        return;
    }

    sscma_client_set_model(sscma_client_handle_, 4);
    model_class_cnt = 0;
    if (sscma_client_get_model(sscma_client_handle_, &model, true) == ESP_OK) {
        printf("ID: %d\n", model->id ? model->id : -1);
        printf("UUID: %s\n", model->uuid ? model->uuid : "N/A");
        printf("Name: %s\n", model->name ? model->name : "N/A");
        printf("Version: %s\n", model->ver ? model->ver : "N/A");
        printf("URL: %s\n", model->url ? model->url : "N/A");
        printf("Checksum: %s\n", model->checksum ? model->checksum : "N/A");
        printf("Classes:\n");
        if (model->classes[0] != NULL)
        {
            for (int i = 0; model->classes[i] != NULL; i++)
            {
                printf("  - %s\n", model->classes[i]);
                model_class_cnt++;
            }
        } else {
            printf("  N/A\n");
        }
    } else {
        printf("get model failed\n");
    }

    ESP_LOGI(TAG, "initialize mcp tools");
    InitializeMcpTools();

    xTaskCreate([](void* arg) {
        auto this_ = (SscmaCamera*)arg;
        bool is_inference = false;
        int64_t last_keepalive_time = esp_timer_get_time();
        while (true)
        {
            if (this_->sscma_restarted_) {
                ESP_LOGI(TAG, "SSCMA restarted detected");
                this_->sscma_restarted_ = false;
                is_inference = false;
            }

            if (esp_timer_get_time() - last_keepalive_time > 10 * 1000000) {
                last_keepalive_time = esp_timer_get_time();
                if (!__himax_keepalive_check(this_->sscma_client_handle_)) {
                    ESP_LOGE(TAG, "restart himax");
                    sscma_client_reset(this_->sscma_client_handle_);
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }

            if (this_->inference_en && Application::GetInstance().GetDeviceState() == kDeviceStateIdle ) {
                if (!is_inference) {
                    ESP_LOGI(TAG, "Start inference (enable=1)");
                    sscma_client_break(this_->sscma_client_handle_);
                    sscma_client_set_model(this_->sscma_client_handle_, 4);
                    sscma_client_set_sensor(this_->sscma_client_handle_, 1, 1, true); // è®¾ç½®åˆ†è¾¨çŽ‡ 416X416
                    sscma_client_invoke(this_->sscma_client_handle_, -1, false, true);
                    is_inference = true;
                }
            } else if (is_inference && (!this_->inference_en || Application::GetInstance().GetDeviceState() != kDeviceStateIdle))  {
                ESP_LOGI(TAG, "Stop inference (enable=%d state=%d)", this_->inference_en, Application::GetInstance().GetDeviceState());
                is_inference = false;
                sscma_client_break(this_->sscma_client_handle_);
            }
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }, "sscma_camera", 4096, this, 1, nullptr);

}

SscmaCamera::~SscmaCamera() {
    if (preview_image_.data) {
        heap_caps_free((void*)preview_image_.data);
        preview_image_.data = nullptr;
    }
    if (sscma_client_handle_) {
        sscma_client_del(sscma_client_handle_);
    }
    if (sscma_data_queue_) {
        vQueueDelete(sscma_data_queue_);
    }
    if (jpeg_data_.buf) {
        heap_caps_free(jpeg_data_.buf);
        jpeg_data_.buf = nullptr;
    }
    if (jpeg_dec_) {
        jpeg_dec_close(jpeg_dec_);
        jpeg_dec_ = nullptr;
    }
    if (jpeg_io_) {
        heap_caps_free(jpeg_io_);
        jpeg_io_ = nullptr;
    }
    if (jpeg_out_) {
        heap_caps_free(jpeg_out_);
        jpeg_out_ = nullptr;
    }
}

void SscmaCamera::InitializeMcpTools() {
    
    Settings settings("model", false);
    detect_threshold = settings.GetInt("threshold", 75);
    detect_invoke_interval_sec = settings.GetInt("interval", 8);
    detect_duration_sec = settings.GetInt("duration", 2);
    detect_target = settings.GetInt("target", 0);
    inference_en = settings.GetInt("enable", 0);

    auto& mcp_server = McpServer::GetInstance();
        // èŽ·å–æ¨¡åž‹å‚æ•°é…ç½®
    mcp_server.AddTool("self.model.param_get",
        "èŽ·å–å½“å‰è§†è§‰æ¨¡åž‹æ£€æµ‹çš„å‚æ•°é…ç½®ä¿¡æ¯ã€‚\n"
        "è¿”å›žç»“æžœåŒ…å«ï¼š\n"
        "  `threshold`: æ£€æµ‹ç½®ä¿¡åº¦é˜ˆå€¼ (0-100)ï¼Œä½ŽäºŽæ­¤å€¼çš„æ£€æµ‹ç»“æžœå°†è¢«å¿½ç•¥ï¼›\n"
        "  `interval`: è§¦å‘å¯¹è¯åŽçš„å†·å´æ—¶é—´(ç§’)ï¼Œé˜²æ­¢é¢‘ç¹æ‰“æ–­ï¼›\n"
        "  `duration`: æŒç»­æ£€æµ‹ç¡®è®¤æ—¶é—´(ç§’)ï¼›\n"
        "  `target`: å½“å‰å…³æ³¨çš„æ£€æµ‹ç›®æ ‡ç´¢å¼•ã€‚",
        PropertyList(),
        [this](const PropertyList& properties) -> ReturnValue {
            Settings settings("model", false);
            int threshold = settings.GetInt("threshold", 75);
            int interval = settings.GetInt("interval", 8);
            int duration = settings.GetInt("duration", 2);
            int target_type = settings.GetInt("target", 0);
            
            std::string result = "{\"threshold\":" + std::to_string(threshold) + 
                            ",\"interval\":" + std::to_string(interval) + 
                            ",\"duration\":" + std::to_string(duration) + 
                            ",\"target_type\":" + std::to_string(target_type) + "}";
            return result;
    });

    
    // è®¾ç½®æ¨¡åž‹å‚æ•°é…ç½®
    mcp_server.AddTool("self.model.param_set",
        "é…ç½®è§†è§‰æ¨¡åž‹æ£€æµ‹å‚æ•°ã€‚å½“ç”¨æˆ·å¸Œæœ›è°ƒæ•´æ£€æµ‹çµæ•åº¦ã€é¢‘çŽ‡æˆ–ç‰¹å®šç›®æ ‡æ—¶ä½¿ç”¨ã€‚\n"
        "å‚æ•°(å‡ä¸ºå¯é€‰ï¼Œæœªæä¾›çš„å‚æ•°å°†ä¿æŒå½“å‰è®¾ç½®ä¸å˜)ï¼š\n"
        "  `threshold`: ç½®ä¿¡åº¦é˜ˆå€¼ (0-100)ã€‚æé«˜æ­¤å€¼å¯å‡å°‘è¯¯æŠ¥ï¼Œä½†å¯èƒ½æ¼æ£€ï¼›\n"
        "  `interval`: å†·å´æ—¶é—´(ç§’)ã€‚è®¾ç½®å¯¹è¯ç»“æŸåŽå¤šä¹…å†…ä¸å†è§¦å‘æ£€æµ‹ï¼›\n"
        "  `duration`: æŒç»­æ£€æµ‹æ—¶é—´(ç§’)ã€‚\n"
        "  `target`: è®¾ç½®æ£€æµ‹ç›®æ ‡çš„ç´¢å¼• IDã€‚",
        PropertyList({
            Property("threshold", kPropertyTypeInteger, -1, -1, 100),
            Property("interval", kPropertyTypeInteger, -1, -1, 60),
            Property("duration", kPropertyTypeInteger, -1, -1, 60),
            Property("target", kPropertyTypeInteger, -1, -1, this->model_class_cnt > 0 ? this->model_class_cnt - 1 : 255)
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            Settings settings("model", true);
            try {
                const Property& threshold_prop = properties["threshold"];
                int threshold = threshold_prop.value<int>();
                if (threshold != -1) {
                    settings.SetInt("threshold", threshold);
                    this->detect_threshold = threshold;
                    ESP_LOGI(TAG, "Set detection threshold to %d", threshold);
                }
            } catch (const std::runtime_error&) {
                // threshold parameter not provided, skip
            }
            
            try {
                const Property& interval_prop = properties["interval"];
                int interval = interval_prop.value<int>();
                if (interval != -1) {
                    settings.SetInt("interval", interval);
                    this->detect_invoke_interval_sec = interval;
                    ESP_LOGI(TAG, "Set detection interval to %d", interval);
                }
            } catch (const std::runtime_error&) {
                // interval parameter not provided, skip
            }
            
            try {
                const Property& duration_prop = properties["duration"];
                int duration = duration_prop.value<int>();
                if (duration != -1) {
                    settings.SetInt("duration", duration);
                    this->detect_duration_sec = duration;
                }
            } catch (const std::runtime_error&) {
                // duration parameter not provided, skip
            }
            
            try {
                const Property& target_prop = properties["target"];
                int target = target_prop.value<int>();
                if (target != -1) {
                    settings.SetInt("target", target);
                    this->detect_target = target;
                    ESP_LOGI(TAG, "Set detection target to %d", target);
                }
            } catch (const std::runtime_error&) {
                // target_type parameter not provided, skip
            }

            return "{\"status\": \"success\", \"message\": \"Detection configuration updated\"}";
        });

    // æŽ¨ç†å¼€å…³èŽ·å–
    mcp_server.AddTool("self.model.enable",
        "æŽ§åˆ¶è§†è§‰æŽ¨ç†(æ‘„åƒå¤´æ£€æµ‹)åŠŸèƒ½çš„å¼€å¯ä¸Žå…³é—­ï¼Œæˆ–æŸ¥è¯¢å½“å‰çŠ¶æ€ã€‚\n"
        "å½“ç”¨æˆ·æŒ‡ä»¤æ¶‰åŠ'å¼€å¯/å…³é—­æŽ¨ç†'ã€'å¼€å§‹/åœæ­¢æ£€æµ‹'æ—¶ä½¿ç”¨ã€‚\n"
        "å‚æ•°ï¼š\n"
        "  `enable`: (å¯é€‰) æ•´æ•°ã€‚1=å¼€å¯æŽ¨ç†ï¼Œ0=å…³é—­æŽ¨ç†ã€‚è‹¥çœç•¥åˆ™è¿”å›žå½“å‰å¼€å…³çŠ¶æ€ã€‚",
        PropertyList({
            Property("enable", kPropertyTypeInteger, inference_en, 0, 1)
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            Settings settings("model", true);
            try {
                const Property& enable_prop = properties["enable"];
                int en = enable_prop.value<int>();
                settings.SetInt("enable", en);
                this->inference_en = en;
                ESP_LOGI(TAG, "Set inference enable to %d", en);
            } catch (const std::runtime_error&) {
                // enable not provided -> treat as query
            }
            // è¿”å›žå½“å‰é…ç½®
            int cur_en = settings.GetInt("enable", this->inference_en);
            return std::string("{\"enable\":") + std::to_string(cur_en) + "}";
        });
}

void SscmaCamera::SetExplainUrl(const std::string& url, const std::string& token) {
    explain_url_ = url;
    explain_token_ = token;
}

bool SscmaCamera::Capture() {

    SscmaData data;
    int ret = 0;
    
    if (sscma_client_handle_ == nullptr) {
        ESP_LOGE(TAG, "SSCMA client handle is not initialized");
        return false;
    }

    if (sscma_client_set_sensor(sscma_client_handle_, 1, 3, true)) {
        ESP_LOGE(TAG, "Failed to set sensor");
        return false;
    }
    ESP_LOGI(TAG, "Capturing image...");
    // himax å¯èƒ½æœ‰ç¼“å­˜æ•°æ®, åªèŽ·å–æœ€æ–°çš„ç…§ç‰‡å³å¯.
    if (sscma_client_sample(sscma_client_handle_, 1) ) {
        ESP_LOGE(TAG, "Failed to capture image from SSCMA client");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(500)); // ç­‰å¾…SSCMAå®¢æˆ·ç«¯å¤„ç†æ•°æ®
    if (xQueueReceive(sscma_data_queue_, &data, pdMS_TO_TICKS(1000)) != pdPASS) {
        ESP_LOGE(TAG, "Failed to receive JPEG data from SSCMA client");
        return false;
    }

    if (jpeg_data_.buf == nullptr) {
        heap_caps_free(data.img);
        return false;
    }

    ret = mbedtls_base64_decode(jpeg_data_.buf, IMG_JPEG_BUF_SIZE, &jpeg_data_.len, data.img, data.len);
    if (ret != 0 || jpeg_data_.len == 0) {
        ESP_LOGE(TAG, "Failed to decode base64 image data, ret: %d, output_len: %zu", ret, jpeg_data_.len);
        heap_caps_free(data.img);
        return false;
    }
    heap_caps_free(data.img);

    //DECODE JPEG
    if (!jpeg_dec_ || !jpeg_io_ || !jpeg_out_ || !preview_image_.data) {
        return true;
    }
    jpeg_io_->inbuf = jpeg_data_.buf;
    jpeg_io_->inbuf_len = jpeg_data_.len;
    ret = jpeg_dec_parse_header(jpeg_dec_, jpeg_io_, jpeg_out_);
    if (ret < 0) {
        ESP_LOGE(TAG, "Failed to parse JPEG header, ret: %d", ret);
        return true;
    }
    jpeg_io_->outbuf = (unsigned char*)preview_image_.data;
    int inbuf_consumed = jpeg_io_->inbuf_len - jpeg_io_->inbuf_remain;
    jpeg_io_->inbuf =  jpeg_data_.buf + inbuf_consumed;
    jpeg_io_->inbuf_len = jpeg_io_->inbuf_remain;

    ret = jpeg_dec_process(jpeg_dec_, jpeg_io_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to decode JPEG image, ret: %d", ret);
        return true;
    }

    // æ˜¾ç¤ºé¢„è§ˆå›¾ç‰‡
    auto display = dynamic_cast<LvglDisplay*>(Board::GetInstance().GetDisplay());
    if (display != nullptr) {
        uint16_t w = preview_image_.header.w;
        uint16_t h = preview_image_.header.h;
        size_t image_size = w * h * 2;
        size_t stride = preview_image_.header.w * 2;

        uint8_t* data = (uint8_t*)heap_caps_malloc(image_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (data == nullptr) {
            ESP_LOGE(TAG, "Failed to allocate memory for display image");
            return true;
        }
        memcpy(data, preview_image_.data, image_size);
        
        auto image = std::make_unique<LvglAllocatedImage>(data, image_size, w, h, stride, LV_COLOR_FORMAT_RGB565);
        display->SetPreviewImage(std::move(image));
    }
    return true;
}
bool SscmaCamera::SetHMirror(bool enabled) {
    return false;
}

bool SscmaCamera::SetVFlip(bool enabled) {
    return false;
}

/**
 * @brief å°†æ‘„åƒå¤´æ•èŽ·çš„å›¾åƒå‘é€åˆ°è¿œç¨‹æœåŠ¡å™¨è¿›è¡ŒAIåˆ†æžå’Œè§£é‡Š
 * 
 * è¯¥å‡½æ•°å°†å½“å‰æ‘„åƒå¤´ç¼“å†²åŒºä¸­çš„å›¾åƒç¼–ç ä¸ºJPEGæ ¼å¼ï¼Œå¹¶é€šè¿‡HTTP POSTè¯·æ±‚
 * ä»¥multipart/form-dataçš„å½¢å¼å‘é€åˆ°æŒ‡å®šçš„è§£é‡ŠæœåŠ¡å™¨ã€‚æœåŠ¡å™¨å°†æ ¹æ®æä¾›çš„
 * é—®é¢˜å¯¹å›¾åƒè¿›è¡ŒAIåˆ†æžå¹¶è¿”å›žç»“æžœã€‚
 * 
 * @param question è¦å‘AIæå‡ºçš„å…³äºŽå›¾åƒçš„é—®é¢˜ï¼Œå°†ä½œä¸ºè¡¨å•å­—æ®µå‘é€
 * @return std::string æœåŠ¡å™¨è¿”å›žçš„JSONæ ¼å¼å“åº”å­—ç¬¦ä¸²
 *         æˆåŠŸæ—¶åŒ…å«AIåˆ†æžç»“æžœï¼Œå¤±è´¥æ—¶åŒ…å«é”™è¯¯ä¿¡æ¯
 *         æ ¼å¼ç¤ºä¾‹ï¼š{"success": true, "result": "åˆ†æžç»“æžœ"}
 *                  {"success": false, "message": "é”™è¯¯ä¿¡æ¯"}
 * 
 * @note è°ƒç”¨æ­¤å‡½æ•°å‰å¿…é¡»å…ˆè°ƒç”¨SetExplainUrl()è®¾ç½®æœåŠ¡å™¨URL
 * @note å‡½æ•°ä¼šç­‰å¾…ä¹‹å‰çš„ç¼–ç çº¿ç¨‹å®ŒæˆåŽå†å¼€å§‹æ–°çš„å¤„ç†
 * @warning å¦‚æžœæ‘„åƒå¤´ç¼“å†²åŒºä¸ºç©ºæˆ–ç½‘ç»œè¿žæŽ¥å¤±è´¥ï¼Œå°†è¿”å›žé”™è¯¯ä¿¡æ¯
 */
std::string SscmaCamera::Explain(const std::string& question) {
    if (explain_url_.empty()) {
        return "{\"success\": false, \"message\": \"Image explain URL or token is not set\"}";
    }

    auto network = Board::GetInstance().GetNetwork();
    auto http = network->CreateHttp(3);
    // æž„é€ multipart/form-dataè¯·æ±‚ä½“
    std::string boundary = "----ESP32_CAMERA_BOUNDARY";
    
    // æž„é€ questionå­—æ®µ
    std::string question_field;
    question_field += "--" + boundary + "\r\n";
    question_field += "Content-Disposition: form-data; name=\"question\"\r\n";
    question_field += "\r\n";
    question_field += question + "\r\n";
    
    // æž„é€ æ–‡ä»¶å­—æ®µå¤´éƒ¨
    std::string file_header;
    file_header += "--" + boundary + "\r\n";
    file_header += "Content-Disposition: form-data; name=\"file\"; filename=\"camera.jpg\"\r\n";
    file_header += "Content-Type: image/jpeg\r\n";
    file_header += "\r\n";
    
    // æž„é€ å°¾éƒ¨
    std::string multipart_footer;
    multipart_footer += "\r\n--" + boundary + "--\r\n";

    // é…ç½®HTTPå®¢æˆ·ç«¯ï¼Œä½¿ç”¨åˆ†å—ä¼ è¾“ç¼–ç 
    http->SetHeader("Device-Id", SystemInfo::GetMacAddress().c_str());
    http->SetHeader("Client-Id", Board::GetInstance().GetUuid().c_str());
    if (!explain_token_.empty()) {
        http->SetHeader("Authorization", "Bearer " + explain_token_);
    }
    http->SetHeader("Content-Type", "multipart/form-data; boundary=" + boundary);
    http->SetHeader("Transfer-Encoding", "chunked");
    if (!http->Open("POST", explain_url_)) {
        ESP_LOGE(TAG, "Failed to connect to explain URL");
        return "{\"success\": false, \"message\": \"Failed to connect to explain URL\"}";
    }
    
    // ç¬¬ä¸€å—ï¼šquestionå­—æ®µ
    http->Write(question_field.c_str(), question_field.size());
    
    // ç¬¬äºŒå—ï¼šæ–‡ä»¶å­—æ®µå¤´éƒ¨
    http->Write(file_header.c_str(), file_header.size());
    
    // ç¬¬ä¸‰å—ï¼šJPEGæ•°æ®
    http->Write((const char*)jpeg_data_.buf, jpeg_data_.len);

    // ç¬¬å››å—ï¼šmultipartå°¾éƒ¨
    http->Write(multipart_footer.c_str(), multipart_footer.size());
    
    // ç»“æŸå—
    http->Write("", 0);

    if (http->GetStatusCode() != 200) {
        ESP_LOGE(TAG, "Failed to upload photo, status code: %d", http->GetStatusCode());
        return "{\"success\": false, \"message\": \"Failed to upload photo\"}";
    }

    std::string result = http->ReadAll();
    http->Close();

    ESP_LOGI(TAG, "Explain image size=%d, question=%s\n%s", jpeg_data_.len, question.c_str(), result.c_str());
    return result;
}
