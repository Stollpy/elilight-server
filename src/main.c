#include "stdio.h"
#include "string.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define DEVICE_NAME "elilight"
#define GATTS_TAG "BLE_LIGHT"
#define PROFILE_NUM 1
#define PROFILE_APP_IDX 0

#define LIGHT_SERVICE_UUID       0x00FF
#define LIGHT_STATE_UUID         0xFF01
#define LIGHT_BRIGHTNESS_UUID    0xFF02
#define LIGHT_FADE_STATE_UUID    0xFF03
#define LIGHT_FADE_UUID          0xFF04

#define CHAR_COUNT 4
#define LED_GPIO                 GPIO_NUM_18
#define LEDC_TIMER               LEDC_TIMER_0
#define LEDC_CHANNEL             LEDC_CHANNEL_0

static uint8_t adv_service_uuid128[16] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT)
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY
};

static uint8_t led_on = 0;
static uint8_t brightness = 255;
static uint16_t fade_time = 255;
static uint8_t fade_on = 0;
static TaskHandle_t fade_task_handle = NULL; 

static uint16_t service_handle;
static uint16_t char_handles[CHAR_COUNT];
static esp_bt_uuid_t char_uuids[CHAR_COUNT];
static int char_add_index = 0;

void apply_led_state() {
    if (!led_on) {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL);
    } else {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL, brightness);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL);
    }
}

void start_fade_effect(void *arg) {
    while (fade_on) {
        ledc_set_fade_time_and_start(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL, brightness, fade_time, LEDC_FADE_WAIT_DONE);
        ledc_set_fade_time_and_start(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL, 0, fade_time, LEDC_FADE_WAIT_DONE);
    }

    led_on = 1;
    apply_led_state();
    
    ESP_LOGI(GATTS_TAG, "Fade task stopping...");
    fade_task_handle = NULL;
    vTaskDelete(NULL);
}

void start_fade() {
    if (fade_task_handle == NULL) {
        fade_on = 1;
        xTaskCreate(start_fade_effect, "fade_task", 2048, NULL, 5, &fade_task_handle);
        ESP_LOGI(GATTS_TAG, "Fade task started");
    } else {
        ESP_LOGI(GATTS_TAG, "Fade task already running");
    }
}

void stop_fade() {
    if (fade_task_handle != NULL) {
        fade_on = 0;
        ESP_LOGI(GATTS_TAG, "Requested fade stop");
    }
}


void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    // if (char_add_index == CHAR_COUNT && event == ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT) {
    //     ESP_LOGI(GATTS_TAG, "All characteristics added, starting advertising");
    //     esp_ble_gap_start_advertising(&adv_params);
    // } else if (event == ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT) {
    //     esp_ble_gap_config_adv_data(&adv_data);
    //     ESP_LOGI(GATTS_TAG, "Resend GAP event %d", char_add_index);
    // }
}

void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT: {
            esp_ble_gap_set_device_name(DEVICE_NAME);
            esp_ble_gap_config_adv_data(&adv_data);

            esp_gatt_srvc_id_t service_id = {
                .is_primary = true,
                .id.inst_id = 0,
                .id.uuid.len = ESP_UUID_LEN_16,
                .id.uuid.uuid.uuid16 = LIGHT_SERVICE_UUID
            };

            esp_ble_gatts_create_service(gatts_if, &service_id, (CHAR_COUNT * 2) + 4);
            break;
        }
        case ESP_GATTS_CREATE_EVT: {
            service_handle = param->create.service_handle;
            esp_ble_gatts_start_service(service_handle);

            char_uuids[0].len = ESP_UUID_LEN_16;
            char_uuids[0].uuid.uuid16 = LIGHT_STATE_UUID;
            
            char_uuids[1].len = ESP_UUID_LEN_16;
            char_uuids[1].uuid.uuid16 = LIGHT_BRIGHTNESS_UUID;
            
            char_uuids[2].len = ESP_UUID_LEN_16;
            char_uuids[2].uuid.uuid16 = LIGHT_FADE_STATE_UUID;
            
            char_uuids[3].len = ESP_UUID_LEN_16;
            char_uuids[3].uuid.uuid16 = LIGHT_FADE_UUID;
            
            char_add_index = 0;
            esp_ble_gatts_add_char(service_handle, &char_uuids[char_add_index], ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE, NULL, NULL);
            ESP_LOGI(GATTS_TAG, "Adding characteristic %d with UUID: 0x%04X", char_add_index, char_uuids[char_add_index].uuid.uuid16);
            break;
        }
        case ESP_GATTS_ADD_CHAR_EVT: {
            char_handles[char_add_index] = param->add_char.attr_handle;
            char_add_index++;
            
            if (char_add_index < CHAR_COUNT) {
                esp_err_t ret = esp_ble_gatts_add_char(service_handle, &char_uuids[char_add_index], ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE, NULL, NULL);
                if (ret != ESP_OK) {
                    ESP_LOGE(GATTS_TAG, "Failed to add char FF04: %s", esp_err_to_name(ret));
                } else {
                    ESP_LOGI(GATTS_TAG, "Adding characteristic %d with UUID: 0x%04X", char_add_index, char_uuids[char_add_index].uuid.uuid16);
                }
            } else {
                ESP_LOGI(GATTS_TAG, "All characteristics added, starting advertising");
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        }
        case ESP_GATTS_CONNECT_EVT: {
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            
            conn_params.latency = 0;
            conn_params.max_int = 0x30;
            conn_params.min_int = 0x10;
            conn_params.timeout = 400;

            ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x",
                param->connect.conn_id,
                param->connect.remote_bda[0],
                param->connect.remote_bda[1],
                param->connect.remote_bda[2],
                param->connect.remote_bda[3],
                param->connect.remote_bda[4],
                param->connect.remote_bda[5]
            );

            esp_ble_gap_update_conn_params(&conn_params);
            break;
        }
        case ESP_GATTS_WRITE_EVT: {
            ESP_LOGI(GATTS_TAG, "Write event, handle: %d, value_len: %d", param->write.handle, param->write.len);
        
            if (!param->write.is_prep) {
                if (param->write.len == 1) {
                    uint16_t handle = param->write.handle;
                    ESP_LOGI(GATTS_TAG, "New value received: %d", param->write.value[0]);
        
                    if (handle == char_handles[0]) {
                        led_on = param->write.value[0];
                        apply_led_state();
                        ESP_LOGI(GATTS_TAG, "LIGHT IS SET ON %d", led_on);
                    } else if (handle == char_handles[1]) {
                        brightness = param->write.value[0];
                        apply_led_state();
                        ESP_LOGI(GATTS_TAG, "LIGHT BRIGHTNESS IS SET ON %d", brightness);
                    } else if (handle == char_handles[2]) {
                        if (param->write.value[0]) {
                            start_fade();
                            ESP_LOGI(GATTS_TAG, "LIGHT FADE START");
                        } else {
                            stop_fade();
                            ESP_LOGI(GATTS_TAG, "LIGHT FADE STOP");
                        }
                    } else if (handle == char_handles[3]) {
                        fade_time = param->write.value[0] * 10;
                        ESP_LOGI(GATTS_TAG, "LIGHT FADE SET ON %d", fade_time);
                    }
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        
                } else {
                    ESP_LOGW(GATTS_TAG, "Invalid value length: %d", param->write.len);
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_INVALID_ATTR_LEN, NULL);
                }
            } else {
                ESP_LOGW(GATTS_TAG, "Prepare write not handled");
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_WRITE_NOT_PERMIT, NULL);
            }
            break;
        }
        case ESP_GATTS_READ_EVT: {
            ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %" PRIu16 ", trans_id %" PRIu32 ", handle %" PRIu16, param->read.conn_id, param->read.trans_id, param->read.handle);
            
            uint16_t handle = param->read.handle;

            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));

            rsp.attr_value.handle = handle;
            rsp.attr_value.len = 1;

            if (handle == char_handles[0]) {
                rsp.attr_value.value[0] = led_on;
            } else if (handle == char_handles[1]) {
                rsp.attr_value.value[0] = brightness;
            } else if (handle == char_handles[2]) {
                rsp.attr_value.value[0] = fade_on;
            } else if (handle == char_handles[3]) {
                rsp.attr_value.value[0] = fade_time;
            }
            
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
            break;
        }
        case ESP_GATTS_DISCONNECT_EVT: {
            ESP_LOGI(GATTS_TAG, "Disconnected, remote "ESP_BD_ADDR_STR", reason 0x%02x",
            ESP_BD_ADDR_HEX(param->disconnect.remote_bda), param->disconnect.reason);
            if (char_add_index == CHAR_COUNT) {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        }
        default:
            break;
    }
}

void app_main(void) {
    nvs_flash_init();
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num = LED_GPIO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);

    ledc_fade_func_install(0);

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(0);

    while (1) {
        ESP_LOGI(GATTS_TAG, "Alive...");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}