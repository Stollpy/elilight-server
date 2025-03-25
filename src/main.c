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
#define LIGHT_FADE_UUID          0xFF03

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
static uint16_t fade_time = 1000;

static uint16_t service_handle;
static uint16_t char_handles[3];

void apply_led_state() {
    if (!led_on) {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL);
    } else {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL, brightness);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL);
    }
}

void start_fade_effect() {
    while (1) {
        ledc_set_fade_time_and_start(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL, brightness, fade_time, LEDC_FADE_WAIT_DONE);
        ledc_set_fade_time_and_start(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL, 0, fade_time, LEDC_FADE_WAIT_DONE);
    }
}

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    if (event == ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT) {
        esp_ble_gap_start_advertising(&adv_params);
    }
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

            esp_ble_gatts_create_service(gatts_if, &service_id, 8);
            break;
        }
        case ESP_GATTS_CREATE_EVT: {
            service_handle = param->create.service_handle;
            esp_ble_gatts_start_service(service_handle);

            esp_bt_uuid_t char_uuid;

            char_uuid.len = ESP_UUID_LEN_16;
            char_uuid.uuid.uuid16 = LIGHT_STATE_UUID;
            esp_ble_gatts_add_char(service_handle, &char_uuid,
                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE,
                                   NULL, NULL);

            char_uuid.uuid.uuid16 = LIGHT_BRIGHTNESS_UUID;
            esp_ble_gatts_add_char(service_handle, &char_uuid,
                                   ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_WRITE,
                                   NULL, NULL);

            char_uuid.uuid.uuid16 = LIGHT_FADE_UUID;
            esp_ble_gatts_add_char(service_handle, &char_uuid,
                                   ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_WRITE,
                                   NULL, NULL);
            break;
        }
        case ESP_GATTS_ADD_CHAR_EVT: {
            static int char_idx = 0;
            char_handles[char_idx++] = param->add_char.attr_handle;
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
            ESP_LOGI("BLE_LIGHT", "Write event, handle: %d, value_len: %d", param->write.handle, param->write.len);
        
            if (!param->write.is_prep) {
                // On vérifie que la valeur envoyée est de taille 1 (par exemple un uint8_t)
                if (param->write.len == 1) {
                    uint16_t handle = param->write.handle;
                    uint8_t value = param->write.value[0];
                    ESP_LOGI("BLE_LIGHT", "New value received: %d", value);
        
                    if (handle == char_handles[0]) {
                        led_on = value;
                        ESP_LOGI(GATTS_TAG, "NEW VALUE %d", value);
                        apply_led_state();
                    } else if (handle == char_handles[1]) {
                        brightness = value;
                        apply_led_state();
                    } else if (handle == char_handles[2]) {
                        fade_time = value * 10;
                        xTaskCreate((TaskFunction_t)start_fade_effect, "fade_task", 2048, NULL, 5, NULL);
                    }
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        
                } else {
                    ESP_LOGW("BLE_LIGHT", "Invalid value length: %d", param->write.len);
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_INVALID_ATTR_LEN, NULL);
                }
            } else {
                ESP_LOGW("BLE_LIGHT", "Prepare write not handled");
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_WRITE_NOT_PERMIT, NULL);
            }
            break;
        }
        case ESP_GATTS_READ_EVT: {
            ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %" PRIu16 ", trans_id %" PRIu32 ", handle %" PRIu16, param->read.conn_id, param->read.trans_id, param->read.handle);

            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = 1;
            rsp.attr_value.value[0] = led_on;
            
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
            break;
        }
        case ESP_GATTS_DISCONNECT_EVT: {
            ESP_LOGI(GATTS_TAG, "Disconnected, remote "ESP_BD_ADDR_STR", reason 0x%02x",
            ESP_BD_ADDR_HEX(param->disconnect.remote_bda), param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
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
        ESP_LOGI("BLE_LIGHT", "Alive...");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}