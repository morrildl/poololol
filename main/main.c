#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_event.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

static const char *POOLOLOL_TAG = "poololol";


/* MQTT state/data variables */
static esp_mqtt_client_handle_t mqtt_client;


/* Bluetooth state variables & constants */
static bool connected = false;
static bool service_found = false;
static struct gattc_profile_state {
  uint16_t gattc_if;
  uint16_t app_id;
  uint16_t conn_id;
  uint16_t service_start_handle;
  uint16_t service_end_handle;
  uint16_t char_handle;
  esp_bd_addr_t remote_bda;
} gattc_profile = { .gattc_if = ESP_GATT_IF_NONE };


/* Timers for the primary data poller, and for a health heartbeat */
static esp_timer_handle_t poller_timer;
static esp_timer_handle_t heartbeat_timer;


/* Bluetooth constants */
#define POLL_PERIOD (1000000ULL) * CONFIG_POOLOLOL_POLL_PERIOD
static const char remote_device_name[] = "tps";
static esp_bt_uuid_t tps_temp_service_uuid = {
  .len = ESP_UUID_LEN_16,
  .uuid = { .uuid16 = CONFIG_POOLOLOL_BT_SERVICE_UUID },
};
static esp_bt_uuid_t tps_temp_characteristic_uuid = {
  .len = ESP_UUID_LEN_16,
  .uuid = { .uuid16 = CONFIG_POOLOLOL_BT_CHARACTERISTIC_UUID },
};
static esp_ble_scan_params_t ble_scan_params = {
  .scan_type = BLE_SCAN_TYPE_ACTIVE,
  .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
  .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
  .scan_interval = 0x50,
  .scan_window = 0x30,
  .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE
};


/** Transmit a temperature reading, stored in the input bytes, over MQTT.
 *  The bytes are interpreted as little-endian representation of a 16-bit signed int
 *  (which is the format emitted by the Inkbird widgets.)
 */
static void transmit_temperature(uint8_t bytes[2]) {
  int16_t reading = (int16_t)((bytes[1] << 8) | bytes[0]);
  ESP_LOGI(POOLOLOL_TAG, "transmitting some bytes: %d", reading);
  char payload[128];
  snprintf(payload, 128, "{\"device\": \"" CONFIG_POOLOLOL_MQTT_DEVICE_NAME "\", \"" CONFIG_POOLOLOL_MQTT_TEMPERATURE_FIELD "\": %.2f}", ((float)reading) / 100);
  esp_mqtt_client_publish(mqtt_client, CONFIG_POOLOLOL_MQTT_TOPIC_PREFIX CONFIG_POOLOLOL_MQTT_DEVICE_NAME, payload, 0, 0, 0);
}

/** Disconnects from Bluetooth and resets state, ready for another scan. */
static void reset_bt() {
  ESP_LOGI(POOLOLOL_TAG, "closing GATT connection handle %d on interface %x", gattc_profile.conn_id, gattc_profile.gattc_if);
  esp_err_t err = esp_ble_gattc_close(gattc_profile.gattc_if, gattc_profile.conn_id);
  if (err) ESP_LOGW(POOLOLOL_TAG, "error closing GATT client: '%s'", esp_err_to_name(err));

  connected = false;
  service_found = false;
  gattc_profile.app_id = 0;
  gattc_profile.service_start_handle = 0;
  gattc_profile.service_end_handle = 0;
  gattc_profile.char_handle = 0;
  gattc_profile.conn_id = 0;
  memset(gattc_profile.remote_bda, 0, sizeof(esp_bd_addr_t));

  ESP_LOGI(POOLOLOL_TAG, "reset_bt() complete");

  //gattc_profile.gattc_if = ESP_GATT_IF_NONE; // don't need to clear this, it's just a hardware handle
}

/** Kicks off the event chain that will scan for, connect to, query, and ultimately read the characteristic for a single sample collection. */
static void poll_once() {
  ESP_LOGI(POOLOLOL_TAG, "new poll sequence started");
  reset_bt();
  esp_ble_gap_start_scanning(CONFIG_POOLOLOL_POLL_PERIOD / 2); // scan for half the configured period, to allow for time to complete the connection
}

/** Sends a heartbeat message to the MQTT server on a separate topic from the data samples. Includes some system status info, because why not. */
static void heartbeat() {
  multi_heap_info_t heap;
  char message[256];
  heap_caps_get_info(&heap, MALLOC_CAP_DEFAULT);
  snprintf(message, 256, 
    "{\"device\": \"%s\", total_free: %d, total_alloc: %d, largest_free_blk: %d, min_free: %d, allocated_blocks: %d, free_blocks: %d, total_blocks: %d}",
    CONFIG_POOLOLOL_MQTT_DEVICE_NAME,
    heap.total_free_bytes, heap.total_allocated_bytes, heap.largest_free_block, heap.minimum_free_bytes, heap.allocated_blocks, heap.free_blocks, heap.total_blocks);
  ESP_LOGI(POOLOLOL_TAG, "Sending heartbeat/health message");
  esp_mqtt_client_publish(mqtt_client, CONFIG_POOLOLOL_MQTT_HEARTBEAT_TOPIC_PREFIX CONFIG_POOLOLOL_MQTT_DEVICE_NAME, message, 0, 0, 1); // retained
}


/*
 * Event Handlers
 */

/** WiFi callback that tries really hard to stay connected, by kicking off a new reconnection whenever WiFi disconnects. */
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
  ESP_LOGE(POOLOLOL_TAG, "%s", "wifi handler");
  if (event_base != WIFI_EVENT) return;
  switch (event_id) {
    case WIFI_EVENT_STA_START:  // fallthrough
    case WIFI_EVENT_STA_DISCONNECTED:
      ESP_LOGE(POOLOLOL_TAG, "%s", "starting WiFi (re)connect");
      esp_wifi_connect();
    default:
      break;
  }
}

/** A basic MQTT callback. There's not much to do in this, since we never subscribe to anything, we only publish. */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
  esp_mqtt_event_handle_t event = event_data;
  //esp_mqtt_client_handle_t client = event->client;
  //int msg_id;

  switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
      ESP_LOGI(POOLOLOL_TAG, "MQTT_EVENT_CONNECTED");
      heartbeat();
      break;

    case MQTT_EVENT_DISCONNECTED:
      ESP_LOGI(POOLOLOL_TAG, "MQTT_EVENT_DISCONNECTED");
      break;

    case MQTT_EVENT_PUBLISHED:
      ESP_LOGI(POOLOLOL_TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
      break;

    case MQTT_EVENT_ERROR:
      ESP_LOGI(POOLOLOL_TAG, "MQTT_EVENT_ERROR");
      if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
        ESP_LOGI(POOLOLOL_TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
      }
      break;

    default:
      ESP_LOGI(POOLOLOL_TAG, "Other event id:%d", event->event_id);
      break;
  }
}

/** A BLE GATT callback where each event handler simply kicks things along to the next, until eventually
 * it completes service discovery and then locates and reads the configured characteristic. */
static void bt_gatt_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *cb_param) {
  switch (event) {
    case ESP_GATTC_REG_EVT:
      if (cb_param->reg.status == ESP_GATT_OK) {
        gattc_profile.gattc_if = gattc_if;
      } else {
        ESP_LOGI(POOLOLOL_TAG, "reg app failed, app_id %04x, status %d", cb_param->reg.app_id, cb_param->reg.status);
        return;
      }
      ESP_LOGI(POOLOLOL_TAG, "REG_EVT");

      ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&ble_scan_params)); // note that this kicks out to the GAP stack/callback, which handles scans
      break;

    case ESP_GATTC_CONNECT_EVT: {
      ESP_LOGI(POOLOLOL_TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d", cb_param->connect.conn_id, gattc_if);
      gattc_profile.conn_id = cb_param->connect.conn_id;
      memcpy(gattc_profile.remote_bda, cb_param->connect.remote_bda, sizeof(esp_bd_addr_t));
      ESP_LOGI(POOLOLOL_TAG, "Remote MAC address:");
      ESP_LOG_BUFFER_HEX(POOLOLOL_TAG, gattc_profile.remote_bda, sizeof(esp_bd_addr_t));
      esp_err_t err = esp_ble_gattc_send_mtu_req(gattc_if, gattc_profile.conn_id);
      if (err) ESP_LOGE(POOLOLOL_TAG, "error starting MTU configuration: %s", esp_err_to_name(err));
      break;
    }

    case ESP_GATTC_OPEN_EVT:
      if (cb_param->open.status != ESP_GATT_OK) {
        ESP_LOGE(POOLOLOL_TAG, "failed to open the remote peer; status=%d", cb_param->open.status);
        break;
      }
      ESP_LOGI(POOLOLOL_TAG, "opened remote peer");
      break;

    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
      if (cb_param->dis_srvc_cmpl.status != ESP_GATT_OK) {
        ESP_LOGE(POOLOLOL_TAG, "failed discovering services; status=%d", cb_param->dis_srvc_cmpl.status);
        break;
      }
      ESP_LOGI(POOLOLOL_TAG, "service discovery complete on conn_id=%d", cb_param->dis_srvc_cmpl.conn_id);
      esp_ble_gattc_search_service(gattc_if, cb_param->dis_srvc_cmpl.conn_id, &tps_temp_service_uuid);
      break;

    case ESP_GATTC_CFG_MTU_EVT:
      if (cb_param->cfg_mtu.status != ESP_GATT_OK) {
        ESP_LOGE(POOLOLOL_TAG, "failed to configure MTU; status=%x", cb_param->cfg_mtu.status);
      }
      ESP_LOGI(
          POOLOLOL_TAG, "MTU configured on conn_id=%d with MTU=%d", cb_param->cfg_mtu.conn_id, cb_param->cfg_mtu.mtu);
      break;

    case ESP_GATTC_SEARCH_RES_EVT: {
      ESP_LOGI(POOLOLOL_TAG, "processing scan hit on conn_id=%x; is_primary service=%d", cb_param->search_res.conn_id, cb_param->search_res.is_primary);
      if (cb_param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && cb_param->search_res.srvc_id.uuid.uuid.uuid16 == tps_temp_service_uuid.uuid.uuid16) {
        service_found = true;
        gattc_profile.service_start_handle = cb_param->search_res.start_handle;
        gattc_profile.service_end_handle = cb_param->search_res.end_handle;
        ESP_LOGI(POOLOLOL_TAG, "located target service; UUID=%x", tps_temp_service_uuid.uuid.uuid16);
      }
      break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT:
      if (cb_param->search_cmpl.status != ESP_GATT_OK) {
        ESP_LOGE(POOLOLOL_TAG, "service search failed; status=%x", cb_param->search_cmpl.status);
        break;
      }
      if (cb_param->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE) {
        ESP_LOGI(POOLOLOL_TAG, "fetched service information from remote device");
      } else if (cb_param->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH) {
        ESP_LOGI(POOLOLOL_TAG, "loaded service information from flash");
      } else {
        ESP_LOGI(POOLOLOL_TAG, "unknown service source");
      }
      if (service_found) {
        uint16_t count = 0;
        esp_gatt_status_t status = esp_ble_gattc_get_attr_count(
            gattc_if, cb_param->search_cmpl.conn_id, ESP_GATT_DB_CHARACTERISTIC,
            gattc_profile.service_start_handle,
            gattc_profile.service_end_handle,
            0, &count);
        if (status != ESP_GATT_OK) {
          ESP_LOGE(POOLOLOL_TAG, "error retrieving service count; status=%d", status);
          break;
        }

        if (count > 0) {
          esp_gattc_char_elem_t *char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
          if (!char_elem_result) {
            ESP_LOGE(POOLOLOL_TAG, "error in malloc() -- no RAM to search for characteristic");
            break;
          } else {
            status = esp_ble_gattc_get_char_by_uuid(
                gattc_if, cb_param->search_cmpl.conn_id,
                gattc_profile.service_start_handle,
                gattc_profile.service_end_handle,
                tps_temp_characteristic_uuid, char_elem_result, &count);
            if (status != ESP_GATT_OK) {
              ESP_LOGE(POOLOLOL_TAG, "error loading characteristic data; status=%d", status);
              free(char_elem_result);
              char_elem_result = NULL;
              break;
            }

            for (int i = 0; i < count; ++i) {
              ESP_LOGD(POOLOLOL_TAG, "examining characteristic UUID: %x vs %x", char_elem_result[i].uuid.uuid.uuid16, tps_temp_characteristic_uuid.uuid.uuid16);
              if (char_elem_result[i].uuid.len == ESP_UUID_LEN_16 && char_elem_result[i].uuid.uuid.uuid16 == tps_temp_characteristic_uuid.uuid.uuid16) {
                esp_err_t err = esp_ble_gattc_read_char(gattc_if, cb_param->search_cmpl.conn_id, char_elem_result[0].char_handle, ESP_GATT_AUTH_REQ_NONE);
                if (err) ESP_LOGI(POOLOLOL_TAG, "characteristic found, but read request failed: %s", esp_err_to_name(err));
                
                break; // for loop, not switch
              }
            }
          }

          free(char_elem_result);
        } else {
          ESP_LOGE(POOLOLOL_TAG, "target service had no characteristics?");
        }
      }
      break;

    case ESP_GATTC_READ_CHAR_EVT:
        ESP_LOGI(POOLOLOL_TAG, "characteristic has been read; status=%x (%x); len=%d", cb_param->read.status, ESP_GATT_OK, cb_param->read.value_len);
        if (cb_param->read.status == ESP_GATT_OK && cb_param->read.value_len >= 2) {
          uint8_t twobytes[2] = { cb_param->read.value[0], cb_param->read.value[1] };
          transmit_temperature(twobytes);
        } else {
          ESP_LOGE(POOLOLOL_TAG, "error reading characteristic, or not enough bytes; %x %d", cb_param->read.status, cb_param->read.value_len);
        }
        reset_bt();
        break;

    case ESP_GATTC_SRVC_CHG_EVT: {
      esp_bd_addr_t bda;
      memcpy(bda, cb_param->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
      ESP_LOGI(POOLOLOL_TAG, "received a BT 'service changed' event? new(?) MAC is:");
      ESP_LOG_BUFFER_HEX(POOLOLOL_TAG, bda, sizeof(esp_bd_addr_t));
      break;
    }

    case ESP_GATTC_DISCONNECT_EVT:
      ESP_LOGI(POOLOLOL_TAG, "GATT disconnected; reason=%d", cb_param->disconnect.reason);
      if (connected) reset_bt();
      break;

    default:
      break;
  }
}

/** Callback for BT GAP (scanning and advertising, basically) events. */
static void bt_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *cb_param) {
  uint8_t *adv_name = NULL;
  uint8_t adv_name_len = 0;
  switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
      // kick off the first scan immediately & start timer for subsequent scans
      poll_once();
      ESP_ERROR_CHECK(esp_timer_start_periodic(poller_timer, POLL_PERIOD));
      break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
      switch (cb_param->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
          adv_name = esp_ble_resolve_adv_data(cb_param->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
          ESP_LOG_BUFFER_CHAR(POOLOLOL_TAG, adv_name, adv_name_len);
          if (adv_name != NULL) {
            if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {
              ESP_LOGI(POOLOLOL_TAG, "matched target device by name: %s", remote_device_name);
              if (connected == false) {
                connected = true;
                ESP_LOGI(POOLOLOL_TAG, "connecting to %s", remote_device_name);
                esp_ble_gap_stop_scanning();
                esp_ble_gattc_open(gattc_profile.gattc_if, cb_param->scan_rst.bda, cb_param->scan_rst.ble_addr_type, true);
              }
            }
          }
          break;

        default:
          break;
      }
      break;
    }

    case ESP_GAP_BLE_SCAN_TIMEOUT_EVT:
      break;

    default:
      break;
  }
}

void app_main() {
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);


  // create timers for the BT poller, and for the heartbeat sender
  const esp_timer_create_args_t bt_poller_timer_args = { .callback = (void *)&poll_once };
  ESP_ERROR_CHECK(esp_timer_create(&bt_poller_timer_args, &poller_timer));

  const esp_timer_create_args_t heartbeat_poller_timer_args = { .callback = (void *)&heartbeat};
  ESP_ERROR_CHECK(esp_timer_create(&heartbeat_poller_timer_args, &heartbeat_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(heartbeat_timer, 1000000ULL * CONFIG_POOLOLOL_MQTT_HEARTBEAT_PERIOD));


  // wiffies
  ESP_ERROR_CHECK(esp_netif_init());  // call only once, ever
  esp_netif_create_default_wifi_sta();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
  wifi_config_t wifi_config = {
      .sta = {
          .ssid = CONFIG_POOLOLOL_WIFI_SSID,
          .password = CONFIG_POOLOLOL_WIFI_WPA2_PASSWORD,
          .threshold.authmode = WIFI_AUTH_WPA2_PSK,
          .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
      },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_LOGE(POOLOLOL_TAG, "%s", "wifi started");


  // configure & start MQTT handler
  esp_mqtt_client_config_t mqtt_cfg = {.broker.address.uri = CONFIG_POOLOLOL_MQTT_URI};
  mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);  // last argument can pass data to handler
  esp_mqtt_client_start(mqtt_client);


  // the blue teeths
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
  ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
  ESP_ERROR_CHECK(esp_bluedroid_init());
  ESP_ERROR_CHECK(esp_bluedroid_enable());
  ESP_ERROR_CHECK(esp_ble_gap_register_callback(bt_gap_event_handler));
  ESP_ERROR_CHECK(esp_ble_gattc_register_callback(bt_gatt_event_handler));
  ESP_ERROR_CHECK(esp_ble_gattc_app_register(0)); // hard-coded app ID, but for us this is fine
  ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(500));
}
