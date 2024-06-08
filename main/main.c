#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "mqtt_client.h"
#include "nvs_flash.h"

static const char *TAG = "wesp32lel";

static esp_mqtt_client_handle_t mqtt_client;

static QueueHandle_t gpio_events = NULL;
#define DOOR_INPUT_PIN 23
#define DOOR_DEBOUNCE_INTERVAL_MS 250
#define DOOR_MQTT_TOPIC "/door/front"

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    ESP_LOGE(TAG, "%s", "wifi handler");
    if (event_base != WIFI_EVENT) return;
    switch (event_id) {
    case WIFI_EVENT_STA_START: // fallthrough
    case WIFI_EVENT_STA_DISCONNECTED:
        ESP_LOGE(TAG, "%s", "starting wifi connect");
        esp_wifi_connect();
    default:
        break;
    }
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;
    ESP_LOGI(TAG, "\tNetmask: " IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "\tGateway: " IPSTR, IP2STR(&ip_info->gw));
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    char buf[128];

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "/foo/bar", "connected", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/foo/qux", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/foo/bar", "subscribed", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_DATA:
        snprintf(buf, 128, "received '%.*s' on %.*s", event->data_len, event->data, event->topic_len, event->topic);
        ESP_LOGI(TAG, "%s", buf);
        esp_mqtt_client_publish(client, "/foo/bar", buf, 0, 0, 0);
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;

    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void IRAM_ATTR gpio_isr(void* arg) {
    uint32_t tmp = (uint32_t)arg;
    if ((uint32_t)arg == DOOR_INPUT_PIN) {
        xQueueSendFromISR(gpio_events, &tmp, NULL);
    }
}

static int last_gpio_level;

static void transmit_door_state(TimerHandle_t timer) {
    char payload[128];
    int state = (int)pvTimerGetTimerID(timer);
    if (last_gpio_level == state) return;
    last_gpio_level = state;
    snprintf(payload, 128, "{\"state\": \"%s\"}", state ? "closed" : "opened");
    esp_mqtt_client_publish(mqtt_client, DOOR_MQTT_TOPIC, payload, 0, 0, 0);
}

static void gpio_handler(void* arg) {
    uint32_t pin;
    int cur_level;
    TimerHandle_t timer = xTimerCreate("debounce", DOOR_DEBOUNCE_INTERVAL_MS / portTICK_PERIOD_MS, pdFALSE, (void*)0, transmit_door_state);
    for (;;) {
        if (xQueueReceive(gpio_events, &pin, portMAX_DELAY)) {
            if (pin != DOOR_INPUT_PIN) continue;
            cur_level = gpio_get_level(pin);
            if (cur_level == last_gpio_level) {
                if (xTimerIsTimerActive(timer)) xTimerStop(timer, 1);
            } else {
                vTimerSetTimerID(timer, (void*)cur_level);
                xTimerStart(timer, 1);
            }
        }
    }
}

void app_main() {
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // init TCP stack
    ESP_ERROR_CHECK(esp_netif_init()); // call only once, ever

    #ifdef CONFIG_USE_ETHERNET
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH(); // defaults to DHCP(?)
    esp_netif_t *eth_netif = esp_netif_new(&cfg);

    // init Ethernet MAC
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp32_emac_config.smi_mdc_gpio_num = CONFIG_EXAMPLE_ETH_MDC_GPIO;
    esp32_emac_config.smi_mdio_gpio_num = CONFIG_EXAMPLE_ETH_MDIO_GPIO;
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);

    // init Ethernet phy
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = CONFIG_EXAMPLE_ETH_PHY_ADDR;
    phy_config.reset_gpio_num = CONFIG_EXAMPLE_ETH_PHY_RST_GPIO;
    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config); // needs to be rtl82xx for later wESP32s

    // final Ethernet config struct from MAC and phy
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &eth_handle));

    // attach Ethernet to TCP/IP stack, register event handlers, and start up Ethernet
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
    #endif

    #ifdef CONFIG_USE_WIFI
    // wiffies
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID, .password = CONFIG_WIFI_WPA2_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK, .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGE(TAG, "%s", "wifi started");
    #endif

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    // configure & start MQTT handler
    esp_mqtt_client_config_t mqtt_cfg = { .broker.address.uri = "mqtt://medea" };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL); // last argument can pass data to handler
    esp_mqtt_client_start(mqtt_client);

    // configure GPIO, queue, handler, and ISR
    gpio_events = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_handler, "gpio_handler_task", 8192, NULL, 10, NULL);
    gpio_config_t io_config = {};
    io_config.pin_bit_mask = 1ULL << DOOR_INPUT_PIN;
    io_config.mode = GPIO_MODE_INPUT;
    io_config.intr_type = GPIO_INTR_ANYEDGE;
    io_config.pull_down_en = 1;
    ESP_ERROR_CHECK(gpio_config(&io_config));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    gpio_isr_handler_add(DOOR_INPUT_PIN, gpio_isr, (void*)DOOR_INPUT_PIN);
}
