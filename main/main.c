#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "ds18b20.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_rom_crc.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "onewire_bus.h"

// NimBLE BLE Headers
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// --- Configuration Constants (Defaults) ---
#define PIN_GRIP_LEFT    GPIO_NUM_18
#define PIN_GRIP_RIGHT   GPIO_NUM_20
#define PIN_SEAT         GPIO_NUM_19
#define PIN_TEMP_SENSOR  GPIO_NUM_17
#define PIN_VOLT_SENSE   GPIO_NUM_0
#define PIN_OIL_PRESSURE GPIO_NUM_1
#define PIN_LED          GPIO_NUM_15
#define PIN_ANT_SW       GPIO_NUM_14
#define PIN_ANT_SW_EN    GPIO_NUM_3

#define VOLTAGE_DIVIDER_R1 33000.0f
#define VOLTAGE_DIVIDER_R2 6800.0f

// PWM Settings
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES       LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY      100
#define LEDC_CHANNEL_GRIP_L LEDC_CHANNEL_0
#define LEDC_CHANNEL_GRIP_R LEDC_CHANNEL_1
#define LEDC_CHANNEL_SEAT   LEDC_CHANNEL_2

// Debounce settings
#define INHIBIT_DEBOUNCE_US 1000000LL

// BLE UUIDs
static const ble_uuid128_t svc_uuid = BLE_UUID128_INIT(0x4b, 0x91, 0x31, 0xc3, 0xc9, 0xc5, 0xcc,
						       0x8f, 0x9e, 0x45, 0xb5, 0x1f, 0x01, 0xc2,
						       0xaf, 0x4f);

static const ble_uuid128_t chr_config_uuid = BLE_UUID128_INIT(0xa8, 0x26, 0x1b, 0x36, 0x07, 0xea,
							      0xf5, 0xb7, 0x88, 0x46, 0xe1, 0x36,
							      0x3e, 0x48, 0xb5, 0xbe);

static const ble_uuid128_t chr_monitor_uuid = BLE_UUID128_INIT(0xa9, 0x26, 0x1b, 0x36, 0x07, 0xea,
							       0xf5, 0xb7, 0x88, 0x46, 0xe1, 0x36,
							       0x3e, 0x48, 0xb5, 0xbe);

static const char *TAG = "motoheato";

struct device_config {
	float voltage_threshold;
	float voltage_hysteresis;
	int start_delay_seconds;
	float grip_left_factor;
	float grip_right_factor;
	float seat_factor;

	// Temperature-based Power
	float temp_setpoint;
	float temp_proportional_gain;
	float temp_max_power;

	// Preheat
	float preheat_gain;
	int preheat_max_time;

	// Features
	uint8_t oil_pressure_enabled;
	uint8_t reserved[3];

	uint32_t crc;
} __attribute__((packed));

static struct device_config g_config;

enum system_state {
	STATE_BOOT,
	STATE_WAIT_START_COND,
	STATE_PREHEAT,
	STATE_RUNNING,
	STATE_INHIBITED
};

struct app_state {
	volatile enum system_state state;
	enum system_state previous_state;
	float voltage;
	float temperature;
	volatile bool oil_pressure_ok;
	int preheat_time_seconds;
	int64_t state_entry_time;
};

static struct app_state g_state = {.state = STATE_BOOT,
				   .previous_state = STATE_BOOT,
				   .voltage = 0.0f,
				   .temperature = 25.0f, // Safe default
				   .oil_pressure_ok = false,
				   .preheat_time_seconds = 0,
				   .state_entry_time = 0};

struct monitoring_data {
	uint16_t company_id;
	uint8_t state;
	uint8_t oil_pressure_ok;
	uint16_t voltage_x100;
	int16_t temp_x100;
	uint16_t duty_l;
	uint16_t duty_r;
	uint16_t duty_s;
} __attribute__((packed));

static adc_oneshot_unit_handle_t adc_handle;
static ds18b20_device_handle_t ds18b20_handle = NULL;
static uint8_t g_ble_addr_type;
static bool g_ble_advertising = false;
static uint16_t g_ble_conn_handle = 0;
static bool g_ble_connected = false;
static uint16_t g_monitor_val_handle;
static TaskHandle_t g_ble_mon_task_handle = NULL;

// --- Helpers ---

static uint32_t calculate_config_crc(const struct device_config *cfg) {
	return esp_rom_crc32_le(0, (uint8_t *)cfg, offsetof(struct device_config, crc));
}

static void save_config(void) {
	nvs_handle_t my_handle;
	if (nvs_open("storage", NVS_READWRITE, &my_handle) != ESP_OK)
		return;
	g_config.crc = calculate_config_crc(&g_config);
	if (nvs_set_blob(my_handle, "config", &g_config, sizeof(struct device_config)) == ESP_OK) {
		nvs_commit(my_handle);
		ESP_LOGI(TAG, "Config saved to NVS (CRC: 0x%08lx)", g_config.crc);
	}
	nvs_close(my_handle);
}

static void load_config(void) {
	// Hardcoded defaults
	g_config.voltage_threshold = 13.5f;
	g_config.voltage_hysteresis = 0.5f;
	g_config.start_delay_seconds = 10;
	g_config.grip_left_factor = 1.0f;
	g_config.grip_right_factor = 0.9f;
	g_config.seat_factor = 1.0f;

	g_config.temp_setpoint = 15.0f;
	g_config.temp_proportional_gain = 5.0f; // e.g., 5% per degree below setpoint
	g_config.temp_max_power = 1.0f;

	g_config.preheat_gain = 20.0f; // e.g., 20s per degree below setpoint
	g_config.preheat_max_time = 300;

	g_config.oil_pressure_enabled = 1;

	nvs_handle_t my_handle;
	if (nvs_open("storage", NVS_READWRITE, &my_handle) != ESP_OK)
		return;
	size_t required_size = sizeof(struct device_config);
	struct device_config loaded_config;
	if (nvs_get_blob(my_handle, "config", &loaded_config, &required_size) == ESP_OK) {
		if (calculate_config_crc(&loaded_config) == loaded_config.crc) {
			memcpy(&g_config, &loaded_config, sizeof(struct device_config));
			ESP_LOGI(TAG, "Config loaded from NVS (CRC OK)");
		}
	} else {
		save_config();
	}
	nvs_close(my_handle);
}

// --- BLE Support ---

static void fill_monitor(struct monitoring_data *mon) {
	mon->company_id = 0xFFFF;
	mon->state = (uint8_t)g_state.state;
	mon->oil_pressure_ok = (uint8_t)g_state.oil_pressure_ok;
	mon->voltage_x100 = (uint16_t)(g_state.voltage * 100);
	mon->temp_x100 = (int16_t)(g_state.temperature * 100);
	mon->duty_l = (uint16_t)ledc_get_duty(LEDC_MODE, LEDC_CHANNEL_GRIP_L);
	mon->duty_r = (uint16_t)ledc_get_duty(LEDC_MODE, LEDC_CHANNEL_GRIP_R);
	mon->duty_s = (uint16_t)ledc_get_duty(LEDC_MODE, LEDC_CHANNEL_SEAT);
}

static void ble_send_notification(void) {
	if (!g_ble_connected) {
		return;
	}

	struct monitoring_data mon;
	fill_monitor(&mon);
	struct os_mbuf *om = ble_hs_mbuf_from_flat(&mon, sizeof(mon));
	ble_gattc_notify_custom(g_ble_conn_handle, g_monitor_val_handle, om);
}

static int device_config_access(uint16_t conn_handle, uint16_t attr_handle,
				struct ble_gatt_access_ctxt *ctxt, void *arg) {
	if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
		os_mbuf_append(ctxt->om, &g_config, sizeof(g_config));
		return 0;
	}
	if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
		if (ctxt->om->om_len == sizeof(struct device_config)) {
			struct device_config new_cfg;
			memcpy(&new_cfg, ctxt->om->om_data, sizeof(new_cfg));
			if (calculate_config_crc(&new_cfg) == new_cfg.crc) {
				memcpy(&g_config, &new_cfg, sizeof(g_config));
				save_config();
				return 0;
			}
		}
		return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
	}
	return BLE_ATT_ERR_UNLIKELY;
}

static int device_monitor_access(uint16_t conn_handle, uint16_t attr_handle,
				 struct ble_gatt_access_ctxt *ctxt, void *arg) {
	struct monitoring_data mon;
	fill_monitor(&mon);
	os_mbuf_append(ctxt->om, &mon, sizeof(mon));
	return 0;
}

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
	{
		.type = BLE_GATT_SVC_TYPE_PRIMARY,
		.uuid = &svc_uuid.u,
		.characteristics =
			(struct ble_gatt_chr_def[]){{
							    .uuid = &chr_config_uuid.u,
							    .access_cb = device_config_access,
							    .flags = BLE_GATT_CHR_F_READ |
								     BLE_GATT_CHR_F_WRITE,
						    },
						    {
							    .uuid = &chr_monitor_uuid.u,
							    .access_cb = device_monitor_access,
							    .flags = BLE_GATT_CHR_F_READ |
								     BLE_GATT_CHR_F_NOTIFY,
							    .val_handle = &g_monitor_val_handle,
						    },
						    {0}},
	},
	{0},
};

static void ble_monitor_task(void *pvParameters) {
	ESP_LOGI(TAG, "BLE Monitor Task started");
	while (1) {
		ble_send_notification();
		ESP_LOGI(TAG, "S: %d | O: %d | V: %.2f | T: %.2f | PWM: %lu/%lu/%lu", g_state.state,
			 g_state.oil_pressure_ok, g_state.voltage, g_state.temperature,
			 ledc_get_duty(LEDC_MODE, LEDC_CHANNEL_GRIP_L),
			 ledc_get_duty(LEDC_MODE, LEDC_CHANNEL_GRIP_R),
			 ledc_get_duty(LEDC_MODE, LEDC_CHANNEL_SEAT));
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

static int ble_gap_event(struct ble_gap_event *event, void *arg);

static void ble_app_advertise(void) {
	if (g_ble_connected || g_ble_advertising)
		return;

	struct ble_hs_adv_fields adv_fields;
	const char *device_name = "Motoheato";
	memset(&adv_fields, 0, sizeof(adv_fields));
	adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
	adv_fields.name = (uint8_t *)device_name;
	adv_fields.name_len = strlen(device_name);
	adv_fields.name_is_complete = 1;

	ble_gap_adv_set_fields(&adv_fields);

	struct ble_gap_adv_params adv_params;
	memset(&adv_params, 0, sizeof(adv_params));
	adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
	adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
	if (ble_gap_adv_start(g_ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event,
			      NULL) == 0) {
		g_ble_advertising = true;
	}
}

static int ble_gap_event(struct ble_gap_event *event, void *arg) {
	switch (event->type) {
	case BLE_GAP_EVENT_CONNECT:
		if (event->connect.status == 0) {
			g_ble_connected = true;
			g_ble_conn_handle = event->connect.conn_handle;
			g_ble_advertising = false;
			if (g_ble_mon_task_handle == NULL) {
				xTaskCreate(ble_monitor_task, "ble_mon_task", 2048, NULL, 5,
					    &g_ble_mon_task_handle);
			}
		} else {
			g_ble_connected = false;
			g_ble_advertising = false;
			ble_app_advertise();
		}
		break;
	case BLE_GAP_EVENT_DISCONNECT:
		g_ble_connected = false;
		g_ble_advertising = false;
		if (g_ble_mon_task_handle != NULL) {
			vTaskDelete(g_ble_mon_task_handle);
			g_ble_mon_task_handle = NULL;
		}
		ble_app_advertise();
		break;
	case BLE_GAP_EVENT_ADV_COMPLETE:
		g_ble_advertising = false;
		break;
	}
	return 0;
}

static void ble_app_on_sync(void) {
	ble_hs_id_infer_auto(0, &g_ble_addr_type);
	ble_app_advertise();
}

static void ble_host_task(void *param) {
	nimble_port_run();
	nimble_port_freertos_deinit();
}

// --- Hardware Control ---

static void set_levels(void) {
	uint32_t duty_max = (1 << 13) - 1;
	uint32_t duty_l = 0, duty_r = 0, duty_s = 0;

	if (g_state.state == STATE_PREHEAT) {
		duty_l = duty_r = duty_s = duty_max;
	} else if (g_state.state == STATE_RUNNING) {
		// P-Control based on temperature
		float temp_error = g_config.temp_setpoint - g_state.temperature;
		float power = temp_error * (g_config.temp_proportional_gain / 100.0f);
		power = fmaxf(0.0f, fminf(g_config.temp_max_power, power));

		duty_s = (uint32_t)(duty_max * power * g_config.seat_factor);
		duty_l = (uint32_t)(duty_max * power * g_config.grip_left_factor);
		duty_r = (uint32_t)(duty_max * power * g_config.grip_right_factor);
	}

	ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_GRIP_L, duty_l);
	ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_GRIP_L);
	ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_GRIP_R, duty_r);
	ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_GRIP_R);
	ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_SEAT, duty_s);
	ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_SEAT);
	gpio_set_level(PIN_LED, g_state.state != STATE_PREHEAT && g_state.state != STATE_RUNNING);
}

static bool check_inhibit(uint32_t now) {
	if (g_state.state == STATE_INHIBITED)
		return false;

	bool voltage_ok = g_state.voltage >=
			  (g_config.voltage_threshold - g_config.voltage_hysteresis);
	bool oil_ok = !g_config.oil_pressure_enabled || g_state.oil_pressure_ok;

	if (!voltage_ok || !oil_ok) {
		g_state.previous_state = g_state.state;
		g_state.state = STATE_INHIBITED;
		g_state.state_entry_time = now;
		return true;
	}

	return false;
}

static void IRAM_ATTR oil_pressure_isr_handler(void *arg) {
	g_state.oil_pressure_ok = gpio_get_level(PIN_OIL_PRESSURE) == 0;
	if (check_inhibit(esp_timer_get_time())) {
		set_levels();
	}
}

static bool init_hardware(void) {
	gpio_config_t led_conf = {.pin_bit_mask = (1ULL << PIN_LED), .mode = GPIO_MODE_OUTPUT};
	gpio_config(&led_conf);
	gpio_set_level(PIN_LED, 1);

	gpio_config_t io_conf = {.pin_bit_mask = (1ULL << PIN_OIL_PRESSURE),
				 .mode = GPIO_MODE_INPUT,
				 .pull_up_en = 1,
				 .intr_type = GPIO_INTR_ANYEDGE};
	gpio_config(&io_conf);
	gpio_install_isr_service(0);
	gpio_isr_handler_add(PIN_OIL_PRESSURE, oil_pressure_isr_handler, NULL);

	adc_oneshot_unit_handle_t handle;
	adc_oneshot_unit_init_cfg_t init_config1 = {.unit_id = ADC_UNIT_1,
						    .clk_src = ADC_DIGI_CLK_SRC_DEFAULT};
	ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &handle));
	adc_handle = handle;
	adc_oneshot_chan_cfg_t config = {.bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12};
	ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, PIN_VOLT_SENSE, &config));

	ledc_timer_config_t ledc_timer = {.speed_mode = LEDC_MODE,
					  .timer_num = LEDC_TIMER,
					  .duty_resolution = LEDC_DUTY_RES,
					  .freq_hz = LEDC_FREQUENCY,
					  .clk_cfg = LEDC_AUTO_CLK};
	ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
	uint32_t phase = (1 << 13) / 3;
	ledc_channel_config_t ledc_channel[] = {{
							.channel = LEDC_CHANNEL_GRIP_L,
							.gpio_num = PIN_GRIP_LEFT,
							.speed_mode = LEDC_MODE,
							.timer_sel = LEDC_TIMER,
							.hpoint = 0,
						},
						{.channel = LEDC_CHANNEL_GRIP_R,
						 .gpio_num = PIN_GRIP_RIGHT,
						 .speed_mode = LEDC_MODE,
						 .timer_sel = LEDC_TIMER,
						 .hpoint = phase},
						{.channel = LEDC_CHANNEL_SEAT,
						 .gpio_num = PIN_SEAT,
						 .speed_mode = LEDC_MODE,
						 .timer_sel = LEDC_TIMER,
						 .hpoint = phase * 2}};

	for (int i = 0; i < 3; i++) {
		ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel[i]));
	}

	onewire_bus_handle_t bus = NULL;
	onewire_bus_config_t bus_config = {.bus_gpio_num = PIN_TEMP_SENSOR, .flags.en_pull_up = true};
	onewire_bus_rmt_config_t rmt_config = {.max_rx_bytes = 10};
	ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));
	onewire_device_iter_handle_t iter = NULL;
	onewire_device_t dev;
	ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
	if (onewire_device_iter_get_next(iter, &dev) != ESP_OK) {
		ESP_LOGE(TAG, "No DS18B20 found!");
		onewire_del_device_iter(iter);
		return false;
	}
	ds18b20_config_t ds_cfg = {};
	ESP_ERROR_CHECK(ds18b20_new_device_from_enumeration(&dev, &ds_cfg, &ds18b20_handle));
	ESP_ERROR_CHECK(ds18b20_set_resolution(ds18b20_handle, DS18B20_RESOLUTION_12B));
	onewire_del_device_iter(iter);

	gpio_config_t ant_en_conf = {.pin_bit_mask = (1ULL << PIN_ANT_SW_EN),
				     .mode = GPIO_MODE_OUTPUT};
	gpio_config(&ant_en_conf);
	gpio_set_level(PIN_ANT_SW_EN, 0);
	gpio_config_t ant_sel_conf = {.pin_bit_mask = (1ULL << PIN_ANT_SW), .mode = GPIO_MODE_OUTPUT};
	gpio_config(&ant_sel_conf);
	gpio_set_level(PIN_ANT_SW, 0);

	return true;
}

static void update_sensors(void) {
	int adc_raw;
	if (adc_oneshot_read(adc_handle, PIN_VOLT_SENSE, &adc_raw) == ESP_OK) {
		float v_pin = (adc_raw / 4095.0f) * 3.3f * 1.06f;
		g_state.voltage =
			(v_pin * ((VOLTAGE_DIVIDER_R1 + VOLTAGE_DIVIDER_R2) / VOLTAGE_DIVIDER_R2) *
			 1.1325f);
	}
	g_state.oil_pressure_ok = gpio_get_level(PIN_OIL_PRESSURE) == 0;
}

// --- Sensor Tasks ---

static void ds18b20_task(void *pvParameters) {
	while (1) {
		ds18b20_trigger_temperature_conversion(ds18b20_handle);
		// Conversion takes up to 750ms
		vTaskDelay(pdMS_TO_TICKS(800));
		float temp;
		if (ds18b20_get_temperature(ds18b20_handle, &temp) == ESP_OK) {
			g_state.temperature = temp;
		}
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}

// --- Main State Machine Task ---

static void state_machine_loop(void *pvParameters) {
	TickType_t last_wake_time = xTaskGetTickCount();
	while (1) {
		update_sensors();
		int64_t now = esp_timer_get_time();
		switch (g_state.state) {
		case STATE_BOOT:
			g_state.state = STATE_WAIT_START_COND;
			g_state.state_entry_time = now;
			break;
		case STATE_WAIT_START_COND:
			if (g_state.voltage >= g_config.voltage_threshold &&
			    (!g_config.oil_pressure_enabled || g_state.oil_pressure_ok)) {
				if ((now - g_state.state_entry_time) >
				    (int64_t)g_config.start_delay_seconds * 1000000LL) {
					g_state.state = STATE_PREHEAT;
					g_state.state_entry_time = now;

					float preheat_error =
						g_config.temp_setpoint - g_state.temperature;
					float preheat_time = preheat_error * g_config.preheat_gain;
					g_state.preheat_time_seconds =
						(int)fmaxf(0.0f,
							   fminf((float)g_config.preheat_max_time,
								 preheat_time));
				}
			} else {
				g_state.state_entry_time = now;
			}
			break;
		case STATE_PREHEAT:
			if ((now - g_state.state_entry_time) >
			    (int64_t)g_state.preheat_time_seconds * 1000000LL) {
				g_state.state = STATE_RUNNING;
				g_state.state_entry_time = now;
			}
			break;
		case STATE_RUNNING:
			break;
		case STATE_INHIBITED:
			if (g_state.voltage >= g_config.voltage_threshold &&
			    (!g_config.oil_pressure_enabled || g_state.oil_pressure_ok)) {
				if ((now - g_state.state_entry_time) < INHIBIT_DEBOUNCE_US) {
					g_state.state = g_state.previous_state;
				} else {
					g_state.state = STATE_WAIT_START_COND;
					g_state.state_entry_time = now;
				}
			}
			break;
		}

		check_inhibit(now);
		set_levels();

		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100));
	}
}

void app_main(void) {
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
	load_config();

	esp_pm_config_t pm_config = {.max_freq_mhz = 80,
				     .min_freq_mhz = 40,
				     .light_sleep_enable = true};
	ESP_ERROR_CHECK(esp_pm_configure(&pm_config));

	if (!init_hardware()) {
		// Error handler: Blink LED at 1Hz if DS18B20 not found
		while (1) {
			gpio_set_level(PIN_LED, 0); // ON
			vTaskDelay(pdMS_TO_TICKS(500));
			gpio_set_level(PIN_LED, 1); // OFF
			vTaskDelay(pdMS_TO_TICKS(500));
		}
	}

	ESP_ERROR_CHECK(nimble_port_init());
	ble_svc_gap_device_name_set("Motoheato");
	ble_svc_gap_init();
	ble_svc_gatt_init();
	ble_gatts_count_cfg(gatt_svr_svcs);
	ble_gatts_add_svcs(gatt_svr_svcs);
	ble_hs_cfg.sync_cb = ble_app_on_sync;
	nimble_port_freertos_init(ble_host_task);

	xTaskCreate(ds18b20_task, "temp_task", 2048, NULL, 5, NULL);
	xTaskCreate(state_machine_loop, "control_task", 4096, NULL, 5, NULL);
}