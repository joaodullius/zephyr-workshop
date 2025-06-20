#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(bmp280_ble, LOG_LEVEL_INF);

static const struct bt_le_adv_param adv_param =
    BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_USE_IDENTITY,
                         BT_GAP_ADV_FAST_INT_MIN_2,
                         BT_GAP_ADV_FAST_INT_MAX_2,
                         NULL);

static uint8_t manuf_data[4];

static struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, manuf_data, sizeof(manuf_data)),
};

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback button_cb_data;
static struct k_work button_work;
static bool adv_enabled;

static const struct device *bmp280;

#define STACK_SIZE 1024
#define PRIORITY 5

void sensor_thread(void);
void blink_thread(void);

K_THREAD_DEFINE(sensor_tid, STACK_SIZE, sensor_thread, NULL, NULL, NULL,
                PRIORITY, 0, 0);
K_THREAD_DEFINE(blink_tid, STACK_SIZE, blink_thread, NULL, NULL, NULL,
                PRIORITY, 0, 0);

void button_work_handler(struct k_work *work)
{
    int err;

    if (adv_enabled) {
        err = bt_le_adv_stop();
        if (err) {
            LOG_ERR("Failed to stop advertising (%d)", err);
            return;
        }
        adv_enabled = false;
        LOG_INF("Advertising stopped");
    } else {
        err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
        if (err) {
            LOG_ERR("Failed to start advertising (%d)", err);
            return;
        }
        adv_enabled = true;
        LOG_INF("Advertising started");
    }
}

void button_pressed_isr(const struct device *dev, struct gpio_callback *cb,
                        uint32_t pins)
{
    k_work_submit(&button_work);
}

void sensor_thread(void)
{
    while (1) {
        if (!adv_enabled) {
            k_sleep(K_SECONDS(1));
            continue;
        }

        struct sensor_value temp, press;

        if (sensor_sample_fetch(bmp280) < 0) {
            LOG_ERR("Failed to fetch sample");
            k_sleep(K_SECONDS(1));
            continue;
        }

        sensor_channel_get(bmp280, SENSOR_CHAN_AMBIENT_TEMP, &temp);
        sensor_channel_get(bmp280, SENSOR_CHAN_PRESS, &press);

        int16_t temp_c = (int16_t)(sensor_value_to_double(&temp) * 100);
        uint16_t press_hpa = (uint16_t)sensor_value_to_double(&press);

        manuf_data[0] = temp_c >> 8;
        manuf_data[1] = temp_c & 0xFF;
        manuf_data[2] = press_hpa >> 8;
        manuf_data[3] = press_hpa & 0xFF;

        bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);

        LOG_INF("Temp: %d.%02d C, Press: %d hPa", temp_c / 100, temp_c % 100,
                press_hpa);

        k_sleep(K_SECONDS(1));
    }
}

void blink_thread(void)
{
    while (1) {
        gpio_pin_toggle_dt(&led);
        k_msleep(500);
    }
}

int main(void)
{
    int err;

    bmp280 = DEVICE_DT_GET_ONE(bosch_bme280);

    if (!device_is_ready(bmp280) || !device_is_ready(led.port) ||
        !device_is_ready(button.port)) {
        LOG_ERR("Device not ready");
        return 0;
    }

    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&button, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&button_cb_data, button_pressed_isr, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);

    k_work_init(&button_work, button_work_handler);

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (%d)", err);
        return 0;
    }

    LOG_INF("Bluetooth initialized");

    err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed to start (%d)", err);
        return 0;
    }

    adv_enabled = true;
    LOG_INF("Advertising started");

    while (1) {
        k_sleep(K_FOREVER);
    }
}
