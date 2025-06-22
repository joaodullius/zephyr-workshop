#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bmp280_ble_sensor, LOG_LEVEL_INF);

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define BUTTON0_NODE DT_ALIAS(sw0)
#define BUTTON1_NODE DT_ALIAS(sw1)

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(BUTTON0_NODE, gpios);
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(BUTTON1_NODE, gpios);

static struct gpio_callback button0_cb_data;
static struct gpio_callback button1_cb_data;

static struct k_work sample_work;
static struct k_timer sample_timer;
static bool sampling_enabled;

static bool adv_enabled;
static struct k_work adv_work;

static const struct device *bmp280;

static uint8_t manuf_data[4];

static const struct bt_le_adv_param adv_param =
    BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_USE_IDENTITY,
                         BT_GAP_ADV_FAST_INT_MIN_2,
                         BT_GAP_ADV_FAST_INT_MAX_2,
                         NULL);

static struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, manuf_data, sizeof(manuf_data)),
};

static void sample_work_handler(struct k_work *work)
{
    struct sensor_value temp, press;

    if (sensor_sample_fetch(bmp280) < 0) {
        LOG_ERR("Failed to fetch sample");
        return;
    }

    sensor_channel_get(bmp280, SENSOR_CHAN_AMBIENT_TEMP, &temp);
    sensor_channel_get(bmp280, SENSOR_CHAN_PRESS, &press);

    double temp_d = sensor_value_to_double(&temp);
    double press_d = sensor_value_to_double(&press);

    LOG_INF("Temp: %0.2f C, Press: %0.2f hPa", temp_d, press_d);

    int16_t temp_c = (int16_t)(temp_d * 100);
    uint16_t press_hpa = (uint16_t)press_d;

    manuf_data[0] = temp_c >> 8;
    manuf_data[1] = temp_c & 0xFF;
    manuf_data[2] = press_hpa >> 8;
    manuf_data[3] = press_hpa & 0xFF;

    LOG_INF("manuf_data: %02x %02x %02x %02x", manuf_data[0], manuf_data[1],
            manuf_data[2], manuf_data[3]);

    if (adv_enabled) {
        bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
    }

    gpio_pin_set_dt(&led0, 1);
    k_msleep(100);
    gpio_pin_set_dt(&led0, 0);
}

static void sample_timer_handler(struct k_timer *timer)
{
    k_work_submit(&sample_work);
}

static void button0_pressed_isr(const struct device *dev, struct gpio_callback *cb,
                                uint32_t pins)
{
    if (sampling_enabled) {
        k_timer_stop(&sample_timer);
        sampling_enabled = false;
        LOG_INF("Sampling stopped");
    } else {
        k_timer_start(&sample_timer, K_NO_WAIT, K_SECONDS(5));
        sampling_enabled = true;
        LOG_INF("Sampling started");
    }
}

static void adv_work_handler(struct k_work *work)
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
            LOG_ERR("Advertising failed to start (%d)", err);
            return;
        }

        adv_enabled = true;
        LOG_INF("Advertising started");
    }
}

static void button1_pressed_isr(const struct device *dev, struct gpio_callback *cb,
                                uint32_t pins)
{
    k_work_submit(&adv_work);
}

void advertising_led_thread(void)
{
    while (1) {
        if (adv_enabled) {
            gpio_pin_toggle_dt(&led1);
            k_msleep(500);
        } else {
            gpio_pin_set_dt(&led1, 0);
            k_msleep(100);
        }
    }
}

#define STACK_SIZE 512
#define PRIORITY 5

K_THREAD_DEFINE(adv_led_tid, STACK_SIZE, advertising_led_thread, NULL, NULL, NULL,
                PRIORITY, 0, 0);

int main(void)
{
    int err;

    bmp280 = DEVICE_DT_GET_ONE(bosch_bme280);

    if (!device_is_ready(bmp280) || !device_is_ready(led0.port) ||
        !device_is_ready(led1.port) || !device_is_ready(button0.port) ||
        !device_is_ready(button1.port)) {
        LOG_ERR("Device not ready");
        return 0;
    }

    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);

    gpio_pin_configure_dt(&button0, GPIO_INPUT);
    gpio_pin_configure_dt(&button1, GPIO_INPUT);

    gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_pin_interrupt_configure_dt(&button1, GPIO_INT_EDGE_TO_ACTIVE);

    gpio_init_callback(&button0_cb_data, button0_pressed_isr, BIT(button0.pin));
    gpio_init_callback(&button1_cb_data, button1_pressed_isr, BIT(button1.pin));
    gpio_add_callback(button0.port, &button0_cb_data);
    gpio_add_callback(button1.port, &button1_cb_data);

    k_work_init(&sample_work, sample_work_handler);
    k_work_init(&adv_work, adv_work_handler);
    k_timer_init(&sample_timer, sample_timer_handler, NULL);

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (%d)", err);
        return 0;
    }

    LOG_INF("Bluetooth initialized");

    while (1) {
        k_sleep(K_FOREVER);
    }
}

