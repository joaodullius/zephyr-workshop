#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

LOG_MODULE_REGISTER(blinky_thread_ble, LOG_LEVEL_INF);

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

static const struct bt_le_adv_param adv_param =
    BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_USE_IDENTITY,
                         BT_GAP_ADV_FAST_INT_MIN_2,
                         BT_GAP_ADV_FAST_INT_MAX_2,
                         NULL);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA, 0x01, 0x02, 0x03, 0x04),
};

void thread_led0(void)
{
    while (1) {
        gpio_pin_toggle_dt(&led0);
        k_msleep(500);
    }
}

void thread_led1(void)
{
    while (1) {
        gpio_pin_toggle_dt(&led1);
        k_msleep(1000);
    }
}

#define STACK_SIZE 512
#define PRIORITY 5

K_THREAD_DEFINE(led0_tid, STACK_SIZE, thread_led0, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(led1_tid, STACK_SIZE, thread_led1, NULL, NULL, NULL, PRIORITY, 0, 0);

int main(void)
{
    int err;

    LOG_INF("Inicializando LEDs e Bluetooth...");

    if (!device_is_ready(led0.port) || !device_is_ready(led1.port)) {
        LOG_ERR("Erro: LED não está pronto");
        return -1;
    }

    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (%d)", err);
        return -1;
    }

    err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed to start (%d)", err);
        return -1;
    }

    return 0;
}
