#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

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

int main(void)
{
    int err;
    const struct device *bmp280 = DEVICE_DT_GET_ONE(bosch_bme280);

    if (!device_is_ready(bmp280)) {
        LOG_ERR("BMP280 device not ready");
        return 0;
    }

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

    LOG_INF("Advertising started");

    while (1) {
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

        LOG_INF("Temp: %d.%02d C, Press: %d hPa", temp_c / 100, temp_c % 100, press_hpa);

        k_sleep(K_SECONDS(1));
    }
}
