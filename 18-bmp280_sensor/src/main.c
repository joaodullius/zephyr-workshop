#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bmp280_sensor, LOG_LEVEL_INF);

int main(void)
{
    const struct device *bmp280 = DEVICE_DT_GET_ONE(bosch_bme280);

    if (!device_is_ready(bmp280)) {
        LOG_ERR("BMP280 device not ready");
        return -1;
    }

    LOG_INF("BMP280 driver ready");

    while (1) {
        struct sensor_value temp, press;

        if (sensor_sample_fetch(bmp280) < 0) {
            LOG_ERR("Failed to fetch sample");
            return -1;
        }

        if (sensor_channel_get(bmp280, SENSOR_CHAN_AMBIENT_TEMP, &temp) < 0) {
            LOG_ERR("Failed to get temperature");
            return -1;
        }

        if (sensor_channel_get(bmp280, SENSOR_CHAN_PRESS, &press) < 0) {
            LOG_ERR("Failed to get pressure");
            return -1;
        }

        LOG_INF("Temperature: %0.3f °C", sensor_value_to_double(&temp));
        LOG_INF("Pressure: %0.3f hPa",sensor_value_to_double(&press));
        
        k_sleep(K_MSEC(1000));
    }
}
