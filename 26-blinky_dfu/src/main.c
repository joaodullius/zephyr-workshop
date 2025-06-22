#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(blinky_dfu, LOG_LEVEL_INF);

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA, 0x01, 0x02, 0x03, 0x04),
};

/* Set Scan Response data */
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
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

static void connected(struct bt_conn *conn, uint8_t err)
{
	struct bt_conn_info info;
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_INF("Connection failed (err %u)\n", err);
		return;
	} else if (bt_conn_get_info(conn, &info)) {
		LOG_INF("Could not parse connection info\n");
	} else {
		bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

		LOG_INF("Connection established!");
		LOG_INF("Connected to: %s", addr);
		LOG_INF("Role: %u", info.role);
		LOG_INF("Connection interval: %u", info.le.interval);
		LOG_INF("Slave latency: %u", info.le.latency);
		LOG_INF("Connection supervisory timeout: %u", info.le.timeout);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason %u)", reason);
	// Additional disconnection handling code
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};


static void bt_ready(int err)
{
	char addr_s[BT_ADDR_LE_STR_LEN];
	bt_addr_le_t addr = {0};
	size_t count = 1;

	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	/* Start advertising */

#ifdef CONFIG_BT_PERIPHERAL
	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
#else
	err = bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
#endif
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}


	/* For connectable advertising you would use
	 * bt_le_oob_get_local().  For non-connectable non-identity
	 * advertising an non-resolvable private address is used;
	 * there is no API to retrieve that.
	 */

	bt_id_get(&addr, &count);
	bt_addr_le_to_str(&addr, addr_s, sizeof(addr_s));

	printk("Beacon started, advertising as %s\n", addr_s);
}

int main(void)
{
    int err;

    printk("Inicializando Blinky DFU\n");

    if (!device_is_ready(led0.port) || !device_is_ready(led1.port)) {
        printk("Erro: LED não está pronto\n");
        return -1;
    }

    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);

	bt_conn_cb_register(&conn_callbacks);

	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}
	return 0;
}
