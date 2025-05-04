/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */
#include <uart_async_adapter.h>
#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>

#include <stdio.h>
#include <string.h>

//add int
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

//add blood pressure services
#define BT_UUID_BLOOD_PRESSURE         BT_UUID_DECLARE_16(0x1810)
#define BT_UUID_BLOOD_PRESSURE_MEAS    BT_UUID_DECLARE_16(0x2A35)

#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

#define CON_STATUS_LED DK_LED2

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

#define BATTERY_UPDATE_INTERVAL K_SECONDS(10)

#define BT_UUID_RACP_VAL 0x2A52
#define BT_UUID_RACP BT_UUID_DECLARE_16(BT_UUID_RACP_VAL)

//中斷
#define BUTTON_NODE DT_ALIAS(sw1)  // 使用 devicetree 定義的按鈕（例如 P0.13）
#define SW1_PIN 13  // P0.13
#define SW1_PORT "GPIO_0"


/* Simulate Battery's data */
static uint8_t battery_level = 100;
static uint8_t battery_state = 0b00000110;	// Charging, Good health
static uint8_t battery_status = 0b00000000; // No alarms

// Simulate blood pressure（mmHg, 120/80）
static uint8_t bp_data[] = {
    0x00,  // Flags
    120, 0, // Systolic
    80, 0,  // Diastolic
    100, 0, // MAP (mean arterial pressure)
    0, 0,   // Time Stamp (optional)
};


static void battery_level_update(struct k_timer *dummy);
static struct k_timer battery_timer;

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart));
static struct k_work_delayable uart_work;

static const char manufacturer_name_str[] = "Dynapack";
static const char model_number_str[] = "Model-1234";
static const char serial_number_str[] = "SN-56789";

// button
const struct device *sw1_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));

static uint8_t racp_data[20]; // �i�ھڻݨD�X�j

// Battery Health Status
static uint16_t cycle_count = 120;		   // 0x78	-> 7800
static int8_t battery_temp = 32;		   // 32	->20
static uint8_t soh = 95;				   // percent	5F
static uint16_t remaining_capacity = 2500; // mAh	9C4
static uint16_t estimated_runtime = 180;   // minutes	B4

//中斷
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(BUTTON_NODE, gpios, {0});
static struct gpio_callback sw1_cb_data;

static ssize_t write_racp(struct bt_conn *conn,
						  const struct bt_gatt_attr *attr,
						  const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	printk("RACP write received: ");
	memcpy(racp_data, buf, len);
	for (int i = 0; i < len; i++)
	{
		printk("%02X ", racp_data[i]);
	}
	printk("\n");

	// �o�̥i�H�ѪR opcode & operator�A��@�޿�
	// Ex: racp_data[0] �O opcode�Aracp_data[1] �O operator

	// �^���Ϊ� Indicate �аѦ� bt_gatt_indicate()

	return len;
}

static void racp_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	printk("RACP indication %s\n", value == BT_GATT_CCC_INDICATE ? "enabled" : "disabled");
}

struct uart_data_t
{
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

#ifdef CONFIG_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
#define async_adapter NULL
#endif

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	static size_t aborted_len;
	struct uart_data_t *buf;
	static uint8_t *aborted_buf;
	static bool disable_req;

	switch (evt->type)
	{
	case UART_TX_DONE:
		LOG_DBG("UART_TX_DONE");
		if ((evt->data.tx.len == 0) ||
			(!evt->data.tx.buf))
		{
			return;
		}

		if (aborted_buf)
		{
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
							   data[0]);
			aborted_buf = NULL;
			aborted_len = 0;
		}
		else
		{
			buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t,
							   data[0]);
		}

		k_free(buf);

		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		if (!buf)
		{
			return;
		}

		if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS))
		{
			LOG_WRN("Failed to send data over UART");
		}

		break;

	case UART_RX_RDY:
		LOG_DBG("UART_RX_RDY");
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data[0]);
		buf->len += evt->data.rx.len;

		if (disable_req)
		{
			return;
		}

		if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
			(evt->data.rx.buf[buf->len - 1] == '\r'))
		{
			disable_req = true;
			uart_rx_disable(uart);
		}

		break;

	case UART_RX_DISABLED:
		LOG_DBG("UART_RX_DISABLED");
		disable_req = false;

		buf = k_malloc(sizeof(*buf));
		if (buf)
		{
			buf->len = 0;
		}
		else
		{
			LOG_WRN("Not able to allocate UART receive buffer");
			k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
			return;
		}

		uart_rx_enable(uart, buf->data, sizeof(buf->data),
					   UART_WAIT_FOR_RX);

		break;

	case UART_RX_BUF_REQUEST:
		LOG_DBG("UART_RX_BUF_REQUEST");
		buf = k_malloc(sizeof(*buf));
		if (buf)
		{
			buf->len = 0;
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
		}
		else
		{
			LOG_WRN("Not able to allocate UART receive buffer");
		}

		break;

	case UART_RX_BUF_RELEASED:
		LOG_DBG("UART_RX_BUF_RELEASED");
		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t,
						   data[0]);

		if (buf->len > 0)
		{
			k_fifo_put(&fifo_uart_rx_data, buf);
		}
		else
		{
			k_free(buf);
		}

		break;

	case UART_TX_ABORTED:
		LOG_DBG("UART_TX_ABORTED");
		if (!aborted_buf)
		{
			aborted_buf = (uint8_t *)evt->data.tx.buf;
		}

		aborted_len += evt->data.tx.len;
		buf = CONTAINER_OF((void *)aborted_buf, struct uart_data_t,
						   data);

		uart_tx(uart, &buf->data[aborted_len],
				buf->len - aborted_len, SYS_FOREVER_MS);

		break;

	default:
		break;
	}
}

static void uart_work_handler(struct k_work *item)
{
	struct uart_data_t *buf;

	buf = k_malloc(sizeof(*buf));
	if (buf)
	{
		buf->len = 0;
	}
	else
	{
		LOG_WRN("Not able to allocate UART receive buffer");
		k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
		return;
	}

	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
}

static bool uart_test_async_api(const struct device *dev)
{
	const struct uart_driver_api *api =
		(const struct uart_driver_api *)dev->api;

	return (api->callback_set != NULL);
}

static int uart_init(void)
{
	int err;
	int pos;
	struct uart_data_t *rx;
	struct uart_data_t *tx;

	if (!device_is_ready(uart))
	{
		return -ENODEV;
	}

	if (IS_ENABLED(CONFIG_USB_DEVICE_STACK))
	{
		err = usb_enable(NULL);
		if (err && (err != -EALREADY))
		{
			LOG_ERR("Failed to enable USB");
			return err;
		}
	}

	rx = k_malloc(sizeof(*rx));
	if (rx)
	{
		rx->len = 0;
	}
	else
	{
		return -ENOMEM;
	}

	k_work_init_delayable(&uart_work, uart_work_handler);

	if (IS_ENABLED(CONFIG_UART_ASYNC_ADAPTER) && !uart_test_async_api(uart))
	{
		/* Implement API adapter */
		uart_async_adapter_init(async_adapter, uart);
		uart = async_adapter;
	}

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err)
	{
		k_free(rx);
		LOG_ERR("Cannot initialize UART callback");
		return err;
	}

	if (IS_ENABLED(CONFIG_UART_LINE_CTRL))
	{
		LOG_INF("Wait for DTR");
		while (true)
		{
			uint32_t dtr = 0;

			uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr);
			if (dtr)
			{
				break;
			}
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
		}
		LOG_INF("DTR set");
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DCD, 1);
		if (err)
		{
			LOG_WRN("Failed to set DCD, ret code %d", err);
		}
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DSR, 1);
		if (err)
		{
			LOG_WRN("Failed to set DSR, ret code %d", err);
		}
	}

	tx = k_malloc(sizeof(*tx));

	if (tx)
	{
		pos = snprintf(tx->data, sizeof(tx->data),
					   "Starting Nordic UART service example\r\n");

		if ((pos < 0) || (pos >= sizeof(tx->data)))
		{
			k_free(rx);
			k_free(tx);
			LOG_ERR("snprintf returned %d", pos);
			return -ENOMEM;
		}

		tx->len = pos;
	}
	else
	{
		k_free(rx);
		return -ENOMEM;
	}

	err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
	if (err)
	{
		k_free(rx);
		k_free(tx);
		LOG_ERR("Cannot display welcome message (err: %d)", err);
		return err;
	}

	err = uart_rx_enable(uart, rx->data, sizeof(rx->data), UART_WAIT_FOR_RX);
	if (err)
	{
		LOG_ERR("Cannot enable uart reception (err: %d)", err);
		/* Free the rx buffer only because the tx buffer will be handled in the callback */
		k_free(rx);
	}

	return err;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err)
	{
		LOG_ERR("Connection failed, err 0x%02x %s", err, bt_hci_err_to_str(err));
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);

	current_conn = bt_conn_ref(conn);

	dk_set_led_on(CON_STATUS_LED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s, reason 0x%02x %s", addr, reason, bt_hci_err_to_str(reason));

	if (auth_conn)
	{
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn)
	{
		bt_conn_unref(current_conn);
		current_conn = NULL;
		dk_set_led_off(CON_STATUS_LED);
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
							 enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err)
	{
		LOG_INF("Security changed: %s level %u", addr, level);
	}
	else
	{
		LOG_WRN("Security failed: %s level %u err %d %s", addr, level, err,
				bt_security_err_to_str(err));
	}
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	auth_conn = bt_conn_ref(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);

	if (IS_ENABLED(CONFIG_SOC_SERIES_NRF54HX) || IS_ENABLED(CONFIG_SOC_SERIES_NRF54LX))
	{
		LOG_INF("Press Button 0 to confirm, Button 1 to reject.");
	}
	else
	{
		LOG_INF("Press Button 1 to confirm, Button 2 to reject.");
	}
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", addr);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing failed conn: %s, reason %d %s", addr, reason,
			bt_security_err_to_str(reason));
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
						  uint16_t len)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received data from: %s", addr);

	for (uint16_t pos = 0; pos != len;)
	{
		struct uart_data_t *tx = k_malloc(sizeof(*tx));

		if (!tx)
		{
			LOG_WRN("Not able to allocate UART send data buffer");
			return;
		}

		/* Keep the last byte of TX buffer for potential LF char. */
		size_t tx_data_size = sizeof(tx->data) - 1;

		if ((len - pos) > tx_data_size)
		{
			tx->len = tx_data_size;
		}
		else
		{
			tx->len = (len - pos);
		}

		memcpy(tx->data, &data[pos], tx->len);

		pos += tx->len;

		/* Append the LF character when the CR character triggered
		 * transmission from the peer.
		 */
		if ((pos == len) && (data[len - 1] == '\r'))
		{
			tx->data[tx->len] = '\n';
			tx->len++;
		}

		err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
		if (err)
		{
			k_fifo_put(&fifo_uart_tx_data, tx);
		}
	}
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

void error(void)
{
	dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);

	while (true)
	{
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void num_comp_reply(bool accept)
{
	if (accept)
	{
		bt_conn_auth_passkey_confirm(auth_conn);
		LOG_INF("Numeric Match, conn %p", (void *)auth_conn);
	}
	else
	{
		bt_conn_auth_cancel(auth_conn);
		LOG_INF("Numeric Reject, conn %p", (void *)auth_conn);
	}

	bt_conn_unref(auth_conn);
	auth_conn = NULL;
}

void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;

	if (auth_conn)
	{
		if (buttons & KEY_PASSKEY_ACCEPT)
		{
			num_comp_reply(true);
		}

		if (buttons & KEY_PASSKEY_REJECT)
		{
			num_comp_reply(false);
		}
	}
}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

static void configure_gpio(void)
{
	int err;

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	err = dk_buttons_init(button_changed);
	if (err)
	{
		LOG_ERR("Cannot init buttons (err: %d)", err);
	}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

	err = dk_leds_init();
	if (err)
	{
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}
}

/* Read Handlers */
static ssize_t read_battery_level(struct bt_conn *conn, const struct bt_gatt_attr *attr,
								  void *buf, uint16_t len, uint16_t offset)
{
	const uint8_t *value = &battery_level;
	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(*value));
	// return bt_gatt_attr_read(conn, attr, buf, len, offset, &battery_level, sizeof(battery_level));
}

static ssize_t read_battery_state(struct bt_conn *conn, const struct bt_gatt_attr *attr,
								  void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &battery_state, sizeof(battery_state));
}

static ssize_t read_battery_status(struct bt_conn *conn, const struct bt_gatt_attr *attr,
								   void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &battery_status, sizeof(battery_status));
}

static ssize_t read_str(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
						uint16_t offset)
{
	const char *value = attr->user_data;
	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, strlen(value));
}

static ssize_t read_u8(struct bt_conn *conn, const struct bt_gatt_attr *attr,
					   void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(uint8_t));
}

static ssize_t read_u16(struct bt_conn *conn, const struct bt_gatt_attr *attr,
						void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(uint16_t));
}

static ssize_t read_bp(struct bt_conn *conn, const struct bt_gatt_attr *attr,
	void *buf, uint16_t len, uint16_t offset)
{
return bt_gatt_attr_read(conn, attr, buf, len, offset, bp_data, sizeof(bp_data));
}

/* GATT Service �w�q */
#if 0
BT_GATT_SERVICE_DEFINE(battery_svc,
					   BT_GATT_PRIMARY_SERVICE(BT_UUID_BAS),
					   BT_GATT_CHARACTERISTIC(BT_UUID_BAS_BATTERY_LEVEL,
											  BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
											  BT_GATT_PERM_READ,
											  read_battery_level, NULL, NULL),
					   BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

					   BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(0x1A, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
																  0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB),
											  BT_GATT_CHRC_READ,
											  BT_GATT_PERM_READ,
											  read_battery_state, NULL, NULL),

					   BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(0x1B, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
																  0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB),
											  BT_GATT_CHRC_READ,
											  BT_GATT_PERM_READ,
											  read_battery_status, NULL, NULL));
#endif

// BT_GATT_SERVICE_DEFINE(dis_svc,
BT_GATT_SERVICE_DEFINE(battery_svc,
					   BT_GATT_PRIMARY_SERVICE(BT_UUID_DIS),
					   BT_GATT_PRIMARY_SERVICE(BT_UUID_BAS),
					   BT_GATT_CHARACTERISTIC(BT_UUID_BAS_BATTERY_LEVEL,
											  BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
											  BT_GATT_PERM_READ,
											  read_battery_level, NULL, NULL),
					   BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
					   BT_GATT_CHARACTERISTIC(BT_UUID_RACP,
											  BT_GATT_CHRC_WRITE | BT_GATT_CHRC_INDICATE,
											  BT_GATT_PERM_WRITE,
											  NULL, write_racp, racp_data),
					   BT_GATT_CCC(racp_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
					   BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(0x1A, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
																  0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB),
											  BT_GATT_CHRC_READ,
											  BT_GATT_PERM_READ,
											  read_u16, NULL, &cycle_count),
					   BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(0x1B, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
																  0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB),
											  BT_GATT_CHRC_READ,
											  BT_GATT_PERM_READ,
											  read_u16, NULL, &battery_temp),

					   BT_GATT_CHARACTERISTIC(BT_UUID_DIS_MANUFACTURER_NAME,
											  BT_GATT_CHRC_READ,
											  BT_GATT_PERM_READ,
											  read_str, NULL, manufacturer_name_str),

					   BT_GATT_CHARACTERISTIC(BT_UUID_DIS_MODEL_NUMBER,
											  BT_GATT_CHRC_READ,
											  BT_GATT_PERM_READ,
											  read_str, NULL, model_number_str),

					   BT_GATT_CHARACTERISTIC(BT_UUID_DIS_SERIAL_NUMBER,
											  BT_GATT_CHRC_READ,
											  BT_GATT_PERM_READ,
											  read_str, NULL, serial_number_str));

static void battery_level_update(struct k_timer *dummy)
{
	bt_gatt_notify(NULL, &battery_svc.attrs[1], &battery_level, sizeof(battery_level));
	printk("Battery level updated: %d%%\n", battery_level);

	if (battery_level > 0)
	{
		battery_level--;
	}
	else
	{
		battery_level = 100;
	}
}

/* BT_GATT_SERVICE_DEFINE(battery_svc,
					   BT_GATT_PRIMARY_SERVICE(BT_UUID_BAS),

					   BT_GATT_CHARACTERISTIC(BT_UUID_BAS_BATTERY_LEVEL,
											  BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
											  BT_GATT_PERM_READ,
											  read_battery_level, NULL, &battery_level),

					   BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)); */

void battery_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	// �i�H�d�šA�ήھڻݭn�}��notify
}

BT_GATT_SERVICE_DEFINE(bp_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_BLOOD_PRESSURE),
    BT_GATT_CHARACTERISTIC(BT_UUID_BLOOD_PRESSURE_MEAS,
                           BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ,
                           read_bp, NULL, NULL),
);

//中斷
// 中斷處理函數
void button_pressed_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    printk("中斷觸發！按鈕被按下！\n");
}


int main(void)
{
	int blink_status = 0;
	int err = 0;
    int ret;

	configure_gpio();

	err = uart_init();
	if (err)
	{
		error();
	}

	if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED))
	{
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err)
		{
			printk("Failed to register authorization callbacks.\n");
			return 0;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err)
		{
			printk("Failed to register authorization info callbacks.\n");
			return 0;
		}
	}

	err = bt_enable(NULL);
	if (err)
	{
		error();
	}

	LOG_INF("Bluetooth initialized");

	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS))
	{
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err)
	{
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return 0;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
						  ARRAY_SIZE(sd));
	if (err)
	{
		LOG_ERR("Advertising failed to start (err %d)", err);
		return 0;
	}

	//printk("Cycle Count value: %d\n", cycle_count);

	#pragma region 
    if (!device_is_ready(button.port)) {
        printk("按鈕 GPIO 不可用！\n");
        return;
    }

    ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (ret != 0) {
        printk("設定按鈕 GPIO 輸入失敗\n");
        return;
    }

    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);  // 邊緣觸發（按下）
    if (ret != 0) {
        printk("設定中斷失敗\n");
        return;
    }

    // gpio_init_callback(&button_cb_data, button_pressed_isr, BIT(button.pin));
    // gpio_add_callback(button.port, &button_cb_data);
    gpio_pin_configure(sw1_dev, SW1_PIN, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure(sw1_dev, SW1_PIN, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&sw1_cb_data, button_pressed_isr, BIT(SW1_PIN));
    gpio_add_callback(sw1_dev, &sw1_cb_data);


    printk("初始化完成，等待中斷...\n");
	#pragma endregion

	k_timer_init(&battery_timer, battery_level_update, NULL);
	k_timer_start(&battery_timer, BATTERY_UPDATE_INTERVAL, BATTERY_UPDATE_INTERVAL);
	
	for (;;)
	{
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}

void ble_write_thread(void)
{
	/* Don't go any further until BLE is initialized */
	k_sem_take(&ble_init_ok, K_FOREVER);
	struct uart_data_t nus_data = {
		.len = 0,
	};

	for (;;)
	{
		/* Wait indefinitely for data to be sent over bluetooth */
		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
											 K_FOREVER);

		int plen = MIN(sizeof(nus_data.data) - nus_data.len, buf->len);
		int loc = 0;

		while (plen > 0)
		{
			memcpy(&nus_data.data[nus_data.len], &buf->data[loc], plen);
			nus_data.len += plen;
			loc += plen;

			if (nus_data.len >= sizeof(nus_data.data) ||
				(nus_data.data[nus_data.len - 1] == '\n') ||
				(nus_data.data[nus_data.len - 1] == '\r'))
			{
				if (bt_nus_send(NULL, nus_data.data, nus_data.len))
				{
					LOG_WRN("Failed to send data over BLE connection");
				}
				nus_data.len = 0;
			}

			plen = MIN(sizeof(nus_data.data), buf->len - loc);
		}

		k_free(buf);
	}
}

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
				NULL, PRIORITY, 0, 0);
