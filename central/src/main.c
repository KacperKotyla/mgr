/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>

#include <zephyr/net/buf.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

#include <zephyr/bluetooth/ead.h>
#include <zephyr/bluetooth/att.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/bluetooth.h>

bool debug = false;

// Peripherals P1 P2 P3 P4
const char *addressArr[] = {
	"C8:08:67:10:6A:25",
	"F7:3E:E2:EA:4B:AC",
	"F5:E6:A8:F0:CC:21",
	"EE:FC:B1:9C:E3:A2",
};

int connTry = 0;
int connTryMax = 50;

#define BT_UUID_READ_WRITE_SERVICE \
	BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x0000fff1, 0x0000, 0x1000, 0x8000, 0x00805f9b34f0))

#define BT_UUID_PERIPHERAL_WRITE \
	BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x0000fff1, 0x0000, 0x1000, 0x8000, 0x00805f9b34f1))

#define BT_UUID_PERIPHERAL_READ \
	BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x0000fff1, 0x0000, 0x1000, 0x8000, 0x00805f9b34f2))

#define BT_UUID_PERIPHERAL_INDICATE \
	BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x0000fff1, 0x0000, 0x1000, 0x8000, 0x00805f9b34f3))

#define BT_UUID_PERIPHERAL_NOTIFY \
	BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x0000fff1, 0x0000, 0x1000, 0x8000, 0x00805f9b34f4))

int addressIdx = 0;
#define n_array (sizeof(addressArr) / sizeof(const char *))

// There are 4 scenarios
// 1 - Connection time (central: from advertisement to connection frame)
// 2 - RTT read (central: just before reading to getting frame with read data)
// 3 - RTT indicate (peripheral: just before sending to getting ACK)
// 4 - RTT notify (peripheral: send signal for central -> central starts time and waits for notify)
int scenarioIdx = 1;
bool write_handle_found = false;
int write_handle = -1;
timing_t start_time, end_time;

// Scenario 1
int connectionCount = 0;
#define connectionMaxCount 10
uint64_t connectionTimes[connectionMaxCount];

// Scenario 2
int readCount = 0;
#define readMaxCount 10
uint64_t readTimes[connectionMaxCount];

// Scenario 3 in server, pin handler here to start time
int indicateCount = 0;
#define indicateMaxCount 10
uint64_t indicateTimes[indicateMaxCount];
bool ackIndicate = false;

// Scenario 4 in server, pin handler here to start time
int notifyCount = 0;
#define notifyMaxCount 10
uint64_t notifyTimes[notifyMaxCount];
bool validNotify = false;

//By looking at hardware there are two GPIO: GPIO1 and GPIO2
//Pins are depending on GPIO. In this example there is P1.15 used. It means GPIO1 and PIN 15
#define GPIO1_LABEL "GPIO_1"
#define GPIO1_PIN15 15
const struct device *gpio1pin15;
static struct gpio_callback gpio1pin15_cb_data;

#define GPIO1_LABEL "GPIO_1"
#define GPIO1_PIN14 14
const struct device *gpio1pin14;
static struct gpio_callback gpio1pin14_cb_data;

#define GPIO1_LABEL "GPIO_1"
#define GPIO1_PIN13 13
const struct device *gpio1pin13;
static struct gpio_callback gpio1pin13_cb_data;

#define GPIO1_LABEL "GPIO_1"
#define GPIO1_PIN12 12
const struct device *gpio1pin12;
static struct gpio_callback gpio1pin12_cb_data;

// Pin configuration to get signal from peripheral and later start counting time
const struct device * configurePin(const char * label, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct device *dev;
	int ret;

	dev = device_get_binding(label);
	if (dev == NULL) {
		if (debug == true) printk("Failed to bind gpio1pin15\n");
		return NULL;
	}

	ret = gpio_pin_configure(dev, pin, flags);
	if (ret < 0) {
		if (debug == true) printk("Failed to configure gpio1pin15\n");
		return NULL;
	}

	ret = gpio_pin_interrupt_configure(dev, pin, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		if (debug == true) printk("Failed to configure gpio1pin15 interrupt\n");
		return NULL;
	}

	return dev;
}

//This callback only launches on pin high state and starts counting time
void gpio1pinCallback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{	
	int ret;
	ret = gpio_pin_get(gpio1pin15, GPIO1_PIN15);

 	if (debug == true) printk("Pin 15 received data at %" PRIu32 " data: %d\n", k_cycle_get_32(), ret);

	validNotify = true;
	timing_start();
	start_time = timing_counter_get();
}

// Conversion of uint64 to str because long causes problems in printing to terminal
char *convertUint64ToStr(uint64_t num)
{
	static char buf[22];
	char *p = &buf[sizeof(buf) - 1];
	*p = '\0';
	do
	{
		*--p = '0' + (num % 10);
		num /= 10;
	} while (num > 0);
	return p;
}

// Saving uart data
void SendUartData(uint64_t times[], int size)
{
	for (size_t i = 0; i < size; i++)
	{
		// Json type string
		printk("{'address': '%s', 'scenario': %d, 'time': %lld}\n", addressArr[addressIdx], scenarioIdx, times[i]);
	}
}

static struct bt_conn *default_conn;
static struct bt_uuid_128 uuid = BT_UUID_INIT_128(0);
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params;
static void start_scan(void);
static uint8_t read_func_cb_sc2(struct bt_conn *conn, uint8_t err, struct bt_gatt_read_params *params, const void *data, uint16_t length);


//This function filters for correct device: connectable, in close proximity and with correct hardcoded address
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
						 struct net_buf_simple *ad)
{
	char addr_str[BT_ADDR_LE_STR_LEN];

	if (default_conn)
	{
		return;
	}

	/* We're only interested in connectable events */
	if (type != BT_GAP_ADV_TYPE_ADV_IND &&
	type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND)
	{
		return;
	}

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	if (debug == true) printk("Device found: %s (RSSI %d)\n", addr_str, rssi);

	/* connect only to devices in close proximity */
	if (rssi < -90)
	{
		return;
	}

	char address[BT_ADDR_LE_STR_LEN];
	strcpy(address, addressArr[addressIdx]);
	if (0 != strncmp(address, addr_str, 17))
	{
		// Try a bunch of times and move to other device
		connTry++;
		if (connTry >= connTryMax)
		{
			connTry = 0;
			addressIdx++;
		}
		return;
	}

	connTry = 0;

	if (bt_le_scan_stop())
	{
		return;
	}

	// Start experiment scenario 1
	if (scenarioIdx == 1)
	{
		timing_start();
		start_time = timing_counter_get();
	}

	int err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
								BT_LE_CONN_PARAM_DEFAULT, &default_conn);
	if (err)
	{
		if (debug == true) printk("Create conn to %s failed (%u)\n", addr_str, err);
		start_scan();
	}
}

//Basic function to start scanning. On device found it will use device_found callback.
static void start_scan(void)
{
	int err;

	struct bt_le_scan_param scan_param = {
		.type = BT_LE_SCAN_TYPE_ACTIVE,
		.options = BT_LE_SCAN_OPT_NONE,
		.interval = BT_GAP_SCAN_FAST_INTERVAL,
		.window = BT_GAP_SCAN_FAST_WINDOW,
	};

	err = bt_le_scan_start(&scan_param, device_found);
	if (err)
	{
		if (debug == true) printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	if (debug == true) printk("\nScanning successfully started\n");
}

static void write_func_cb(struct bt_conn *conn, uint8_t err,
		    struct bt_gatt_write_params *params)
{
	if (err) {
		if (debug == true) printk("Failed to write (err %d)\n", err);
	}
	else
	{
		if (debug == true) printk("Scenario initialized\n");
	}

	return;
}

static void writeScenarioIdx(struct bt_conn *conn, uint16_t handle)
{	
	int err;

	struct bt_gatt_write_params write_params;
	write_params.func = write_func_cb;
	write_params.handle = handle;
	write_params.offset = 0;
	write_params.data = &scenarioIdx;
	write_params.length = sizeof(scenarioIdx);
	err = bt_gatt_write(conn, &write_params);

	if (err) {
		if (debug == true) printk("Write param failed (err %d)\n", err);

		if (debug == true) printk("Disconnecting because of failed scenario init (write param)\n");
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}

	return;
}

static void read(struct bt_conn *conn)
{
	int err;

	struct bt_gatt_read_params read_params;
	read_params.func = read_func_cb_sc2;
	read_params.handle_count = 0;
	read_params.by_uuid.start_handle = BT_ATT_FIRST_ATTTRIBUTE_HANDLE;
	read_params.by_uuid.end_handle = BT_ATT_LAST_ATTTRIBUTE_HANDLE;
	read_params.by_uuid.uuid = BT_UUID_PERIPHERAL_READ;
	err = bt_gatt_read(conn, &read_params);
	if (err) {
		if (debug == true) printk("Read param failed (err %d)\n", err);
	}
	else{
		if (debug == true) printk("Read param successful \n");
	}
}

static uint8_t read_func_cb_sc2(struct bt_conn *conn, uint8_t err, struct bt_gatt_read_params *params, const void *data, uint16_t length)
{
	// End of experiment 2
	end_time = timing_counter_get();
	readTimes[readCount] = timing_cycles_to_ns(timing_cycles_get(&start_time, &end_time));
	
    if ((data != NULL) && (err == 0))
    {
		uint8_t *d = (uint8_t *)data;
		if (debug == true) printk("length: %2x data:", length);
		for (int i =0; i < length; i++){
			if (debug == true) printk("%2x ", d[i]);
		}
		if (debug == true) printk("\n");
    }
	else{
		if (debug == true) printk("No data\n");
	}

	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	readCount++;

	if (readCount >= readMaxCount)
	{
		readCount = 0;
		if (debug == true) printk("Scenario 2 ended for peripheral: %s\n", addressArr[addressIdx]);
		SendUartData(readTimes, readMaxCount);

		addressIdx += 1;

		if (debug == true) printk("Disconnecting (expected for scenario 2)\n");
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);

		if (addressIdx >= n_array)
		{
			addressIdx = 0;
			scenarioIdx++;
		}
	}

	//Sleep for 100 ms to avoid overspam
	k_msleep(100);
	start_time = timing_counter_get();
	read(conn);

    return BT_GATT_ITER_STOP;
}

static uint8_t notify_func(struct bt_conn *conn,
			   struct bt_gatt_subscribe_params *params,
			   const void *data, uint16_t length)
{	
	uint32_t data_raw;

	if (!data) {
		if (debug == true) printk("[UNSUBSCRIBED] no data\n");
		params->value_handle = 0U;

		return BT_GATT_ITER_STOP;
	}

	/* data value display */
	data_raw = sys_le32_to_cpu(*(uint32_t *)data);
	
	if (scenarioIdx == 3)
	{
		if (ackIndicate == false)
		{
			// End of experiment 3 with no ACK
			if (debug == true) printk("Getting indication 1\n");
			ackIndicate = true;
		}
		else {
			// End of experiment 3 with ACK
			indicateTimes[indicateCount] = data_raw;

			if (debug == true) printk("Getting indication 2 (ACK information)\n");
			if (debug == true) printk("Spent time indicating (ACK): ");
			if (debug == true) printk("%d", data_raw);
			if (debug == true) printk("\n");

			if (scenarioIdx == 3)
			{
				if (debug == true) printk("Indicate count is %d.\n", indicateCount);
				indicateCount++;
				if (indicateCount >= indicateMaxCount)
				{
					indicateCount = 0;
					if (debug == true) printk("Scenario 3 ended for peripheral: %s\n", addressArr[addressIdx]);
					SendUartData(indicateTimes, indicateMaxCount);
					addressIdx += 1;

					if (addressIdx >= n_array)
					{
						addressIdx = 0;
						scenarioIdx++;
					}

					if (debug == true) printk("Disconnecting (expected for scenario 3)\n");
					bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
				}
			}
			ackIndicate = false;
		}
	}
	
	if (scenarioIdx == 4)
	{
		// End of experiment 4
		if (validNotify == true)
		{
			end_time = timing_counter_get();
			notifyTimes[notifyCount] = timing_cycles_to_ns(timing_cycles_get(&start_time, &end_time));
			validNotify = false;
			
			if (debug == true) printk("Spent time notifyng (no ACK): ");
			if (debug == true) printk("%s", convertUint64ToStr(notifyTimes[notifyCount]));
			if (debug == true) printk("\n");

			if (debug == true) printk("Notify count is %d.\n", notifyCount);
			notifyCount++;
			if (notifyCount >= notifyMaxCount)
			{
				notifyCount = 0;
				if (debug == true) printk("Scenario 4 ended for peripheral: %s\n", addressArr[addressIdx]);
				SendUartData(notifyTimes, notifyMaxCount);
				addressIdx += 1;

				if (addressIdx >= n_array)
				{
					addressIdx = 0;
					scenarioIdx = 1;
				}

				if (debug == true) printk("Disconnecting (expected for scenario 4)\n");
				bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
			}
		}
	}
	return BT_GATT_ITER_CONTINUE;
}

static uint8_t discover_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	int err;

	if (!attr) {
		if (debug == true) printk("Discover complete\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	if (debug == true) printk("[ATTRIBUTE] handle %u\n", attr->handle);

	
	if (scenarioIdx == 2)
	{
		timing_start();

		start_time = timing_counter_get();

		read(conn);
	} 
	
	if (scenarioIdx == 1)
	{
		if (debug == true) printk("Disconnecting (expected for scenario 1/2)\n");
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}

	if (scenarioIdx == 3)
	{
		if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_READ_WRITE_SERVICE)) {
			memcpy(&uuid, BT_UUID_PERIPHERAL_INDICATE, sizeof(uuid));
			discover_params.uuid = &uuid.uuid;
			discover_params.start_handle = attr->handle + 1;
			discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

			err = bt_gatt_discover(conn, &discover_params);
			if (err) {
				if (debug == true) printk("Discover failed (err %d)\n", err);
			}
		}
		else if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_PERIPHERAL_INDICATE)) {
			memcpy(&uuid, BT_UUID_GATT_CCC, sizeof(uuid));
			discover_params.uuid = &uuid.uuid;
			discover_params.start_handle = attr->handle + 2;
			discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
			subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);

			err = bt_gatt_discover(conn, &discover_params);
			if (err) {
				if (debug == true) printk("Discover failed (err %d)\n", err);
			}
		}
		else {
			subscribe_params.notify = notify_func;
			subscribe_params.value = BT_GATT_CCC_INDICATE;
			subscribe_params.ccc_handle = attr->handle;

			err = bt_gatt_subscribe(conn, &subscribe_params);
			if (err && err != -EALREADY) {
				if (debug == true) printk("Subscribe failed (err %d)\n", err);
			} else {
				if (debug == true) printk("[SUBSCRIBED INDICATE]\n");
				if (debug == true) printk("Writing scenario to peripheral\n");
				writeScenarioIdx(conn, write_handle);
			}

			return BT_GATT_ITER_STOP;
		}
	}

	if (scenarioIdx == 4)
	{
		if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_READ_WRITE_SERVICE)) {
			memcpy(&uuid, BT_UUID_PERIPHERAL_NOTIFY, sizeof(uuid));
			discover_params.uuid = &uuid.uuid;
			discover_params.start_handle = attr->handle + 1;
			discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

			err = bt_gatt_discover(conn, &discover_params);
			if (err) {
				if (debug == true) printk("Discover failed (err %d)\n", err);
			}
		}
		else if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_PERIPHERAL_NOTIFY)) {
			memcpy(&uuid, BT_UUID_GATT_CCC, sizeof(uuid));
			discover_params.uuid = &uuid.uuid;
			discover_params.start_handle = attr->handle + 2;
			discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
			subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);

			err = bt_gatt_discover(conn, &discover_params);
			if (err) {
				if (debug == true) printk("Discover failed (err %d)\n", err);
			}
		}
		else {
			subscribe_params.notify = notify_func;
			subscribe_params.value = BT_GATT_CCC_NOTIFY;
			subscribe_params.ccc_handle = attr->handle;

			err = bt_gatt_subscribe(conn, &subscribe_params);
			if (err && err != -EALREADY) {
				if (debug == true) printk("Subscribe failed (err %d)\n", err);
			} else {
				if (debug == true) printk("[SUBSCRIBED NOTIFY]\n");
				if (debug == true) printk("Writing scenario to peripheral\n");
				writeScenarioIdx(conn, write_handle);
			}

			return BT_GATT_ITER_STOP;
		}
	}

	return BT_GATT_ITER_STOP;
}

static uint8_t discover_write_characteristic_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	int err;

	if (!attr) {
		if (debug == true) printk("Discover complete\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	if (debug == true) printk("[ATTRIBUTE] handle %u\n", attr->handle);

	if (write_handle_found == false)
	{
		// For all scenarios write scenario idx to peripheral
		if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_READ_WRITE_SERVICE)) {
			memcpy(&uuid, BT_UUID_PERIPHERAL_WRITE, sizeof(uuid));
			discover_params.uuid = &uuid.uuid;
			discover_params.start_handle = attr->handle + 1;
			discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

			err = bt_gatt_discover(conn, &discover_params);
			if (err) {
				if (debug == true) printk("Discover failed (err %d)\n", err);
			}
		}
		else
		{
			write_handle = attr->handle + 1;
			write_handle_found = true;

			if (debug == true) printk("Found write handle %d\n", write_handle);

			if (debug == true) printk("Rediscover service, but this time go for scenarios\n");
			memcpy(&uuid, BT_UUID_READ_WRITE_SERVICE, sizeof(uuid));
			discover_params.uuid = &uuid.uuid;
			discover_params.func = discover_func;
			discover_params.start_handle = BT_ATT_FIRST_ATTTRIBUTE_HANDLE;
			discover_params.end_handle = BT_ATT_LAST_ATTTRIBUTE_HANDLE;
			discover_params.type = BT_GATT_DISCOVER_PRIMARY;

			err = bt_gatt_discover(conn, &discover_params);
			if (err) {
				if (debug == true) printk("Discover failed(err %d)\n", err);
				return;
			}
		}
	}

	return BT_GATT_ITER_STOP;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (scenarioIdx == 1)
	{
		// End of experiment 1
		end_time = timing_counter_get();
		connectionTimes[connectionCount] = timing_cycles_to_ns(timing_cycles_get(&start_time, &end_time));
	}

	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	if (debug == true) printk("Connected: %s\n", addr);

	if (scenarioIdx == 1)
	{
		if (debug == true) printk("Spent time connecting: ");
		if (debug == true) printk("%s", convertUint64ToStr(connectionTimes[connectionCount]));
		if (debug == true) printk("\n");
		if (debug == true) printk("Connection count %d\n", connectionCount);
	}

	if (err)
	{
		if (debug == true) printk("Failed to connect to %s (%u)\n", addr, err);

		bt_conn_unref(default_conn);
		default_conn = NULL;

		start_scan();
		return;
	}

	if (conn != default_conn)
	{
		return;
	}
	
	//After connection initiate service discovery
	if (conn == default_conn) {
		memcpy(&uuid, BT_UUID_READ_WRITE_SERVICE, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;

		//First time find write handle for later
		if (write_handle_found == false)
		{
			discover_params.func = discover_write_characteristic_func;
		}
		else if (write_handle_found == true)
		{
			//Next iterations focus on scenarios
			discover_params.func = discover_func;
		}
		
		discover_params.start_handle = BT_ATT_FIRST_ATTTRIBUTE_HANDLE;
		discover_params.end_handle = BT_ATT_LAST_ATTTRIBUTE_HANDLE;
		discover_params.type = BT_GATT_DISCOVER_PRIMARY;

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			if (debug == true) printk("Discover failed(err %d)\n", err);
			return;
		}
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != default_conn)
	{
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (debug == true) printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);

	bt_conn_unref(default_conn);
	default_conn = NULL;

	if (scenarioIdx == 1)
	{
		connectionCount += 1;

		if (connectionCount >= connectionMaxCount)
		{
			connectionCount = 0;
			if (debug == true) printk("Scenario 1 ended for peripheral: %s\n", addressArr[addressIdx]);
			SendUartData(connectionTimes, connectionMaxCount);
			addressIdx += 1;

			if (addressIdx >= n_array)
			{
				addressIdx = 0;
				scenarioIdx++;
			}
		}

		//Sleep for 100ms to not overspam with connections
		k_msleep(100);
	}

	validNotify = false;

	start_scan();
}

//Basic define which determines which function will be called on connection and disconnection
BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};


void readCb()
{
	read(default_conn);
}

void configurePins()
{
	gpio1pin15 = configurePin(GPIO1_LABEL, GPIO1_PIN15, GPIO_INPUT);
	if (gpio1pin15 == NULL)
	{
		if (debug == true) printk("Failed to initialize gpio1pin15");
		return;
	} 

	gpio_init_callback(&gpio1pin15_cb_data, gpio1pinCallback, BIT(GPIO1_PIN15));
 	gpio_add_callback(gpio1pin15, &gpio1pin15_cb_data);

	// 

	gpio1pin14 = configurePin(GPIO1_LABEL, GPIO1_PIN14, GPIO_INPUT);
	if (gpio1pin14 == NULL)
	{
		if (debug == true) printk("Failed to initialize gpio1pin14");
		return;
	} 

	gpio_init_callback(&gpio1pin14_cb_data, gpio1pinCallback, BIT(GPIO1_PIN14));
 	gpio_add_callback(gpio1pin14, &gpio1pin14_cb_data);

	// 

	gpio1pin13 = configurePin(GPIO1_LABEL, GPIO1_PIN13, GPIO_INPUT);
	if (gpio1pin13 == NULL)
	{
		if (debug == true) printk("Failed to initialize gpio1pin13");
		return;
	} 

	gpio_init_callback(&gpio1pin13_cb_data, gpio1pinCallback, BIT(GPIO1_PIN13));
 	gpio_add_callback(gpio1pin13, &gpio1pin13_cb_data);

	// 

	gpio1pin12 = configurePin(GPIO1_LABEL, GPIO1_PIN12, GPIO_INPUT);
	if (gpio1pin12 == NULL)
	{
		if (debug == true) printk("Failed to initialize gpio1pin13");
		return;
	} 

	gpio_init_callback(&gpio1pin12_cb_data, gpio1pinCallback, BIT(GPIO1_PIN12));
 	gpio_add_callback(gpio1pin12, &gpio1pin12_cb_data);

}
void main(void)
{
	k_msleep(2000);
	int err;

	timing_init();

	configurePins();

	err = bt_enable(NULL);
	if (err)
	{
		if (debug == true) printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	if (debug == true) printk("Bluetooth initialized\n");

	start_scan();
}