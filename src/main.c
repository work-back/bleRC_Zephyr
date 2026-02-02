#include <string.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/shell/shell.h>

/* HID 报告描述符: 键盘 */
static const uint8_t hid_report_map[] = {
    0x05, 0x01, 0x09, 0x06, 0xa1, 0x01, 0x05, 0x07,
    0x19, 0xe0, 0x29, 0xe7, 0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x95, 0x01,
    0x75, 0x08, 0x81, 0x01, 0x95, 0x06, 0x75, 0x08,
    0x15, 0x00, 0x25, 0x65, 0x05, 0x07, 0x19, 0x00,
    0x29, 0x65, 0x81, 0x00, 0xc0
};

/* HID 信息 */
struct hid_info {
    uint16_t bcd_hid;
    uint8_t  b_country_code;
    uint8_t  flags;
} __packed;

static const struct hid_info info = {
    .bcd_hid = 0x0111,
    .b_country_code = 0x00,
    .flags = 0x01,
};

/* 定义广播数据 (Advertising Data) */
static const struct bt_data ad[] = {
    // 1. 设置 Flags：一般设为有限发现模式或普通发现模式
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    // 2. 设置外观 (Appearance)：384 表示 Generic Remote Control
    BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE, 0x80, 0x01), // 小端序: 0x0180 = 384
    // 3. 广播 HID 服务 UUID
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL)),
};

/* 定义扫描响应数据 (Scan Response Data) */
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* GATT 属性定义 */
static uint8_t report_val[8]; // 键盘报告数据缓冲区

BT_GATT_SERVICE_DEFINE(hid_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_HIDS),
    // HID 信息
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_INFO, BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ, NULL, NULL, (void *)&info),
    // 报告描述符
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT_MAP, BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ, NULL, NULL, (void *)hid_report_map),
    // 实际报告值 (Input Report)
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ_AUTHEN, NULL, NULL, report_val),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    // 协议模式 (Required for HIDS)
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_CTRL_POINT, BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE, NULL, NULL, NULL)
);

/* 蓝牙连接回调 */
static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		printk("Failed to connect to %s, err 0x%02x %s\n", addr,
		       err, bt_hci_err_to_str(err));
		return;
	}

	printk("Connected %s\n", addr);

	if (bt_conn_set_security(conn, BT_SECURITY_L2)) {
		printk("Failed to set security\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected from %s, reason 0x%02x %s\n", addr,
	       reason, bt_hci_err_to_str(reason));
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
	.disconnected = disconnected,
};

/* Shell 命令实现 */
static int cmd_send_key(const struct shell *sh, size_t argc, char **argv) {
    if (argc < 2) return -EINVAL;

    uint8_t key = (uint8_t)strtol(argv[1], NULL, 16);
    
    // 1. 按下按键
    memset(report_val, 0, 8);
    report_val[2] = key;
    bt_gatt_notify(NULL, &hid_svc.attrs[5], report_val, 8); // attrs[5] 对应 Report 属性

    k_msleep(50);

    // 2. 松开按键
    memset(report_val, 0, 8);
    bt_gatt_notify(NULL, &hid_svc.attrs[5], report_val, 8);

    shell_print(sh, "Sent keycode 0x%02x", key);
    return 0;
}

SHELL_CMD_REGISTER(send, NULL, "Send keycode: send <hex>", cmd_send_key);

int main(void) {
	int err;

    err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

    /*
    err = bt_conn_cb_register(&conn_callbacks);
	if (err) {
        printk("Connection callbacks register failed.\n")
		return 0;
    }
    */

    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return 0;
	}

	printk("Advertising successfully started\n");

    while(1) {
		k_sleep(K_MSEC(1000));
    }

    return 0;
}

