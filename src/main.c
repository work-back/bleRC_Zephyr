#include <string.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/shell/shell.h>

enum {
	HIDS_REMOTE_WAKE = BIT(0),
	HIDS_NORMALLY_CONNECTABLE = BIT(1),
};

/* HID 信息 */
struct hids_info {
    uint16_t bcd_hid;
    uint8_t  b_country_code;
    uint8_t  flags;
} __packed;

struct hids_report {
	uint8_t id; /* report id */
	uint8_t type; /* report type */
} __packed;

static struct hids_info info = {
    .bcd_hid = 0x0111,
    .b_country_code = 0x00,
    .flags = 0x01,
};

enum {
	HIDS_INPUT = 0x01,
	HIDS_OUTPUT = 0x02,
	HIDS_FEATURE = 0x03,
};

static struct hids_report input = {
	.id = 0x01,
	.type = HIDS_INPUT,
};

static uint8_t simulate_input_allow;
static uint8_t ctrl_point;

/* HID 报告描述符: 键盘 */
static const uint8_t report_map[] = {
    0x05, 0x01,       /* Usage Page (Generic Desktop) - 通用桌面设备 */
    0x09, 0x06,       /* Usage (Keyboard) - 明确是一个键盘 */
    0xa1, 0x01,       /* Collection (Application) - 开始应用集合 */

        /* Byte 0 */
        0x05, 0x07,       /* Usage Page (Key Codes) - 使用按键码页面 */
        0x19, 0xe0,       /* Usage Minimum (224) - 对应左 Ctrl */
        0x29, 0xe7,       /* Usage Maximum (231) - 对应右 GUI (Win键) */
        0x15, 0x00,       /* Logical Minimum (0) - 最小值 0 (没按) */
        0x25, 0x01,       /* Logical Maximum (1) - 最大值 1 (按了) */
        0x75, 0x01,       /* Report Size (1) - 每个按键占 1 bit */
        0x95, 0x08,       /* Report Count (8) - 总共 8 个按键 (刚好 1 字节) */
        0x81, 0x02,       /* Input (Data, Variable, Absolute) - 变量输入 */


        /* Byte 1 */
        0x95, 0x01,       /* Report Count (1) - 1 个单位 */
        0x75, 0x08,       /* Report Size (8) - 占 8 bit (1 字节) */
        0x81, 0x01,       /* Input (Constant) - 常量输入 (填充用) */

        /* Byte 2-7 */
        0x95, 0x06,       /* Report Count (6) - 允许同时按下 6 个键 */
        0x75, 0x08,       /* Report Size (8) - 每个键占 8 bit (1 字节) */
        0x15, 0x00,       /* Logical Minimum (0) */
        0x25, 0xff,       /* Logical Maximum (255) - 允许的最大键值是 255 (0xff) */
        0x05, 0x07,       /* Usage Page (Key Codes) */
        0x19, 0x00,       /* Usage Minimum (0) */
        0x29, 0xff,       /* Usage Maximum (255) - 允许的最大 Usage 也是 255 (0xff) */
        0x81, 0x00,       /* Input (Data, Array) - 数组输入 */

    0xc0              /* End Collection - 结束应用集合 */
};

static ssize_t read_info(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr, void *buf,
			  uint16_t len, uint16_t offset)
{
    printk("[GATT SRV CB] ----> Handle [%s]\n", __func__);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,
				             sizeof(struct hids_info));
}

static ssize_t read_report_map(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr, void *buf,
			       uint16_t len, uint16_t offset)
{
    printk("[GATT SRV CB] ----> Handle [%s]\n", __func__);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, report_map,
				 sizeof(report_map));
}

static ssize_t read_report(struct bt_conn *conn,
			   const struct bt_gatt_attr *attr, void *buf,
			   uint16_t len, uint16_t offset)
{
    printk("[GATT SRV CB] ----> Handle [%s]\n", __func__);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,
				 sizeof(struct hids_report));
}

static void input_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    printk("[GATT SRV CB] ----> Handle [%s]\n", __func__);

	simulate_input_allow = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
}

static ssize_t read_input_report(struct bt_conn *conn,
				 const struct bt_gatt_attr *attr, void *buf,
				 uint16_t len, uint16_t offset)
{
    printk("[GATT SRV CB] ----> Handle [%s]\n", __func__);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, NULL, 0);
}

static ssize_t write_ctrl_point(struct bt_conn *conn,
				const struct bt_gatt_attr *attr,
				const void *buf, uint16_t len, uint16_t offset,
				uint8_t flags)
{
	uint8_t *value = attr->user_data;

    printk("[GATT SRV CB] ----> Handle [%s]\n", __func__);

	if (offset + len > sizeof(ctrl_point)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);

	return len;
}

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

    // HID Info (Index 1, 2)
	BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_INFO, BT_GATT_CHRC_READ,
			               BT_GATT_PERM_READ, read_info, NULL, &info),

    // Report Map (Index 3, 4)
	BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT_MAP, BT_GATT_CHRC_READ,
			               BT_GATT_PERM_READ, read_report_map, NULL, NULL),

    // Report Value (Index 5, 6)
	BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			               BT_GATT_PERM_READ /* _AUTHEN */ ,
                           /* _AUTHEN 对应的安全等级为 BT_SECURITY_L3, 否则 对端会有: Error Code: Insufficient Authentication (0x05) */
                           read_input_report, NULL, NULL),

    // CCC (Index 7)
	// BT_GATT_CCC(input_ccc_changed, SAMPLE_BT_PERM_READ | SAMPLE_BT_PERM_WRITE),
	BT_GATT_CCC(input_ccc_changed, BT_GATT_PERM_READ /* _AUTHEN */ | BT_GATT_PERM_WRITE /* _AUTHEN */ ),


    // Report Reference (Index 8)
	BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ,
			           read_report, NULL, &input),

    // Control Point (Index 9, 10)
	BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_CTRL_POINT,
                           BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE,
                           NULL, write_ctrl_point, &ctrl_point),
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

static struct bt_gatt_attr *report_decl = NULL;
static inline struct bt_gatt_attr * get_attrs(void)
{
    if (!report_decl) {
        report_decl = bt_gatt_find_by_uuid(hid_svc.attrs, hid_svc.attr_count, BT_UUID_HIDS_REPORT);
        if (!report_decl) {
            printk("Error: HID Report Characteristic not found\n");
            return NULL;
        }
    }

    return report_decl + 1;
}

static int cmd_send_key(const struct shell *sh, size_t argc, char **argv) {
    if (argc < 2) return -EINVAL;

    uint8_t key = (uint8_t)strtol(argv[1], NULL, 16);

    struct bt_gatt_attr * rpt_val_att = get_attrs();
    if (!report_decl) {
        printk("Error: HID Report Characteristic not found\n");
        return -1;
    }

    // 1. 按下按键
    memset(report_val, 0, 8);
    report_val[2] = key;
    //bt_gatt_notify(NULL, rpt_val_att, report_val, 8);
    bt_gatt_notify(NULL, &hid_svc.attrs[6], report_val, sizeof(report_val));

    k_msleep(50);

    // 2. 松开按键
    memset(report_val, 0, 8);
    //bt_gatt_notify(NULL, rpt_val_att, report_val, 8);
    bt_gatt_notify(NULL, &hid_svc.attrs[6], report_val, sizeof(report_val));

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

