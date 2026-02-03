#include <string.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/shell/shell.h>
#include <zephyr/settings/settings.h>

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
#if 0
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
#endif

/* copyed from miRC :
 * $ adb shell cat /sys/class/input/event2/device/device/report_descriptor > mirc.bin
 * $ hid-decode mirc.bin
 * */
static const uint8_t report_map[] = {
    0x05, 0x01,                    // Usage Page (Generic Desktop)        0
    0x09, 0x06,                    // Usage (Keyboard)                    2
    0xa1, 0x01,                    // Collection (Application)            4
    0x05, 0x07,                    //  Usage Page (Keyboard)              6
    0x09, 0x06,                    //  Usage (c and C)                    8
    0xa1, 0x01,                    //  Collection (Application)           10
    0x85, 0x01,                    //   Report ID (1)                     12
    0x95, 0x03,                    //   Report Count (3)                  14
    0x75, 0x10,                    //   Report Size (16)                  16
    0x15, 0x00,                    //   Logical Minimum (0)               18
    0x25, 0xfe,                    //   Logical Maximum (254)             20
    0x19, 0x00,                    //   Usage Minimum (0)                 22
    0x29, 0xfe,                    //   Usage Maximum (254)               24
    0x81, 0x00,                    //   Input (Data,Arr,Abs)              26
    0xc0,                          //  End Collection                     28
    0x06, 0x00, 0xff,              //  Usage Page (Vendor Defined Page 1) 29
    0x09, 0x00,                    //  Usage (Undefined)                  32
    0xa1, 0x01,                    //  Collection (Application)           34
    0x85, 0x06,                    //   Report ID (6)                     36
    0x75, 0x08,                    //   Report Size (8)                   38
    0x95, 0x78,                    //   Report Count (120)                40
    0x15, 0x00,                    //   Logical Minimum (0)               42
    0x25, 0xff,                    //   Logical Maximum (255)             44
    0x19, 0x00,                    //   Usage Minimum (0)                 46
    0x29, 0xff,                    //   Usage Maximum (255)               48
    0x81, 0x00,                    //   Input (Data,Arr,Abs)              50
    0x85, 0x07,                    //   Report ID (7)                     52
    0x75, 0x08,                    //   Report Size (8)                   54
    0x95, 0x78,                    //   Report Count (120)                56
    0x15, 0x00,                    //   Logical Minimum (0)               58
    0x25, 0xff,                    //   Logical Maximum (255)             60
    0x19, 0x00,                    //   Usage Minimum (0)                 62
    0x29, 0xff,                    //   Usage Maximum (255)               64
    0x81, 0x00,                    //   Input (Data,Arr,Abs)              66
    0x85, 0x08,                    //   Report ID (8)                     68
    0x75, 0x08,                    //   Report Size (8)                   70
    0x95, 0x78,                    //   Report Count (120)                72
    0x15, 0x00,                    //   Logical Minimum (0)               74
    0x25, 0xff,                    //   Logical Maximum (255)             76
    0x19, 0x00,                    //   Usage Minimum (0)                 78
    0x29, 0xff,                    //   Usage Maximum (255)               80
    0x81, 0x00,                    //   Input (Data,Arr,Abs)              82
    0xc0,                          //  End Collection                     84
    0xc0,                          // End Collection                      85
};


static volatile bool is_adv_running;
static struct k_work adv_work;

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

int g_id = -1;

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


static void advertising_continue(void)
{
    struct bt_le_adv_param adv_param;

#if CONFIG_BT_DIRECTED_ADVERTISING
    bt_addr_le_t addr;

    if (!k_msgq_get(&bonds_queue, &addr, K_NO_WAIT)) {
        char addr_buf[BT_ADDR_LE_STR_LEN];
        int err;

        if (is_adv_running) {
            err = bt_le_adv_stop();
            if (err) {
                printk("Advertising failed to stop (err %d)\n", err);
                return;
            }
            is_adv_running = false;
        }

        adv_param = *BT_LE_ADV_CONN_DIR(&addr);
        adv_param.options |= BT_LE_ADV_OPT_DIR_ADDR_RPA;


        err = bt_le_adv_start(&adv_param, NULL, 0, NULL, 0);

        if (err) {
            printk("Directed advertising failed to start (err %d)\n", err);
            return;
        }

        bt_addr_le_to_str(&addr, addr_buf, BT_ADDR_LE_STR_LEN);
        printk("Direct advertising to %s started\n", addr_buf);
    } else
#endif
    {
        int err;

        if (is_adv_running) {
            return;
        }

        adv_param = *BT_LE_ADV_CONN_FAST_1;
        adv_param.options |= BT_LE_ADV_OPT_USE_IDENTITY;
        if (g_id >= 0) {
            printk("Use mac form g_id!\n");
            adv_param.id = g_id;
        }

        err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
        if (err) {
            printk("Advertising failed to start (err %d)\n", err);
            return;
        }

        printk("Regular advertising started\n");
    }

    is_adv_running = true;
}

static void advertising_start(void)
{
#if CONFIG_BT_DIRECTED_ADVERTISING
    k_msgq_purge(&bonds_queue);
    bt_foreach_bond(BT_ID_DEFAULT, bond_find, NULL);
#endif

    k_work_submit(&adv_work);
}

static void advertising_process(struct k_work *work)
{
    advertising_continue();
}

/* 蓝牙连接回调 */
static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    is_adv_running = false;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err) {
        if (err == BT_HCI_ERR_ADV_TIMEOUT) {
            printk("Direct advertising to %s timed out\n", addr);
            k_work_submit(&adv_work);
        } else {
            printk("Failed to connect to %s 0x%02x %s\n", addr, err,
                   bt_hci_err_to_str(err));
        }
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

    advertising_start();
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

    #if 0
    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return 0;
    }

    printk("Advertising successfully started\n");
    #endif

    k_work_init(&adv_work, advertising_process);

    advertising_start();

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    while(1) {
        k_sleep(K_MSEC(1000));
    }

    return 0;
}

