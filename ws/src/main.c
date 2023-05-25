#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/drivers/gpio.h>

// default sample time is 2s
#define DEFAULT_SAMPLE_TIME 2

static int sample_time = DEFAULT_SAMPLE_TIME;
static struct sensor_value pres;
static struct sensor_value temp, humidity;
static struct sensor_value co2, voc, voltage, current;

/*
 * Start LPS22HB sensor to get pressure.
 */
extern void lps22hb_thread(void)
{
    // Get LPS22Hb sensor from dts
    const struct device *const dev = DEVICE_DT_GET_ONE(st_lps22hb_press);

    // Check is device ready
    if (!device_is_ready(dev)) {
        printk("LPS22HB not ready.\n");
        return;
    }

    // Fetch data every 2s
    while (true) {
        sensor_sample_fetch(dev);
        sensor_channel_get(dev, SENSOR_CHAN_PRESS, &pres);

        k_sleep(K_SECONDS(sample_time));
    }
}

/*
 * Start HTS221 sensor to get temperature and humidity.
 */
extern void hts221_thread(void)
{   
    // Get HTS221 sensor from dts
    const struct device *const dev = DEVICE_DT_GET_ONE(st_hts221);

    // Check is device ready
    if (!device_is_ready(dev)) {
        printk("HTS221 not ready.\n");
        return;
    }

    // Fetch data every 2s
    while (true) {
        sensor_sample_fetch(dev);
        sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
        sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity);
    }
}

/*
 * Start CCS811 sensor to get CO2 and VOC.
 */
extern void ccs811_thread(void)
{
    // Get CCS811 sensor from dts
    const struct device *const dev = DEVICE_DT_GET_ONE(ams_ccs811);

    // Check is device ready
    if (!device_is_ready(dev)) {
		printk("CCS811 is not ready.\n");
		return;
	}

    // Fetch data every 2s
    while (true) {
        sensor_sample_fetch(dev);
        sensor_channel_get(dev, SENSOR_CHAN_CO2, &co2);
		sensor_channel_get(dev, SENSOR_CHAN_VOC, &voc);
		sensor_channel_get(dev, SENSOR_CHAN_VOLTAGE, &voltage);
		sensor_channel_get(dev, SENSOR_CHAN_CURRENT, &current);
    }
}

/* sensor double values */
static double pres_value;
static double temp_value, humidity_value;
static double co2_value, voc_value, voltage_value, current_value;

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* 
 * Print sensor values.
 */
extern double report_sensor_values(int sensor_did)
{   
    double value = 0.0;

    if (sensor_did == 1) {
        value = sensor_value_to_double(&pres);
    } else if (sensor_did == 2) {
        value = sensor_value_to_double(&temp);
    } else if (sensor_did == 3) {
        value = sensor_value_to_double(&humidity);
    } else if (sensor_did == 4) {
        value = sensor_value_to_double(&co2);
    } else if (sensor_did == 5) {
        value = sensor_value_to_double(&voc);
    }

    return value;
}

/* 
 * Sensor threads.
 */
K_THREAD_DEFINE(lps22hb_tid, MY_STACK_SIZE, 
                lps22hb_thread, NULL, NULL, NULL, 
                MY_PRIORITY, 0, 0);
K_THREAD_DEFINE(hts221_tid, MY_STACK_SIZE, 
                hts221_thread, NULL, NULL, NULL, 
                MY_PRIORITY, 0, 0);
K_THREAD_DEFINE(ccs811_tid, MY_STACK_SIZE, 
                ccs811_thread, NULL, NULL, NULL, 
                MY_PRIORITY, 0, 0);

                /* main.c - Application main entry point */

/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/printk.h>

#include <zephyr/settings/settings.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/mesh.h>

#define OP_ONOFF_GET       BT_MESH_MODEL_OP_2(0x82, 0x01)
#define OP_ONOFF_SET       BT_MESH_MODEL_OP_2(0x82, 0x02)
#define OP_ONOFF_SET_UNACK BT_MESH_MODEL_OP_2(0x82, 0x03)
#define OP_ONOFF_STATUS    BT_MESH_MODEL_OP_2(0x82, 0x04)

static const char *const onoff_str[] = { "off", "on" };

static struct {
	bool val;
	uint8_t tid;
	uint16_t src;
	uint32_t transition_time;
	struct k_work_delayable work;
} onoff;

/* OnOff messages' transition time and remaining time fields are encoded as an
 * 8 bit value with a 6 bit step field and a 2 bit resolution field.
 * The resolution field maps to:
 * 0: 100 ms
 * 1: 1 s
 * 2: 10 s
 * 3: 20 min
 */
static const uint32_t time_res[] = {
	100,
	MSEC_PER_SEC,
	10 * MSEC_PER_SEC,
	10 * 60 * MSEC_PER_SEC,
};

static inline int32_t model_time_decode(uint8_t val)
{
	uint8_t resolution = (val >> 6) & BIT_MASK(2);
	uint8_t steps = val & BIT_MASK(6);

	if (steps == 0x3f) {
		return SYS_FOREVER_MS;
	}

	return steps * time_res[resolution];
}

static inline uint8_t model_time_encode(int32_t ms)
{
	if (ms == SYS_FOREVER_MS) {
		return 0x3f;
	}

	for (int i = 0; i < ARRAY_SIZE(time_res); i++) {
		if (ms >= BIT_MASK(6) * time_res[i]) {
			continue;
		}

		uint8_t steps = ceiling_fraction(ms, time_res[i]);

		return steps | (i << 6);
	}

	return 0x3f;
}

static int onoff_status_send(struct bt_mesh_model *model,
			     struct bt_mesh_msg_ctx *ctx)
{
	uint32_t remaining;

	BT_MESH_MODEL_BUF_DEFINE(buf, OP_ONOFF_STATUS, 3);
	bt_mesh_model_msg_init(&buf, OP_ONOFF_STATUS);

	remaining = k_ticks_to_ms_floor32(
			    k_work_delayable_remaining_get(&onoff.work)) +
		    onoff.transition_time;

	/* Check using remaining time instead of "work pending" to make the
	 * onoff status send the right value on instant transitions. As the
	 * work item is executed in a lower priority than the mesh message
	 * handler, the work will be pending even on instant transitions.
	 */
	if (remaining) {
		net_buf_simple_add_u8(&buf, !onoff.val);
		net_buf_simple_add_u8(&buf, onoff.val);
		net_buf_simple_add_u8(&buf, model_time_encode(remaining));
	} else {
		net_buf_simple_add_u8(&buf, onoff.val);
	}

	return bt_mesh_model_send(model, ctx, &buf, NULL, NULL);
}

static void onoff_timeout(struct k_work *work)
{
	if (onoff.transition_time) {
		/* Start transition.
		 *
		 * The LED should be on as long as the transition is in
		 * progress, regardless of the target value, according to the
		 * Bluetooth Mesh Model specification, section 3.1.1.
		 */
		board_led_set(true);

		k_work_reschedule(&onoff.work, K_MSEC(onoff.transition_time));
		onoff.transition_time = 0;
		return;
	}

	board_led_set(onoff.val);
}

/* Generic OnOff Server message handlers */

static int gen_onoff_get(struct bt_mesh_model *model,
			 struct bt_mesh_msg_ctx *ctx,
			 struct net_buf_simple *buf)
{
	onoff_status_send(model, ctx);
	return 0;
}

static int gen_onoff_set_unack(struct bt_mesh_model *model,
			       struct bt_mesh_msg_ctx *ctx,
			       struct net_buf_simple *buf)
{
	uint8_t val = net_buf_simple_pull_u8(buf);
	uint8_t tid = net_buf_simple_pull_u8(buf);
	int32_t trans = 0;
	int32_t delay = 0;

	if (buf->len) {
		trans = model_time_decode(net_buf_simple_pull_u8(buf));
		delay = net_buf_simple_pull_u8(buf) * 5;
	}

	/* Only perform change if the message wasn't a duplicate and the
	 * value is different.
	 */
	if (tid == onoff.tid && ctx->addr == onoff.src) {
		/* Duplicate */
		return 0;
	}

	if (val == onoff.val) {
		/* No change */
		return 0;
	}

	printk("set: %s delay: %d ms time: %d ms\n", onoff_str[val], delay,
	       trans);

	onoff.tid = tid;
	onoff.src = ctx->addr;
	onoff.val = val;
	onoff.transition_time = trans;

	/* Schedule the next action to happen on the delay, and keep
	 * transition time stored, so it can be applied in the timeout.
	 */
	k_work_reschedule(&onoff.work, K_MSEC(delay));

	return 0;
}

static int gen_onoff_set(struct bt_mesh_model *model,
			 struct bt_mesh_msg_ctx *ctx,
			 struct net_buf_simple *buf)
{
	(void)gen_onoff_set_unack(model, ctx, buf);
	onoff_status_send(model, ctx);

	return 0;
}

static const struct bt_mesh_model_op gen_onoff_srv_op[] = {
	{ OP_ONOFF_GET,       BT_MESH_LEN_EXACT(0), gen_onoff_get },
	{ OP_ONOFF_SET,       BT_MESH_LEN_MIN(2),   gen_onoff_set },
	{ OP_ONOFF_SET_UNACK, BT_MESH_LEN_MIN(2),   gen_onoff_set_unack },
	BT_MESH_MODEL_OP_END,
};

/* Generic OnOff Client */

static int gen_onoff_status(struct bt_mesh_model *model,
			    struct bt_mesh_msg_ctx *ctx,
			    struct net_buf_simple *buf)
{
	uint8_t present = net_buf_simple_pull_u8(buf);

	if (buf->len) {
		uint8_t target = net_buf_simple_pull_u8(buf);
		int32_t remaining_time =
			model_time_decode(net_buf_simple_pull_u8(buf));

		printk("OnOff status: %s -> %s: (%d ms)\n", onoff_str[present],
		       onoff_str[target], remaining_time);
		return 0;
	}

	printk("OnOff status: %s\n", onoff_str[present]);

	return 0;
}

static const struct bt_mesh_model_op gen_onoff_cli_op[] = {
	{OP_ONOFF_STATUS, BT_MESH_LEN_MIN(1), gen_onoff_status},
	BT_MESH_MODEL_OP_END,
};

/* This application only needs one element to contain its models */
static struct bt_mesh_model models[] = {
	BT_MESH_MODEL_CFG_SRV,
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_srv_op, NULL,
		      NULL),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, gen_onoff_cli_op, NULL,
		      NULL),
};

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, models, BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
	.cid = BT_COMP_ID_LF,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

/* Provisioning */
static int output_number(bt_mesh_output_action_t action, uint32_t number)
{
	printk("OOB Number: %u\n", number);

	board_output_number(action, number);

	return 0;
}

/*
 * Prvisioner complete function call.
 */
static void prov_complete(uint16_t net_idx, uint16_t addr)
{
	board_prov_complete();
}

/*
 * Prvisioner reset function call.
 */
static void prov_reset(void)
{
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}

static uint8_t dev_uuid[16];

static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
	.output_size = 4,
	.output_actions = BT_MESH_DISPLAY_NUMBER,
	.output_number = output_number,
	.complete = prov_complete,
	.reset = prov_reset,
};

/** Send an OnOff Set message from the Generic OnOff Client to all nodes. */
static int gen_onoff_send(bool val)
{
	struct bt_mesh_msg_ctx ctx = {
		.app_idx = models[3].keys[0], /* Use the bound key */
		.addr = BT_MESH_ADDR_ALL_NODES,
		.send_ttl = BT_MESH_TTL_DEFAULT,
	};
	static uint8_t tid;

	if (ctx.app_idx == BT_MESH_KEY_UNUSED) {
		printk("The Generic OnOff Client must be bound to a key before "
		       "sending.\n");
		return -ENOENT;
	}

    // send message to base node
	BT_MESH_MODEL_BUF_DEFINE(buf, OP_ONOFF_SET_UNACK, 2);
	bt_mesh_model_msg_init(&buf, OP_ONOFF_SET_UNACK);
	net_buf_simple_add_u8(&buf, val);
	net_buf_simple_add_u8(&buf, tid++);

	printk("Sending OnOff Set: %s\n", onoff_str[val]);

	return bt_mesh_model_send(&models[3], &ctx, &buf, NULL, NULL);
}

/*
 * Check if bluetooth is ready to work.
 */
static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

    // init bluetooth mesh
	err = bt_mesh_init(&prov, &comp);
	if (err) {
		printk("Initializing mesh failed (err %d)\n", err);
		return;
	}

    // enable config settings
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	/* This will be a no-op if settings_load() loaded provisioning info */
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);

	printk("Mesh initialized\n");
}

void main(void)
{
	static struct k_work button_work;
	int err = -1;

    // init bluetooth
	printk("Initializing...\n");

	if (IS_ENABLED(CONFIG_HWINFO)) {
		err = hwinfo_get_device_id(dev_uuid, sizeof(dev_uuid));
	}

    // set provisioner uuid
	if (err < 0) {
		dev_uuid[0] = 0xdd;
		dev_uuid[1] = 0xdd;
	}

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}

    // keep fetch data from senseors
    while (1) {
        for (int i = 0; i < 5; i++) {
            value = report_sensor_values(i + 1);
        }
        k_sleep(K_MESC(500));
    }
}
