#include "ws_sensors.h"

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
// extern void ccs811_thread(void)
// {
//     // Get CCS811 sensor from dts
//     const struct device *const dev = DEVICE_DT_GET_ONE(ams_ccs811);

//     // Check is device ready
//     if (!device_is_ready(dev)) {
// 		printk("CCS811 is not ready.\n");
// 		return;
// 	}

//     // Fetch data every 2s
//     while (true) {
//         sensor_sample_fetch(dev);
//         sensor_channel_get(dev, SENSOR_CHAN_CO2, &co2);
// 		sensor_channel_get(dev, SENSOR_CHAN_VOC, &voc);
// 		sensor_channel_get(dev, SENSOR_CHAN_VOLTAGE, &voltage);
// 		sensor_channel_get(dev, SENSOR_CHAN_CURRENT, &current);
//     }
// }

static double pres_value;
static double temp_value, humidity_value;
static double co2_value, voc_value, voltage_value, current_value;

/* 
 * Print sensor values.
 */
extern void report_sensor_values(void)
{   
    while (1) {
        pres_value = sensor_value_to_double(&pres);
        temp_value = sensor_value_to_double(&temp);
        humidity_value = sensor_value_to_double(&humidity);
        // co2_value = sensor_value_to_double(&co2);
        // voc_value = sensor_value_to_double(&voc);
        // voltage_value = sensor_value_to_double(&voltage);
        // current_value = sensor_value_to_double(&current);

        printk("Pressure: %f kPa\nTemperature: %f C\nHumidity: %.1f%%\n",
        pres_value, temp_value, humidity_value);
        k_sleep(K_SECONDS(2));
    }
}