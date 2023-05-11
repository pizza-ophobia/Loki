#ifndef WS_SENSORS_H
#define WS_SENSORS_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

// default sample time is 2s
#define DEFAULT_SAMPLE_TIME 2

extern void lps22hb_thread(void);
extern void hts221_thread(void);
extern void ccs811_thread(void);
extern void report_sensor_values(void);

#endif