#include <zephyr/kernel.h>
#include "ws_sensors.h"

#define MY_STACK_SIZE 1024
#define MY_PRIORITY 5

/* 
 * Sensor threads.
 */
K_THREAD_DEFINE(lps22hb_tid, MY_STACK_SIZE, 
                lps22hb_thread, NULL, NULL, NULL, 
                MY_PRIORITY, 0, 0);
K_THREAD_DEFINE(hts221_tid, MY_STACK_SIZE, 
                hts221_thread, NULL, NULL, NULL, 
                MY_PRIORITY, 0, 0);
// K_THREAD_DEFINE(ccs811_tid, MY_STACK_SIZE, 
//                 ccs811_thread, NULL, NULL, NULL, 
//                 MY_PRIORITY, 0, 0);
K_THREAD_DEFINE(report_tid, MY_STACK_SIZE, 
                report_sensor_values, NULL, NULL, NULL, 
                MY_PRIORITY, 0, 0);