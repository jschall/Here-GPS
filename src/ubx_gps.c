#include "ch.h"
#include "hal.h"
#include <modules/timing/timing.h>
#include <common/helpers.h>
#include <modules/gps/gps.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <modules/uavcan/uavcan.h>
#include <modules/pubsub/pubsub.h>
#include <modules/worker_thread/worker_thread.h>

#define WT hpwork_thread
WORKER_THREAD_DECLARE_EXTERN(WT)

static struct gps_handle_s gps_handle;
static struct worker_thread_timer_task_s ubx_gps_task;
static void ubx_gps_task_func(struct worker_thread_timer_task_s* task);

RUN_AFTER(INIT_END) {
    gps_init(&gps_handle);
    worker_thread_add_timer_task(&WT, &ubx_gps_task, ubx_gps_task_func, NULL, MS2ST(1), true);
}

static void ubx_gps_task_func(struct worker_thread_timer_task_s* task) {
    (void)task;
}
