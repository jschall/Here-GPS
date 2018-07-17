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
#include <modules/uavcan_debug/uavcan_debug.h>
#include <ubx_msgs.h>

#define WT lpwork_thread
WORKER_THREAD_DECLARE_EXTERN(WT)

#define UBX_MSG_TOPIC_GROUP PUBSUB_DEFAULT_TOPIC_GROUP
PUBSUB_TOPIC_GROUP_DECLARE_EXTERN(UBX_MSG_TOPIC_GROUP)

#define GPS_CFG_BAUD 115200U
static const uint32_t baudrates[] = {9600U, 115200U, 4800U, 19200U, 38400U, 57600U, 230400U};
char init_blob[] = "\265\142\006\001\003\000\001\006\001\022\117$PUBX,41,1,0023,0001,115200,0*1C\r\n";

#define UBX_PORT_ID 1

static SerialConfig gps_default_sercfg =
{
  9600,
  0,
  USART_CR2_STOP1_BITS,
  0
};

struct  __attribute__((__packed__)) ubx_header {
    uint8_t hdr[2];
    uint8_t class_id;
    uint8_t msg_id;
    uint16_t payload_len;
};

struct ubx_gps_handle_s {
    bool initialised;
    bool configured;
    SerialDriver *serial;
    SerialConfig sercfg;
    uint8_t baudrate_index;
    uint8_t cfg_step;
    uint8_t cfg_msg_index;
    bool do_cfg;
    uint32_t last_baud_change_ms;
    struct worker_thread_timer_task_s ubx_gps_init_task;
    uint8_t total_msg_cfgs;
} ubx_handle;

struct ubx_msg_cfg_s {
    uint8_t class_id;
    uint8_t msg_id;
    uint8_t rate;
    struct pubsub_topic_s topic;
    struct worker_thread_listener_task_s listener_task;
    pubsub_message_handler_func_ptr handler;
};

static struct gps_handle_s gps_handle;
static void ubx_gps_spinner(void *ctx);
static void ubx_init(struct ubx_gps_handle_s *ubx_handle, SerialDriver* serial, SerialConfig *sercfg);
static void ubx_gps_init_loop(struct worker_thread_timer_task_s *task);
static void send_init_blob(SerialDriver* serial);
static void ubx_gps_configure_msgs(void);

uint8_t parsed_msg_buffer[1024];

enum ubx_cfg_steps {
    STEP_CFG_RATE,
    STEP_CFG_MSG
};

//Msg topics and listeners
//NAV-SOL
struct pubsub_topic_s ubx_nav_sol_topic;
struct worker_thread_listener_task_s ubx_nav_sol_listener;
static void ubx_nav_sol_handler(size_t msg_size, const void* msg, void* ctx);

//ACK-ACK
struct pubsub_topic_s ubx_ack_ack_topic;
struct worker_thread_listener_task_s ubx_ack_ack_listener;
static void ubx_ack_ack_handler(size_t msg_size, const void* msg, void* ctx);

//CFG-RATE
struct pubsub_topic_s ubx_cfg_rate_topic;
struct worker_thread_listener_task_s ubx_cfg_rate_listener;
static void ubx_cfg_rate_handler(size_t msg_size, const void* msg, void* ctx);

//CFG-MSG1
struct pubsub_topic_s ubx_cfg_msg_topic;
struct worker_thread_listener_task_s ubx_cfg_msg_listener;
static void ubx_cfg_msg_handler(size_t msg_size, const void* msg, void* ctx);

//NAV-SVINFO
static void ubx_nav_svinfo_handler(size_t msg_size, const void* msg, void* ctx);
//NAV-POSLLH
static void ubx_nav_posllh_handler(size_t msg_size, const void* msg, void* ctx);

static void ubx_nav_svinfo_handler(size_t msg_size, const void* msg, void* ctx);

struct ubx_msg_cfg_s ubx_cfg_list[] = { \
    {UBX_NAV_SVINFO_CLASS_ID, UBX_NAV_SVINFO_MSG_ID, 2, {0}, {0}, ubx_nav_svinfo_handler}, \
    {UBX_NAV_POSLLH_CLASS_ID, UBX_NAV_POSLLH_MSG_ID, 2, {0}, {0}, ubx_nav_posllh_handler} \
};


THD_WORKING_AREA(ubx_gps_thd_wa, 512);
RUN_AFTER(INIT_END) {
    gps_init(&gps_handle);
    ubx_init(&ubx_handle, &GPS_SERIAL, &gps_default_sercfg);
    chThdCreateStatic(ubx_gps_thd_wa,
                        sizeof(ubx_gps_thd_wa),
                        HIGHPRIO,               // Initial priority.
                        ubx_gps_spinner,             // Thread function.
                        NULL);              // Thread parameter.
    worker_thread_add_timer_task(&WT, &ubx_handle.ubx_gps_init_task, ubx_gps_init_loop, &ubx_handle, MS2ST(100), true);
}

static void ubx_init(struct ubx_gps_handle_s *ubx_handle, SerialDriver* serial, SerialConfig *sercfg)
{
    if (ubx_handle == NULL) {
        return;
    }
    memset(ubx_handle, 0, sizeof(struct ubx_gps_handle_s));
    memcpy(&ubx_handle->sercfg, sercfg, sizeof(SerialConfig));
    ubx_handle->serial = serial;
    ubx_handle->last_baud_change_ms = millis();
    sdStart(ubx_handle->serial, &ubx_handle->sercfg);
    send_init_blob(ubx_handle->serial);
    ubx_handle->total_msg_cfgs = sizeof(ubx_cfg_list)/sizeof(struct ubx_msg_cfg_s);
    //Setup UBX message subscribers
    //NAV-SOL
    pubsub_init_topic(&ubx_nav_sol_topic, &UBX_MSG_TOPIC_GROUP);
    if (gps_ubx_init_msg_topic(&gps_handle, UBX_NAV_SOL_CLASS_ID, UBX_NAV_SOL_MSG_ID, parsed_msg_buffer, sizeof(parsed_msg_buffer), &ubx_nav_sol_topic)) {
        worker_thread_add_listener_task(&WT, &ubx_nav_sol_listener, &ubx_nav_sol_topic, ubx_nav_sol_handler, ubx_handle);
        uavcan_send_debug_msg(LOG_LEVEL_INFO, "GPS", "Registered Topic for 0x%x 0x%x", UBX_NAV_SOL_CLASS_ID, UBX_NAV_SOL_MSG_ID);
    }
    //ACK-ACK
    pubsub_init_topic(&ubx_ack_ack_topic, &UBX_MSG_TOPIC_GROUP);
    if (gps_ubx_init_msg_topic(&gps_handle, UBX_ACK_ACK_CLASS_ID, UBX_ACK_ACK_MSG_ID, parsed_msg_buffer, sizeof(parsed_msg_buffer), &ubx_ack_ack_topic)) {
        worker_thread_add_listener_task(&WT, &ubx_ack_ack_listener, &ubx_ack_ack_topic, ubx_ack_ack_handler, ubx_handle);
        uavcan_send_debug_msg(LOG_LEVEL_INFO, "GPS", "Registered Topic for 0x%x 0x%x", UBX_ACK_ACK_CLASS_ID, UBX_ACK_ACK_MSG_ID);
    }

    //CFG-RATE
    pubsub_init_topic(&ubx_cfg_rate_topic, &UBX_MSG_TOPIC_GROUP);
    if (gps_ubx_init_msg_topic(&gps_handle, UBX_CFG_RATE_CLASS_ID, UBX_CFG_RATE_MSG_ID, parsed_msg_buffer, sizeof(parsed_msg_buffer), &ubx_cfg_rate_topic)) {
        worker_thread_add_listener_task(&WT, &ubx_cfg_rate_listener, &ubx_cfg_rate_topic, ubx_cfg_rate_handler, ubx_handle);
        uavcan_send_debug_msg(LOG_LEVEL_INFO, "GPS", "Registered Topic for 0x%x 0x%x", UBX_CFG_RATE_CLASS_ID, UBX_CFG_RATE_MSG_ID);
    }

    //CFG-MSG
    pubsub_init_topic(&ubx_cfg_msg_topic, &UBX_MSG_TOPIC_GROUP);
    if (gps_ubx_init_msg_topic(&gps_handle, UBX_CFG_MSG_CLASS_ID, UBX_CFG_MSG_MSG_ID, parsed_msg_buffer, sizeof(parsed_msg_buffer), &ubx_cfg_msg_topic)) {
        worker_thread_add_listener_task(&WT, &ubx_cfg_msg_listener, &ubx_cfg_msg_topic, ubx_cfg_msg_handler, ubx_handle);
        uavcan_send_debug_msg(LOG_LEVEL_INFO, "GPS", "Registered Topic for 0x%x 0x%x", UBX_CFG_MSG_CLASS_ID, UBX_CFG_MSG_MSG_ID);
    }

    //Register Messages in the cfg list
    for (uint8_t i = 0; i < ubx_handle->total_msg_cfgs; i++) {
        pubsub_init_topic(&ubx_cfg_list[i].topic, &UBX_MSG_TOPIC_GROUP);
        if (gps_ubx_init_msg_topic(&gps_handle, ubx_cfg_list[i].class_id, ubx_cfg_list[i].msg_id, parsed_msg_buffer, sizeof(parsed_msg_buffer), &ubx_cfg_list[i].topic)) {
            worker_thread_add_listener_task(&WT, &ubx_cfg_list[i].listener_task, &ubx_cfg_list[i].topic, ubx_cfg_list[i].handler, ubx_handle);
            uavcan_send_debug_msg(LOG_LEVEL_INFO, "GPS", "Registered Topic for 0x%x 0x%x", ubx_cfg_list[i].class_id, ubx_cfg_list[i].msg_id);
        }
    }
}

static void send_init_blob(SerialDriver* serial)
{
    sdWrite(serial, (const uint8_t*)init_blob, sizeof(init_blob));
}

static void send_message(uint8_t class_id, uint8_t msg_id, uint8_t* payload, size_t payload_len)
{
    struct ubx_header header;

    header.hdr[0] = 0xB5;
    header.hdr[1] = 0x62;
    header.class_id = class_id;
    header.msg_id = msg_id;
    header.payload_len = payload_len;
    uint8_t crc[2] = {0};

    for(uint16_t i = 2; i < sizeof(header); i++) {
        crc[0] += ((uint8_t*)&header)[i];
        crc[1] += crc[0];
    }
    for(uint16_t i = 0; i < payload_len; i++) {
        crc[0] += payload[i];
        crc[1] += crc[0];
    }
    sdWrite(ubx_handle.serial, (uint8_t*)&header, sizeof(header));
    if (payload_len) {
        sdWrite(ubx_handle.serial, payload, payload_len);
    }
    sdWrite(ubx_handle.serial, crc, sizeof(crc));
}

static void request_message(uint8_t class_id, uint8_t msg_id)
{
    send_message(class_id, msg_id, NULL, 0);
}

static void ubx_gps_spinner(void *ctx)
{
    (void)ctx;
    int16_t byte;
    while(true) {
        byte = chnGetTimeout(ubx_handle.serial, TIME_INFINITE);
        if (gps_spin(&gps_handle, (uint8_t)byte) && ubx_handle.sercfg.speed == GPS_CFG_BAUD) {
            ubx_handle.initialised = true;
        }
    }
}

static void ubx_gps_init_loop(struct worker_thread_timer_task_s *task)
{
    struct ubx_gps_handle_s *handle = (struct ubx_gps_handle_s *)task->ctx;
    static uint8_t try_cnt;
    if (!handle->initialised && (try_cnt % 10)) {
        sdStop(handle->serial);
        handle->baudrate_index++;
        handle->baudrate_index %= sizeof(baudrates)/sizeof(baudrates[0]);
        handle->sercfg.speed = baudrates[handle->baudrate_index];
        sdStart(handle->serial, &handle->sercfg);
        send_init_blob(handle->serial);
    } else if (!handle->configured) {
        ubx_gps_configure_msgs();
    }
    try_cnt++;
}

//MSG Configure
static void ubx_gps_configure_msgs()
{
    struct ubx_cfg_msg1_getset_s cfg_msg1;
    switch(ubx_handle.cfg_step) {
        case STEP_CFG_RATE: { //CFG_RATE
            if (ubx_handle.do_cfg) {
                struct ubx_cfg_rate_getset_s cfg_rate;
                cfg_rate.measRate = 200;
                cfg_rate.navRate = 1;
                cfg_rate.timeRef = 0;
                send_message(UBX_CFG_RATE_CLASS_ID, UBX_CFG_RATE_MSG_ID, (uint8_t*)&cfg_rate, sizeof(cfg_rate));
                ubx_handle.do_cfg = false;
            } else {
                request_message(UBX_CFG_RATE_CLASS_ID, UBX_CFG_RATE_MSG_ID);
            }
            break;
        }
        case STEP_CFG_MSG: {
            if (ubx_handle.cfg_msg_index >= ubx_handle.total_msg_cfgs) {
                ubx_handle.cfg_step++;
                break;
            }
            if (ubx_handle.do_cfg) {
                cfg_msg1.msgClass = ubx_cfg_list[ubx_handle.cfg_msg_index].class_id;
                cfg_msg1.msgID = ubx_cfg_list[ubx_handle.cfg_msg_index].msg_id;
                cfg_msg1.rate = ubx_cfg_list[ubx_handle.cfg_msg_index].rate;
                send_message(UBX_CFG_MSG1_CLASS_ID, UBX_CFG_MSG1_MSG_ID, (uint8_t*)&cfg_msg1, sizeof(cfg_msg1));
                ubx_handle.do_cfg = false;
            } else { //request current config
                struct ubx_cfg_msg_pollrequest_s cfg_msg;
                cfg_msg.msgClass = ubx_cfg_list[ubx_handle.cfg_msg_index].class_id;
                cfg_msg.msgID = ubx_cfg_list[ubx_handle.cfg_msg_index].msg_id;
                send_message(UBX_CFG_MSG_CLASS_ID, UBX_CFG_MSG_MSG_ID, (uint8_t*)&cfg_msg, sizeof(cfg_msg));
            }
            break;
        }
        default:
            break;
    }

}

//MSG Handlers
//NAV-SOL
static void ubx_nav_sol_handler(size_t msg_size, const void* msg, void* ctx)
{
    (void)msg_size;
    (void)ctx;
    const struct gps_msg *parsed_msg = msg;
    struct ubx_nav_sol_periodicpolled_s *nav_sol = ubx_parse_ubx_nav_sol_periodicpolled(parsed_msg->frame_buffer, parsed_msg->msg_len);
    if (nav_sol != NULL) {
        uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "NAV-SOL", "Time: %d Fix: %d SatUsed: %d posX: %ld posY: %ld posZ: %ld", nav_sol->iTOW, nav_sol->gpsFix, nav_sol->numSV, nav_sol->ecefX, nav_sol->ecefY, nav_sol->ecefZ);
    } else {
        uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "NAV-SOL", "BAD MSG");
    }
}

//NAV-SVINFO
static void ubx_nav_svinfo_handler(size_t msg_size, const void* msg, void* ctx)
{
    (void)msg_size;
    (void)ctx;
    const struct gps_msg *parsed_msg = msg;
    struct ubx_nav_svinfo_periodicpolled_s *nav_svinfo = ubx_parse_ubx_nav_svinfo_periodicpolled(parsed_msg->frame_buffer, parsed_msg->msg_len);

    if (nav_svinfo != NULL) {
        uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "NAV-SVINFO", "Time: %d NumSats: %d", nav_svinfo->iTOW, nav_svinfo->numCh);
    } else {
        uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "NAV-SVINFO", "BAD MSG");
    }
}

//NAV-POSLLH
static void ubx_nav_posllh_handler(size_t msg_size, const void* msg, void* ctx)
{
    (void)msg_size;
    (void)ctx;
    const struct gps_msg *parsed_msg = msg;
    struct ubx_nav_posllh_periodicpolled_s *nav_posllh = ubx_parse_ubx_nav_posllh_periodicpolled(parsed_msg->frame_buffer, parsed_msg->msg_len);

    if (nav_posllh != NULL) {
        uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "NAV-POSLLH", "Time: %d lon: %d lat: %d height: %d", nav_posllh->iTOW, nav_posllh->lon, nav_posllh->lat, nav_posllh->height);
    } else {
        uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "NAV-POSLLH", "BAD MSG");
    }
}


//ACK-ACK
static void ubx_ack_ack_handler(size_t msg_size, const void* msg, void* ctx)
{
    (void)msg_size;
    struct ubx_gps_handle_s* _handle = (struct ubx_gps_handle_s*)ctx;
    const struct gps_msg *parsed_msg = msg;
    struct ubx_ack_ack_output_s *ack_ack = ubx_parse_ubx_ack_ack_output(parsed_msg->frame_buffer, parsed_msg->msg_len);
    if (ack_ack != NULL) {
        uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "ACK-ACK", "CFG Ack %d %d", _handle->cfg_step, _handle->cfg_msg_index);
    } else {
        uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "ACK-ACK", "BAD MSG");
    }
}

//CFG-MSG
static void ubx_cfg_msg_handler(size_t msg_size, const void* msg, void* ctx)
{
    (void)msg_size;
    struct ubx_gps_handle_s* _handle = (struct ubx_gps_handle_s*)ctx;
    const struct gps_msg *parsed_msg = msg;
    struct ubx_cfg_msg_getset_s *cfg_msg = ubx_parse_ubx_cfg_msg_getset(parsed_msg->frame_buffer, parsed_msg->msg_len);
    if (cfg_msg != NULL) {
        if (_handle->cfg_step == STEP_CFG_MSG && _handle->cfg_msg_index < _handle->total_msg_cfgs) {
            if (cfg_msg->msgClass == ubx_cfg_list[_handle->cfg_msg_index].class_id && 
                cfg_msg->msgID == ubx_cfg_list[_handle->cfg_msg_index].msg_id &&
                cfg_msg->rate[UBX_PORT_ID] == ubx_cfg_list[_handle->cfg_msg_index].rate) {
                uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "CFG-MSG", "MSG CFG (%d/%d) for 0x%x 0x%x set", _handle->cfg_msg_index + 1, _handle->total_msg_cfgs, ubx_cfg_list[_handle->cfg_msg_index].class_id, ubx_cfg_list[_handle->cfg_msg_index].msg_id, _handle->total_msg_cfgs);
                _handle->cfg_msg_index++;
                _handle->do_cfg = false;
            } else {
                _handle->do_cfg = true;
                uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "CFG-MSG", "MSG CFG for 0x%x 0x%x Not set Received 0x%x 0x%x %d", ubx_cfg_list[_handle->cfg_msg_index].class_id, ubx_cfg_list[_handle->cfg_msg_index].msg_id, cfg_msg->msgClass, cfg_msg->msgID, cfg_msg->rate[UBX_PORT_ID]);
            }
        } else {
            uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "CFG-MSG", "WRONG CFG");
        }
    } else {
        uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "CFG-MSG", "BAD MSG");
    }
}

//CFG-RATE
static void ubx_cfg_rate_handler(size_t msg_size, const void* msg, void* ctx)
{
    (void)msg_size;
    struct ubx_gps_handle_s* _handle = (struct ubx_gps_handle_s*)ctx;
    const struct gps_msg *parsed_msg = msg;
    struct ubx_cfg_rate_getset_s *cfg_rate = ubx_parse_ubx_cfg_rate_getset(parsed_msg->frame_buffer, parsed_msg->msg_len);
    if (cfg_rate != NULL) {
        if (cfg_rate->measRate == 200 && cfg_rate->navRate == 1 && cfg_rate->timeRef == 0) {
            _handle->cfg_step++;
            _handle->do_cfg = false;
            uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "CFG-RATE", "CFG Rate Set");
        } else {
            uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "CFG-RATE", "CFG Rate Not Set %d %d %d", cfg_rate->measRate, cfg_rate->navRate, cfg_rate->timeRef);
            _handle->do_cfg = true;
        }
    } else {
        uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "CFG-RATE", "BAD MSG");
    }
}