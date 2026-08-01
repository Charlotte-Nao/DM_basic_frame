//
// Created by charlotte on 7/21/26.
//

#include "./usb_task.h"
#include "cmsis_os2.h"
#include <stdint.h>
#include "../../bsp/usb/usb.h"
#include "../../protocol/protocol.h"
#include"../../application/global_data.h"
#include "../../bsp/LED/LED.h"

#define ACTION_SEQUENCE_START_MARKER 5
#define ACTION_SEQUENCE_END_MARKER   8
#define ACTION_SEQUENCE_LOAD_TIMEOUT_MS 30000U

static uint32_t action_sequence_last_rx_tick;

static int protocol_data_is_marker(const struct protocol_data *data,
                                   int16_t marker)
{
    if (data == NULL) {
        return 0;
    }

    return data->x == (float)marker &&
           data->y == (float)marker &&
           data->z == (float)marker &&
           data->roll == marker &&
           data->action == (uint8_t)marker;
}

static void publish_legacy_aim_pose(const struct protocol_data *data)
{
    if (data == NULL) {
        return;
    }

    aim_pose.x = data->x;
    aim_pose.y = data->y;
    aim_pose.z = data->z;
    aim_pose.roll = (float)data->roll / 10.0f;
    aim_pose.action = data->action;
}

static void action_sequence_begin(void)
{
    int32_t lock_state = osKernelLock();

    action_sequence_last_rx_tick = osKernelGetTickCount();
    action_sequence.count = 0U;
    action_sequence.active_index = 0U;
    action_sequence.state = ACTION_SEQUENCE_STATE_LOADING;
    action_sequence.overflow = 0U;
    action_sequence.generation++;

    (void)osKernelRestoreLock(lock_state);
}

static void action_sequence_finish(void)
{
    int32_t lock_state = osKernelLock();

    action_sequence_last_rx_tick = osKernelGetTickCount();
    if (action_sequence.state == ACTION_SEQUENCE_STATE_LOADING &&
        action_sequence.count > 0U &&
        action_sequence.overflow == 0U) {
        action_sequence.active_index = 0U;
        action_sequence.state = ACTION_SEQUENCE_STATE_READY;
    } else if (action_sequence.state == ACTION_SEQUENCE_STATE_LOADING) {
        action_sequence.active_index = 0U;
        action_sequence.state = ACTION_SEQUENCE_STATE_IDLE;
    }

    action_sequence.generation++;

    (void)osKernelRestoreLock(lock_state);
}

static void action_sequence_append(const struct protocol_data *data)
{
    uint16_t index;
    int32_t lock_state;

    if (data == NULL) {
        return;
    }

    lock_state = osKernelLock();

    if (action_sequence.state != ACTION_SEQUENCE_STATE_LOADING) {
        (void)osKernelRestoreLock(lock_state);
        return;
    }

    if (action_sequence.count >= ACTION_SEQUENCE_MAX_COUNT) {
        action_sequence.overflow = 1U;
        action_sequence.state = ACTION_SEQUENCE_STATE_ERROR;
        action_sequence.generation++;
        (void)osKernelRestoreLock(lock_state);
        LED_RED_SET();
        return;
    }

    index = action_sequence.count;
    action_sequence.frames[index] = *data;
    action_sequence.count = (uint16_t)(index + 1U);
    action_sequence_last_rx_tick = osKernelGetTickCount();

    (void)osKernelRestoreLock(lock_state);
}

static void action_sequence_monitor_timeout(void)
{
    int32_t lock_state;
    uint8_t timed_out = 0U;
    uint32_t now = osKernelGetTickCount();

    lock_state = osKernelLock();
    if (action_sequence.state == ACTION_SEQUENCE_STATE_LOADING &&
        (uint32_t)(now - action_sequence_last_rx_tick) >= ACTION_SEQUENCE_LOAD_TIMEOUT_MS) {
        action_sequence.state = ACTION_SEQUENCE_STATE_ERROR;
        action_sequence.generation++;
        timed_out = 1U;
    }
    (void)osKernelRestoreLock(lock_state);

    if (timed_out != 0U) {
        LED_RED_SET();
    }
}

static void handle_protocol_frame(const struct protocol_data *data,
                                  void *user_context)
{
    action_sequence_state_t state;
    int32_t lock_state;

    (void)user_context;

    if (data == NULL) {
        return;
    }

    if (protocol_data_is_marker(data, ACTION_SEQUENCE_START_MARKER)) {
        action_sequence_begin();
        LED_GREEN_SET();
        return;
    }

    if (protocol_data_is_marker(data, ACTION_SEQUENCE_END_MARKER)) {
        action_sequence_finish();
        LED_SKY_SET();
        return;
    }

    lock_state = osKernelLock();
    state = action_sequence.state;
    (void)osKernelRestoreLock(lock_state);

    if (state == ACTION_SEQUENCE_STATE_LOADING) {
        action_sequence_append(data);
        return;
    }

    if (state == ACTION_SEQUENCE_STATE_IDLE ||
        state == ACTION_SEQUENCE_STATE_DONE ||
        state == ACTION_SEQUENCE_STATE_ERROR) {
        publish_legacy_aim_pose(data);
    }
}

void usb_task(void)
{
    struct usb_device *usb;
    uint8_t rx_buffer[128];
    uint8_t question_tx_buffer[64];
    uint16_t question_tx_length;
    int result;

    usb = usb_get_device("usb_cdc");

    if (usb == NULL) {
        for (;;) {
            osDelay(1000U);
        }
    }

    for (;;)
    {
        result = usb->usb_read(usb, rx_buffer, sizeof(rx_buffer));

        if (result > 0) {
            (void)protocol_parse_each(rx_buffer,
                                      (uint16_t)result,
                                      handle_protocol_frame,
                                      NULL);
        }

        question_tx_length = question_number_uart7_copy_ascii(
            question_tx_buffer,
            sizeof(question_tx_buffer)
        );
        if (question_tx_length > 0U &&
            usb->usb_send_bytes(
                usb,
                question_tx_buffer,
                question_tx_length
            ) == USB_DEVICE_OK) {
            question_number_uart7_consume_ascii(question_tx_length);
            LED_PINK_SET();
        }

        action_sequence_monitor_timeout();
        osDelay(1U);
    }
}
