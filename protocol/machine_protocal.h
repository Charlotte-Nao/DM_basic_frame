//
// Created by charlotte on 7/29/26.
//

#ifndef DM_MACHINE_PROTOCAL_H
#define DM_MACHINE_PROTOCAL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MACHINE_PROTOCOL_TX_BUFFER_SIZE 128U
#define MACHINE_PROTOCOL_RX_LINE_SIZE 256U
#define MACHINE_PROTOCOL_DEFAULT_FEED_MM_PER_MIN 9000.0f

enum machine_protocol_controller_state {
    MACHINE_PROTOCOL_STATE_UNKNOWN = 0,
    MACHINE_PROTOCOL_STATE_IDLE,
    MACHINE_PROTOCOL_STATE_RUN,
    MACHINE_PROTOCOL_STATE_HOME,
    MACHINE_PROTOCOL_STATE_HOLD,
    MACHINE_PROTOCOL_STATE_ALARM,
    MACHINE_PROTOCOL_STATE_ERROR,
    MACHINE_PROTOCOL_STATE_OTHER,
};

enum machine_protocol_event {
    MACHINE_PROTOCOL_EVENT_NONE = 0U,
    MACHINE_PROTOCOL_EVENT_READY = 1U << 0,
    MACHINE_PROTOCOL_EVENT_OK = 1U << 1,
    MACHINE_PROTOCOL_EVENT_ERROR = 1U << 2,
    MACHINE_PROTOCOL_EVENT_ALARM = 1U << 3,
    MACHINE_PROTOCOL_EVENT_STATUS = 1U << 4,
    MACHINE_PROTOCOL_EVENT_LINE_OVERFLOW = 1U << 5,
};

struct machine_protocol_context {
    char rx_line[MACHINE_PROTOCOL_RX_LINE_SIZE];
    uint16_t rx_length;
    uint8_t discard_line;
    enum machine_protocol_controller_state controller_state;
    int32_t error_code;
    int32_t alarm_code;
    float machine_position[3];
    uint8_t machine_position_valid;
};

void machine_protocol_init(struct machine_protocol_context *context);

int machine_protocol_pack(const float target_xyz[3], float feed_mm_per_min,
                          char command[MACHINE_PROTOCOL_TX_BUFFER_SIZE],
                          uint16_t command_capacity);
uint32_t machine_protocol_parse(struct machine_protocol_context *context, const uint8_t *bytes,
                                uint16_t length);

#ifdef __cplusplus
}
#endif

#endif //DM_MACHINE_PROTOCAL_H
