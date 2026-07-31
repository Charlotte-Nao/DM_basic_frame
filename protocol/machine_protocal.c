//
// Created by charlotte on 7/29/26.
//

#include "machine_protocal.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static enum machine_protocol_controller_state machine_protocol_parse_controller_state(const char *line)
{
    if (strncmp(line, "<Idle|", 6U) == 0 || strcmp(line, "<Idle>") == 0) {
        return MACHINE_PROTOCOL_STATE_IDLE;
    }
    if (strncmp(line, "<Run|", 5U) == 0 || strcmp(line, "<Run>") == 0) {
        return MACHINE_PROTOCOL_STATE_RUN;
    }
    if (strncmp(line, "<Home|", 6U) == 0 || strcmp(line, "<Home>") == 0) {
        return MACHINE_PROTOCOL_STATE_HOME;
    }
    if (strncmp(line, "<Hold|", 6U) == 0 || strcmp(line, "<Hold>") == 0) {
        return MACHINE_PROTOCOL_STATE_HOLD;
    }
    if (strncmp(line, "<Alarm|", 7U) == 0 || strcmp(line, "<Alarm>") == 0) {
        return MACHINE_PROTOCOL_STATE_ALARM;
    }
    return MACHINE_PROTOCOL_STATE_OTHER;
}

static void machine_protocol_parse_machine_position(struct machine_protocol_context *context,
                                                     const char *line)
{
    const char *position_text;
    char *end;
    float position[3];
    position_text = strstr(line, "MPos:");
    if (position_text == NULL) {
        return;
    }
    position_text += 5;
    position[0] = strtof(position_text, &end);
    if (end == position_text || *end != ',') {
        return;
    }
    position_text = end + 1;
    position[1] = strtof(position_text, &end);
    if (end == position_text || *end != ',') {
        return;
    }
    position_text = end + 1;
    position[2] = strtof(position_text, &end);
    if (end == position_text) {
        return;
    }
    context->machine_position[0] = position[0];
    context->machine_position[1] = position[1];
    context->machine_position[2] = position[2];
    context->machine_position_valid = 1U;
}

static uint32_t machine_protocol_process_line(struct machine_protocol_context *context, char *line)
{
    char *end;
    char *line_end;
    long code;
    size_t line_length;
    while (isspace((unsigned char)*line) != 0) {
        ++line;
    }
    line_end = line + strlen(line);
    while (line_end > line && isspace((unsigned char)line_end[-1]) != 0) {
        --line_end;
        *line_end = '\0';
    }
    if (strcmp(line, "ok") == 0) {
        return MACHINE_PROTOCOL_EVENT_OK;
    }
    if (strncmp(line, "error:", 6U) == 0) {
        code = strtol(&line[6], &end, 10);
        context->error_code = (end == &line[6]) ? -1 : (int32_t)code;
        context->controller_state = MACHINE_PROTOCOL_STATE_ERROR;
        return MACHINE_PROTOCOL_EVENT_ERROR;
    }
    if (strncmp(line, "ALARM:", 6U) == 0) {
        code = strtol(&line[6], &end, 10);
        context->alarm_code = (end == &line[6]) ? -1 : (int32_t)code;
        context->controller_state = MACHINE_PROTOCOL_STATE_ALARM;
        return MACHINE_PROTOCOL_EVENT_ALARM;
    }
    line_length = strlen(line);
    if (line_length >= 2U && line[0] == '<' && line[line_length - 1U] == '>') {
        context->controller_state = machine_protocol_parse_controller_state(line);
        machine_protocol_parse_machine_position(context, line);
        return MACHINE_PROTOCOL_EVENT_STATUS;
    }
    if (strncmp(line, "Grbl ", 5U) == 0) {
        context->controller_state = MACHINE_PROTOCOL_STATE_UNKNOWN;
        return MACHINE_PROTOCOL_EVENT_READY;
    }
    return MACHINE_PROTOCOL_EVENT_NONE;
}

void machine_protocol_init(struct machine_protocol_context *context)
{
    if (context == NULL) {
        return;
    }
    memset(context, 0, sizeof(*context));
    context->controller_state = MACHINE_PROTOCOL_STATE_UNKNOWN;
}

int machine_protocol_pack(const float target_xyz[3], float feed_mm_per_min,
                          char command[MACHINE_PROTOCOL_TX_BUFFER_SIZE],
                          uint16_t command_capacity)
{
    int length;
    if (target_xyz == NULL || command == NULL || command_capacity == 0U) {
        return -1;
    }
    if (!isfinite(target_xyz[0]) || !isfinite(target_xyz[1]) ||
        !isfinite(target_xyz[2]) || !isfinite(feed_mm_per_min) ||
        feed_mm_per_min <= 0.0f) {
        return -1;
    }
    length = snprintf(command, command_capacity,
                      "G21 G90 G94 G1 X%.3f Y%.3f Z%.3f F%.0f\n",
                      target_xyz[0], target_xyz[1], target_xyz[2], feed_mm_per_min);
    if (length <= 0 || length >= (int)command_capacity) {
        return -1;
    }
    return length;
}

uint32_t machine_protocol_parse(struct machine_protocol_context *context, const uint8_t *bytes,
                                uint16_t length)
{
    uint32_t events = MACHINE_PROTOCOL_EVENT_NONE;
    if (context == NULL || bytes == NULL) {
        return MACHINE_PROTOCOL_EVENT_NONE;
    }
    for (uint16_t i = 0U; i < length; i++) {
        uint8_t byte = bytes[i];
        if (byte == (uint8_t)'\r') {
            continue;
        }
        if (byte == (uint8_t)'\n') {
            if (context->discard_line != 0U) {
                context->discard_line = 0U;
                context->rx_length = 0U;
                continue;
            }
            if (context->rx_length == 0U) {
                continue;
            }
            context->rx_line[context->rx_length] = '\0';
            events |= machine_protocol_process_line(context, context->rx_line);
            context->rx_length = 0U;
            continue;
        }
        if (context->discard_line != 0U) {
            continue;
        }
        if (context->rx_length >= MACHINE_PROTOCOL_RX_LINE_SIZE - 1U) {
            context->rx_length = 0U;
            context->discard_line = 1U;
            events |= MACHINE_PROTOCOL_EVENT_LINE_OVERFLOW;
            continue;
        }
        context->rx_line[context->rx_length] = (char)byte;
        context->rx_length++;
    }
    return events;
}
