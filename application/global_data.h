//
// Created by charlotte on 7/13/26.
//

#ifndef DM_GLOBAL_DATA_H
#define DM_GLOBAL_DATA_H

#include <stdint.h>

#include "../protocol/protocol.h"

struct four_axis_robotic_arm;

#define ACTION_SEQUENCE_MAX_COUNT 128U

typedef enum {
    ACTION_SEQUENCE_STATE_IDLE = 0,
    ACTION_SEQUENCE_STATE_LOADING,
    ACTION_SEQUENCE_STATE_READY,
    ACTION_SEQUENCE_STATE_RUNNING,
    ACTION_SEQUENCE_STATE_DONE,
    ACTION_SEQUENCE_STATE_ERROR,
} action_sequence_state_t;

/* Shared IMU attitude.  The sensor task is the sole writer. */
typedef struct {
    volatile float imu_roll_rad;
    volatile float imu_pitch_rad;
    volatile float imu_yaw_rad;
    volatile uint32_t imu_update_tick;
    volatile uint8_t imu_ready;
} global_data_t;

typedef struct host_data {
    float x;
    float y;
    float z;
    float roll;
    uint8_t action;
} aim_pose_t;

typedef struct {
    struct protocol_data frames[ACTION_SEQUENCE_MAX_COUNT];
    volatile uint16_t count;
    volatile uint16_t active_index;
    volatile action_sequence_state_t state;
    volatile uint8_t overflow;
    volatile uint32_t generation;
} action_sequence_t;


extern volatile global_data_t global_data;
extern struct four_axis_robotic_arm arm;
extern  volatile aim_pose_t aim_pose;
extern action_sequence_t action_sequence;
extern volatile uint8_t question_number;

void question_number_uart7_init(void);
uint16_t question_number_uart7_copy_ascii(uint8_t *data, uint16_t max_length);
void question_number_uart7_consume_ascii(uint16_t length);

#endif //DM_GLOBAL_DATA_H
