//
// Created by charlotte on 7/29/26.
//

#ifndef DM_MACHINE_PROTOCAL_H
#define DM_MACHINE_PROTOCAL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MACHINE_PROTOCAL_AIM_POSE_COUNT 5U
#define MACHINE_PROTOCAL_FRAME_SIZE (MACHINE_PROTOCAL_AIM_POSE_COUNT * sizeof(float))

int machine_protocol_pack(const float aim_pose[MACHINE_PROTOCAL_AIM_POSE_COUNT],
                          uint8_t machine_aim[MACHINE_PROTOCAL_FRAME_SIZE]);

#ifdef __cplusplus
}
#endif

#endif //DM_MACHINE_PROTOCAL_H
