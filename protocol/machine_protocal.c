//
// Created by charlotte on 7/29/26.
//

#include "machine_protocal.h"

#include <string.h>

int machine_protocol_pack(const float aim_pose[MACHINE_PROTOCAL_AIM_POSE_COUNT],
                          uint8_t machine_aim[MACHINE_PROTOCAL_FRAME_SIZE])
{
    if (aim_pose == NULL || machine_aim == NULL) {
        return -1;
    }

    /*
     * Placeholder protocol:
     * keep the four aim pose floats as raw little-endian payload bytes until
     * the machine-side frame header/checksum/tail are finalized.
     */
    memcpy(machine_aim, aim_pose, MACHINE_PROTOCAL_FRAME_SIZE);

    return 0;
}
