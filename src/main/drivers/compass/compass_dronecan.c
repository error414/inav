/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#if defined(USE_MAG_DRONECAN)

#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"
#include "common/time.h"

#include "drivers/time.h"
#include "drivers/compass/compass.h"
#include "drivers/compass/compass_dronecan.h"

#include "sensors/boardalignment.h"

#define DRONECAN_MAG_TIMEOUT_MS     250     // Less than 4Hz updates is considered a failure
#define DRONECAN_MAG_GAUSS_TO_RAW   1000.0f // Raw units are milligauss, the same scale ArduPilot uses

static float dronecanMagData[XYZ_AXIS_COUNT];
static timeMs_t dronecanMagLastUpdateMs;

// The first (node, sensor) pair heard owns the compass until it goes silent for
// DRONECAN_MAG_TIMEOUT_MS. Without this, two magnetometer nodes on one bus would
// interleave samples taken in two different frames.
static bool dronecanMagSourceLocked;
static uint8_t dronecanMagNodeId;
static uint8_t dronecanMagSensorId;

static bool dronecanMagIsStale(void)
{
    return (millis() - dronecanMagLastUpdateMs) > DRONECAN_MAG_TIMEOUT_MS;
}

static bool dronecanMagInit(magDev_t *magDev)
{
    UNUSED(magDev);
    dronecanMagData[X] = 0;
    dronecanMagData[Y] = 0;
    dronecanMagData[Z] = 0;
    dronecanMagLastUpdateMs = 0;
    dronecanMagSourceLocked = false;
    return true;
}

void dronecanMagReceiveField(uint8_t nodeId, uint8_t sensorId, const float fieldGauss[3])
{
    // float16 fields on the wire can carry NaN for "not populated"
    if (isnan(fieldGauss[X]) || isnan(fieldGauss[Y]) || isnan(fieldGauss[Z])) {
        return;
    }

    if (!dronecanMagSourceLocked || dronecanMagIsStale()) {
        dronecanMagSourceLocked = true;
        dronecanMagNodeId = nodeId;
        dronecanMagSensorId = sensorId;
    } else if (nodeId != dronecanMagNodeId || sensorId != dronecanMagSensorId) {
        return;
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        dronecanMagData[axis] = constrainf(fieldGauss[axis] * DRONECAN_MAG_GAUSS_TO_RAW, INT16_MIN, INT16_MAX);
    }

    // AP_Periph publishes the field in the ArduPilot body frame, the same frame its
    // MSP compass messages use, so apply the same pre-rotation as the MSP mag driver
    applySensorAlignment(dronecanMagData, dronecanMagData, CW90_DEG_FLIP);

    dronecanMagLastUpdateMs = millis();
}

static bool dronecanMagRead(magDev_t *magDev)
{
    if (dronecanMagIsStale()) {
        return false;
    }

    magDev->magADCRaw[X] = lrintf(dronecanMagData[X]);
    magDev->magADCRaw[Y] = lrintf(dronecanMagData[Y]);
    magDev->magADCRaw[Z] = lrintf(dronecanMagData[Z]);

    return true;
}

bool dronecanMagDetect(magDev_t *mag)
{
    mag->init = dronecanMagInit;
    mag->read = dronecanMagRead;
    return true;
}

#endif
