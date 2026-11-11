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

/*
 * DroneCAN compass driver tests: scaling, pre-rotation, staleness and
 * source locking of drivers/compass/compass_dronecan.c.
 */

#include <cmath>
#include <cstdint>
#include <cstring>

extern "C" {
#include "platform.h"
#include "common/axis.h"
#include "drivers/compass/compass.h"
#include "drivers/compass/compass_dronecan.h"
#include "fc/runtime_config.h"

/* Controllable time source for drivers/time.h millis() */
static uint32_t mock_time_ms = 0;
uint32_t millis(void) { return mock_time_ms; }

/* boardalignment.c reads STATE(TAILSITTER) */
uint32_t stateFlags = 0;
}

#include "gtest/gtest.h"

class DroneCANCompassTest : public ::testing::Test {
protected:
    magDev_t dev;

    void SetUp() override {
        memset(&dev, 0, sizeof(dev));
        mock_time_ms = 1000;
        ASSERT_TRUE(dronecanMagDetect(&dev));
        ASSERT_TRUE(dev.init(&dev));
    }

    void receive(uint8_t nodeId, float x, float y, float z) {
        const float field[3] = { x, y, z };
        dronecanMagReceiveField(nodeId, field);
    }
};

TEST_F(DroneCANCompassTest, NoDataYet_ReadFails)
{
    EXPECT_FALSE(dev.read(&dev));
}

TEST_F(DroneCANCompassTest, FieldScaledToMilligaussAndPreRotatedLikeMspMag)
{
    receive(10, 0.1f, 0.2f, 0.3f);

    ASSERT_TRUE(dev.read(&dev));
    /* CW90_DEG_FLIP: X <- Y, Y <- X, Z <- -Z, in milligauss */
    EXPECT_EQ(dev.magADCRaw[X], 200);
    EXPECT_EQ(dev.magADCRaw[Y], 100);
    EXPECT_EQ(dev.magADCRaw[Z], -300);
}

TEST_F(DroneCANCompassTest, FieldClampedToInt16Range)
{
    receive(10, 100.0f, -100.0f, -100.0f);

    ASSERT_TRUE(dev.read(&dev));
    EXPECT_EQ(dev.magADCRaw[X], -INT16_MAX);
    EXPECT_EQ(dev.magADCRaw[Y], INT16_MAX);
    /* Z is negated by the pre-rotation; the clamp must leave room for that */
    EXPECT_EQ(dev.magADCRaw[Z], INT16_MAX);
}

TEST_F(DroneCANCompassTest, NonFiniteSampleIgnored)
{
    receive(10, 0.1f, 0.2f, 0.3f);
    receive(10, NAN, 0.0f, 0.0f);
    receive(10, 0.0f, INFINITY, 0.0f);
    receive(10, 0.0f, 0.0f, -INFINITY);

    ASSERT_TRUE(dev.read(&dev));
    EXPECT_EQ(dev.magADCRaw[X], 200);
    EXPECT_EQ(dev.magADCRaw[Y], 100);
    EXPECT_EQ(dev.magADCRaw[Z], -300);
}

TEST_F(DroneCANCompassTest, AnonymousNodeIgnored)
{
    receive(0, 0.1f, 0.2f, 0.3f);
    EXPECT_FALSE(dev.read(&dev));

    /* and it did not take the lock */
    receive(10, 0.1f, 0.2f, 0.3f);
    ASSERT_TRUE(dev.read(&dev));
    EXPECT_EQ(dev.magADCRaw[X], 200);
}

TEST_F(DroneCANCompassTest, ReadFailsAfterTimeout)
{
    receive(10, 0.1f, 0.2f, 0.3f);
    ASSERT_TRUE(dev.read(&dev));

    mock_time_ms += 250;
    EXPECT_TRUE(dev.read(&dev));

    mock_time_ms += 1;
    EXPECT_FALSE(dev.read(&dev));
}

TEST_F(DroneCANCompassTest, SecondSourceIgnoredWhileFirstIsAlive)
{
    receive(10, 0.1f, 0.2f, 0.3f);
    receive(11, 0.5f, 0.5f, 0.5f);

    ASSERT_TRUE(dev.read(&dev));
    EXPECT_EQ(dev.magADCRaw[X], 200);
    EXPECT_EQ(dev.magADCRaw[Y], 100);
    EXPECT_EQ(dev.magADCRaw[Z], -300);
}

TEST_F(DroneCANCompassTest, SecondSourceNeverTakesOverAfterFirstGoesStale)
{
    receive(10, 0.1f, 0.2f, 0.3f);

    mock_time_ms += 251;
    receive(11, 0.5f, 0.6f, 0.7f);

    /* no failover: the compass goes unhealthy instead */
    EXPECT_FALSE(dev.read(&dev));

    /* and the original source is picked up again when it comes back */
    receive(10, 0.1f, 0.2f, 0.3f);
    ASSERT_TRUE(dev.read(&dev));
    EXPECT_EQ(dev.magADCRaw[X], 200);
    EXPECT_EQ(dev.magADCRaw[Y], 100);
    EXPECT_EQ(dev.magADCRaw[Z], -300);
}

TEST_F(DroneCANCompassTest, InitUnlocksSource)
{
    receive(10, 0.1f, 0.2f, 0.3f);
    ASSERT_TRUE(dev.init(&dev));
    EXPECT_FALSE(dev.read(&dev));

    receive(11, 0.5f, 0.6f, 0.7f);
    ASSERT_TRUE(dev.read(&dev));
    EXPECT_EQ(dev.magADCRaw[X], 600);
}
