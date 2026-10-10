/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Non-blocking read path of sensors/pitotmeter.c.
 *
 * Real sensors/pitotmeter.c is linked and driven through pitotUpdate() with a
 * fake pitot device that has readStart() and a fake bus. A sample is made of
 * one or more transfers: readStart() is called with firstStep for the first
 * one and again after each completed one, get() parses the data once the last
 * transfer is done, and a round that only waits for the bus is reported as a
 * retry with no new sample.
 */

#include <cstdint>
#include <cstring>
#include <cmath>

#include "gtest/gtest.h"

extern "C" {
    #include "platform.h"

    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/time.h"
    #include "config/parameter_group.h"
    #include "config/parameter_group_ids.h"
    #include "drivers/pitotmeter/pitotmeter.h"
    #include "fc/runtime_config.h"
    #include "flight/pid.h"
    #include "flight/wind_estimator.h"
    #include "io/gps.h"
    #include "sensors/barometer.h"
    #include "navigation/navigation.h"
    #define _Static_assert static_assert
    #include "navigation/navigation_private.h"
    #undef _Static_assert
    #include "sensors/pitotmeter.h"
    #include "sensors/sensors.h"

    gpsSolutionData_t gpsSol;
    uint8_t detectedSensors[SENSOR_INDEX_COUNT] = { 0 };
    uint32_t stateFlags = 0;
    uint32_t armingFlags = 0;
    baro_t baro;
    navigationPosControl_t posControl;

    static timeMs_t fakeMillis = 100000;

    // Link surface of pitotmeter.c that this test never reaches
    pidProfile_t pidProfile_Instance;
    pidProfile_t *pidProfile_ProfileCurrent = &pidProfile_Instance;
    uint8_t requestedSensors[SENSOR_INDEX_COUNT] = { 0 };
    simulatorData_t simulatorData;
    bool ms4525Detect(pitotDev_t *) { return false; }
    bool ms5525Detect(pitotDev_t *) { return false; }
    bool dlvrDetect(pitotDev_t *) { return false; }
    bool mspPitotmeterDetect(struct pitotDev_s *) { return false; }
    float _logf(float x) { return logf(x); }
    bool feature(uint32_t) { return false; }
    void sensorsSet(uint32_t) {}
    void sensorsClear(uint32_t) {}
    float getEstimatedWindSpeed(int) { return 0.0f; }
    void imuTransformVectorEarthToBody(fpVector3_t *) {}
    bool isEstimatedWindSpeedValid(void) { return false; }

    timeMs_t millis(void) { return fakeMillis; }
    timeUs_t micros(void) { return (timeUs_t)fakeMillis * 1000; }

    // Fake bus: the transfer started by readStart() is busy for one poll, then completes or fails
    static busDevice_t fakeBusDevice;
    static bool fakeTransferPending;
    static bool fakeTransferFails;
    static int fakeBusyPolls;

    bool busIsBusy(const busDevice_t *dev, bool *error)
    {
        EXPECT_EQ(&fakeBusDevice, dev);
        *error = false;

        if (!fakeTransferPending) {
            return false;
        }
        if (fakeBusyPolls-- > 0) {
            return true;
        }

        fakeTransferPending = false;
        *error = fakeTransferFails;
        return false;
    }

    // Fake driver: a sample takes fakeStepsPerSample transfers, started one by one through readStart()
    static int fakeStepsPerSample;
    static bool fakeBusTaken;           // another device holds the bus, readStart() cannot start anything
    static int fakeStepsStarted;        // transfers started for the sample in progress
    static int startCalls;
    static int getCalls;
    static int readStartCalls;
    static bool lastFirstStep;

    static bool fakeStart(pitotDev_t *) { startCalls++; return true; }
    static bool fakeGet(pitotDev_t *) { getCalls++; return true; }
    static void fakeCalculate(pitotDev_t *, float *pressure, float *temperature)
    {
        *pressure = 0.0f;
        *temperature = 293.15f;
    }

    static busReadStepResult_e fakeReadStart(pitotDev_t *, bool firstStep)
    {
        readStartCalls++;
        lastFirstStep = firstStep;

        if (fakeBusTaken) {
            return BUS_READ_STEP_BUSY;
        }
        if (firstStep) {
            fakeStepsStarted = 0;
        }
        fakeStepsStarted++;
        fakeTransferPending = true;
        fakeBusyPolls = 1;
        return (fakeStepsStarted < fakeStepsPerSample) ? BUS_READ_STEP_NEXT : BUS_READ_STEP_LAST;
    }
}

class PitotAsyncRead : public ::testing::Test {
protected:
    bool newSampleReady = false;
    uint32_t period = 0;

    void tick() { fakeMillis += 20; period = pitotUpdate(&newSampleReady); }

    void SetUp() override {
        memset(&gpsSol, 0, sizeof(gpsSol));
        memset(&posControl, 0, sizeof(posControl));
        memset(&pidProfile_Instance, 0, sizeof(pidProfile_Instance));
        detectedSensors[SENSOR_INDEX_PITOT] = PITOT_MS4525;
        stateFlags = 0;
        armingFlags = 0;
        pitotmeterConfigMutable()->pitot_scale = 1.0f;
        pitotmeterConfigMutable()->pitot_lpf_milli_hz = 0;

        fakeTransferPending = false;
        fakeTransferFails = false;
        fakeBusyPolls = 0;
        fakeStepsPerSample = 2;
        fakeBusTaken = false;
        fakeStepsStarted = 0;
        startCalls = 0;
        getCalls = 0;
        readStartCalls = 0;
        lastFirstStep = false;

        memset(&pitot, 0, sizeof(pitot));
        pitot.dev.busDev = &fakeBusDevice;
        pitot.dev.delay = 0;
        pitot.dev.calibThreshold = 1.0f;
        pitot.dev.start = fakeStart;
        pitot.dev.readStart = fakeReadStart;
        pitot.dev.get = fakeGet;
        pitot.dev.calculate = fakeCalculate;
        pitotStartCalibration();
    }

    // Every test ends with a parsed sample so the read state machine is idle for the next one
    void finishSample() {
        for (int i = 0; i < 10 && getCalls == 0; i++) tick();
        ASSERT_EQ(1, getCalls);
    }
};

TEST_F(PitotAsyncRead, SampleIsParsedAfterTheLastTransferCompletes) {
    tick();     // measurement triggered, first transfer started
    EXPECT_EQ(1, startCalls);
    EXPECT_EQ(1, readStartCalls);
    EXPECT_TRUE(lastFirstStep);
    EXPECT_EQ(0, getCalls);
    EXPECT_FALSE(newSampleReady);
    EXPECT_EQ(PITOT_READ_RETRY_US, period);

    tick();     // on the bus: nothing else is started
    EXPECT_EQ(1, readStartCalls);
    EXPECT_FALSE(newSampleReady);
    EXPECT_EQ(PITOT_READ_RETRY_US, period);

    tick();     // first transfer done, second one started
    EXPECT_EQ(2, readStartCalls);
    EXPECT_FALSE(lastFirstStep);
    EXPECT_EQ(0, getCalls);
    EXPECT_FALSE(newSampleReady);

    tick();     // on the bus
    EXPECT_EQ(0, getCalls);
    EXPECT_FALSE(newSampleReady);

    tick();     // last transfer done: parsed, healthy, next measurement triggered, regular period
    EXPECT_EQ(1, getCalls);
    EXPECT_EQ(2, startCalls);
    EXPECT_EQ(2, readStartCalls);
    EXPECT_EQ(fakeMillis, pitot.lastSeenHealthyMs);
    EXPECT_TRUE(newSampleReady);
    EXPECT_EQ(PITOT_UPDATE_PERIOD_US, period);
}

TEST_F(PitotAsyncRead, SingleTransferSampleIsParsedWhenItCompletes) {
    fakeStepsPerSample = 1;

    tick();     // the only transfer started
    tick();     // on the bus
    EXPECT_EQ(1, readStartCalls);
    EXPECT_EQ(0, getCalls);
    EXPECT_FALSE(newSampleReady);

    tick();     // done: parsed without another readStart()
    EXPECT_EQ(1, readStartCalls);
    EXPECT_EQ(1, getCalls);
    EXPECT_TRUE(newSampleReady);
}

TEST_F(PitotAsyncRead, FailedTransferDropsTheSampleAndStartsOver) {
    fakeTransferFails = true;

    tick();     // first transfer started
    const timeMs_t healthyBefore = pitot.lastSeenHealthyMs;
    tick();     // on the bus
    tick();     // failed: nothing parsed, not healthy
    EXPECT_EQ(1, readStartCalls);
    EXPECT_EQ(0, getCalls);
    EXPECT_EQ(healthyBefore, pitot.lastSeenHealthyMs);

    fakeTransferFails = false;
    tick();     // next attempt begins with the first transfer again
    EXPECT_EQ(2, readStartCalls);
    EXPECT_TRUE(lastFirstStep);

    finishSample();
}

TEST_F(PitotAsyncRead, BusyBusIsRetriedWithoutStartingTheSample) {
    fakeBusTaken = true;

    tick();     // readStart() could not start anything
    EXPECT_EQ(1, readStartCalls);
    EXPECT_TRUE(lastFirstStep);
    EXPECT_FALSE(newSampleReady);
    EXPECT_EQ(PITOT_READ_RETRY_US, period);

    tick();     // tried again, still from the first step
    EXPECT_EQ(2, readStartCalls);
    EXPECT_TRUE(lastFirstStep);
    EXPECT_FALSE(newSampleReady);

    fakeBusTaken = false;
    finishSample();
    EXPECT_EQ(4, readStartCalls);   // first and second transfer of the sample
}
