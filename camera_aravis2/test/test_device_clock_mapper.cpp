// Copyright (c) 2026 WATonomous and contributors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Fraunhofer IOSB nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

// Std
#include <cstdint>
#include <cstdlib>
#include <random>

#include "camera_aravis2/device_clock_mapper.h"

using camera_aravis2::DeviceClockMapper;
using camera_aravis2::DeviceClockStats;

namespace
{

constexpr int64_t kMs          = 1000000;
constexpr int64_t kSec         = 1000 * kMs;
constexpr int64_t kFramePeriod = 50 * kMs;  // 20 fps
constexpr int64_t kMinDelay    = 10 * kMs;  // constant part of the transfer delay

struct Frame
{
    int64_t capture_ns;  // true capture time on the host clock
    int64_t device_ns;   // camera counter
    int64_t arrival_ns;  // host arrival time
};

/// One simulated camera at 20 fps: device counter with drift, delay = 10 ms + exp(3 ms) + extra.
class CameraSim
{
  public:
    explicit CameraSim(double drift_ppm = 0.0, uint32_t seed = 42) :
      drift_(drift_ppm * 1e-6),
      rng_(seed),
      jitter_(1.0 / (3.0 * kMs))
    {
    }

    Frame next(int64_t extra_delay_ns = 0)
    {
        const double ELAPSED_NS = static_cast<double>(capture_ns_ - kHostStart);
        const int64_t JITTER_NS = static_cast<int64_t>(jitter_(rng_));

        Frame f;
        f.capture_ns = capture_ns_;
        f.device_ns  = device_base_ns_ + static_cast<int64_t>(ELAPSED_NS * (1.0 + drift_));
        f.arrival_ns = capture_ns_ + kMinDelay + JITTER_NS + extra_delay_ns;
        capture_ns_ += kFramePeriod;
        return f;
    }

    void skipFrames(int n)
    {
        capture_ns_ += n * kFramePeriod;
    }
    /// Host clock jumps by step_ns; the device counter keeps running undisturbed.
    void stepHostClock(int64_t step_ns)
    {
        capture_ns_ += step_ns;
        device_base_ns_ -= step_ns;
    }
    void rebootCamera()
    {
        device_base_ns_ = 5 * kMs - (capture_ns_ - kHostStart);
    }

    static constexpr int64_t kHostStart = 1760000000LL * kSec;  // ~2025 epoch in ns

  private:
    double drift_;
    std::mt19937 rng_;
    std::exponential_distribution<double> jitter_;
    int64_t capture_ns_     = kHostStart;
    int64_t device_base_ns_ = 123 * kSec;  // camera powered on 123 s "ago"
};

DeviceClockMapper makeMapper(int64_t latency_ns = 0)
{
    DeviceClockMapper m;
    m.configure(10 * kSec, 10, latency_ns);
    return m;
}

}  // namespace

TEST(DeviceClockMapper, ReturnsArrivalUntilWindowFilled)
{
    auto m = makeMapper();
    CameraSim cam;
    for (int i = 0; i < 199; ++i)  // 199 frames < 10 s of arrivals
    {
        Frame f  = cam.next();
        auto res = m.map(f.device_ns, f.arrival_ns);
        EXPECT_FALSE(res.converged);
        EXPECT_EQ(res.stamp_ns, f.arrival_ns);
    }
    bool converged = false;
    for (int i = 0; i < 5; ++i)
    {
        Frame f   = cam.next();
        converged = m.map(f.device_ns, f.arrival_ns).converged;
    }
    EXPECT_TRUE(converged);
}

TEST(DeviceClockMapper, StampDeltasWithinOneMsDespiteDelaySpikes)
{
    auto m = makeMapper();
    CameraSim cam;
    int64_t prev_stamp = -1;
    for (int i = 0; i < 20 * 120; ++i)  // 2 min
    {
        Frame f  = cam.next((i % 37 == 0) ? 40 * kMs : 0);  // periodic 40 ms spikes
        auto res = m.map(f.device_ns, f.arrival_ns);
        if (!res.converged)
            continue;
        if (prev_stamp >= 0)
        {
            EXPECT_NEAR(static_cast<double>(res.stamp_ns - prev_stamp), kFramePeriod, 1.0 * kMs);
        }
        prev_stamp = res.stamp_ns;
    }
    ASSERT_GE(prev_stamp, 0);
}

TEST(DeviceClockMapper, StampIsCaptureTimePlusMinDelayWithZeroLatency)
{
    auto m = makeMapper();
    CameraSim cam;
    for (int i = 0; i < 20 * 60; ++i)
    {
        Frame f  = cam.next();
        auto res = m.map(f.device_ns, f.arrival_ns);
        if (res.converged)
        {
            const int64_t ERR = res.stamp_ns - (f.capture_ns + kMinDelay);
            EXPECT_GE(ERR, 0);
            EXPECT_LT(ERR, 1 * kMs);
        }
    }
}

TEST(DeviceClockMapper, LatencyCorrectionRecoversCaptureTime)
{
    auto m = makeMapper(kMinDelay);
    CameraSim cam;
    for (int i = 0; i < 20 * 60; ++i)
    {
        Frame f  = cam.next();
        auto res = m.map(f.device_ns, f.arrival_ns);
        if (res.converged)
        {
            EXPECT_LT(std::llabs(res.stamp_ns - f.capture_ns), 1 * kMs);
        }
    }
}

TEST(DeviceClockMapper, StampNeverLaterThanArrivalAndNeverClamped)
{
    auto m = makeMapper();
    CameraSim cam;
    for (int i = 0; i < 20 * 120; ++i)
    {
        Frame f  = cam.next((i % 11 == 0) ? 25 * kMs : 0);
        auto res = m.map(f.device_ns, f.arrival_ns);
        EXPECT_LE(res.stamp_ns, f.arrival_ns);
        EXPECT_FALSE(res.clamped);
    }
}

TEST(DeviceClockMapper, NegativeLatencyIsClampedToArrival)
{
    auto m = makeMapper(-5 * kMs);
    CameraSim cam;
    bool any_clamped = false;
    for (int i = 0; i < 20 * 30; ++i)
    {
        Frame f  = cam.next();
        auto res = m.map(f.device_ns, f.arrival_ns);
        EXPECT_LE(res.stamp_ns, f.arrival_ns);
        any_clamped = any_clamped || res.clamped;
    }
    EXPECT_TRUE(any_clamped);
}

TEST(DeviceClockMapper, TracksPlusMinus50PpmDriftOverTenMinutes)
{
    for (double ppm : {50.0, -50.0})
    {
        auto m = makeMapper(kMinDelay);
        CameraSim cam(ppm);
        int64_t prev_offset = 0;
        bool have_prev      = false;
        for (int i = 0; i < 20 * 600; ++i)
        {
            Frame f  = cam.next();
            auto res = m.map(f.device_ns, f.arrival_ns);
            if (!res.converged)
                continue;
            EXPECT_LT(std::llabs(res.stamp_ns - f.capture_ns), 1 * kMs) << "ppm=" << ppm;
            if (have_prev)
            {
                EXPECT_LT(std::llabs(res.offset_ns - prev_offset), 1 * kMs) << "offset jump";
            }
            prev_offset = res.offset_ns;
            have_prev   = true;
        }
    }
}

TEST(DeviceClockMapper, DroppedFramesDoNotResetOrDegrade)
{
    auto m = makeMapper(kMinDelay);
    CameraSim cam;
    for (int i = 0; i < 20 * 15; ++i)
    {
        Frame f = cam.next();
        m.map(f.device_ns, f.arrival_ns);
    }
    cam.skipFrames(30);  // 1.5 s gap
    for (int i = 0; i < 20 * 5; ++i)
    {
        Frame f  = cam.next();
        auto res = m.map(f.device_ns, f.arrival_ns);
        EXPECT_FALSE(res.was_reset);
        EXPECT_TRUE(res.converged);
        EXPECT_LT(std::llabs(res.stamp_ns - f.capture_ns), 1 * kMs);
    }
}

TEST(DeviceClockMapper, ResetsAndReconvergesAfterCameraReboot)
{
    auto m = makeMapper(kMinDelay);
    CameraSim cam;
    for (int i = 0; i < 20 * 15; ++i)
    {
        Frame f = cam.next();
        m.map(f.device_ns, f.arrival_ns);
    }
    cam.rebootCamera();
    Frame f  = cam.next();
    auto res = m.map(f.device_ns, f.arrival_ns);
    EXPECT_TRUE(res.was_reset);
    EXPECT_FALSE(res.converged);
    EXPECT_EQ(res.stamp_ns, f.arrival_ns);

    for (int i = 0; i < 20 * 15; ++i)
    {
        f   = cam.next();
        res = m.map(f.device_ns, f.arrival_ns);
        EXPECT_FALSE(res.was_reset);
    }
    EXPECT_TRUE(res.converged);
    EXPECT_LT(std::llabs(res.stamp_ns - f.capture_ns), 1 * kMs);
}

TEST(DeviceClockMapper, ResetsOnHostClockStep)
{
    for (int64_t step : {2 * kSec, -2 * kSec})
    {
        auto m = makeMapper();
        CameraSim cam;
        for (int i = 0; i < 20 * 15; ++i)
        {
            Frame f = cam.next();
            m.map(f.device_ns, f.arrival_ns);
        }
        cam.stepHostClock(step);
        Frame f = cam.next();
        EXPECT_TRUE(m.map(f.device_ns, f.arrival_ns).was_reset) << "step=" << step;
    }
}

TEST(DeviceClockMapper, StaleMinimaExpireAfterLongStall)
{
    auto m = makeMapper();
    CameraSim cam;
    for (int i = 0; i < 20 * 15; ++i)
    {
        Frame f = cam.next();
        m.map(f.device_ns, f.arrival_ns);
    }
    cam.skipFrames(20 * 15);     // camera stalls for 15 s (longer than the window) ...
    cam.stepHostClock(5 * kMs);  // ... while NTP nudges the host clock by 5 ms
    for (int i = 0; i < 20 * 5; ++i)
    {
        Frame f  = cam.next();
        auto res = m.map(f.device_ns, f.arrival_ns);
        if (!res.converged)
            continue;
        const int64_t ERR = res.stamp_ns - (f.capture_ns + kMinDelay);
        EXPECT_GE(ERR, 0);  // pre-stall minima would make stamps ~5 ms too early
        EXPECT_LT(ERR, 1 * kMs);
    }
}

TEST(DeviceClockMapper, ConfigureClampsBucketCount)
{
    DeviceClockMapper m;
    m.configure(10 * kSec, 1000, 0);  // > kMaxBuckets must not overflow storage
    CameraSim cam;
    for (int i = 0; i < 20 * 12; ++i)
    {
        Frame f = cam.next();
        m.map(f.device_ns, f.arrival_ns);
    }
    m.configure(10 * kSec, 0, 0);  // < 1 must not divide by zero
    Frame f = cam.next();
    EXPECT_EQ(m.map(f.device_ns, f.arrival_ns).stamp_ns, f.arrival_ns);
}

TEST(DeviceClockStats, AccumulatesAndClears)
{
    DeviceClockStats s;
    DeviceClockMapper::Result r;
    r.stamp_ns  = 100;
    r.offset_ns = 7;
    r.converged = true;
    s.add(r, 110);
    r.stamp_ns = 100;
    r.clamped  = true;
    s.add(r, 130);
    EXPECT_EQ(s.n_frames, 2);
    EXPECT_EQ(s.n_clamped, 1);
    EXPECT_EQ(s.min_arrival_minus_stamp_ns, 10);
    EXPECT_EQ(s.max_arrival_minus_stamp_ns, 30);
    EXPECT_EQ(s.sum_arrival_minus_stamp_ns, 40);
    EXPECT_EQ(s.offset_ns, 7);
    s.clear();
    EXPECT_EQ(s.n_frames, 0);
}
