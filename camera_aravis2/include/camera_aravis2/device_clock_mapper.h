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

#ifndef CAMERA_ARAVIS2__DEVICE_CLOCK_MAPPER_H_
#define CAMERA_ARAVIS2__DEVICE_CLOCK_MAPPER_H_

// Std
#include <array>
#include <cstdint>

namespace camera_aravis2
{

/**
 * @brief Maps the free-running device timestamp of a camera onto the host clock.
 *
 * For every frame, arrival - device = clock offset + transfer delay, where the delay is always
 * at least some constant minimum. The minimum of (arrival - device) over a sliding window is
 * therefore the best estimate of (offset + min delay). The window is split into fixed buckets
 * keyed by arrival time, so each frame costs O(n_buckets) and no memory is allocated.
 *
 * Not thread-safe. Intended to be owned by exactly one stream processing thread.
 */
class DeviceClockMapper
{
  public:
    /// Upper bound on the number of window buckets (fixed storage, no allocation).
    static constexpr int kMaxBuckets = 64;

    /// Jump of (arrival - device) beyond which the estimator is reset (camera reboot, host
    /// clock step).
    static constexpr int64_t kResetJumpNs = 1000000000;

    /**
     * @brief Result of mapping a single frame.
     */
    struct Result
    {
        /// Stamp to publish. Equal to the arrival time until the estimator has converged.
        int64_t stamp_ns = 0;

        /// Current estimate of min(arrival - device) over the window.
        int64_t offset_ns = 0;

        /// True once a full window of samples has been observed since the last reset.
        bool converged = false;

        /// True if the computed stamp was later than the arrival time and was clamped.
        bool clamped = false;

        /// True if the estimator was reset on this frame.
        bool was_reset = false;
    };

    /**
     * @brief Configure the estimator and reset it.
     *
     * @param[in] window_ns Length of the sliding window in ns. Also the warm-up time.
     * @param[in] n_buckets Number of buckets the window is split into. Clamped to
     * [1, kMaxBuckets].
     * @param[in] latency_ns Constant latency subtracted from the stamp.
     */
    void configure(int64_t window_ns, int n_buckets, int64_t latency_ns);

    /**
     * @brief Forget all samples. The next frame starts a new warm-up window.
     */
    void reset();

    /**
     * @brief Map one frame's device timestamp onto the host clock.
     *
     * @param[in] device_ns Device timestamp of the frame in ns.
     * @param[in] arrival_ns Host (system) timestamp of the frame in ns.
     * @return Stamp and estimator state for this frame.
     */
    Result map(int64_t device_ns, int64_t arrival_ns);

  private:
    int64_t window_ns_  = 10000000000;
    int n_buckets_      = 10;
    int64_t bucket_ns_  = 1000000000;
    int64_t latency_ns_ = 0;

    std::array<int64_t, kMaxBuckets> bucket_min_{};
    std::array<int64_t, kMaxBuckets> bucket_epoch_{};

    bool has_samples_         = false;
    int64_t first_arrival_ns_ = 0;
    int64_t last_device_ns_   = 0;
    int64_t last_offset_ns_   = 0;
};

/**
 * @brief Accumulates DeviceClockMapper results between two diagnostic reports.
 */
struct DeviceClockStats
{
    int64_t n_frames               = 0;
    int64_t n_clamped              = 0;
    int64_t n_resets               = 0;
    int64_t sum_arrival_minus_stamp_ns = 0;
    int64_t min_arrival_minus_stamp_ns = 0;
    int64_t max_arrival_minus_stamp_ns = 0;
    int64_t offset_ns              = 0;
    bool converged                 = false;

    /// Add one frame's result.
    void add(const DeviceClockMapper::Result& result, int64_t arrival_ns);

    /// Clear all counters (keeps nothing).
    void clear();
};

}  // namespace camera_aravis2

#endif  // CAMERA_ARAVIS2__DEVICE_CLOCK_MAPPER_H_
