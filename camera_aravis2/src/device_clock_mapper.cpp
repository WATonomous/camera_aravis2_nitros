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


#include "camera_aravis2/device_clock_mapper.h"

// Std
#include <algorithm>
#include <cstdlib>
#include <limits>

namespace camera_aravis2
{

//==================================================================================================
void DeviceClockMapper::configure(int64_t window_ns, int n_buckets, int64_t latency_ns)
{
    n_buckets_  = std::clamp(n_buckets, 1, kMaxBuckets);
    window_ns_  = std::max<int64_t>(window_ns, n_buckets_);
    bucket_ns_  = window_ns_ / n_buckets_;
    latency_ns_ = latency_ns;
    reset();
}

//==================================================================================================
void DeviceClockMapper::reset()
{
    bucket_min_.fill(std::numeric_limits<int64_t>::max());
    bucket_epoch_.fill(std::numeric_limits<int64_t>::min());
    has_samples_ = false;
}

//==================================================================================================
DeviceClockMapper::Result DeviceClockMapper::map(int64_t device_ns, int64_t arrival_ns)
{
    Result result;
    const int64_t DIFF_NS = arrival_ns - device_ns;

    //--- reset if the device counter restarted or the clocks jumped against each other
    if (has_samples_ &&
        (device_ns <= last_device_ns_ || std::llabs(DIFF_NS - last_offset_ns_) > kResetJumpNs))
    {
        reset();
        result.was_reset = true;
    }

    if (!has_samples_)
    {
        has_samples_      = true;
        first_arrival_ns_ = arrival_ns;
    }
    last_device_ns_ = device_ns;

    //--- insert the sample BEFORE taking the minimum, so that offset <= DIFF_NS always holds
    const int64_t EPOCH = arrival_ns / bucket_ns_;
    const int SLOT      = static_cast<int>(EPOCH % n_buckets_);
    if (bucket_epoch_[SLOT] != EPOCH)
    {
        bucket_epoch_[SLOT] = EPOCH;
        bucket_min_[SLOT]   = DIFF_NS;
    }
    else
    {
        bucket_min_[SLOT] = std::min(bucket_min_[SLOT], DIFF_NS);
    }

    //--- minimum over all buckets that are still inside the window
    int64_t offset_ns = std::numeric_limits<int64_t>::max();
    for (int i = 0; i < n_buckets_; ++i)
    {
        if (EPOCH - bucket_epoch_[i] < n_buckets_)
            offset_ns = std::min(offset_ns, bucket_min_[i]);
    }
    last_offset_ns_ = offset_ns;

    result.offset_ns = offset_ns;
    result.converged = (arrival_ns - first_arrival_ns_) >= window_ns_;

    if (!result.converged)
    {
        result.stamp_ns = arrival_ns;
        return result;
    }

    result.stamp_ns = device_ns + offset_ns - latency_ns_;
    if (result.stamp_ns > arrival_ns)
    {
        result.stamp_ns = arrival_ns;
        result.clamped  = true;
    }
    return result;
}

//==================================================================================================
void DeviceClockStats::add(const DeviceClockMapper::Result& result, int64_t arrival_ns)
{
    const int64_t LAG_NS = arrival_ns - result.stamp_ns;
    if (n_frames == 0)
    {
        min_arrival_minus_stamp_ns = LAG_NS;
        max_arrival_minus_stamp_ns = LAG_NS;
    }
    else
    {
        min_arrival_minus_stamp_ns = std::min(min_arrival_minus_stamp_ns, LAG_NS);
        max_arrival_minus_stamp_ns = std::max(max_arrival_minus_stamp_ns, LAG_NS);
    }
    sum_arrival_minus_stamp_ns += LAG_NS;
    n_frames++;
    n_clamped += result.clamped ? 1 : 0;
    n_resets += result.was_reset ? 1 : 0;
    offset_ns = result.offset_ns;
    converged = result.converged;
}

//==================================================================================================
void DeviceClockStats::clear()
{
    *this = DeviceClockStats();
}

}  // namespace camera_aravis2
