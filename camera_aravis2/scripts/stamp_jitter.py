#!/usr/bin/env python3
# Copyright (c) 2026 WATonomous and contributors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Fraunhofer IOSB nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""Measure per-camera header.stamp jitter from camera_info topics.

camera_info carries a copy of the image header (camera_driver.cpp fillCameraInfoMsg) and is
published on the CPU topic even when NITROS is enabled, so this measures the image stamps
without deserializing any images.

Usage (live):  python3 stamp_jitter.py --duration 120 --csv before.csv
Usage (offline, from a CSV written earlier):  python3 stamp_jitter.py --from-csv before.csv
"""

import argparse
import csv
import statistics
import sys
import time

FRAME_PERIOD_MS = 50.0  # 20 fps


def summarize(stamps_ns):
    """Return stats (ms) of consecutive stamp deltas, or None if fewer than 2 stamps."""
    stamps_ns = sorted(stamps_ns)
    deltas = [(b - a) / 1e6 for a, b in zip(stamps_ns, stamps_ns[1:])]
    if not deltas:
        return None
    # A delta of ~2x the frame period or more is a dropped frame, not jitter.
    kept = [d for d in deltas if d < 1.5 * FRAME_PERIOD_MS]
    dropped = len(deltas) - len(kept)
    if not kept:
        return None
    errs = sorted(abs(d - FRAME_PERIOD_MS) for d in kept)
    return {
        'n': len(kept),
        'mean': statistics.fmean(kept),
        'std': statistics.pstdev(kept),
        'min': min(kept),
        'max': max(kept),
        'p99_abs_err': errs[min(len(errs) - 1, int(0.99 * len(errs)))],
        'dropped': dropped,
    }


def print_table(per_topic):
    hdr = f"{'topic':45s} {'n':>6s} {'mean':>8s} {'std':>7s} {'min':>8s} {'max':>8s} " \
          f"{'p99|err|':>9s} {'drop':>5s}"
    print(hdr)
    print('-' * len(hdr))
    for topic in sorted(per_topic):
        s = summarize(per_topic[topic])
        if s is None:
            print(f'{topic:45s} (not enough samples)')
            continue
        print(f"{topic:45s} {s['n']:6d} {s['mean']:8.3f} {s['std']:7.3f} {s['min']:8.3f} "
              f"{s['max']:8.3f} {s['p99_abs_err']:9.3f} {s['dropped']:5d}")
    print('(all values in ms; deltas >= 75 ms are counted as drops and excluded)')


def read_csv(path):
    per_topic = {}
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            per_topic.setdefault(row['topic'], []).append(int(row['stamp_ns']))
    return per_topic


def record_live(duration_s, csv_path):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo

    rclpy.init()
    node = rclpy.create_node('stamp_jitter')
    time.sleep(2.0)  # let discovery fill the graph
    topics = sorted(name for name, types in node.get_topic_names_and_types()
                    if 'sensor_msgs/msg/CameraInfo' in types and name.endswith('camera_info')
                    and not name.startswith('/multi_camera_sync'))
    if not topics:
        print('No camera_info topics found.', file=sys.stderr)
        return {}
    print(f'Recording {len(topics)} topics for {duration_s} s ...')

    per_topic = {t: [] for t in topics}
    rows = []

    def make_cb(topic):
        def cb(msg):
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            per_topic[topic].append(stamp_ns)
            rows.append((topic, stamp_ns, node.get_clock().now().nanoseconds))
        return cb

    subs = [node.create_subscription(CameraInfo, t, make_cb(t), qos_profile_sensor_data)
            for t in topics]
    end = time.monotonic() + duration_s
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    del subs
    node.destroy_node()
    rclpy.shutdown()

    if csv_path:
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['topic', 'stamp_ns', 'receive_ns'])
            w.writerows(rows)
        print(f'Wrote {len(rows)} rows to {csv_path}')
    return per_topic


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--duration', type=float, default=120.0, help='seconds to record (live)')
    ap.add_argument('--csv', help='write raw stamps to this CSV (live)')
    ap.add_argument('--from-csv', help='analyse a CSV recorded earlier instead of recording')
    args = ap.parse_args()

    per_topic = read_csv(args.from_csv) if args.from_csv else record_live(args.duration, args.csv)
    if per_topic:
        print_table(per_topic)


if __name__ == '__main__':
    main()
