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
"""Check device timestamps from the driver's DEBUG log lines.

Reads lines like
  [camera_pano_nn]: (camera_pano_nn) device_ns=123000000000 arrival_ns=1760000000010000000
from a file or stdin (run the driver with --log-level <node>:=debug and tee its output).

Prints, per camera:
  * consecutive device-timestamp deltas (expect 50 ms +- <0.1 ms at 20 fps) -> units check
  * min / median of (arrival - device) -> compare between fixed-exposure runs to find out
    whether the device timestamp is latched at the start or the end of exposure
"""

import argparse
import re
import statistics
import sys

LINE_RE = re.compile(r'\((?P<cam>[^)]+)\) device_ns=(?P<dev>-?\d+) arrival_ns=(?P<arr>-?\d+)')


def parse(lines):
    per_cam = {}
    for line in lines:
        m = LINE_RE.search(line)
        if m:
            per_cam.setdefault(m['cam'], []).append((int(m['dev']), int(m['arr'])))
    return per_cam


def analyse(samples):
    devs = [d for d, _ in samples]
    deltas_ms = [(b - a) / 1e6 for a, b in zip(devs, devs[1:]) if b > a]
    deltas_ms = [d for d in deltas_ms if d < 75.0]  # drop frame gaps
    diffs_ms = [(a - d) / 1e6 for d, a in samples]
    return {
        'frames': len(samples),
        'dev_delta_mean': statistics.fmean(deltas_ms) if deltas_ms else float('nan'),
        'dev_delta_std': statistics.pstdev(deltas_ms) if deltas_ms else float('nan'),
        'dev_delta_maxerr': max((abs(d - 50.0) for d in deltas_ms), default=float('nan')),
        'diff_min': min(diffs_ms),
        'diff_median': statistics.median(diffs_ms),
        'arrival_mid_s': statistics.median(a for _, a in samples) / 1e9,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('log', nargs='?', help='log file (default: stdin)')
    args = ap.parse_args()
    lines = open(args.log) if args.log else sys.stdin
    per_cam = parse(lines)
    if not per_cam:
        print('No "device_ns=... arrival_ns=..." lines found.', file=sys.stderr)
        sys.exit(1)
    for cam, samples in sorted(per_cam.items()):
        r = analyse(samples)
        print(f"{cam}: frames={r['frames']} device delta mean={r['dev_delta_mean']:.4f} ms "
              f"std={r['dev_delta_std']:.4f} ms max|err|={r['dev_delta_maxerr']:.4f} ms | "
              f"arrival-device min={r['diff_min']:.3f} ms median={r['diff_median']:.3f} ms "
              f"run mid-time={r['arrival_mid_s']:.1f} s")


if __name__ == '__main__':
    main()
