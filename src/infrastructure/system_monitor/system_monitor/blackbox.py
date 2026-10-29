# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Black box: a CSV log the car keeps writing, so the seconds before a sudden power cut survive.

Rows are written at a fixed rate and flushed to disk (fsync) about once a second, so a cut loses at
most that last second. One file per start; old files are deleted beyond keep_files. A clean stop
writes a last row with alarms = CLEAN_STOP; a file without it ended in a power cut or a crash.
"""

import csv
import glob
import os
import time

CLEAN_STOP = "clean shutdown"

FIELDS = (
    "time",
    "unix_s",
    "pack_voltage_v",
    "drive_current_a",
    "drive_power_w",
    "compute_power_w",
    "total_power_w",
    "cpu_percent",
    "ram_used_percent",
    "max_temperature_c",
    "gpu_percent",
    "alarms",
)


class BlackBox:
    def __init__(self, directory, keep_files=30, flush_period_s=1.0, now=time.time):
        os.makedirs(directory, exist_ok=True)
        self.now = now
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(now()))
        # Never overwrite: after a power cut the clock can repeat a start time we already used
        self.path = os.path.join(directory, f"blackbox_{stamp}.csv")
        n = 1
        while os.path.exists(self.path):
            self.path = os.path.join(directory, f"blackbox_{stamp}_{n}.csv")
            n += 1
        self.file = open(self.path, "x", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=FIELDS)
        self.writer.writeheader()
        self.flush_period_s = flush_period_s
        self.last_flush = now()
        self._prune(directory, keep_files)

    def _prune(self, directory, keep_files):
        # Never this run's file: after a power cut the Jetson can start with a wrong clock, so this
        # file's name (its start time) may sort before older runs
        files = sorted(
            f
            for f in glob.glob(os.path.join(directory, "blackbox_*.csv"))
            if f != self.path
        )
        for old in files[: max(0, len(files) - (keep_files - 1))]:
            os.remove(old)

    def write(self, values):
        t = self.now()
        row = {k: values.get(k, "") for k in FIELDS}
        row["time"] = (
            time.strftime("%H:%M:%S", time.localtime(t)) + f".{int(t * 10) % 10}"
        )
        row["unix_s"] = f"{t:.1f}"
        for k, v in row.items():
            if isinstance(v, float):
                row[k] = f"{v:.3f}"
        self.writer.writerow(row)
        if t - self.last_flush >= self.flush_period_s:
            self.flush()

    def flush(self):
        self.file.flush()
        os.fsync(self.file.fileno())
        self.last_flush = self.now()

    def close(self):
        self.write({"alarms": CLEAN_STOP})
        self.flush()
        self.file.close()
