import re
import time
import threading
import csv
import sys
import os
from datetime import datetime

import numpy as np
import serial
from serial.tools import list_ports


MAX_DEVICES = 4
BAUDRATE = 115200
SYNC_RE = re.compile(r"^SYNC:([^:]+):(\d+)\s*$")
BASELINE_SECONDS = 2.0

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CAL_DIR = "C:/Users/jwi/Nico/wiiboard/MultiWiiboardhandler/calibrationWeight"
SAVE_DIR = "C:/Users/jwi/Nico/wiiboard/MultiWiiboardhandler/Readings"


def load_calibration_slope(cal_dir, cal_filename=None):
    if not os.path.isdir(cal_dir):
        print(f"Calibration directory not found: {cal_dir}")
        sys.exit(1)

    target_path = None

    if cal_filename:
        if os.path.isabs(cal_filename) or os.path.sep in cal_filename:
            candidate = cal_filename
        else:
            candidate = os.path.join(cal_dir, cal_filename)

        if not os.path.isfile(candidate):
            print(f"Calibration file not found: {candidate}")
            sys.exit(1)

        target_path = candidate
    else:
        for fname in os.listdir(cal_dir):
            if fname.endswith("_AVG_calibration.csv"):
                target_path = os.path.join(cal_dir, fname)
                break

        if target_path is None:
            for fname in os.listdir(cal_dir):
                if fname.lower().endswith(".csv"):
                    target_path = os.path.join(cal_dir, fname)
                    break

        if target_path is None:
            print(f"No calibration CSV found in {cal_dir}")
            sys.exit(1)

    forces, avg_raws = [], []
    with open(target_path, "r", newline="", encoding="utf-8-sig") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        force_idx, raw_idx = 0, 1

        if header:
            hl = [h.strip().lower() for h in header]
            for i, h in enumerate(hl):
                if "force" in h:
                    force_idx = i
                if "avg" in h or "mean" in h:
                    raw_idx = i

        for row in reader:
            if not row or len(row) < 2:
                continue
            try:
                forces.append(float(row[force_idx]))
                avg_raws.append(float(row[raw_idx]))
            except Exception:
                pass

    if len(forces) < 2:
        print(f"Not enough calibration points in {target_path}")
        sys.exit(1)

    forces = np.asarray(forces, dtype=float)
    avg_raws = np.asarray(avg_raws, dtype=float)
    m, b = np.polyfit(avg_raws, forces, 1)

    print(f"Calibration fit from {os.path.basename(target_path)}:")
    print(f"    Force_N = {m:.6f}·AvgRaw + {b:.6f}")
    return m


def list_serial_devices():
    ports = []
    for p in list_ports.comports():
        ports.append(p.device)

    uniq = []
    seen = set()
    for p in ports:
        if p not in seen:
            uniq.append(p)
            seen.add(p)
    return uniq


def try_open_serial(port):
    try:
        ser = serial.Serial(
            port=port,
            baudrate=BAUDRATE,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1,
        )
        time.sleep(1.5)
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception:
            pass
        return ser
    except Exception as e:
        print(f"[{port}] open failed: {e}")
        return None


def find_calibration_file(cal_dir, device_name):
    print("directory di ricerca " + cal_dir)
    print("nome device di ricerca " + device_name)
    if not os.path.isdir(cal_dir):
        return None

    for fname in os.listdir(cal_dir):
        if device_name.lower() in fname.lower() and fname.lower().endswith(".csv"):
            return os.path.join(cal_dir, fname)
    return None


def get_board_time_offset(ser, timeout=2.0):
    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    ser.write(b"s")
    ser.flush()

    start = time.perf_counter()
    while time.perf_counter() - start < timeout:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            continue

        m = SYNC_RE.match(line)
        if not m:
            continue

        t_host_recv = time.perf_counter()
        board_name = m.group(1)

        calibration_file_path = find_calibration_file(CAL_DIR, board_name)
        if calibration_file_path is None:
            raise FileNotFoundError(
                f"No calibration file found for {board_name} in {CAL_DIR}"
            )

        board_us = int(m.group(2))
        board_s = board_us / 1_000_000.0
        offset_s = t_host_recv - board_s
        m_slope = load_calibration_slope(CAL_DIR, calibration_file_path)

        print(
            f"Offset for board {board_name} ({ser.port}): "
            f"host ≈ board + {offset_s:.6f} s"
        )

        return {
            "board_name": board_name,
            "baseline_raw": None,
            "calibration_file": calibration_file_path,
            "off_set": offset_s,
            "port": ser.port,
            "m_slope": m_slope,
        }

    raise TimeoutError(f"No SYNC response from board on {ser.port} within timeout.")


class DeviceController:
    def __init__(self, port, ser, board_info):
        self.port = port
        self.ser = ser
        self.port_sanitized = (
            port.replace("/", "_").replace("\\", "_").replace(":", "_")
        )

        self.board_name = board_info["board_name"]
        self.M = board_info["m_slope"]
        self.offset_s = board_info["off_set"]
        self.calibration_file = board_info["calibration_file"]

        self.baseline_done = False
        self.baseline_start = None
        self.baseline_samples = []
        self.baseline_raw = 0.0

        self.lock = threading.Lock()
        self.buffer = []
        self.record_buffer = []
        self.index = 0

        self.is_recording = False
        self.recording_started_at = None
        self.record_start_host_time = None
        self.record_synced_index = 0

        self.stop_event = threading.Event()
        self.reader_thread = None

    def start_reader(self):
        self.reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.reader_thread.start()

    def _read_loop(self):
        pattern = re.compile(
            r"Time:(-?\d+),V1:(-?\d+(?:\.\d+)?),"
            r"V2:(-?\d+(?:\.\d+)?),V3:(-?\d+(?:\.\d+)?),V4:(-?\d+(?:\.\d+)?)"
        )

        while not self.stop_event.is_set():
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                match = pattern.match(line)
                if not match:
                    continue

                t_us = int(match.group(1))
                t_host = (t_us / 1_000_000.0) + self.offset_s

                v1, v2, v3, v4 = [float(match.group(i)) for i in range(2, 6)]
                avg_raw = (v1 + v2 + v3 + v4) / 4.0

                if not self.baseline_done:
                    now = time.perf_counter()
                    if self.baseline_start is None:
                        self.baseline_start = now

                    self.baseline_samples.append(avg_raw)

                    elapsed = now - self.baseline_start
                    if elapsed >= BASELINE_SECONDS and self.baseline_samples:
                        self.baseline_raw = float(np.mean(self.baseline_samples))
                        self.baseline_done = True
                        print(
                            f"[{self.board_name}] Baseline established over "
                            f"{elapsed:.2f}s: baseline_raw = {self.baseline_raw:.3f}"
                        )

                    if not self.baseline_done:
                        continue

                raw_corr = avg_raw - self.baseline_raw
                f_total = self.M * raw_corr

                raw_sum = v1 + v2 + v3 + v4
                if raw_sum != 0:
                    weights = [v1 / raw_sum, v2 / raw_sum, v3 / raw_sum, v4 / raw_sum]
                else:
                    weights = [0.25, 0.25, 0.25, 0.25]

                f1 = float(np.round(f_total * weights[0], 3))
                f2 = float(np.round(f_total * weights[1], 3))
                f3 = float(np.round(f_total * weights[2], 3))
                f4 = float(np.round(f_total * weights[3], 3))
                f_total_rounded = float(np.round(f_total, 3))

                live_row = (
                    t_host,
                    self.index,
                    v1,
                    v2,
                    v3,
                    v4,
                    f1,
                    f2,
                    f3,
                    f4,
                    f_total_rounded,
                )

                with self.lock:
                    self.buffer.append(live_row)

                    if self.is_recording and self.record_start_host_time is not None:
                        if t_host >= self.record_start_host_time:
                            recorded_row = (
                                t_host,
                                self.index,
                                self.record_synced_index,
                                v1,
                                v2,
                                v3,
                                v4,
                                f1,
                                f2,
                                f3,
                                f4,
                                f_total_rounded,
                            )
                            self.record_buffer.append(recorded_row)
                            self.record_synced_index += 1

                    self.index += 1

            except Exception as e:
                print(f"[{self.board_name}] Read error: {e}")

    def start_recording(self, start_host_time, session_name):
        with self.lock:
            self.record_buffer = []
            self.is_recording = True
            self.recording_started_at = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.record_start_host_time = start_host_time
            self.record_synced_index = 0
            self.session_name = session_name

        print(
            f"[{self.board_name}] Recording armed at synced host time "
            f"{start_host_time:.6f}"
        )

    def stop_recording_and_save(self):
        with self.lock:
            rows = list(self.record_buffer)
            self.is_recording = False
            ts = self.recording_started_at or datetime.now().strftime("%Y%m%d_%H%M%S")
            self.recording_started_at = None
            self.record_start_host_time = None
            self.record_synced_index = 0

        os.makedirs(SAVE_DIR, exist_ok=True)
        try:
            print("Session name found")
            filename = os.path.join(
                SAVE_DIR, f"{self.session_name}_session_{ts}_{self.board_name}_{self.port_sanitized}.csv"
            )
        except:
                print("Session name doesn't exist")
                filename = os.path.join(
                SAVE_DIR, f"session_{ts}_{self.board_name}_{self.port_sanitized}.csv"
            )

        with open(filename, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(
                [
                    "host_time_s",
                    "sample_index",
                    "synced_index",
                    "v1_raw",
                    "v2_raw",
                    "v3_raw",
                    "v4_raw",
                    "v1_force_N",
                    "v2_force_N",
                    "v3_force_N",
                    "v4_force_N",
                    "total_force_N",
                ]
            )
            for row in rows:
                w.writerow(list(row))

        print(f"[{self.board_name}] Saved {len(rows)} recorded rows to {filename}")

    def get_plot_data(self, history):
        with self.lock:
            if len(self.buffer) < 2:
                return [], []
            recent = self.buffer[-history:]
            x = [r[0] for r in recent]
            y = [r[10] for r in recent]
        return x, y

    def get_total_series(self):
        with self.lock:
            return [r[10] for r in self.buffer]

    def close(self):
        self.stop_event.set()
        try:
            if self.reader_thread and self.reader_thread.is_alive():
                self.reader_thread.join(timeout=2)
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass


class WiiBoardController:
    def __init__(self):
        self.available_boards = {}
        self.devices = []
        self.is_recording = False
        self.recording_start_host_time = None
        self.session_name = ""

    def discover_and_connect(self):
        ports = list_serial_devices()
        if not ports:
            raise RuntimeError("No COM ports found.")

        print("Detected serial ports:")
        for p in ports:
            print(f"  - {p}")

        valid_pairs = []

        for p in ports:
            if len(valid_pairs) >= MAX_DEVICES:
                break

            ser = try_open_serial(p)
            if not ser:
                continue

            try:
                board_info = get_board_time_offset(ser, timeout=2.0)
                self.available_boards[p] = board_info
                valid_pairs.append((p, ser, board_info))
                print(f"[{p}] synced successfully as board '{board_info['board_name']}'")
            except Exception as e:
                print(f"[{p}] sync failed: {e}")
                try:
                    ser.close()
                except Exception:
                    pass

        if not valid_pairs:
            raise RuntimeError("No boards successfully synced.")

        for port, ser, board_info in valid_pairs:
            dev = DeviceController(port, ser, board_info)
            dev.start_reader()
            self.devices.append(dev)
            print(f"[{port}] Reader started.")

    def start_recording(self):
        if self.is_recording:
            print("Already recording.")
            return

        shared_start_host_time = time.perf_counter()
        self.recording_start_host_time = shared_start_host_time
        self.is_recording = True

        for dev in self.devices:
            dev.start_recording(shared_start_host_time, self.session_name)

        print(f"Recording started with session: {self.session_name}")

    def stop_recording(self):
        if not self.is_recording:
            print("Not currently recording.")
            return

        for dev in self.devices:
            dev.stop_recording_and_save()

        self.is_recording = False
        self.recording_start_host_time = None
        print("Recording stopped.")

    def close_all(self):
        for dev in self.devices:
            dev.close()


def main():
    print("Windows serial recorder starting...")
    print(f"Base directory: {BASE_DIR}")
    print(f"Calibration directory: {CAL_DIR}")
    print(f"Save directory: {SAVE_DIR}")
    print()

    controller = WiiBoardController()

    try:
        controller.discover_and_connect()
        print()
        print("Commands:")
        print("  r = start recording")
        print("  s = stop recording and save")
        print("  q = quit")
        print()

        while True:
            cmd = input("Enter command [r/s/q]: ").strip().lower()

            if cmd == "r":
                controller.start_recording()
            elif cmd == "s":
                controller.stop_recording()
            elif cmd == "q":
                if controller.is_recording:
                    controller.stop_recording()
                break
            else:
                print("Unknown command.")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        if controller.is_recording:
            controller.stop_recording()
    except Exception as e:
        print(f"Fatal error: {e}")
    finally:
        controller.close_all()
        print("All ports closed. Exiting.")


if __name__ == "__main__":
    main()