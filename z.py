# esp32_ble_monitor.py
import asyncio
import argparse
import csv
import sys
import time
import struct
from typing import Optional, List, Tuple

from bleak import BleakScanner, BleakClient

# ==== MUST MATCH YOUR ESP32 SKETCH ====
DEVICE_NAME = "ESP32-DataLogger"  # from BLEDevice::init("ESP32-DataLogger")
SERVICE_UUID = "12345678-1234-1234-1234-123456789ABC".lower()
DATA_CHAR_UUID = "87654321-4321-4321-4321-CBA987654321".lower()
STATS_CHAR_UUID = "11111111-2222-3333-4444-555555555555".lower()
CONTROL_CHAR_UUID = "AAAAAAAA-BBBB-CCCC-DDDD-EEEEEEEEEEEE".lower()

# Your packed structs (with __attribute__((packed)) in the sketch):
# BLEDataPacket header:
#   uint8  packet_type
#   uint32 timestamp_ms
#   uint16 sequence
#   uint8  data_count
#   uint8  reserved
DATA_HDR_FMT = "<B I H B B"           # = 1 + 4 + 2 + 1 + 1 = 9 bytes
DATA_HDR_SIZE = struct.calcsize(DATA_HDR_FMT)

# BLECombinedData (per sample):
#   uint16 frame_idx
#   uint8  sample_idx
#   uint8  flags
#   int32  local_lc[4]
#   int32  remote_lc[4]
SAMPLE_FMT = "<H B B 8i"              # = 2 + 1 + 1 + 8*4 = 36 bytes
SAMPLE_SIZE = struct.calcsize(SAMPLE_FMT)

# BLEStatsPacket:
#   uint8  packet_type
#   uint32 timestamp
#   uint32 local_samples_total
#   uint32 remote_samples_total
#   uint32 combined_samples_total
#   uint32 local_samples_rate
#   uint32 remote_samples_rate
#   uint32 combined_samples_rate
#   uint32 local_frames_total
#   uint32 remote_frames_total
#   uint16 buffer_usage_percent
#   uint8  connection_quality
#   uint8  reserved
STATS_FMT = "<B I 8I H B B"           # = 1 + 4 + (8*4) + 2 + 1 + 1 = 41 bytes
STATS_SIZE = struct.calcsize(STATS_FMT)
# ======================================

class CsvWriter:
    def __init__(self, path: Optional[str]):
        self.path = path
        self.f = None
        self.w = None

    def open(self):
        if not self.path:
            return
        self.f = open(self.path, "w", newline="", encoding="utf-8")
        self.w = csv.writer(self.f)
        # Columns: host_time, dev_ts_ms, seq, idx_in_batch, frame_idx, sample_idx, flags, 8 channels
        cols = ["host_time", "dev_ts_ms", "sequence", "batch_idx", "frame_idx", "sample_idx", "flags"] + [f"ch{i+1}" for i in range(8)]
        self.w.writerow(cols)

    def write_sample(self, host_time: float, ts_ms: int, seq: int, batch_idx: int, frame_idx: int, sample_idx: int, flags: int, chans: List[int]):
        if self.w:
            self.w.writerow([f"{host_time:.3f}", ts_ms, seq, batch_idx, frame_idx, sample_idx, flags] + list(chans))

    def close(self):
        if self.f:
            self.f.flush()
            self.f.close()
            self.f = None
            self.w = None


class Monitor:
    def __init__(self, csv_path: Optional[str], verbose: bool):
        self.csv = CsvWriter(csv_path)
        self.verbose = verbose
        self.total_samples = 0
        self.last_print_t = time.time()
        self.last_total = 0

    def open(self):
        self.csv.open()

    def close(self):
        self.csv.close()

    def on_data_batch(self, ts_ms: int, seq: int, samples: List[Tuple[int,int,int,List[int]]]):
        """samples: list of (frame_idx, sample_idx, flags, [8 ints])"""
        self.total_samples += len(samples)
        now = time.time()
        for idx, (frame_idx, sample_idx, flags, chans) in enumerate(samples):
            self.csv.write_sample(now, ts_ms, seq, idx, frame_idx, sample_idx, flags, chans)

        if now - self.last_print_t >= 1.0:
            delta = self.total_samples - self.last_total
            sps = delta / (now - self.last_print_t)
            self.last_print_t = now
            self.last_total = self.total_samples
            if self.verbose and samples:
                f, s, flg, ch = samples[-1]
                ch_preview = " ".join(str(v) for v in ch[:8])
                print(f"[DATA] {sps:6.1f} sps | total={self.total_samples} | seq={seq} frame={f} sidx={s} flags=0x{flg:02X} | ch[0..]= {ch_preview}")
            else:
                print(f"[DATA] {sps:6.1f} sps | total={self.total_samples}")

    def on_stats(self, stats_tuple):
        # If you want to print stats every second as sent by ESP32:
        (ptype, ts, l_tot, r_tot, c_tot, l_rate, r_rate, c_rate, l_frames, r_frames, buf_pct, cq, _res) = stats_tuple
        print(f"[STAT] t={ts}ms | L:{l_rate} R:{r_rate} C:{c_rate} sps | buf={buf_pct}% frames L={l_frames} R={r_frames} cq={cq}")


def parse_data_notification(data: bytes) -> Optional[Tuple[int, int, List[Tuple[int,int,int,List[int]]]]]:
    """
    Returns (timestamp_ms, sequence, samples[list]) or None if malformed.
    """
    if len(data) < DATA_HDR_SIZE:
        return None

    (ptype, ts_ms, seq, count, _res) = struct.unpack_from(DATA_HDR_FMT, data, 0)
    if ptype != 0:
        # not a combined_data packet
        return None

    samples = []
    off = DATA_HDR_SIZE
    for i in range(count):
        if off + SAMPLE_SIZE > len(data):
            break
        frame_idx, sample_idx, flags, *vals = struct.unpack_from(SAMPLE_FMT, data, off)
        off += SAMPLE_SIZE
        samples.append((frame_idx, sample_idx, flags, vals))
    return (ts_ms, seq, samples)


def parse_stats_notification(data: bytes):
    if len(data) < STATS_SIZE:
        return None
    return struct.unpack_from(STATS_FMT, data, 0)


async def find_device(address: Optional[str], name: Optional[str]) -> Optional[str]:
    if address:
        return address
    print("Scanning for BLE devices (6s)...")
    devices = await BleakScanner.discover(timeout=6.0)
    # Prefer name match
    if name:
        for d in devices:
            if d.name and name.lower() in d.name.lower():
                print(f"Found by name: {d.name} [{d.address}]")
                return d.address
    # Fallback: try by service UUID presence (if provided by OS)
    for d in devices:
        uuids = set((d.metadata or {}).get("uuids", []) or [])
        if any(u.lower() == SERVICE_UUID for u in uuids):
            print(f"Found by service UUID: {d.name or '?'} [{d.address}]")
            return d.address

    print("Device not found. Candidates:")
    for d in devices:
        print(f"  {d.name or '?':20}  {d.address}")
    return None


async def run(address: Optional[str], name: Optional[str], csv_path: Optional[str], verbose: bool):
    # Windows event loop policy
    if sys.platform.startswith("win"):
        try:
            asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
        except Exception:
            pass

    addr = await find_device(address, name or DEVICE_NAME)
    if not addr:
        print("ERROR: Could not find target device. Try --address AA:BB:CC:DD:EE:FF")
        return

    mon = Monitor(csv_path, verbose)
    mon.open()

    async with BleakClient(addr, timeout=15.0) as client:
        if not await client.is_connected():
            print("Failed to connect.")
            return
        print(f"Connected to {addr}")

        # Optional: check that expected service exists
        svcs = await client.get_services()
        have_service = any(s.uuid.lower() == SERVICE_UUID for s in svcs)
        if not have_service:
            print("WARNING: Expected SERVICE_UUID not found in GATT. Continuing…")

        # --- DATA notifications ---
        def on_data(_handle, payload: bytes):
            parsed = parse_data_notification(payload)
            if parsed:
                ts_ms, seq, samples = parsed
                mon.on_data_batch(ts_ms, seq, samples)

        # --- STATS notifications (optional) ---
        def on_stats(_handle, payload: bytes):
            st = parse_stats_notification(payload)
            if st:
                mon.on_stats(st)

        # Start notifications if these characteristics exist
        try:
            await client.start_notify(DATA_CHAR_UUID, on_data)
            print("Subscribed to DATA characteristic.")
        except Exception as e:
            print("Could not subscribe DATA_CHAR:", e)

        try:
            await client.start_notify(STATS_CHAR_UUID, on_stats)
            print("Subscribed to STATS characteristic.")
        except Exception as e:
            print("Note: STATS subscribe failed (optional):", e)

        print("Monitoring…  (Ctrl+C to stop)")
        try:
            while True:
                await asyncio.sleep(0.25)
        except KeyboardInterrupt:
            print("Stopping…")
        finally:
            try:
                await client.stop_notify(DATA_CHAR_UUID)
            except Exception:
                pass
            try:
                await client.stop_notify(STATS_CHAR_UUID)
            except Exception:
                pass

    mon.close()


def main():
    ap = argparse.ArgumentParser(description="ESP32 BLE Combined Data Monitor")
    ap.add_argument("--address", help="BLE MAC address (Windows format AA:BB:CC:DD:EE:FF)")
    ap.add_argument("--name", default=DEVICE_NAME, help="Device name to search (default: ESP32-DataLogger)")
    ap.add_argument("--csv", help="Write CSV here (optional)")
    ap.add_argument("-v", "--verbose", action="store_true", help="Print sample values in the console")
    args = ap.parse_args()

    try:
        asyncio.run(run(args.address, args.name, args.csv, args.verbose))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
