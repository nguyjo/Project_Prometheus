#!/usr/bin/env python3
"""
prometheus_log_decode.py — Decoder for Prometheus flight log files.

USAGE:
  # Method 1: Capture log dump from serial monitor to a file, then decode
  python3 prometheus_log_decode.py captured_output.txt

  # Method 2: Decode directly from raw binary if you have it as a file
  python3 prometheus_log_decode.py --binary flight_001.log

OUTPUT:
  Writes a CSV with one row per record. Open in Excel, plot in Python,
  whatever you like.

WORKFLOW for getting a log off the rocket:
  1. Connect rocket via USB
  2. In serial monitor: send "LOG_LIST" to see what's there
  3. In serial monitor: send "LOG_DUMP flight_000.log"
  4. Save the monitor output to a file
  5. Run this decoder on that file
"""

import sys
import struct
import csv
import argparse

# Must match LogRecord struct in FlashLogger.h exactly.
# 64 bytes packed.
RECORD_STRUCT = struct.Struct('<'    # little-endian
                              'I'    # timestamp_us (uint32)
                              'B'    # flight_state (uint8)
                              'B'    # pyro_states (uint8)
                              'B'    # pyro_continuity (uint8)
                              'B'    # flags (uint8)
                              'f'    # altitude_m (float32)
                              'f'    # velocity_mps
                              'f'    # accel_axial_mps2
                              'f'    # tilt_deg
                              'f'    # gyro_x_dps
                              'f'    # gyro_y_dps
                              'f'    # gyro_z_dps
                              'f'    # baro_altitude_m
                              'f'    # imu_accel_g
                              'f'    # highg_accel_g
                              'f'    # baro_pressure_mbar
                              'f'    # imu_lat1_g
                              'f'    # imu_lat2_g
                              'bbbb' # canards (4x int8)
                              )

assert RECORD_STRUCT.size == 64, f"Struct size {RECORD_STRUCT.size}, expected 64"

# Session header has the same size but different layout.
HEADER_STRUCT = struct.Struct('<'
                              'I'      # magic (uint32) — 0x474F4C46 "FLOG"
                              'I'      # version (uint32)
                              'I'      # boot_time_ms (uint32)
                              '8s'     # stage[8]
                              '12s'    # build_date[12]
                              '9s'     # build_time[9]
                              '19s')   # reserved[19]

assert HEADER_STRUCT.size == 64, f"Header size {HEADER_STRUCT.size}, expected 64"

LOG_MAGIC = 0x474F4C46  # "FLOG"

FLIGHT_STATES = {
    # Booster states
    0: 'PRE_LIFTOFF',
    1: 'BOOST',
    2: 'COAST',
    3: 'DESCENT_DROGUE',
    4: 'MAIN_FIRED',
    5: 'DESCENT_MAIN',
    6: 'TOUCHDOWN',
    # NOTE: sustainer uses different state values. Update this map per stage.
}


def decode_record(data):
    """Decode a 64-byte LogRecord. Returns a dict."""
    fields = RECORD_STRUCT.unpack(data)
    rec = {
        'timestamp_us':       fields[0],
        'flight_state':       fields[1],
        'flight_state_name':  FLIGHT_STATES.get(fields[1], f'STATE_{fields[1]}'),
        'pyro0_state':        (fields[2] >> 0) & 0x3,
        'pyro1_state':        (fields[2] >> 2) & 0x3,
        'pyro2_state':        (fields[2] >> 4) & 0x3,
        'pyro3_state':        (fields[2] >> 6) & 0x3,
        'pyro0_continuity':   bool(fields[3] & 0x01),
        'pyro1_continuity':   bool(fields[3] & 0x02),
        'pyro2_continuity':   bool(fields[3] & 0x04),
        'pyro3_continuity':   bool(fields[3] & 0x08),
        'rotation_fault':     bool(fields[4] & 0x01),
        'using_highg':        bool(fields[4] & 0x02),
        'flight_armed':       bool(fields[4] & 0x04),
        'baro_calibrated':    bool(fields[4] & 0x08),
        'altitude_m':         fields[5],
        'velocity_mps':       fields[6],
        'accel_axial_mps2':   fields[7],
        'tilt_deg':           fields[8],
        'gyro_x_dps':         fields[9],
        'gyro_y_dps':         fields[10],
        'gyro_z_dps':         fields[11],
        'baro_altitude_m':    fields[12],
        'imu_accel_g':        fields[13],
        'highg_accel_g':      fields[14],
        'baro_pressure_mbar': fields[15],
        'imu_lat1_g':         fields[16],
        'imu_lat2_g':         fields[17],
        'canard1_deg':        fields[18],
        'canard2_deg':        fields[19],
        'canard3_deg':        fields[20],
        'canard4_deg':        fields[21],
    }
    return rec


def decode_header(data):
    """Decode a 64-byte SessionHeader. Returns a dict."""
    fields = HEADER_STRUCT.unpack(data)
    return {
        'magic':         fields[0],
        'version':       fields[1],
        'boot_time_ms':  fields[2],
        'stage':         fields[3].split(b'\0', 1)[0].decode('ascii', errors='replace'),
        'build_date':    fields[4].split(b'\0', 1)[0].decode('ascii', errors='replace'),
        'build_time':    fields[5].split(b'\0', 1)[0].decode('ascii', errors='replace'),
    }


def parse_serial_dump(filename):
    """Parse output captured from the LOG_DUMP serial command.
    
    Expects lines like 'REC <128 hex chars>'. Returns a list of 64-byte
    binary chunks (one per record/header).
    """
    chunks = []
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if not line.startswith('REC '):
                continue
            hex_str = line[4:]
            if len(hex_str) != 128:
                print(f"WARN: skipping malformed line ({len(hex_str)} hex chars): {line[:80]}")
                continue
            try:
                chunks.append(bytes.fromhex(hex_str))
            except ValueError as e:
                print(f"WARN: hex decode failed: {e}")
                continue
    return chunks


def parse_binary_file(filename):
    """Parse a raw binary log file directly."""
    chunks = []
    with open(filename, 'rb') as f:
        while True:
            chunk = f.read(64)
            if len(chunk) != 64:
                break
            chunks.append(chunk)
    return chunks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', help='Input file (serial dump or binary)')
    parser.add_argument('--binary', action='store_true',
                        help='Input is a raw binary file (not a serial dump)')
    parser.add_argument('--output', '-o', help='Output CSV (default: <input>.csv)')
    args = parser.parse_args()

    if args.binary:
        chunks = parse_binary_file(args.input)
    else:
        chunks = parse_serial_dump(args.input)

    if not chunks:
        print("ERROR: no records found in input file")
        return 1

    # First chunk should be the session header (magic must match)
    first = chunks[0]
    magic, = struct.unpack('<I', first[:4])
    if magic == LOG_MAGIC:
        hdr = decode_header(first)
        print(f"Session header:")
        print(f"  Stage:     {hdr['stage']}")
        print(f"  Build:     {hdr['build_date']} {hdr['build_time']}")
        print(f"  Boot time: {hdr['boot_time_ms']} ms")
        print(f"  Version:   {hdr['version']}")
        record_chunks = chunks[1:]
    else:
        print("WARN: no session header found, treating all chunks as records")
        record_chunks = chunks

    print(f"Decoding {len(record_chunks)} records...")

    # Decode all records
    records = [decode_record(c) for c in record_chunks]

    # Write CSV
    output = args.output or args.input.rsplit('.', 1)[0] + '.csv'
    with open(output, 'w', newline='') as f:
        if records:
            writer = csv.DictWriter(f, fieldnames=list(records[0].keys()))
            writer.writeheader()
            writer.writerows(records)

    print(f"Wrote {len(records)} records to {output}")

    # Print a quick summary
    if records:
        first_t = records[0]['timestamp_us']
        last_t = records[-1]['timestamp_us']
        duration_s = (last_t - first_t) / 1_000_000
        max_alt = max(r['altitude_m'] for r in records)
        max_vel = max(r['velocity_mps'] for r in records)
        max_accel = max(abs(r['accel_axial_mps2']) for r in records)
        max_tilt = max(r['tilt_deg'] for r in records)
        print(f"\nFlight summary:")
        print(f"  Duration:     {duration_s:.1f} sec")
        print(f"  Max altitude: {max_alt:.1f} m  ({max_alt*3.281:.0f} ft)")
        print(f"  Max velocity: {max_vel:.1f} m/s")
        print(f"  Max accel:    {max_accel:.1f} m/s² ({max_accel/9.81:.1f} g)")
        print(f"  Max tilt:     {max_tilt:.1f} deg")
        # State transitions
        last_state = None
        print(f"\nState transitions:")
        for r in records:
            if r['flight_state'] != last_state:
                t_s = (r['timestamp_us'] - first_t) / 1_000_000
                print(f"  T+{t_s:6.2f}s: {r['flight_state_name']}")
                last_state = r['flight_state']

    return 0


if __name__ == '__main__':
    sys.exit(main())