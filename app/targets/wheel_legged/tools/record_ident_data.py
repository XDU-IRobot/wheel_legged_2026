#!/usr/bin/env python3
"""
Serial data recorder for gimbal identification.

Records CSV data from the STM32 identification mode via serial port.
CSV format matches gimbal_ident.hpp output:
  time_ms, yaw_tau, yaw_pos, yaw_vel, pitch_tau, pitch_pos, pitch_vel, pitch_static

Usage:
  python record_ident_data.py
  python record_ident_data.py --port COM7 --baud 115200 --output ident_data.csv
"""

import argparse
import csv
import sys

try:
    import serial
except ImportError:
    print("pyserial not installed. Run: pip install pyserial")
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM7", help="serial port (default: COM7)")
    parser.add_argument("--baud", type=int, default=115200, help="baud rate (default: 115200)")
    parser.add_argument("--output", default="ident_data.csv", help="output CSV file (default: ident_data.csv)")
    args = parser.parse_args()

    print(f"Opening {args.port} @ {args.baud}...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"Cannot open serial port: {e}")
        sys.exit(1)

    print(f"Connected. Recording to {args.output} ...")
    print("Press Ctrl+C to stop.\n")

    try:
        with open(args.output, mode="w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["time_ms", "yaw_tau", "yaw_pos", "yaw_vel",
                             "pitch_tau", "pitch_pos", "pitch_vel", "pitch_static"])

            ser.reset_input_buffer()

            count = 0
            static_count = 0
            while True:
                line = ser.readline()
                if not line:
                    continue

                try:
                    line_str = line.decode("utf-8").strip()
                    data = line_str.split(",")

                    if len(data) == 8:
                        writer.writerow([
                            float(data[0]),  # time_ms
                            float(data[1]),  # yaw_tau
                            float(data[2]),  # yaw_pos
                            float(data[3]),  # yaw_vel
                            float(data[4]),  # pitch_tau
                            float(data[5]),  # pitch_pos
                            float(data[6]),  # pitch_vel
                            int(data[7]),    # pitch_static
                        ])

                        count += 1
                        if data[7] == "1":
                            static_count += 1

                        if count % 100 == 0:
                            print(f"  recorded {count} rows ({static_count} static)")

                except (ValueError, UnicodeDecodeError):
                    continue

    except KeyboardInterrupt:
        print(f"\n\nStopped. Total {count} rows ({static_count} static points).")
    finally:
        ser.close()
        print(f"Saved to {args.output}")


if __name__ == "__main__":
    main()
