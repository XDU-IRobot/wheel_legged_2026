import argparse
import csv
import serial
import sys


PORT = 'COM6'
BAUDRATE = 115200
OUTPUT_FILES = {
    'gravity': 'gravity_static.csv',
    'friction': 'friction_sweep.csv',
    'dynamic': 'dynamic_harmonics.csv',
}

CSV_COLUMNS = [
    'time', 'q1', 'q2', 'dq1', 'dq2', 'tau1', 'tau2', 'dt',
    'q1_ref', 'q2_ref', 'dq1_ref', 'dq2_ref', 'ddq1_ref', 'ddq2_ref',
    'frequency_hz', 'frequency_index', 'cycle_index', 'valid_for_fit',
]


def parse_args():
    parser = argparse.ArgumentParser(description='采集云台辨识串口数据并保存为 CSV。')
    parser.add_argument('mode', choices=OUTPUT_FILES.keys(),
                        help='gravity=静态重力，friction=摩擦扫描，dynamic=动态惯量/谐波')
    parser.add_argument('--port', default=PORT, help=f'串口号，默认 {PORT}')
    parser.add_argument('--baudrate', type=int, default=BAUDRATE, help=f'波特率，默认 {BAUDRATE}')
    parser.add_argument('--output', help='自定义输出文件名')
    return parser.parse_args()


def decode_line(data):
    """Decode both the legacy 7-field frame and the extended 18-field frame."""
    if len(data) not in (7, 18):
        return None

    values = [float(item) for item in data]
    time_s = values[0] / 1000.0
    row = {
        'time': time_s,
        'tau1': values[1],
        'q1': values[2],
        'dq1': values[3],
        'tau2': values[4],
        'q2': values[5],
        'dq2': values[6],
    }

    if len(values) == 18:
        row.update({
            'dt': values[7],
            'q1_ref': values[8],
            'q2_ref': values[9],
            'dq1_ref': values[10],
            'dq2_ref': values[11],
            'ddq1_ref': values[12],
            'ddq2_ref': values[13],
            'frequency_hz': values[14],
            'frequency_index': int(values[15]),
            'cycle_index': int(values[16]),
            'valid_for_fit': int(values[17]),
        })
    return row


def main():
    args = parse_args()
    output_file = args.output or OUTPUT_FILES[args.mode]

    print(f'采集模式: {args.mode}')
    print(f'输出文件: {output_file}')
    print(f'正在打开串口 {args.port} @ {args.baudrate}...')
    try:
        ser = serial.Serial(args.port, args.baudrate, timeout=1.0)
    except serial.SerialException as exc:
        print(f'无法打开串口: {exc}')
        sys.exit(1)

    print('串口已连接，按 Ctrl+C 停止并保存。')
    try:
        with open(output_file, mode='w', newline='', encoding='utf-8') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=CSV_COLUMNS)
            writer.writeheader()
            ser.reset_input_buffer()
            count = 0

            while True:
                raw = ser.readline()
                if not raw:
                    continue
                try:
                    line = raw.decode('utf-8').strip()
                    row = decode_line(line.split(','))
                    if row is None:
                        continue
                    writer.writerow(row)
                    count += 1
                    if count % 100 == 0:
                        print(f'已记录 {count} 条数据...')
                except (ValueError, UnicodeDecodeError):
                    continue
    except KeyboardInterrupt:
        print('\n停止记录。')
    finally:
        ser.close()
        print(f'数据已保存至 {output_file}。')


if __name__ == '__main__':
    main()
