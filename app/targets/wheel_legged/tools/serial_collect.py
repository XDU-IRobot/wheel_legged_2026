import serial
import csv
import sys
import argparse

PORT = 'COM6'
BAUDRATE = 115200
OUTPUT_FILES = {
    'gravity': 'gravity_static.csv',
    'friction': 'friction_sweep.csv',
    'dynamic': 'dynamic_harmonics.csv',
}


def parse_args():
    parser = argparse.ArgumentParser(description='采集云台辨识数据并按辨识阶段保存到不同 CSV 文件。')
    parser.add_argument(
        'mode',
        choices=OUTPUT_FILES.keys(),
        help='采集阶段: gravity=静态重力, friction=低速摩擦, dynamic=谐波动态项',
    )
    parser.add_argument('--port', default=PORT, help=f'串口号，默认 {PORT}')
    parser.add_argument('--baudrate', type=int, default=BAUDRATE, help=f'波特率，默认 {BAUDRATE}')
    parser.add_argument('--output', help='自定义输出文件名；不填则按 mode 自动选择')
    return parser.parse_args()


def main():
    args = parse_args()
    output_file = args.output or OUTPUT_FILES[args.mode]

    print(f"采集模式: {args.mode}")
    print(f"输出文件: {output_file}")
    print(f"正在打开串口 {args.port} @ {args.baudrate}...")
    try:
        ser = serial.Serial(args.port, args.baudrate, timeout=1.0)
    except serial.SerialException as e:
        print(f"无法打开串口: {e}")
        sys.exit(1)

    print(f"已连接串口。开始将数据记录到 {output_file} ...")
    print("按 Ctrl+C 停止记录并保存文件。")

    try:
        with open(output_file, mode='w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['time', 'q1', 'q2', 'dq1', 'dq2', 'tau1', 'tau2'])

            ser.reset_input_buffer()

            count = 0
            while True:
                line = ser.readline()
                if not line:
                    continue

                try:
                    line_str = line.decode('utf-8').strip()
                    data = line_str.split(',')

                    if len(data) == 7:
                        # STM32 发送顺序: timestamp_ms, tau_yaw, q_yaw, dq_yaw, tau_pitch, q_pitch, dq_pitch
                        t_ms = float(data[0])
                        time_s = t_ms / 1000.0

                        tau1 = float(data[1])
                        q1 = float(data[2])
                        dq1 = float(data[3])
                        tau2 = float(data[4])
                        q2 = float(data[5])
                        dq2 = float(data[6])

                        # CSV 表头顺序: time, q1, q2, dq1, dq2, tau1, tau2
                        writer.writerow([time_s, q1, q2, dq1, dq2, tau1, tau2])

                        count += 1
                        if count % 100 == 0:
                            print(f"已记录 {count} 条数据...")

                except ValueError:
                    print(f"解析错误，跳过该行: {line_str}")
                    continue
                except UnicodeDecodeError:
                    continue

    except KeyboardInterrupt:
        print("\n\nCtrl+C 已按下，停止记录。")
    finally:
        ser.close()
        print(f"串口已关闭，数据已安全保存至 {output_file}。")


if __name__ == '__main__':
    main()
