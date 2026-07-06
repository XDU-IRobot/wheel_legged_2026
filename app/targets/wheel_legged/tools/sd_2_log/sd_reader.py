"""
sd_reader.py — 从 SD 卡导出原始扇区为 .bin 文件

用法（管理员终端！）:
  # 列出所有磁盘，找到 SD 卡
  python sd_reader.py --list

  # 导出日志扇区（-d 必须用 PhysicalDrive，不能用盘符）
  python sd_reader.py -d PhysicalDrive2 -s 2048 -c 500 --dump-bin

  # 参数说明
  #   -d PhysicalDrive2   SD 卡物理磁盘（靠容量和插拔辨认）
  #   -s 2048              起始 LBA（= STM32 端 base_lba）
  #   -c 500               扇区数（256KB，覆盖 60s 日志绰绰有余）
  #   --dump-bin           生成 .bin 文件

输出自动创建在 sd_out/<时间戳>/ 子文件夹内。
"""
import argparse
import ctypes
from ctypes import wintypes
import os
import struct
import sys
import time
from pathlib import Path

# ====================== 全局配置 ======================
SCRIPT_DIR = Path(__file__).resolve().parent
LOG_OUT_DIR = SCRIPT_DIR / "sd_out"
SECTOR_SIZE = 512
MAX_SECTORS = 8192  # 单次读取上限 4MB，防止内存耗尽
# 禁止读取的系统硬盘，运行时动态检测替代硬编码黑名单
BLACKLIST_PHYSICAL = set()  # 由 detect_system_physical_drives() 动态填充
# Windows 文件打开常量
GENERIC_READ = 0x80000000
OPEN_EXISTING = 3
FILE_SHARE_READ = 0x00000001
IOCTL_DISK_GET_LENGTH_INFO = 0x0007405C
IOCTL_DISK_GET_PARTITION_INFO_EX = 0x00070080
IOCTL_VOLUME_GET_VOLUME_DISK_EXTENTS = 0x00560000
# 写入保护常量：任何写操作必须显式设置此标志
_FILE_WRITE_MODE = "wb"  # 未来扩展写功能时用作哨兵，当前禁止

# Windows终端强制UTF8输出
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

# ====================== Windows底层工具函数 ======================
def is_admin() -> bool:
    """检测当前是否管理员权限"""
    try:
        return ctypes.windll.shell32.IsUserAnAdmin()
    except Exception:
        return False

def win32_open_device(dev_path: str):
    """安全打开Windows磁盘设备，带共享只读参数"""
    handle = ctypes.windll.kernel32.CreateFileW(
        dev_path,
        GENERIC_READ,
        FILE_SHARE_READ,
        None,
        OPEN_EXISTING,
        0,
        None
    )
    if handle == ctypes.c_void_p(-1).value:
        return None
    return handle

def get_disk_total_bytes(dev_path: str) -> int:
    """获取磁盘总字节大小，失败时抛出 OSError"""
    handle = win32_open_device(dev_path)
    if handle is None:
        err = ctypes.get_last_error()
        raise OSError(f"无法打开设备 {dev_path}，错误码: {err}（可能需要管理员权限）")
    try:
        class GET_LENGTH_INFORMATION(ctypes.Structure):
            _fields_ = [("Length", ctypes.c_longlong)]
        out = GET_LENGTH_INFORMATION()
        ret_bytes = wintypes.DWORD()
        res = ctypes.windll.kernel32.DeviceIoControl(
            handle, IOCTL_DISK_GET_LENGTH_INFO,
            None, 0, ctypes.byref(out), ctypes.sizeof(out),
            ctypes.byref(ret_bytes), None
        )
        if not res:
            err = ctypes.get_last_error()
            raise OSError(f"获取磁盘大小失败 {dev_path}，DeviceIoControl错误码: {err}")
        return out.Length
    finally:
        ctypes.windll.kernel32.CloseHandle(handle)

def get_partition_start_sector(vol_path: str) -> int:
    """获取逻辑分区盘符对应的硬件起始LBA扇区（解决逻辑盘偏移问题），失败抛出 OSError"""
    handle = win32_open_device(vol_path)
    if handle is None:
        err = ctypes.get_last_error()
        raise OSError(f"无法打开卷设备 {vol_path}，错误码: {err}")
    try:
        class PARTITION_INFORMATION_EX(ctypes.Structure):
            _fields_ = [
                ("PartitionStyle", ctypes.c_uint),
                ("StartingOffset", ctypes.c_longlong),
                ("PartitionLength", ctypes.c_longlong),
                ("HiddenSectors", ctypes.c_ulong),
                ("PartitionNumber", ctypes.c_uint),
                ("RewritePartition", ctypes.c_ubyte),
                ("PartitionType", ctypes.c_ubyte * 16),
                ("BootIndicator", ctypes.c_ubyte),
                ("RecognizedPartition", ctypes.c_ubyte),
                ("RewritePartitionType", ctypes.c_ubyte),
                ("ExtraData", ctypes.c_ubyte * 12)
            ]
        part_info = PARTITION_INFORMATION_EX()
        ret_bytes = wintypes.DWORD()
        res = ctypes.windll.kernel32.DeviceIoControl(
            handle, IOCTL_DISK_GET_PARTITION_INFO_EX,
            None, 0, ctypes.byref(part_info), ctypes.sizeof(part_info),
            ctypes.byref(ret_bytes), None
        )
        if not res:
            err = ctypes.get_last_error()
            raise OSError(f"获取分区信息失败 {vol_path}，错误码: {err}（可能是可移动介质无分区表）")
        return int(part_info.StartingOffset // SECTOR_SIZE)
    finally:
        ctypes.windll.kernel32.CloseHandle(handle)

def detect_system_physical_drives() -> set:
    """
    通过 IOCTL_VOLUME_GET_VOLUME_DISK_EXTENTS 动态检测包含系统卷(C:)的物理磁盘
    替代硬编码 PhysicalDrive0 黑名单，适配多硬盘/非标系统盘配置
    返回: {"PhysicalDrive0", "PhysicalDrive1", ...}
    """
    system_drives = set()
    # 系统可能安装在非C盘（如PE/双系统），优先检测C:，辅助检测包含\Windows的盘符
    candidates = ["C:"] + [f"{l}:" for l in "DEFGHIJKLMNOPQRSTUVWXYZ"
                           if os.path.exists(f"{l}:\\Windows")]
    for letter_path in candidates:
        vol_path = f"\\\\.\\{letter_path}"
        if not os.path.exists(vol_path):
            continue
        handle = win32_open_device(vol_path)
        if handle is None:
            continue
        try:
            # VOLUME_DISK_EXTENTS: NumberOfDiskExtents (DWORD) + array of DISK_EXTENT
            # Each DISK_EXTENT: DiskNumber (DWORD), StartingOffset (LARGE_INTEGER), ExtentLength (LARGE_INTEGER)
            buf_size = ctypes.sizeof(wintypes.DWORD) + 3 * ctypes.sizeof(ctypes.c_longlong)
            out_buf = ctypes.create_string_buffer(buf_size)
            ret_bytes = wintypes.DWORD()
            res = ctypes.windll.kernel32.DeviceIoControl(
                handle, IOCTL_VOLUME_GET_VOLUME_DISK_EXTENTS,
                None, 0, ctypes.byref(out_buf), buf_size,
                ctypes.byref(ret_bytes), None
            )
            if res:
                extent_count = struct.unpack_from("I", out_buf.raw)[0]
                for i in range(extent_count):
                    # DiskNumber at offset 4 + i * 24
                    disk_num = struct.unpack_from("I", out_buf.raw, 4 + i * 24)[0]
                    system_drives.add(f"PhysicalDrive{disk_num}")
        finally:
            ctypes.windll.kernel32.CloseHandle(handle)
    # 兜底：如果API失败或检测不到，保留C:对应盘的传统启发式检测
    if not system_drives and os.path.exists("\\\\.\\C:"):
        system_drives.add("PhysicalDrive0")
    return system_drives

def format_size(byte_num: int) -> str:
    """字节转人类可读容量"""
    if byte_num <= 0:
        return "未知(权限不足)"
    gb = byte_num / (1024 ** 3)
    mb = byte_num / (1024 ** 2)
    if gb >= 1:
        return f"{gb:.2f} GB | 总扇区: {byte_num // SECTOR_SIZE:,}"
    else:
        return f"{mb:.1f} MB | 总扇区: {byte_num // SECTOR_SIZE:,}"

# ====================== 磁盘枚举 ======================
def list_all_devices():
    """枚举所有逻辑盘符 + 物理磁盘，区分类型"""
    dev_list = []
    # 1. 逻辑盘符 A-Z
    for letter in "ABCDEFGHIJKLMNOPQRSTUVWXYZ":
        raw_path = f"\\\\.\\{letter}:"
        if not os.path.exists(raw_path):
            continue
        try:
            total_byte = get_disk_total_bytes(raw_path)
        except OSError:
            total_byte = 0  # 无权限时跳过，不阻塞枚举
        try:
            partition_offset = get_partition_start_sector(raw_path)
        except OSError:
            partition_offset = 0  # 可移动介质可能无分区表
        dev_list.append({
            "dev_type": "volume",
            "name": f"{letter}:",
            "win_path": raw_path,
            "size_bytes": total_byte,
            "partition_offset": partition_offset
        })
    # 2. 物理磁盘 PhysicalDrive0~9
    for idx in range(10):
        name = f"PhysicalDrive{idx}"
        raw_path = f"\\\\.\\{name}"
        if not os.path.exists(raw_path):
            continue
        try:
            total_byte = get_disk_total_bytes(raw_path)
        except OSError:
            total_byte = 0
        dev_list.append({
            "dev_type": "physical",
            "name": name,
            "win_path": raw_path,
            "size_bytes": total_byte,
            "partition_offset": 0
        })
    return dev_list

def print_device_list(dev_list):
    """格式化打印磁盘列表，标注危险设备、寻址差异"""
    print("=" * 80)
    print("磁盘设备列表（【物理盘】=真实硬件LBA | 【盘符】=分区带偏移）")
    print("=" * 80)
    vol_idx = 0
    phy_idx = 0
    vol_devs = [d for d in dev_list if d["dev_type"] == "volume"]
    phy_devs = [d for d in dev_list if d["dev_type"] == "physical"]

    print("\n[ 逻辑分区盘符（普通用户可读，扇区带分区偏移）]")
    print(f"{'序号':<4}{'盘符':<5}{'总容量':<30}{'分区起始硬件扇区'}")
    print("-" * 70)
    for idx, dev in enumerate(vol_devs):
        vol_idx = idx
        offset = dev["partition_offset"]
        tip = f"偏移{offset}" if offset != 0 else "无偏移"
        print(f"[{idx:<3}] {dev['name']:<5} {format_size(dev['size_bytes']):<30} {tip}")

    print("\n[ 物理磁盘（必须管理员运行，读取真实硬件LBA）]")
    print(f"{'序号':<4}{'设备名':<15}{'总容量':<30}{'安全提示'}")
    print("-" * 70)
    base_num = len(vol_devs)
    for idx, dev in enumerate(phy_devs):
        phy_idx = idx
        num = base_num + idx
        warn = "⚠️ 禁止读取系统硬盘" if dev["name"] in BLACKLIST_PHYSICAL else "正常SD/U盘"
        print(f"[{num:<3}] {dev['name']:<15} {format_size(dev['size_bytes']):<30} {warn}")
    print("\n提示：STM32裸SPI写入硬件LBA，优先选择【物理磁盘】读取，避免偏移全0！")

def interactive_select_device(dev_list):
    """交互式选择磁盘，拦截危险系统盘"""
    print_device_list(dev_list)
    all_dev = dev_list
    if not all_dev:
        print("错误: 未检测到任何可用磁盘设备！")
        sys.exit(1)
    choice = input("\n输入磁盘序号(回车退出): ").strip()
    if not choice:
        print("已取消操作，退出程序")
        sys.exit(0)
    try:
        select_idx = int(choice)
    except ValueError:
        print(f"错误：输入 '{choice}' 不是有效数字！请输入列表中的序号")
        sys.exit(1)
    if select_idx < 0 or select_idx >= len(all_dev):
        print(f"错误：序号 {select_idx} 超出范围 0~{len(all_dev)-1}")
        sys.exit(1)
    target_dev = all_dev[select_idx]
    # 拦截系统硬盘
    if target_dev["name"] in BLACKLIST_PHYSICAL:
        print(f"安全拦截：{target_dev['name']} 包含系统卷，禁止直接读取！")
        print(f"当前系统盘列表: {BLACKLIST_PHYSICAL}")
        sys.exit(1)
    # 物理盘校验管理员权限
    if target_dev["dev_type"] == "physical" and not is_admin():
        print("权限错误：读取物理磁盘必须【右键Python → 以管理员身份运行】")
        sys.exit(1)
    return target_dev

# ====================== 扇区底层读取 ======================
def read_raw_sector(dev_win_path: str, real_hardware_lba: int, sector_count: int, total_disk_byte: int) -> bytes:
    """
    读取指定硬件真实LBA扇区（严格只读，禁止写操作）
    :param dev_win_path: 设备路径 \\.\G: 或 \\.\PhysicalDriveX
    :param real_hardware_lba: 硬件原始LBA（已处理分区偏移）
    :param sector_count: 读取扇区数量
    :param total_disk_byte: 磁盘总字节，越界校验
    :return: 原始字节数据
    """
    # 写入保护守卫：此函数禁止任何写操作，防止误修改导致数据破坏
    READ_ONLY_MODE = "rb"
    target_offset = real_hardware_lba * SECTOR_SIZE
    read_total_len = sector_count * SECTOR_SIZE
    # 内存上限检查，防止OOM
    if sector_count > MAX_SECTORS:
        print(f"错误：单次读取扇区数不能超过 {MAX_SECTORS}（{MAX_SECTORS * SECTOR_SIZE // (1024*1024)}MB），当前: {sector_count}")
        sys.exit(1)
    # 越界校验
    if total_disk_byte > 0 and target_offset + read_total_len > total_disk_byte:
        print(f"警告：读取范围超出磁盘边界！磁盘总字节：{total_disk_byte:,}")
        sys.exit(1)
    if target_offset < 0:
        print("错误：扇区号不能为负数！")
        sys.exit(1)
    # 二进制只读打开（READ_ONLY_MODE 守卫确保不会误改为写入）
    # 如需扩展写入功能，必须使用独立函数且要求 --write 显式参数
    try:
        with open(dev_win_path, READ_ONLY_MODE, buffering=0) as f:
            f.seek(target_offset)
            raw_data = f.read(read_total_len)
    except PermissionError:
        print("权限不足！物理磁盘需要管理员运行，盘符请关闭占用文件管理器")
        sys.exit(1)
    except OSError as e:
        print(f"读取设备失败：{str(e)}")
        print("排查：读卡器断开、SD卡损坏、设备被占用")
        sys.exit(1)
    if len(raw_data) < read_total_len:
        print(f"警告：仅读取到 {len(raw_data)}/{read_total_len} 字节，磁盘末尾截断")
    return raw_data

# ====================== 数据解析与校验 ======================
def check_test_data(buf: bytes, match_threshold_pct: int = 95) -> dict:
    """校验STM32写入的测试数据：0~99索引 = 值1~100
    :param buf: 原始扇区数据
    :param match_threshold_pct: 判定成功的匹配百分比阈值，默认95
    """
    # 阈值边界保护
    match_threshold_pct = max(1, min(100, match_threshold_pct))
    res = {
        "is_all_zero": all(b == 0 for b in buf),
        "match_test": False,
        "error_list": [],
        "match_count": 0,
        "match_threshold_pct": match_threshold_pct
    }
    if res["is_all_zero"]:
        return res
    match_cnt = 0
    for i in range(min(100, len(buf))):
        expect = i + 1
        real = buf[i]
        if real == expect:
            match_cnt += 1
        else:
            res["error_list"].append((i, real, expect))
    res["match_count"] = match_cnt
    if match_cnt >= match_threshold_pct:
        res["match_test"] = True
    return res

def generate_text_log(raw_buf: bytes, hardware_lba: int, dev_info: dict, sdsc_mode: bool, match_threshold_pct: int = 95) -> list[str]:
    """生成可读文本日志，包含校验结果、十六进制预览"""
    check_ret = check_test_data(raw_buf, match_threshold_pct)
    lines = []
    lines.append("=" * 70)
    lines.append(f"SD卡读取日志 | 生成时间: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"设备类型: {'物理磁盘(真实LBA)' if dev_info['dev_type'] == 'physical' else '逻辑分区盘符(带偏移)'}")
    lines.append(f"设备名称: {dev_info['name']}")
    lines.append(f"读取硬件LBA扇区: {hardware_lba} | SDSC字节寻址模式: {'开启' if sdsc_mode else '关闭(SDHC标准块寻址)'}")
    lines.append(f"读取总字节: {len(raw_buf)} | 扇区数量: {len(raw_buf)//SECTOR_SIZE}")
    lines.append("=" * 70)
    # 写入状态校验
    if check_ret["is_all_zero"]:
        lines.append("\n⚠️ 【检测结果】读取数据全部为0！")
        lines.append("排查方案：")
        lines.append("  1. 优先切换【物理磁盘】读取，盘符存在分区偏移必然错位")
        lines.append("  2. 确认STM32 SD驱动SpiRx函数已修复（无0xFF时钟会读写失效）")
        lines.append("  3. SDSC卡开启--sdsc参数，块号会×512字节偏移")
    elif check_ret["match_test"]:
        lines.append(f"\n✅ 【检测结果】测试数据1~100写入成功！前100字节匹配{check_ret['match_count']}/100（阈值≥{check_ret['match_threshold_pct']}%）")
    else:
        lines.append(f"\n❌ 【检测结果】数据不匹配，写入失败或寻址错误（匹配{check_ret['match_count']}/100，阈值≥{check_ret['match_threshold_pct']}%）")
        for idx, real, exp in check_ret["error_list"][:10]:
            lines.append(f"  索引{idx}: 读到{real} 预期{exp}")
    # 十六进制预览（前256字节）
    lines.append("\n【十六进制预览（前256字节）】")
    preview_len = min(256, len(raw_buf))
    for offset in range(0, preview_len, 16):
        chunk = raw_buf[offset:offset+16]
        hex_str = " ".join(f"{b:02X}" for b in chunk)
        ascii_str = "".join([chr(b) if 32 <= b < 127 else "." for b in chunk])
        lines.append(f"0x{offset:04X} | {hex_str:<48} | {ascii_str}")
    # 完整数值列表前100
    lines.append("\n【前100字节数值列表】")
    val_chunk = raw_buf[:100]
    for i in range(0, len(val_chunk), 10):
        slice_data = val_chunk[i:i+10]
        val_text = " ".join(f"{v:3d}" for v in slice_data)
        lines.append(f"索引{i:02d}~{i+9:02d}: {val_text}")
    return lines

# ====================== 主入口 ======================
def main():
    parser = argparse.ArgumentParser(
        description="STM32 SPI SD卡原始扇区读取工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
参数说明：
  -d --drive     指定磁盘设备，如 G: / PhysicalDrive2
  -s --sector    STM32写入的块号（SDHC直接LBA，SDSC传入字节地址自动÷512）
  -c --count     读取扇区数量，默认1，最大{MAX_SECTORS}
  --sdsc         SDSC卡模式，-s参数为字节地址（自动÷512转LBA扇区）
  --dump-bin     导出原始二进制bin文件
  --list         仅列出所有磁盘设备，不读取
  --match-threshold  测试数据匹配阈值百分比，默认95，范围1~100
        """
    )
    parser.add_argument("--list", action="store_true", help="仅打印磁盘列表")
    parser.add_argument("-d", "--drive", type=str, default=None, help="指定磁盘设备")
    parser.add_argument("-s", "--sector", type=int, default=1000, help="目标块号，默认1000")
    parser.add_argument("-c", "--count", type=int, default=1, help="读取扇区数量，默认1")
    parser.add_argument("--sdsc", action="store_true", help="SDSC卡字节寻址模式（用户传入字节地址，自动÷512转LBA）")
    parser.add_argument("--dump-bin", action="store_true", help="导出原始bin文件")
    parser.add_argument("--match-threshold", type=int, default=95, metavar="PCT",
                        help="测试数据校验匹配百分比阈值，默认95%%，范围1~100")
    args = parser.parse_args()

    # 动态检测系统盘（替代硬编码 PhysicalDrive0）
    global BLACKLIST_PHYSICAL
    BLACKLIST_PHYSICAL = detect_system_physical_drives()
    if BLACKLIST_PHYSICAL:
        print(f"[系统检测] 已识别系统物理磁盘: {BLACKLIST_PHYSICAL}")

    # 预创建输出文件夹
    LOG_OUT_DIR.mkdir(parents=True, exist_ok=True)
    all_devices = list_all_devices()

    # 仅列出设备
    if args.list:
        print_device_list(all_devices)
        return

    # 参数边界校验
    if args.sector < 0 or args.count < 1:
        print("参数错误：扇区号不能为负，读取数量必须≥1")
        sys.exit(1)
    if args.count > MAX_SECTORS:
        print(f"参数错误：单次读取扇区数不能超过 {MAX_SECTORS}（{MAX_SECTORS * SECTOR_SIZE // (1024*1024)}MB），当前: {args.count}")
        sys.exit(1)
    if not (1 <= args.match_threshold <= 100):
        print(f"参数错误：匹配阈值必须在 1~100 之间，当前: {args.match_threshold}")
        sys.exit(1)

    # 选择磁盘设备
    target_dev = None
    if args.drive:
        input_name = args.drive.strip()
        # 补全设备路径前缀
        win_path = f"\\\\.\\{input_name}" if not input_name.startswith("\\\\.\\") else input_name
        # 匹配设备
        for dev in all_devices:
            if dev["win_path"] == win_path or dev["name"] == input_name:
                target_dev = dev
                break
        if target_dev is None:
            print(f"找不到设备：{input_name}，使用--list查看可用设备")
            sys.exit(1)
        # 安全拦截系统盘
        if target_dev["name"] in BLACKLIST_PHYSICAL:
            print(f"安全拦截：禁止读取系统硬盘 {target_dev['name']}")
            sys.exit(1)
        # 物理盘校验管理员
        if target_dev["dev_type"] == "physical" and not is_admin():
            print("权限错误：读取物理磁盘必须管理员运行Python！")
            sys.exit(1)
    else:
        # 交互式选择
        target_dev = interactive_select_device(all_devices)

    # 计算真实硬件LBA
    base_lba = args.sector
    # SDSC模式：STM32端使用字节寻址(CMD17/CMD24传入字节地址)，
    # 用户传入的是字节地址，需转换为LBA扇区号（÷512）
    if args.sdsc:
        real_hardware_lba = base_lba // SECTOR_SIZE
        print(f"已开启SDSC字节寻址模式，字节地址{base_lba} → 硬件LBA扇区{real_hardware_lba}")
    else:
        real_hardware_lba = base_lba
    # 逻辑盘符叠加分区偏移
    if target_dev["dev_type"] == "volume":
        partition_offset = target_dev["partition_offset"]
        final_hardware_lba = real_hardware_lba + partition_offset
        print(f"逻辑盘符自动叠加分区偏移{partition_offset}，最终硬件LBA：{final_hardware_lba}")
    else:
        final_hardware_lba = real_hardware_lba

    # 读取原始扇区数据
    print(f"\n正在读取设备 {target_dev['win_path']} 硬件LBA {final_hardware_lba}，共{args.count}扇区...")
    raw_buffer = read_raw_sector(
        dev_win_path=target_dev["win_path"],
        real_hardware_lba=final_hardware_lba,
        sector_count=args.count,
        total_disk_byte=target_dev["size_bytes"]
    )
    print(f"读取完成，获取 {len(raw_buffer)} 字节原始数据")

    # 生成文本日志
    log_lines = generate_text_log(raw_buffer, final_hardware_lba, target_dev, args.sdsc, args.match_threshold)
    time_tag = time.strftime("%Y%m%d_%H%M%S")
    session_dir = LOG_OUT_DIR / time_tag
    session_dir.mkdir(parents=True, exist_ok=True)
    txt_path = session_dir / f"sd_log_{time_tag}_lba{final_hardware_lba}.txt"
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("\n".join(log_lines))
    print(f"文本日志已保存: {txt_path.resolve()}")

    # 导出原始bin文件
    if args.dump_bin:
        bin_path = session_dir / f"sd_raw_{time_tag}_lba{final_hardware_lba}.bin"
        with open(bin_path, "wb") as f:
            f.write(raw_buffer)
        print(f"原始二进制文件已保存: {bin_path.resolve()}")

    # 控制台打印简短预览
    print("\n==== 数据预览（前128字节）====")
    preview = raw_buffer[:min(128, len(raw_buffer))]
    for off in range(0, len(preview), 16):
        chunk = preview[off:off+16]
        hex_txt = " ".join(f"{b:02X}" for b in chunk)
        asc_txt = "".join([chr(b) if 32 <= b < 127 else "." for b in chunk])
        print(f"{off:04X}  {hex_txt:<48}  {asc_txt}")

if __name__ == "__main__":
    main()