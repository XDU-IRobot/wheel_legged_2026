# SD Logger — 懒人式完整操作指南

## 这是什么

把 STM32 控制循环里的任意变量（腿长、摆角、速度…）和时间戳一起写入 SD 卡，拔卡插电脑一键解析为交互式图表。

**当前测试配置：** 开机 1 秒后自动开始，100Hz 记录 60 秒。两个自增 float 测试变量 `test_val_a`、`test_val_b`。

---

## 一、PC 环境准备（一次性）

### 1. Python 环境

```bash
# 激活 conda
conda activate productivity-tools

# 进入工具目录
cd C:\Users\Asuna\Desktop\wheel_legged_2026\app\targets\wheel_legged\tools\sd_2_log

# 安装依赖
pip install plotly pandas
```

### 2. 三个脚本简介

| 脚本 | 作用 | 依赖 |
|------|------|------|
| `sd_reader.py` | 从 SD 卡导出原始扇区 → `.bin` | **零依赖**（纯标准库） |
| `parse_log.py` | `.bin` → CSV 表格 | 零依赖 |
| `visualize_log.py` | `.bin` → 交互式 HTML 图表 | `plotly` `pandas` |

---

## 二、完整操作流程（每次）

### Step 1：烧录 + 上电

```
编译 → 烧录 wheel_legged.elf → 上电
```

上电后：
1. SD 卡自动初始化
2. **1 秒后自动开始记录**（无需手动操作）
3. 60 秒后自动停止（或随时断电也安全）

### Step 2：断电拔卡

**等 5~60 秒，直接断电拔卡。** 不需要手动停止。

> 💡 每次刷写扇区时同步更新 header，断电最多丢失 ring buffer 中未刷写的 ~1 秒数据，已写入 SD 的数据全部可恢复。

### Step 3：SD 卡插入 PC 读卡器

### Step 4：导出原始数据

**⚠️ 必须用物理磁盘路径，不能只用盘符！**（盘符可能读到 Windows 文件系统缓存）

```bash
# 以管理员身份打开终端！

# 4a. 找到 SD 卡对应的物理磁盘（靠容量和插拔辨认）
python sd_reader.py --list
```

输出类似：
```
[ 逻辑分区盘符（普通用户可读）]
[4  ] G:    1.87 GB | 总扇区: 3,920,640

[ 物理磁盘（必须管理员运行）]
[7  ] PhysicalDrive2  1.87 GB    正常SD/U盘
```

1.87GB 的那个就是 SD 卡 → `PhysicalDrive2`。

**不确定是哪个？** 拔掉 SD 卡再跑一次 `--list`，消失的那个就是。

```bash
# 4b. 管理员运行，导出扇区
python sd_reader.py -d PhysicalDrive2 -s 2048 -c 500 --dump-bin
```

参数说明：
| 参数 | 值 | 含义 |
|------|-----|------|
| `-d` | `PhysicalDrive2` | SD 卡物理磁盘（**不能是 G:**） |
| `-s` | `2048` | 起始 LBA（= C++ 端 `base_lba`） |
| `-c` | `500` | 读 500 个扇区（256KB，覆盖 60s 绰绰有余） |
| `--dump-bin` | | 生成 `.bin` 文件 |

输出文件结构（每次运行自动创建时间戳子文件夹）：
```
sd_out/
├── 20260706_133712/
│   ├── sd_raw_20260706_133712_lba2048.bin
│   └── sd_log_20260706_133712_lba2048.txt
└── 20260706_150000/
    ├── ...
```

**验证数据正确：** 输出中的 `数据预览` 前 4 字节应该是：
```
0000  47 4C 44 53 ...   G.L.D.S    ← ✅ 正确
```
如果是 `7B 00 36 00...` 或其他 → 用错盘符了，换 PhysicalDrive。

### Step 5：解析为 CSV

```bash
python parse_log.py sd_out/20260706_133712/sd_raw_20260706_133712_lba2048.bin
```

CSV 自动生成在同一个文件夹里。

输出预览：
```
Reading: sd_raw_20260706_133712_lba2048.bin  (256,000 bytes)
  Magic:      0x53444C47 ✓
  Fields:     2 (test_val_a, test_val_b)
  Record sz:  12 B
  Records:    5250 / 6000
  Duration:   52.5s
  Stop:       graceful
Parsed 5250 records.

Preview (first 3 records):
  [    1010 ms] test_val_a=4.4100 test_val_b=95.5891
  [    1020 ms] test_val_a=4.4600 test_val_b=95.5390
  [    1030 ms] test_val_a=4.5100 test_val_b=95.4890
  ... (5247 more)
CSV written: sd_out/20260706_133712/..._log.csv  (5250 rows)
```

### Step 6：生成交互式图表

```bash
python visualize_log.py sd_out/20260706_133712/sd_raw_20260706_133712_lba2048.bin --open
```

浏览器自动打开，包含：
- **📊 统计表**：min / max / mean / std
- **📈 时间序列**：每字段一个子图，共享时间轴，hover 显示数值，框选缩放
- **🔵 散点矩阵**：字段间两两关系，按时间着色

### Step 7（可选）：只导出指定内容

```bash
# 只要统计
python visualize_log.py xxx.bin --no-ts --no-scatter

# 只要时间序列
python visualize_log.py xxx.bin --no-scatter --no-stats

# 自定义标题
python visualize_log.py xxx.bin --title "楼梯测试第3次"
```

---

## 三、常用参数速查

### sd_reader.py

| 参数 | 说明 | 示例 |
|------|------|------|
| `--list` | 列出所有磁盘 | `python sd_reader.py --list` |
| `-d` | 设备名 | `-d PhysicalDrive2` |
| `-s` | 起始 LBA | `-s 2048` |
| `-c` | 扇区数（默认 1，最大 8192） | `-c 500` |
| `--dump-bin` | 同时生成 .bin | `--dump-bin` |

### visualize_log.py

| 参数 | 说明 |
|------|------|
| `-o` | 输出 .html 路径（默认 `<input>_viz.html`） |
| `--open` | 生成后自动打开浏览器 |
| `--title` | 自定义标题 |
| `--no-ts` | 跳过时间序列图 |
| `--no-scatter` | 跳过散点矩阵 |
| `--no-stats` | 跳过统计表 |

---

## 四、核对清单（出问题时过一遍）

| 步骤 | 检查项 | ✅ 标准 |
|------|--------|---------|
| STM32 | SD 卡在位 | 卡正确插入卡槽 |
| STM32 | 等待 >1s | 上电 1s 后才开始记录 |
| PC | 管理员运行终端 | 右键 → 以管理员身份运行 |
| PC | 激活 conda | `conda activate productivity-tools` |
| PC | 找对磁盘 | 靠容量大小匹配 |
| PC | 用 PhysicalDrive | **不用 G:** |
| PC | 读 LBA 2048 | `-s 2048` |
| PC | 前 4 字节是 `47 4C 44 53` | 不是则磁盘不对 |
| PC | parse_log 有 `Magic ✓` | header 正确 |
| PC | Records > 0 | 有数据 |

---

## 五、二进制格式（仅供调试）

```
LBA 2048: Header 扇区
  0x00: magic      = 0x53444C47 ("SDLG")
  0x08: record_sz  (4 + N_fields * 4)
  0x0C: total_records
  0x18: num_fields
  0x20: field_descriptors[] (each: name[16] + type[1] + reserved[1])

LBA 2049+: 数据扇区
  [tick_ms: uint32 LE] [field0: 4B] [field1: 4B] ...
  扇区末尾不足一条记录时补零，不跨扇区
```

---

## 六、故障排查

| 现象 | 原因 | 解决 |
|------|------|------|
| sd_reader 输出前 4 字节不是 `47 4C 44 53` | 用了盘符或磁盘不对 | 用 `--list` + 物理磁盘 + 管理员运行 |
| `物理磁盘` 权限不足 | 没以管理员运行 | 右键终端 → 以管理员身份运行 |
| parse_log 显示 0 records | header 里的 total_records=0（断电太早） | 确保上电 >1s 后再断电 |
| CSV 里数值是整数而不是 float | 字段类型注册错 | C++ 端 RegisterField 用 kFloat32 |
| `ModuleNotFoundError: plotly` | 没装 | `pip install plotly pandas` |
