#pragma once

#include <cstdint>

#include "sd_spi.hpp"

/**
 * @brief SD 卡数据日志器——在原始块驱动之上提供结构化浮点/整数日志记录
 *
 * 设计目标：
 *   - 热路径 Tick() 极其轻量（< 2µs），只做降采样 + 值拷贝到 ring buffer
 *   - 后台 ProcessWrite() 在 main 循环中执行实际的 SD 卡扇区写入（~0.5-1ms）
 *   - 通过指针注册要记录的变量，无需修改现有数据结构
 *
 * 使用示例：
 *   @code
 *   SdLogger logger;
 *   SdLogger::Config cfg;
 *   cfg.decimation = 10;           // 50 Hz from 500 Hz
 *   cfg.max_records = 0;           // auto-calc
 *   cfg.max_duration_s = 300;      // 5 minutes
 *   cfg.base_lba = 2048;
 *   cfg.sd = &sd_card;
 *   logger.Configure(cfg);
 *   logger.RegisterField("l_l_m", SdLogger::FieldType::kFloat32, &some_float_var);
 *   logger.Start();
 *   // ── 在 500 Hz 控制循环中 ──
 *   logger.Tick(HAL_GetTick());
 *   // ── 在后台 main 循环中 ──
 *   logger.ProcessWrite();
 *   @endcode
 */

class SdLogger {
 public:
  // ── 类型定义 ──────────────────────────────────────────────

  enum class State : uint8_t {
    kIdle,       ///< 未启动或已停止，无错误
    kRecording,  ///< 正在记录
    kFlushing,   ///< 停止请求已收到，正在排空剩余数据
    kFull,       ///< 达到 max_records，自动停止
    kError,      ///< 写入失败，不可恢复
  };

  enum class FieldType : uint8_t {
    kFloat32 = 0,  ///< 32 位浮点数
    kUint32 = 1,   ///< 32 位无符号整数
  };

  struct Config {
    uint32_t decimation = 10;       ///< 降采样因子（Hz =循环频率/decimation）
    uint32_t max_records = 0;       ///< 最大记录数（0 = 根据容量和时长自动计算）
    uint32_t max_duration_s = 300;  ///< 时长限制秒数（仅 max_records==0 时生效）
    uint32_t base_lba = 2048;       ///< 起始 LBA，header 写在此地址
    SdSpi* sd = nullptr;            ///< SD 卡驱动指针（外部初始化好的 SdSpi 实例）
  };

  struct Stats {
    uint32_t records_buffered;  ///< ring buffer 中待刷写的记录数
    uint32_t sectors_written;   ///< 累计写入 SD 的扇区数
    uint32_t write_errors;      ///< WriteBlock 失败次数
    uint32_t ring_overruns;     ///< ring buffer 满导致丢弃的记录数
  };

  // ── 常量 ──────────────────────────────────────────────────

  static constexpr uint32_t kMagic = 0x53444C47;  // "SDLG"
  static constexpr uint16_t kVersion = 0x0001;
  static constexpr uint32_t kMaxFields = 26;  // 受 header 480 字节限制
  static constexpr uint32_t kMaxNameLen = 15;
  static constexpr uint32_t kSectorSize = 512;
  static constexpr uint32_t kRingSectors = 4;  // ring buffer = 4 × 512 = 2048 字节
  static constexpr uint32_t kRingSize = kRingSectors * kSectorSize;
  static constexpr uint32_t kFieldDescSize = 18;  // 单个字段描述符大小

  // ── 生命周期 ──────────────────────────────────────────────

  SdLogger() = default;
  ~SdLogger() = default;

  /// @brief 配置日志器。必须在 RegisterField / Start 之前调用。
  /// @return false 若 sd 为 nullptr 或 decimation == 0
  bool Configure(const Config& cfg);

  /// @brief 注册一个要记录的变量
  /// @param name  字段名（最长 15 字符，null-terminated）
  /// @param type  字段类型
  /// @param ptr   指向变量的指针（日志器会读取 4 字节，调用者保证生命周期）
  /// @return false 若字段数已满
  bool RegisterField(const char* name, FieldType type, const void* ptr);

  // ── 启停控制 ──────────────────────────────────────────────

  /// @brief 开始记录，写入 header block 到 SD 卡
  /// @return false 若无注册字段或 SD 写入失败
  bool Start();

  /// @brief 立即停止：循环 ProcessWrite 直到排空，更新 header
  /// @note  可能在调用线程阻塞约 N × 1ms（N = ring 中扇区数）
  void Stop();

  /// @brief 请求优雅停止：在后台逐步排空后自动停止
  /// @note  非阻塞，由 ProcessWrite 完成后自动转换到 kIdle
  void RequestStop();

  // ── 热路径 ────────────────────────────────────────────────

  /// @brief  每个控制周期调用一次，自动处理降采样
  /// @param  tick_ms  当前系统毫秒时间戳（HAL_GetTick()）
  /// @note   必须在 500 Hz 控制循环中调用；内部执行 < 2 µs
  void Tick(uint32_t tick_ms);

  // ── 后台 ──────────────────────────────────────────────────

  /// @brief  在后台循环中调用，每次刷写最多一个完整扇区到 SD
  /// @return true 若本调用实际写了一个扇区
  /// @note   调用间隔应 < 2 ms（高速模式）或 < 50 ms（低速模式）
  bool ProcessWrite();

  // ── 查询 ──────────────────────────────────────────────────

  State GetState() const { return state_; }
  uint32_t GetRecordedCount() const { return recorded_count_; }
  uint32_t GetMaxRecords() const { return config_.max_records; }
  Stats GetStats() const { return stats_; }

  /// @brief 根据参数计算最大可记录条数
  /// @param record_size      单条记录字节数
  /// @param decimation       降采样因子
  /// @param max_duration_s   最大时长（秒）
  /// @param sd_block_count   SD 卡总块数
  static uint32_t CalculateMaxRecords(uint32_t record_size, uint32_t decimation, uint32_t max_duration_s,
                                      uint32_t sd_block_count);

 private:
  // ── 内部字段描述符 ────────────────────────────────────────

  struct FieldDesc {
    char name[16]{};  // 15 字符 + null
    FieldType type{FieldType::kFloat32};
    uint8_t reserved{};
    const void* ptr{nullptr};  // 指向外部变量的指针
  };

  // ── Header 序列化 / 更新 ──────────────────────────────────

  void SerializeHeader(uint8_t* buf) const;
  bool WriteHeaderBlock();
  bool UpdateHeaderTotalRecords();

  // ── Ring Buffer 辅助 ──────────────────────────────────────

  /// @brief 计算 ring buffer 中可用字节数
  uint32_t RingFreeSpace() const;

  /// @brief 计算 ring buffer 中已用字节数
  uint32_t RingUsedSpace() const;

  // ── 成员变量 ──────────────────────────────────────────────

  Config config_{};
  State state_{State::kIdle};
  bool stop_pending_{false};

  // ── 字段注册 ──────────────────────────────────────────────
  FieldDesc fields_[kMaxFields]{};
  uint16_t num_fields_{0};
  uint32_t record_size_{0};  // = 4 + num_fields * 4

  // ── 计数 ──────────────────────────────────────────────────
  uint32_t recorded_count_{0};
  uint32_t tick_counter_{0};
  uint32_t current_data_lba_{0};
  uint32_t flushed_records_{0};  // 已安全写入 SD 的记录数（用于断电恢复）

  // ── Ring Buffer（4 × 512 = 2048 字节） ────────────────────
  uint8_t ring_[kRingSize]{};
  uint32_t ring_head_{0};     // 写游标（字节偏移）
  uint32_t ring_tail_{0};     // 刷游标（字节偏移）
  bool sector_ready_{false};  // ring_head_ 跨过扇区边界标志

  // ── 刷写暂存 ──────────────────────────────────────────────
  uint8_t flush_buf_[kSectorSize]{};

  // ── 统计 ──────────────────────────────────────────────────
  Stats stats_{};
};
