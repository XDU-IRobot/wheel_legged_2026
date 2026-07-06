#include "sd_log.hpp"

#include <algorithm>
#include <cstring>

// ═══════════════════════════════════════════════════════════════════════════
// Ring Buffer 内部辅助
// ═══════════════════════════════════════════════════════════════════════════

namespace {

/// @brief 向 ring buffer 写入 len 字节，自动处理首尾回绕
void RingWriteBytes(uint8_t* ring, uint32_t ring_size, uint32_t& head, const uint8_t* data,
                    uint32_t len) {
  uint32_t first = ring_size - head;
  if (len <= first) {
    std::memcpy(&ring[head], data, len);
    head += len;
  } else {
    std::memcpy(&ring[head], data, first);
    std::memcpy(&ring[0], data + first, len - first);
    head = len - first;
  }
}

/// @brief 向 ring buffer 写入 len 个零字节
void RingWriteZero(uint8_t* ring, uint32_t ring_size, uint32_t& head, uint32_t len) {
  uint32_t first = ring_size - head;
  if (len <= first) {
    std::memset(&ring[head], 0, len);
    head += len;
  } else {
    std::memset(&ring[head], 0, first);
    std::memset(&ring[0], 0, len - first);
    head = len - first;
  }
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// SdLogger 实现
// ═══════════════════════════════════════════════════════════════════════════

bool SdLogger::Configure(const Config& cfg) {
  if (cfg.sd == nullptr) return false;
  if (cfg.decimation == 0) return false;
  config_ = cfg;
  return true;
}

bool SdLogger::RegisterField(const char* name, FieldType type, const void* ptr) {
  if (num_fields_ >= kMaxFields) return false;
  if (ptr == nullptr) return false;

  FieldDesc& fd = fields_[num_fields_];

  // 拷贝名称（最多 kMaxNameLen 字符）
  uint32_t i = 0;
  for (; i < kMaxNameLen && name[i] != '\0'; ++i) {
    fd.name[i] = name[i];
  }
  fd.name[i] = '\0';

  fd.type = type;
  fd.reserved = 0;
  fd.ptr = ptr;

  num_fields_++;
  return true;
}

bool SdLogger::Start() {
  if (num_fields_ == 0) return false;
  if (config_.sd == nullptr) return false;

  record_size_ = 4 + static_cast<uint32_t>(num_fields_) * 4;

  // 若 max_records == 0，根据容量和时长自动计算
  if (config_.max_records == 0) {
    uint32_t sd_blocks = config_.sd->GetBlockCount();
    config_.max_records =
        CalculateMaxRecords(record_size_, config_.decimation, config_.max_duration_s, sd_blocks);
    if (config_.max_records == 0) return false;  // SD 卡太小或参数无效
  }

  // 验证地址范围（header + 最少 1 个扇区）
  uint32_t data_sectors = (static_cast<uint64_t>(config_.max_records) * record_size_ + kSectorSize - 1) / kSectorSize;
  uint32_t sd_blocks = config_.sd->GetBlockCount();
  if (config_.base_lba + 1 + data_sectors > sd_blocks) {
    return false;  // 超出 SD 卡容量
  }

  // 重置状态
  recorded_count_ = 0;
  flushed_records_ = 0;
  tick_counter_ = 0;
  current_data_lba_ = config_.base_lba + 1;
  ring_head_ = 0;
  ring_tail_ = 0;
  sector_ready_ = false;
  stop_pending_ = false;
  stats_ = {};

  // 写入 header
  if (!WriteHeaderBlock()) return false;

  state_ = State::kRecording;
  return true;
}

void SdLogger::Stop() {
  if (state_ == State::kIdle || state_ == State::kError) return;

  // 先切到 kFlushing 以触发排空
  if (state_ == State::kRecording) {
    state_ = State::kFlushing;
  }

  // 循环排空（阻塞调用线程）
  while (RingUsedSpace() > 0) {
    (void)ProcessWrite();
  }

  // 更新 header 中的 total_records
  (void)UpdateHeaderTotalRecords();

  state_ = State::kIdle;
}

void SdLogger::RequestStop() {
  if (state_ == State::kRecording) {
    stop_pending_ = true;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// Tick() — 热路径：每个控制周期调用
// ═══════════════════════════════════════════════════════════════════════════

void SdLogger::Tick(uint32_t tick_ms) {
  // 仅 kRecording 状态处理
  if (state_ != State::kRecording) return;

  // 降采样
  tick_counter_++;
  if (tick_counter_ % config_.decimation != 0) return;

  // 达到上限
  if (recorded_count_ >= config_.max_records) {
    state_ = State::kFull;
    return;
  }

  // 收到停止请求
  if (stop_pending_) {
    state_ = State::kFlushing;
    return;
  }

  // 检查当前扇区剩余空间，若不足一条记录则填零补齐
  uint32_t sector_offset = ring_head_ % kSectorSize;
  if (sector_offset + record_size_ > kSectorSize) {
    uint32_t pad = kSectorSize - sector_offset;
    RingWriteZero(ring_, kRingSize, ring_head_, pad);
    sector_ready_ = true;
  }

  // 检查 ring buffer 剩余空间
  uint32_t free = RingFreeSpace();
  if (free < record_size_) {
    stats_.ring_overruns++;
    return;  // 丢弃本条记录
  }

  // 写入时间戳（4 字节，little-endian，STM32 原生）
  RingWriteBytes(ring_, kRingSize, ring_head_, reinterpret_cast<const uint8_t*>(&tick_ms), 4);

  // 写入各字段值（每个 4 字节）
  for (uint16_t i = 0; i < num_fields_; i++) {
    RingWriteBytes(ring_, kRingSize, ring_head_, static_cast<const uint8_t*>(fields_[i].ptr), 4);
  }

  // 写入后检查是否恰好在扇区边界
  if (ring_head_ % kSectorSize == 0) {
    sector_ready_ = true;
  }

  recorded_count_++;
}

// ═══════════════════════════════════════════════════════════════════════════
// ProcessWrite() — 后台：从 ring buffer 刷写一个扇区到 SD 卡
// ═══════════════════════════════════════════════════════════════════════════

bool SdLogger::ProcessWrite() {
  if (state_ != State::kRecording && state_ != State::kFlushing) return false;

  uint32_t used = RingUsedSpace();

  // 在 kRecording 模式下，只在有完整扇区时才刷
  if (state_ == State::kRecording && used < kSectorSize) return false;

  // kFlushing 模式下且 ring 已空 → 更新 header 并完成
  if (used == 0) {
    if (state_ == State::kFlushing) {
      (void)UpdateHeaderTotalRecords();
      state_ = State::kIdle;
    }
    return false;
  }

  // 组装一个完整扇区到 flush_buf_
  if (used >= kSectorSize) {
    // 完整扇区
    uint32_t first = kRingSize - ring_tail_;
    if (first >= kSectorSize) {
      std::memcpy(flush_buf_, &ring_[ring_tail_], kSectorSize);
      ring_tail_ += kSectorSize;
    } else {
      std::memcpy(flush_buf_, &ring_[ring_tail_], first);
      std::memcpy(flush_buf_ + first, &ring_[0], kSectorSize - first);
      ring_tail_ = kSectorSize - first;
    }
  } else {
    // 不足一个扇区（仅在 kFlushing 时发生）：拷贝剩余数据并填零
    uint32_t first = kRingSize - ring_tail_;
    if (first >= used) {
      std::memcpy(flush_buf_, &ring_[ring_tail_], used);
    } else {
      std::memcpy(flush_buf_, &ring_[ring_tail_], first);
      std::memcpy(flush_buf_ + first, &ring_[0], used - first);
    }
    std::memset(flush_buf_ + used, 0, kSectorSize - used);
    ring_tail_ = (ring_tail_ + used) % kRingSize;  // effectively set to ring_head_
  }

  // 写入 SD 卡
  SdSpi::Error err = config_.sd->WriteBlock(current_data_lba_, flush_buf_);
  if (err != SdSpi::Error::kNone) {
    stats_.write_errors++;
    state_ = State::kError;
    return false;
  }

  current_data_lba_++;
  stats_.sectors_written++;
  sector_ready_ = false;

  // 更新已安全写入 SD 的记录计数
  if (used >= kSectorSize) {
    flushed_records_ += kSectorSize / record_size_;  // 完整扇区
  } else {
    flushed_records_ += used / record_size_;          // 尾部部分扇区
  }

  // 每次刷写后更新 header 中的 total_records，保证断电时数据可恢复
  (void)UpdateHeaderTotalRecords();

  // 检查是否还有更多完整扇区
  if (RingUsedSpace() >= kSectorSize) {
    sector_ready_ = true;
  }

  return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// 查询 / 计算
// ═══════════════════════════════════════════════════════════════════════════

uint32_t SdLogger::CalculateMaxRecords(uint32_t record_size, uint32_t decimation,
                                       uint32_t max_duration_s, uint32_t sd_block_count) {
  if (record_size == 0 || decimation == 0) return 0;

  // 按空间计算
  uint32_t records_per_sector = kSectorSize / record_size;
  if (records_per_sector == 0) return 0;  // 单条记录超过一个扇区，不支持

  // data_sectors 是基于全卡容量的上限（实际由 base_lba 控制，此处给一个保守估计）
  uint32_t max_records_by_space = sd_block_count * records_per_sector;

  // 按时长计算
  uint32_t max_records_by_time = UINT32_MAX;
  if (max_duration_s > 0) {
    uint32_t effective_rate_hz = 500 / decimation;
    if (effective_rate_hz == 0) effective_rate_hz = 1;
    // 防止乘法溢出
    if (max_duration_s > UINT32_MAX / effective_rate_hz) {
      max_records_by_time = UINT32_MAX;
    } else {
      max_records_by_time = max_duration_s * effective_rate_hz;
    }
  }

  return (max_records_by_space < max_records_by_time) ? max_records_by_space : max_records_by_time;
}

// ═══════════════════════════════════════════════════════════════════════════
// Ring Buffer 空间计算
// ═══════════════════════════════════════════════════════════════════════════

uint32_t SdLogger::RingFreeSpace() const {
  if (ring_head_ >= ring_tail_) {
    return kRingSize - (ring_head_ - ring_tail_) - 1;
  } else {
    return (ring_tail_ - ring_head_) - 1;
  }
}

uint32_t SdLogger::RingUsedSpace() const {
  if (ring_head_ >= ring_tail_) {
    return ring_head_ - ring_tail_;
  } else {
    return kRingSize - (ring_tail_ - ring_head_);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// Header 序列化
// ═══════════════════════════════════════════════════════════════════════════

void SdLogger::SerializeHeader(uint8_t* buf) const {
  std::memset(buf, 0, kSectorSize);

  uint32_t off = 0;

  // magic (4 B)
  buf[off++] = static_cast<uint8_t>(kMagic);
  buf[off++] = static_cast<uint8_t>(kMagic >> 8);
  buf[off++] = static_cast<uint8_t>(kMagic >> 16);
  buf[off++] = static_cast<uint8_t>(kMagic >> 24);

  // version (2 B)
  buf[off++] = static_cast<uint8_t>(kVersion);
  buf[off++] = static_cast<uint8_t>(kVersion >> 8);

  // flags (2 B) — bit0 = graceful_stop
  uint16_t flags = 0;
  buf[off++] = static_cast<uint8_t>(flags);
  buf[off++] = static_cast<uint8_t>(flags >> 8);

  // record_size_bytes (4 B)
  buf[off++] = static_cast<uint8_t>(record_size_);
  buf[off++] = static_cast<uint8_t>(record_size_ >> 8);
  buf[off++] = static_cast<uint8_t>(record_size_ >> 16);
  buf[off++] = static_cast<uint8_t>(record_size_ >> 24);

  // total_records (4 B) — 已安全刷入 SD 的记录数
  buf[off++] = static_cast<uint8_t>(flushed_records_);
  buf[off++] = static_cast<uint8_t>(flushed_records_ >> 8);
  buf[off++] = static_cast<uint8_t>(flushed_records_ >> 16);
  buf[off++] = static_cast<uint8_t>(flushed_records_ >> 24);

  // max_records (4 B)
  buf[off++] = static_cast<uint8_t>(config_.max_records);
  buf[off++] = static_cast<uint8_t>(config_.max_records >> 8);
  buf[off++] = static_cast<uint8_t>(config_.max_records >> 16);
  buf[off++] = static_cast<uint8_t>(config_.max_records >> 24);

  // decimation (4 B)
  buf[off++] = static_cast<uint8_t>(config_.decimation);
  buf[off++] = static_cast<uint8_t>(config_.decimation >> 8);
  buf[off++] = static_cast<uint8_t>(config_.decimation >> 16);
  buf[off++] = static_cast<uint8_t>(config_.decimation >> 24);

  // num_fields (2 B)
  buf[off++] = static_cast<uint8_t>(num_fields_);
  buf[off++] = static_cast<uint8_t>(num_fields_ >> 8);

  // sample_rate_hz (2 B) = 500
  buf[off++] = 0xF4;  // 0x01F4 = 500
  buf[off++] = 0x01;

  // reserved (4 B) — skipped (already zeroed)

  // field descriptors (从 offset 32 开始，每个 18 字节)
  off = 32;
  for (uint16_t i = 0; i < num_fields_; i++) {
    // name (16 B)
    uint32_t name_len = 0;
    for (; name_len < sizeof(fields_[i].name) && fields_[i].name[name_len] != '\0'; ++name_len) {
    }
    uint32_t copy_len = (name_len < 16U) ? name_len : 15U;
    for (uint32_t j = 0; j < copy_len; ++j) {
      buf[off + j] = static_cast<uint8_t>(fields_[i].name[j]);
    }
    off += 16;

    // type (1 B)
    buf[off++] = static_cast<uint8_t>(fields_[i].type);

    // reserved (1 B)
    buf[off++] = 0;
  }
}

bool SdLogger::WriteHeaderBlock() {
  SerializeHeader(flush_buf_);
  SdSpi::Error err = config_.sd->WriteBlock(config_.base_lba, flush_buf_);
  return err == SdSpi::Error::kNone;
}

bool SdLogger::UpdateHeaderTotalRecords() {
  // 读取当前 header block
  SdSpi::Error err = config_.sd->ReadBlock(config_.base_lba, flush_buf_);
  if (err != SdSpi::Error::kNone) return false;

  // 更新 offset 12-15: total_records（使用已安全刷入 SD 的记录数）
  flush_buf_[12] = static_cast<uint8_t>(flushed_records_);
  flush_buf_[13] = static_cast<uint8_t>(flushed_records_ >> 8);
  flush_buf_[14] = static_cast<uint8_t>(flushed_records_ >> 16);
  flush_buf_[15] = static_cast<uint8_t>(flushed_records_ >> 24);

  // 更新 offset 6-7: flags — bit0 = graceful_stop
  flush_buf_[6] |= 0x01;

  err = config_.sd->WriteBlock(config_.base_lba, flush_buf_);
  return err == SdSpi::Error::kNone;
}
