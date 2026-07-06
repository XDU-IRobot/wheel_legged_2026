#pragma once

#include <cstdint>
#include "spi.h"
#include "librm/hal/stm32/spi.hpp"

/**
 * @brief SD 卡 SPI 模式驱动（基于 librm）
 *
 * 硬件要求：
 *   - SPI1：MOSI=PD7, SCK=PB3, MISO=PB4（CubeMX 已配）
 *   - CS 引脚：需在 CubeMX 中配置一个 GPIO Output（推荐 PD6，命名为 SD_CS）
 *
 * 使用示例：
 *   @code
 *   SdSpi sd(hspi1, SD_CS_GPIO_Port, SD_CS_Pin);
 *   if (sd.Init() != SdSpi::Error::kNone) { // 初始化失败 }
 *   uint8_t buf[512];
 *   sd.ReadBlock(0, buf);   // 读第 0 块
 *   sd.WriteBlock(0, buf);  // 写第 0 块
 *   @endcode
 */
class SdSpi {
 public:
  /// @brief 错误码
  enum class Error : uint8_t {
    kNone = 0,          ///< 成功
    kTimeout,           ///< 超时
    kInitFailed,        ///< 初始化失败（未检测到 SD 卡或 CMD0/CMD8/ACMD41 失败）
    kWriteFailed,       ///< 写入失败
    kReadFailed,        ///< 读取失败
    kAddrOutOfRange,    ///< 地址超出范围
    kCrcError,          ///< CRC 校验错误
    kNotInitialized,    ///< 未初始化
  };

  /// @brief SD 卡类型
  enum class Type : uint8_t {
    kUnknown,  ///< 未知
    kSdV1,     ///< SD V1.x
    kSdV2,     ///< SD V2 (SDSC)
    kSdhc,     ///< SDHC/SDXC (high capacity)
  };

  // ─── 构造函数 ───────────────────────────────────────────────

  /**
   * @brief 构造 SD SPI 驱动
   * @param hspi       SPI 外设句柄（如 hspi1）
   * @param cs_port    CS 引脚所在的 GPIO 端口（如 SD_CS_GPIO_Port）
   * @param cs_pin     CS 引脚编号（如 SD_CS_Pin）
   * @param timeout_ms SPI 传输超时 [ms]
   */
  SdSpi(SPI_HandleTypeDef &hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint32_t timeout_ms = 200);

  // ─── 生命周期 ───────────────────────────────────────────────

  /**
   * @brief 初始化 SD 卡（上电 → CMD0 → CMD8 → ACMD41 → 准备就绪）
   * @return kNone 成功，其他值表示失败
   */
  Error Init();

  /**
   * @brief SD 卡是否已就绪（Init 成功后返回 true）
   */
  [[nodiscard]] bool IsReady() const { return ready_; }

  // ─── 块读写 ─────────────────────────────────────────────────

  /**
   * @brief 读取单个 512 字节块
   * @param block_addr 块地址（LBA，对于 SDHC 为块号；对于 SDSC 为字节地址）
   * @param dst        目标缓冲区（至少 512 字节）
   * @return kNone 成功
   */
  Error ReadBlock(uint32_t block_addr, uint8_t *dst);

  /**
   * @brief 写入单个 512 字节块
   * @param block_addr 块地址
   * @param src        数据源（至少 512 字节）
   * @return kNone 成功
   */
  Error WriteBlock(uint32_t block_addr, const uint8_t *src);

  /**
   * @brief 读取多个连续 512 字节块（比循环单块读快）
   * @param block_addr 起始块地址
   * @param count      块数
   * @param dst        目标缓冲区
   * @return kNone 成功
   */
  Error ReadBlocks(uint32_t block_addr, uint32_t count, uint8_t *dst);

  /**
   * @brief 写入多个连续 512 字节块
   * @param block_addr 起始块地址
   * @param count      块数
   * @param src        数据源
   * @return kNone 成功
   */
  Error WriteBlocks(uint32_t block_addr, uint32_t count, const uint8_t *src);

  // ─── 信息查询 ───────────────────────────────────────────────

  /** @brief 获取 SD 卡类型 */
  [[nodiscard]] Type GetType() const { return type_; }

  /** @brief 获取总块数（LBA，每块 512 字节） */
  [[nodiscard]] uint32_t GetBlockCount() const { return block_count_; }

  /** @brief 获取总容量 [MB] */
  [[nodiscard]] uint32_t GetCapacityMB() const { return block_count_ / 2048U; }

 private:
  // ── SPI 通信 ──
  void CsLow();
  void CsHigh();
  uint8_t SpiTxRx(uint8_t data);
  void SpiTx(const uint8_t *data, uint32_t size);
  void SpiRx(uint8_t *dst, uint32_t size);
  void SpiDummy(uint32_t count);

  // ── SD 命令 ──
  uint8_t SendCmd(uint8_t cmd, uint32_t arg);
  uint8_t SendCmd8(uint32_t arg);
  uint8_t SendAcmd41(uint32_t arg);
  Error WaitNotBusy(uint32_t timeout_ms);

  // ── 成员 ──
  rm::hal::stm32::Spi spi_;
  GPIO_TypeDef *cs_port_;
  uint16_t cs_pin_;

  bool ready_{false};
  Type type_{Type::kUnknown};
  uint32_t block_count_{0};

  // ── 常量 ──
  static constexpr uint8_t kCmd0 = 0x00;        ///< GO_IDLE_STATE
  static constexpr uint8_t kCmd1 = 0x01;        ///< SEND_OP_COND (MMC)
  static constexpr uint8_t kCmd8 = 0x08;        ///< SEND_IF_COND
  static constexpr uint8_t kCmd9 = 0x09;        ///< SEND_CSD
  static constexpr uint8_t kCmd10 = 0x0A;       ///< SEND_CID
  static constexpr uint8_t kCmd12 = 0x0C;       ///< STOP_TRANSMISSION
  static constexpr uint8_t kCmd16 = 0x10;       ///< SET_BLOCKLEN
  static constexpr uint8_t kCmd17 = 0x11;       ///< READ_SINGLE_BLOCK
  static constexpr uint8_t kCmd18 = 0x12;       ///< READ_MULTIPLE_BLOCK
  static constexpr uint8_t kCmd24 = 0x18;       ///< WRITE_BLOCK
  static constexpr uint8_t kCmd25 = 0x19;       ///< WRITE_MULTIPLE_BLOCK
  static constexpr uint8_t kCmd41 = 0x29;       ///< SD_SEND_OP_COND (ACMD41 prefix)
  static constexpr uint8_t kCmd55 = 0x37;       ///< APP_CMD
  static constexpr uint8_t kCmd58 = 0x3A;       ///< READ_OCR

  static constexpr uint8_t kR1Idle = 0x01;      ///< R1: IDLE 状态
  static constexpr uint8_t kR1Ready = 0x00;      ///< R1: 就绪
  static constexpr uint8_t kDataToken = 0xFE;    ///< 数据起始令牌
  static constexpr uint8_t kDataAccept = 0x05;   ///< 写入数据被接受
  static constexpr uint8_t kDataRejectCrc = 0x0B;///< 写入数据因 CRC 被拒绝
  static constexpr uint8_t kDataRejectWrite = 0x0D;///< 写入数据被拒绝
};
