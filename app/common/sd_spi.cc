/**
 * @file  app/common/bsp/sd_spi.cc
 * @brief SD 卡 SPI 模式驱动实现（基于 librm + HAL）
 */

#include "sd_spi.hpp"
#include <cstring>

// ═══════════════════════════════════════════════════════════════
// 构造
// ═══════════════════════════════════════════════════════════════

SdSpi::SdSpi(SPI_HandleTypeDef &hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint32_t timeout_ms)
    : spi_(hspi, timeout_ms), cs_port_(cs_port), cs_pin_(cs_pin) {
  CsHigh();
}

// ═══════════════════════════════════════════════════════════════
// CS 片选控制
// ═══════════════════════════════════════════════════════════════

void SdSpi::CsLow() {
  HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
}

void SdSpi::CsHigh() {
  HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
}

// ═══════════════════════════════════════════════════════════════
// SPI 底层封装
// ═══════════════════════════════════════════════════════════════

uint8_t SdSpi::SpiTxRx(uint8_t data) {
  uint8_t rx;
  spi_.ReadWrite(&data, &rx, 1);
  return rx;
}

void SdSpi::SpiTx(const uint8_t *data, uint32_t size) {
  spi_.Write(data, size);
}

void SdSpi::SpiRx(uint8_t *dst, uint32_t size) {
  // 使用全双工发送 0xFF 确保 MOSI 在读取期间保持高电平，
  // 避免半双工 Receive 下 TXDR 残留值被误解析为 SD 命令。
  static constexpr uint32_t kChunkSize = 64;
  uint8_t dummy[kChunkSize];
  std::memset(dummy, 0xFF, kChunkSize);

  while (size > 0) {
    uint32_t chunk = (size > kChunkSize) ? kChunkSize : size;
    spi_.ReadWrite(dummy, dst, chunk);
    dst += chunk;
    size -= chunk;
  }
}

void SdSpi::SpiDummy(uint32_t count) {
  // 批量发送 0xFF，避免逐字节 HAL 调用。
  static constexpr uint32_t kChunkSize = 64;
  uint8_t buf[kChunkSize];
  std::memset(buf, 0xFF, kChunkSize);

  while (count > 0) {
    uint32_t chunk = (count > kChunkSize) ? kChunkSize : count;
    spi_.Write(buf, chunk);
    count -= chunk;
  }
}

// ═══════════════════════════════════════════════════════════════
// SD 命令发送
// ═══════════════════════════════════════════════════════════════

/**
 * @brief 发送 SD 命令（6 字节：0x40|cmd + 4字节arg + crc）
 * @return R1 响应字节
 */
uint8_t SdSpi::SendCmd(uint8_t cmd, uint32_t arg) {
  uint8_t tx[6];
  tx[0] = 0x40 | cmd;            // 命令码 + 起始位
  tx[1] = static_cast<uint8_t>(arg >> 24);
  tx[2] = static_cast<uint8_t>(arg >> 16);
  tx[3] = static_cast<uint8_t>(arg >> 8);
  tx[4] = static_cast<uint8_t>(arg);

  // CRC 仅对 CMD0（0x95）和 CMD8（0x87）有效，其他命令在 SPI 模式忽略
  if (cmd == kCmd0) {
    tx[5] = 0x95;
  } else if (cmd == kCmd8) {
    tx[5] = 0x87;
  } else {
    tx[5] = 0x01;  // dummy CRC
  }

  SpiTx(tx, 6);

  // 读 R1 响应：最多等 8 个字节（SD spec says up to 8 clocks）
  for (uint32_t retry = 0; retry < 8; ++retry) {
    uint8_t r1 = SpiTxRx(0xFF);
    if ((r1 & 0x80) == 0) {  // MSB=0 表示有效响应
      return r1;
    }
  }
  return 0xFF;  // 超时
}

/**
 * @brief 发送 CMD8（SEND_IF_COND），检测电压和 SDHC
 */
uint8_t SdSpi::SendCmd8(uint32_t arg) {
  return SendCmd(kCmd8, arg);
}

/**
 * @brief 发送 ACMD41（APP_CMD + SD_SEND_OP_COND）
 */
uint8_t SdSpi::SendAcmd41(uint32_t arg) {
  SendCmd(kCmd55, 0);  // CMD55: prefix for ACMD
  return SendCmd(kCmd41, arg);
}

// ═══════════════════════════════════════════════════════════════
// 初始化
// ═══════════════════════════════════════════════════════════════

SdSpi::Error SdSpi::Init() {
  ready_ = false;
  type_ = Type::kUnknown;
  block_count_ = 0;

  // ── 1. 上电延时：至少 74+ 时钟周期 = 10 字节 0xFF（SPI 低速 ≤400kHz）──
  CsHigh();
  SpiDummy(10);

  // ── 2. CMD0：复位到 IDLE ──
  CsLow();
  uint32_t t0 = HAL_GetTick();
  uint8_t r1;
  do {
    r1 = SendCmd(kCmd0, 0);
    if (HAL_GetTick() - t0 > 500) {
      CsHigh();
      return Error::kTimeout;
    }
  } while (r1 != kR1Idle);  // 应返回 0x01 (IDLE state)
  CsHigh();

  // ── 3. CMD8：检测电压范围 + SDHC 探测 ──
  CsLow();
  r1 = SendCmd8(0x000001AA);  // 2.7~3.6V, check pattern 0xAA
  // CMD8 有响应且无非法命令错误 → SD V2 卡
  // V1 卡会返回 illegal command (bit 2)，需排除。
  bool is_v2 = (r1 != 0xFF) && ((r1 & 0x04) == 0);

  if (is_v2) {
    // 读 CMD8 R7 响应的 4 字节
    uint8_t r7[4];
    SpiRx(r7, 4);
    if ((r7[2] & 0x01) == 0 || r7[3] != 0xAA) {
      CsHigh();
      return Error::kInitFailed;  // 电压不匹配或 check pattern 不对
    }
  }
  CsHigh();

  // ── 4. ACMD41：等待卡就绪 ──
  bool is_hc = false;
  uint32_t acmd41_timeout = is_v2 ? 1000U : 2000U;  // V2 超时短一些
  t0 = HAL_GetTick();
  do {
    CsLow();
    r1 = SendAcmd41(is_v2 ? 0x40000000U : 0x00000000U);  // HCS=1 for SDHC
    if (is_v2 && r1 <= 1) {
      // 读取 OCR（CMD58）获取 CCS 位，使用独立变量避免覆盖 ACMD41 的 r1
      CsLow();
      uint8_t r1_ocr = SendCmd(kCmd58, 0);
      if (r1_ocr <= 1) {
        uint8_t ocr[4];
        SpiRx(ocr, 4);
        is_hc = (ocr[0] & 0x40) != 0;  // CCS bit (bit 30)
      }
      CsHigh();
    }
    CsHigh();

    if (HAL_GetTick() - t0 > acmd41_timeout) {
      return Error::kTimeout;
    }
  } while (r1 != kR1Ready);

  // ── 5. 确定卡类型 ──
  if (is_v2) {
    type_ = is_hc ? Type::kSdhc : Type::kSdV2;
  } else {
    type_ = Type::kSdV1;
  }

  // ── 6. 获取卡容量（读 CSD）──
  CsLow();
  r1 = SendCmd(kCmd9, 0);  // SEND_CSD
  if (r1 != kR1Ready) {
    CsHigh();
    return Error::kInitFailed;
  }

  // 等待数据令牌 0xFE
  t0 = HAL_GetTick();
  uint8_t token;
  do {
    token = SpiTxRx(0xFF);
    if (HAL_GetTick() - t0 > 200) {
      CsHigh();
      return Error::kTimeout;
    }
  } while (token != kDataToken);

  // 读 16 字节 CSD
  uint8_t csd[16];
  SpiRx(csd, 16);
  // 跳过 2 字节 CRC
  SpiTxRx(0xFF);
  SpiTxRx(0xFF);
  CsHigh();

  // 解析 CSD 获取容量
  uint8_t csd_ver = (csd[0] >> 6) & 0x03;  // CSD_STRUCTURE
  if (csd_ver == 0) {
    // CSD V1.0 (SDSC)
    uint32_t c_size = ((static_cast<uint32_t>(csd[6]) & 0x03) << 10) |
                      (static_cast<uint32_t>(csd[7]) << 2) |
                      ((static_cast<uint32_t>(csd[8]) >> 6) & 0x03);
    uint32_t c_size_mult = ((static_cast<uint32_t>(csd[9]) & 0x03) << 1) |
                            ((static_cast<uint32_t>(csd[10]) >> 7) & 0x01);
    uint32_t read_bl_len = csd[5] & 0x0F;
    block_count_ = (c_size + 1) << (c_size_mult + read_bl_len - 7);
  } else if (csd_ver == 1) {
    // CSD V2.0 (SDHC/SDXC)
    uint32_t c_size = (static_cast<uint32_t>(csd[7]) & 0x3F) << 16 |
                      (static_cast<uint32_t>(csd[8]) << 8) |
                      static_cast<uint32_t>(csd[9]);
    block_count_ = (c_size + 1) << 10;  // 每块 512 字节
  } else {
    return Error::kInitFailed;
  }

  // ── 7. 设置块大小为 512 字节（非 SDHC 卡需要）──
  if (type_ != Type::kSdhc) {
    CsLow();
    r1 = SendCmd(kCmd16, 512);  // SET_BLOCKLEN
    CsHigh();
    if (r1 != kR1Ready) {
      return Error::kInitFailed;
    }
  }

  ready_ = true;
  return Error::kNone;
}

// ═══════════════════════════════════════════════════════════════
// 忙等
// ═══════════════════════════════════════════════════════════════

SdSpi::Error SdSpi::WaitNotBusy(uint32_t timeout_ms) {
  uint32_t t0 = HAL_GetTick();
  while (SpiTxRx(0xFF) != 0xFF) {
    if (HAL_GetTick() - t0 > timeout_ms) {
      return Error::kTimeout;
    }
  }
  return Error::kNone;
}

// ═══════════════════════════════════════════════════════════════
// 单块读（CMD17）
// ═══════════════════════════════════════════════════════════════

SdSpi::Error SdSpi::ReadBlock(uint32_t block_addr, uint8_t *dst) {
  if (!ready_) return Error::kNotInitialized;

  // SDSC 使用字节地址，SDHC 使用块地址
  uint32_t addr = (type_ == Type::kSdhc) ? block_addr : (block_addr * 512);

  CsLow();
  uint8_t r1 = SendCmd(kCmd17, addr);
  if (r1 != kR1Ready) {
    CsHigh();
    return Error::kReadFailed;
  }

  // 等待数据令牌 0xFE（超时 200ms）
  uint32_t t0 = HAL_GetTick();
  uint8_t token;
  do {
    token = SpiTxRx(0xFF);
    if (HAL_GetTick() - t0 > 200) {
      CsHigh();
      return Error::kTimeout;
    }
  } while (token == 0xFF);  // wait for non-FF

  if (token != kDataToken) {
    CsHigh();
    return Error::kReadFailed;
  }

  // 读 512 字节数据 + 2 字节 CRC
  uint8_t crc[2];
  SpiRx(dst, 512);
  SpiRx(crc, 2);

  CsHigh();
  return Error::kNone;
}

// ═══════════════════════════════════════════════════════════════
// 单块写（CMD24）
// ═══════════════════════════════════════════════════════════════

SdSpi::Error SdSpi::WriteBlock(uint32_t block_addr, const uint8_t *src) {
  if (!ready_) return Error::kNotInitialized;

  uint32_t addr = (type_ == Type::kSdhc) ? block_addr : (block_addr * 512);

  CsLow();
  uint8_t r1 = SendCmd(kCmd24, addr);
  if (r1 != kR1Ready) {
    CsHigh();
    return Error::kWriteFailed;
  }

  // 发送数据起始令牌
  SpiTxRx(kDataToken);

  // 发送 512 字节数据
  SpiTx(src, 512);

  // 发送 dummy CRC (SPI 模式 CRC 通常被忽略)
  SpiTxRx(0xFF);
  SpiTxRx(0xFF);

  // 读数据响应令牌（低 5 位有效）
  uint8_t resp = SpiTxRx(0xFF);
  if ((resp & 0x1F) != kDataAccept) {
    CsHigh();
    return Error::kWriteFailed;
  }

  // 等待写入完成（卡忙时输出 0x00）
  Error err = WaitNotBusy(500);
  CsHigh();

  return err;
}

// ═══════════════════════════════════════════════════════════════
// 多块读（CMD18）
// ═══════════════════════════════════════════════════════════════

SdSpi::Error SdSpi::ReadBlocks(uint32_t block_addr, uint32_t count, uint8_t *dst) {
  if (!ready_) return Error::kNotInitialized;
  if (count == 0) return Error::kNone;
  if (count == 1) return ReadBlock(block_addr, dst);

  uint32_t addr = (type_ == Type::kSdhc) ? block_addr : (block_addr * 512);

  CsLow();
  uint8_t r1 = SendCmd(kCmd18, addr);  // READ_MULTIPLE_BLOCK
  if (r1 != kR1Ready) {
    CsHigh();
    return Error::kReadFailed;
  }

  for (uint32_t i = 0; i < count; ++i) {
    // 等待数据令牌
    uint32_t t0 = HAL_GetTick();
    uint8_t token;
    do {
      token = SpiTxRx(0xFF);
      if (HAL_GetTick() - t0 > 200) {
        SendCmd(kCmd12, 0);  // STOP_TRANSMISSION
        CsHigh();
        return Error::kTimeout;
      }
    } while (token == 0xFF);

    if (token != kDataToken) {
      SendCmd(kCmd12, 0);
      CsHigh();
      return Error::kReadFailed;
    }

    // 读 512 字节 + 2 字节 CRC
    uint8_t crc[2];
    SpiRx(dst + i * 512, 512);
    SpiRx(crc, 2);
  }

  // 发送 STOP_TRANSMISSION 停止多块读
  SendCmd(kCmd12, 0);
  // 等待卡完成内部操作
  SpiTxRx(0xFF);
  CsHigh();

  return Error::kNone;
}

// ═══════════════════════════════════════════════════════════════
// 多块写（CMD25）
// ═══════════════════════════════════════════════════════════════

SdSpi::Error SdSpi::WriteBlocks(uint32_t block_addr, uint32_t count, const uint8_t *src) {
  if (!ready_) return Error::kNotInitialized;
  if (count == 0) return Error::kNone;
  if (count == 1) return WriteBlock(block_addr, src);

  uint32_t addr = (type_ == Type::kSdhc) ? block_addr : (block_addr * 512);

  CsLow();
  uint8_t r1 = SendCmd(kCmd25, addr);  // WRITE_MULTIPLE_BLOCK
  if (r1 != kR1Ready) {
    CsHigh();
    return Error::kWriteFailed;
  }

  for (uint32_t i = 0; i < count; ++i) {
    // 发送起始令牌
    SpiTxRx(0xFC);  // 多块写使用 0xFC 作为起始令牌

    // 发送 512 字节
    SpiTx(src + i * 512, 512);

    // dummy CRC
    SpiTxRx(0xFF);
    SpiTxRx(0xFF);

    // 读响应
    uint8_t resp = SpiTxRx(0xFF);
    if ((resp & 0x1F) != kDataAccept) {
      CsHigh();
      return Error::kWriteFailed;
    }

    // 等待写入完成
    if (WaitNotBusy(500) != Error::kNone) {
      CsHigh();
      return Error::kTimeout;
    }
  }

  // 发送停止传输令牌
  SpiTxRx(0xFD);  // STOP_TRAN token for multi-block write

  // 等待卡回到就绪
  SpiTxRx(0xFF);
  CsHigh();

  return Error::kNone;
}
