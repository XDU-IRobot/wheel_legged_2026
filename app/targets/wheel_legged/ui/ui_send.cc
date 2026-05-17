//
// C-to-C++ bridge for referee_uart.WriteAsync(). Used by SEND_MESSAGE from C code.
//

#include "../include/globals_no_dtcm.hpp"

extern "C" {
void ui_send_message(const uint8_t *data, int len) {
  if (!globals_no_dtcm.referee_uart.IsTxBusy()) {
    globals_no_dtcm.referee_uart.WriteAsync(data, static_cast<rm::usize>(len), nullptr);
  }
}
}
