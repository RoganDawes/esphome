#if defined ARDUINO_ARCH_STM32
#include "Arduino.h"
#include "uart.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/application.h"
#include "esphome/core/defines.h"

namespace esphome {
namespace uart {
static const char *TAG = "uart_stm32";

uint32_t UARTComponent::get_config() {
  /*
   * All bits numbers below come from
   * Arduino_Core_STM32/cores/arduino/HardwareSerial.h
   */

  if (this->data_bits_ == 8 && this->parity_ == UART_CONFIG_PARITY_NONE && this->stop_bits_ == 1) {
    return SERIAL_8N1;
  } else if (this->data_bits_ == 8 && this->parity_ == UART_CONFIG_PARITY_NONE && this->stop_bits_ == 2) {
    return SERIAL_8N2;
  } else if (this->data_bits_ == 7 && this->parity_ == UART_CONFIG_PARITY_EVEN && this->stop_bits_ == 1) {
    return SERIAL_7E1;
  } else if (this->data_bits_ == 8 && this->parity_ == UART_CONFIG_PARITY_EVEN && this->stop_bits_ == 1) {
    return SERIAL_8E1;
  } else if (this->data_bits_ == 7 && this->parity_ == UART_CONFIG_PARITY_EVEN && this->stop_bits_ == 2) {
    return SERIAL_7E2;
  } else if (this->data_bits_ == 8 && this->parity_ == UART_CONFIG_PARITY_EVEN && this->stop_bits_ == 2) {
    return SERIAL_8E2;
  } else if (this->data_bits_ == 7 && this->parity_ == UART_CONFIG_PARITY_ODD && this->stop_bits_ == 1) {
    return SERIAL_7O1;
  } else if (this->data_bits_ == 8 && this->parity_ == UART_CONFIG_PARITY_ODD && this->stop_bits_ == 1) {
    return SERIAL_8O1;
  } else if (this->data_bits_ == 7 && this->parity_ == UART_CONFIG_PARITY_ODD && this->stop_bits_ == 2) {
    return SERIAL_7O2;
  } else if (this->data_bits_ == 8 && this->parity_ == UART_CONFIG_PARITY_ODD && this->stop_bits_ == 2) {
    return SERIAL_8O2;
  }
  return 0;
}

void UARTComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up UART...");
  // Use Arduino HardwareSerial UARTs if all used pins match the ones
  // preconfigured by the platform. For example if RX disabled but TX pin
  // is 1 we still want to use Serial.
  // FIXME: Deal with the case where USB Serial is not enabled?
  int8_t tx = this->tx_pin_.has_value() ? *this->tx_pin_ : -1;
  int8_t rx = this->rx_pin_.has_value() ? *this->rx_pin_ : -1;
  if (tx == 0 && rx == 0) {
    this->usb_serial_ = &Serial;
  } else {
    this->hw_serial_ = new HardwareSerial(rx , tx);
    this->hw_serial_->begin(this->baud_rate_, get_config());
  }
  // Define alternate pins if needed according to:
  // https://github.com/stm32duino/wiki/wiki/API#hardwareserial
  //
  // Receive buffer can be defined using a build-time flag:
  // build_flags = -D SERIAL_RX_BUFFER_SIZE=256
  // this->hw_serial_->setRxBufferSize(this->rx_buffer_size_);
  ESP_LOGCONFIG(TAG, "Done setting up UART...");
}

void UARTComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "UART Bus:");
  if (this->tx_pin_.has_value()) {
    ESP_LOGCONFIG(TAG, "  TX Pin: GPIO%d", *this->tx_pin_);
  }
  if (this->rx_pin_.has_value()) {
    ESP_LOGCONFIG(TAG, "  RX Pin: GPIO%d", *this->rx_pin_);
    ESP_LOGCONFIG(TAG, "  RX Buffer Size: %u", this->rx_buffer_size_);
  }
  ESP_LOGCONFIG(TAG, "  Baud Rate: %u baud", this->baud_rate_);
  ESP_LOGCONFIG(TAG, "  Data Bits: %u", this->data_bits_);
  ESP_LOGCONFIG(TAG, "  Parity: %s", parity_to_str(this->parity_));
  ESP_LOGCONFIG(TAG, "  Stop bits: %u", this->stop_bits_);
  this->check_logger_conflict_();
}

void UARTComponent::write_byte(uint8_t data) {
  if (this->hw_serial_ != nullptr)
    this->hw_serial_->write(data);
  else if (this->usb_serial_ != nullptr)
    this->usb_serial_->write(data);
  else
    return;
  ESP_LOGVV(TAG, "    Wrote 0b" BYTE_TO_BINARY_PATTERN " (0x%02X)", BYTE_TO_BINARY(data), data);
}
void UARTComponent::write_array(const uint8_t *data, size_t len) {
  if (this->hw_serial_ != nullptr)
    this->hw_serial_->write(data, len);
  else if (this->usb_serial_ != nullptr)
    this->usb_serial_->write(data, len);
  else
    return;
  for (size_t i = 0; i < len; i++) {
    ESP_LOGVV(TAG, "    Wrote 0b" BYTE_TO_BINARY_PATTERN " (0x%02X)", BYTE_TO_BINARY(data[i]), data[i]);
  }
}
void UARTComponent::write_str(const char *str) {
  Serial.print("in write_str ["); Serial.print(str); Serial.println("]");
  if (this->hw_serial_ != nullptr)
    this->hw_serial_->write(str);
  else if (this->usb_serial_ != nullptr)
    this->usb_serial_->write(str);
  else
    return;
  ESP_LOGVV(TAG, "    Wrote \"%s\"", str);
}
bool UARTComponent::read_byte(uint8_t *data) {
  if (!this->check_read_timeout_())
    return false;
  if (this->hw_serial_ != nullptr)
    *data = this->hw_serial_->read();
  else if (this->usb_serial_ != nullptr)
    *data = this->usb_serial_->read();
  else
    return false;
  ESP_LOGVV(TAG, "    Read 0b" BYTE_TO_BINARY_PATTERN " (0x%02X)", BYTE_TO_BINARY(*data), *data);
  return true;
}
bool UARTComponent::peek_byte(uint8_t *data) {
  if (!this->check_read_timeout_())
    return false;
  if (this->hw_serial_ != nullptr)
    *data = this->hw_serial_->peek();
  else if (this->usb_serial_ != nullptr)
    *data = this->usb_serial_->peek();
  else
    return false;
  return true;
}
bool UARTComponent::read_array(uint8_t *data, size_t len) {
  if (!this->check_read_timeout_(len))
    return false;
  if (this->hw_serial_ != nullptr)
    this->hw_serial_->readBytes(data, len);
  else if (this->usb_serial_ != nullptr)
    this->usb_serial_->readBytes((char *)data, len);
  else
    return false;
  for (size_t i = 0; i < len; i++) {
    ESP_LOGVV(TAG, "    Read 0b" BYTE_TO_BINARY_PATTERN " (0x%02X)", BYTE_TO_BINARY(data[i]), data[i]);
  }

  return true;
}
bool UARTComponent::check_read_timeout_(size_t len) {
  if (this->available() >= len)
    return true;

  uint32_t start_time = millis();
  while (this->available() < len) {
    if (millis() - start_time > 1000) {
      ESP_LOGE(TAG, "Reading from UART timed out at byte %u!", this->available());
      return false;
    }
    yield();
  }
  return true;
}
int UARTComponent::available() { 
  if (this->hw_serial_ != nullptr)
    return this->hw_serial_->available();
  else if (this->usb_serial_ != nullptr)
    return this->usb_serial_->available();
  else
    return false;
}
void UARTComponent::flush() {
  if (this->hw_serial_ != nullptr)
    this->hw_serial_->flush();
  else if (this->usb_serial_ != nullptr)
    this->usb_serial_->flush();
  else
    return;
  ESP_LOGVV(TAG, "    Flushing...");
}

}  // namespace uart
}  // namespace esphome
#endif  // ARDUINO_ARCH_STM32
