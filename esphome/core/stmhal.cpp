#if defined ARDUINO_ARCH_STM32

#include "esphome/core/esphal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/defines.h"
#include "esphome/core/log.h"

namespace esphome {

static const char *const TAG = "stmhal";

GPIOPin::GPIOPin(uint8_t pin, uint8_t mode, bool inverted)
    : pin_(pin),
      mode_(mode),
      inverted_(inverted),
      gpio_read_(0),
      gpio_mask_(0)
{
}

const char *GPIOPin::get_pin_mode_name() const {
  const char *mode_s;
  switch (this->mode_) {
    case INPUT:
      mode_s = "INPUT";
      break;
    case OUTPUT:
      mode_s = "OUTPUT";
      break;
    case INPUT_PULLUP:
      mode_s = "INPUT_PULLUP";
      break;
    case OUTPUT_OPEN_DRAIN:
      mode_s = "OUTPUT_OPEN_DRAIN";
      break;

    default:
      mode_s = "UNKNOWN";
      break;
  }

  return mode_s;
}

unsigned char GPIOPin::get_pin() const { return this->pin_; }
unsigned char GPIOPin::get_mode() const { return this->mode_; }

bool GPIOPin::is_inverted() const { return this->inverted_; }
void GPIOPin::setup() { this->pin_mode(this->mode_); }
bool ICACHE_RAM_ATTR HOT GPIOPin::digital_read() {
  return digitalRead(this->pin_) != this->inverted_;
}
bool ICACHE_RAM_ATTR HOT ISRInternalGPIOPin::digital_read() {
  return digitalRead(this->pin_) != this->inverted_;
}
void ICACHE_RAM_ATTR HOT GPIOPin::digital_write(bool value) {
  digitalWrite(this->pin_, value != this->inverted_);
}
void ICACHE_RAM_ATTR HOT ISRInternalGPIOPin::digital_write(bool value) {
  digitalWrite(this->pin_, value != this->inverted_);
}
ISRInternalGPIOPin::ISRInternalGPIOPin(uint8_t pin,
                                       volatile uint32_t *gpio_read, uint32_t gpio_mask, bool inverted)
    : pin_(pin),
      inverted_(inverted),
      gpio_read_(gpio_read),
      gpio_mask_(gpio_mask)
{
}
void ICACHE_RAM_ATTR ISRInternalGPIOPin::clear_interrupt() {
}

void ICACHE_RAM_ATTR HOT GPIOPin::pin_mode(uint8_t mode) {
  pinMode(this->pin_, mode);
}

void GPIOPin::detach_interrupt() const { this->detach_interrupt_(); }
void GPIOPin::detach_interrupt_() const {
  detachInterrupt(get_pin());
}
void GPIOPin::attach_interrupt_(void (*func)(void *), void *arg, int mode) const {
  if (this->inverted_) {
    if (mode == RISING) {
      mode = FALLING;
    } else if (mode == FALLING) {
      mode = RISING;
    }
  }
  attachInterrupt(digitalPinToInterrupt(this->pin_), std::bind(func, arg), mode);
}

ISRInternalGPIOPin *GPIOPin::to_isr() const {
  return new ISRInternalGPIOPin(this->pin_,
                                this->gpio_read_, this->gpio_mask_, this->inverted_);
}

void force_link_symbols() {
}

}  // namespace esphome

#endif
