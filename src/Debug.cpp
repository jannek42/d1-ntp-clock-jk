//
// Output a debug-message if DEBUG is defined.
//
#include <Arduino.h>
#include "Debug.h"

void DebugClass::to_serial(const char *format, ...) {
  if (!this->_enabled) return;
  va_list arguments;
  va_start(arguments, format);
  vsnprintf(this->_buffer, sizeof(this->_buffer), format, arguments);
  Serial.print(this->_buffer);
  va_end(arguments);
}

void DebugClass::to_serial_ln(const char *format, ...) {
  if (!this->_enabled) return;
  va_list arguments;
  va_start(arguments, format);
  vsnprintf(this->_buffer, sizeof(this->_buffer), format, arguments);
  Serial.print(this->_buffer);
  Serial.print("\r\n");
  va_end(arguments);
}

bool DebugClass::is_enabled(void) { return this->_enabled; }
void DebugClass::set(bool state)         { this->_enabled = state; }
void DebugClass::enable(void)            { this->_enabled = true;  }
void DebugClass::disable(void)           { this->_enabled = false; }

