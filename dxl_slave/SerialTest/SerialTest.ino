// Test that ring buffer overrun can be detected.
#include "avr_dxl_serial.h"
#include "util/delay.h"

void setup() {
  usart_init(207); // 9600 baud
  init_timer();
  DDRB |= _BV(PB7); // Set pin 13 (PB7)
}



void loop() {
}
