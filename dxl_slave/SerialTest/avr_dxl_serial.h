#ifndef AVR_DXL_SERIAL_H_H
#define AVR_DXL_SERIAL_H_H

#include <avr/io.h>
#include <avr/interrupt.h>

/**
 * @class usart_register_t
 * @brief Addresses of USART registers.
 */
typedef struct {
  volatile uint8_t* ucsra;  /**< USART Control and Status Register A. */
  volatile uint8_t* ucsrb;  /**< USART Control and Status Register B. */
  volatile uint8_t* ucsrc;  /**< USART Control and Status Register C. */
  volatile uint8_t* ubrrl;  /**< USART Baud Rate Register Low. */
  volatile uint8_t* ubrrh;  /**< USART Baud Rate Register High. */
  volatile uint8_t* udr;    /**< USART I/O Data Register. */
} usart_register_t;

/**
 * @class timer_register_t
 * @brief Addresses of 8 bit timer registers.
 */
typedef struct {
  volatile uint8_t* tcnt;  /**< Timer/Counter Register. */
  volatile uint8_t* tccra; /**< Timer/Counter Control Register A. */
  volatile uint8_t* tccrb; /**< Timer/Counter Control Register B. */
  volatile uint8_t* ocra;  /**< Compare Match Register A. */
  volatile uint8_t* ocrb;  /**< Compare Match Register B. */
  volatile uint8_t* timsk; /**< Timer/Counter Interrupt Mask Register. */
} timer_register_t;

// USART Registers for USB serial (USART0) on Arduino Mega
static const usart_register_t usart = {&UCSR0A, &UCSR0B, &UCSR0C, &UBRR0L, &UBRR0H, &UDR0};

// Timer 2 Registers
static const timer_register_t timer = {&TCNT2, &TCCR2A, &TCCR2B, &OCR2A, &OCR2B, &TIMSK2};

uint8_t dxl_tx_buff[32];
uint8_t dxl_tx_size = 0U;
uint8_t dxl_tx_idx = 0U;

void usart_write(uint8_t size);

ISR(USART0_RX_vect)
{
  uint8_t data = *usart.udr;
  if (data==97)
  {
    dxl_tx_buff[0] = data+1;
    dxl_tx_buff[1] = data+2;
    usart_write(2U);
  }

}

ISR(USART0_UDRE_vect)
{
  // Send data
  if (dxl_tx_idx < dxl_tx_size)
  {
    *usart.udr = dxl_tx_buff[dxl_tx_idx++];
  }
  else
  {
    // Disable interrupt
    *usart.ucsrb &= ~(1 << UDRIE0);
  }
}

void usart_init(uint8_t baud)
{
  // Disable USART interrupts. Set UCSRB to reset values.
  *usart.ucsrb = 0U;

  // Dynamixel serial use 8 bit character size, one stop bit, none parity
  // Only need set USART character size, others are default
  *usart.ucsrc = 1U << UCSZ00 | 1U << UCSZ01;

  // Config high speed mode
  *usart.ucsra = 1U << U2X0;

  // Assign the baud_setting (USART Baud Rate Register)
  *usart.ubrrh = 0U;
  *usart.ubrrl = baud;

  // Enable RX, TX and Receive Complete Interrupt
  *usart.ucsrb = 1U << TXEN0 | 1U << RXEN0 | 1U << RXCIE0;
}

void usart_write(uint8_t data_size)
{
  dxl_tx_size = data_size;
  dxl_tx_idx = 0U;

  // Enable timer 2
  *timer.tcnt = 0; // Reset timer 2 count
  *timer.tccrb |= (1 << CS21) | (1 << CS20) | (1 << WGM22); // Set prescaler for timer 2 at clk/64
  *timer.ocra = 125; // Compare register value, 500 us
  *timer.timsk |= (1 << OCIE2A);  // Enable timer 2 compare match interrupt
}

void init_timer()
{
  // Reset configuration registers for timer 2
  *timer.tccra = 0;
  *timer.tccrb = 0;
}

// Timer 2  overflow interrupt
ISR(TIMER2_COMPA_vect)
{
  *timer.tccrb = 0; // Reset prescaler for timer 2
  *timer.timsk &= ~(1 << OCIE2A);   // Disable timer 2 overflow interrupt
  
  PORTB ^= _BV(PB7); // Toggle LED pin
  // Enable USART interrupts
  *usart.ucsrb |= 1 << UDRIE0;
}

#endif //AVR_DXL_SERIAL_H_H
