; Define constants
.equ DDRB, 0x24
.equ PORTB, 0x25
.equ TCNT1, 0x84
.equ TCCR1B, 0x81
.equ CS12, 3
.equ CS10, 1

; Reset vector
.org 0x0000
  rjmp init

; Main program initialization
init:
  ; Set PB0 as output (pin 14 on Arduino)
  sbi DDRB, 0

main_loop:
  ; Toggle PB0 (pin 14 on Arduino)
  sbi PORTB, 0

  ; Setup Timer/Counter1
  ldi r16, (1<<CS12) | (1<<CS10)
  out TCCR1B, r16

delay_1s:
  ; Wait for Timer/Counter1 to overflow
  in r16, TCNT1
  cpi r16, 0xFF
  brne delay_1s

  ; Reset Timer/Counter1
  ldi r16, 0x00
  out TCNT1, r16

  ; Toggle PB0 (pin 14 on Arduino)
  cbi PORTB, 0

delay_1s_2:
  ; Wait for Timer/Counter1 to overflow
  in r16, TCNT1
  cpi r16, 0xFF
  brne delay_1s_2

  ; Reset Timer/Counter1
  ldi r16, 0x00
  out TCNT1, r16

  ; Repeat main loop
  rjmp main_loop
