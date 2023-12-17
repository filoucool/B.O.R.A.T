; Constants
.equ DDRB, 0x24
.equ PORTB, 0x25
.equ OCR1A, 0x88
.equ TCCR1A, 0x80
.equ TCCR1B, 0x81
.equ WGM11, 1
.equ WGM12, 3
.equ WGM13, 4
.equ COM1A1, 7
.equ CS11, 1
.equ DELAY_COUNT, 24000 ; ~15 ms delay at 16 MHz

; Variables
.org 0x0100
pos: .byte 1

; Reset vector
.org 0x0000
  rjmp init

; Main program initialization
init:
  ; Set PB1 as output (pin 9 on Arduino)
  sbi DDRB, 1

  ; Initialize Timer1 for Phase Correct PWM mode
  ldi r16, (1 << COM1A1) | (1 << WGM11)
  out TCCR1A, r16
  ldi r16, (1 << WGM13) | (1 << WGM12) | (1 << CS11)
  out TCCR1B, r16
  ldi r16, 0x1F
  out OCR1A, r16

main_loop:
  ; Sweep servo from 0 to 180 degrees
  ldi r26, 0
  ldi r27, 0
  ldi ZL, pos
  ldi ZH, 0
  st Z, r26

sweep_forward:
  ld r16, Z
  ldi r17, 180
  cp r16, r17
  brge sweep_backward

  ; Set servo position
  add r16, r26
  ldi r17, 544
  add r16, r17
  out OCR1A, r16

  ; Delay ~15 ms
  call delay_ms

  ; Increment position
  ld r16, Z
  inc r16
  st Z, r16
  rjmp sweep_forward

  ; Sweep servo from 180 to 0 degrees
sweep_backward:
  ld r16, Z
  cpi r16, 0
  brlt main_loop

  ; Set servo position
  sub r16, r26
  ldi r17, 544
  add r16, r17
  out OCR1A, r16

  ; Delay ~15 ms
  call delay_ms

  ; Decrement position
  ld r16, Z
  dec r16
  st Z, r16
  rjmp sweep_backward

; Delay for approximately 15 ms
delay_ms:
  ldi r18, DELAY_COUNT & 0xFF
  ldi r19, DELAY_COUNT >> 8

delay_loop:
  sbiw r18, 1
  brne delay_loop
  ret
