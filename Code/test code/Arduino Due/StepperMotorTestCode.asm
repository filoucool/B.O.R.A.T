; Constants
.equ DDRB, 0x24
.equ PORTB, 0x25
.equ ADMUX, 0x7C
.equ ADCSRA, 0x7A
.equ ADCL, 0x78
.equ ADCH, 0x79
.equ ADPS2, 2
.equ ADPS1, 1
.equ ADPS0, 0
.equ ADEN, 7
.equ ADSC, 6
.equ ADIF, 4

.equ STEPS_PER_REVOLUTION, 200
.equ REVS_DIVISOR, 100

; Variables
.org 0x0100
stepCount: .byte 1

; Reset vector
.org 0x0000
  rjmp init

; Main program initialization
init:
  ; Set pins 8 through 11 (PB0 - PB3) as output
  ldi r16, 0x0F
  out DDRB, r16

  ; Initialize ADC (A0)
  ldi r16, 0x00 ; A0 = 0
  out ADMUX, r16
  ldi r16, (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0)
  out ADCSRA, r16

main_loop:
  ; Read the sensor value from A0
  sbi ADCSRA, ADSC
  in r16, ADCSRA
  sbrs r16, ADIF
  rjmp main_loop

  ; Read ADC result
  in r17, ADCL
  in r18, ADCH

  ; Reset ADIF flag
  sbi ADCSRA, ADIF

  ; Map it to a range from 0 to 100
  ; int motorSpeed = (sensorReading * 100) / 1023
  ldi r19, 100
  mul r18, r19
  ldi r19, 1023
  div r17, r19

  ; Set the motor speed and step 1/100 of a revolution
  ; if (motorSpeed > 0)
  cpi r17, 0
  brle main_loop

  ; Perform 1/100 of a revolution
  ldi r20, STEPS_PER_REVOLUTION / REVS_DIVISOR
step_loop:
  ; Step the motor
  call step_motor

  ; Decrement step count
  dec r20
  brne step_loop

  rjmp main_loop
