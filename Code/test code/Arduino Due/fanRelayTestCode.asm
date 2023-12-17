; Constants
.equ relayPin = 40
.equ toggleInterval = 2000
.equ F_CPU = 16000000 ; 16MHz clock

; Registers
.def temp = r16
.def loopCounter = r17
.def delayCounter1 = r18
.def delayCounter2 = r19
.def delayCounter3 = r20

; Initialization
rjmp init

; Set pin mode to output
init:
    ldi temp, (1 << DDB5) ; Pin 40 is PD5
    sts DDRD, temp

; Main loop
loop:
    ; Turn relay ON (HIGH)
    lds temp, PORTD
    ori temp, (1 << PORTD5)
    sts PORTD, temp

    ; Delay
    ldi delayCounter1, low(toggleInterval * 2) ; Toggle interval in milliseconds
    ldi delayCounter2, high(toggleInterval * 2)
    rcall delay_ms

    ; Turn relay OFF (LOW)
    lds temp, PORTD
    andi temp, ~(1 << PORTD5)
    sts PORTD, temp

    ; Delay
    rcall delay_ms

    rjmp loop

; Delay function
; Input: delayCounter1 (low byte), delayCounter2 (high byte)
delay_ms:
    ldi loopCounter, 125
delay_ms_loop:
    push delayCounter1
    push delayCounter2

    ; delayCounter1 * delayCounter2 * loopCounter
    ldi delayCounter3, 4 ; ((F_CPU / 1000) / 4) / 256 = 16
delay_inner_loop:
    dec delayCounter3
    brne delay_inner_loop

    pop delayCounter2
    pop delayCounter1
    sbiw delayCounter1, 1
    brne delay_ms_loop
    dec loopCounter
    brne delay_ms_loop
    ret
