#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Rotary encoder pins
#define ENCODER_A PD0
#define ENCODER_B PD1

// Stepper motor pins
#define STEPPER_A PB0
#define STEPPER_B PB1
#define STEPPER_C PB2
#define STEPPER_D PB3

volatile int16_t encoder_position = 0;
volatile int16_t target_position = 0;
volatile uint8_t last_encoder_state = 0;

const uint8_t stepper_sequence[] = { 0b0001, 0b0010, 0b0100, 0b1000 };
uint8_t stepper_index = 0;

void initialize_encoder() {
    // Set encoder pins as input
    DDRD &= ~((1 << ENCODER_A) | (1 << ENCODER_B));
    // Enable internal pull-ups for encoder pins
    PORTD |= (1 << ENCODER_A) | (1 << ENCODER_B);

    // Enable pin change interrupts for encoder pins
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT16) | (1 << PCINT17);
}

void initialize_stepper() {
    // Set stepper motor pins as output
    DDRB |= (1 << STEPPER_A) | (1 << STEPPER_B) | (1 << STEPPER_C) | (1 << STEPPER_D);
}

void set_stepper_output(uint8_t output) {
    PORTB = (PORTB & 0xF0) | (output & 0x0F);
}

void step_forward() {
    stepper_index = (stepper_index + 1) % 4;
    set_stepper_output(stepper_sequence[stepper_index]);
    _delay_ms(1);
}

void step_backward() {
    stepper_index = (stepper_index - 1) % 4;
    set_stepper_output(stepper_sequence[stepper_index]);
    _delay_ms(1);
}

ISR(PCINT2_vect) {
    uint8_t encoder_state = ((PIND >> ENCODER_A) & 1) | (((PIND >> ENCODER_B) & 1) << 1);

    if (encoder_state == ((last_encoder_state + 1) % 4)) {
        encoder_position++;
    } else if (encoder_state == ((last_encoder_state - 1) % 4)) {
        encoder_position--;
    }

    target_position = encoder_position;
    last_encoder_state = encoder_state;
}

int main() {
    initialize_encoder();
    initialize_stepper();
    sei(); // Enable global interrupts

    while (1) {
        if (encoder_position < target_position) {
            step_forward();
            encoder_position++;
        } else if (encoder_position > target_position) {
            step_backward();
            encoder_position--;
        }
    }

    return 0;
}
