#include <avr/io.h>
#include <util/twi.h>
#include <avr/interrupt.h>

#define SLAVE_ADDR 0x12

volatile uint8_t data = 0;

ISR(TWI_vect) {
	if ((TWSR & 0xF8) == TW_SR_SLA_ACK) {
		TWCR |= _BV(TWINT) | _BV(TWEA);
		} else if ((TWSR & 0xF8) == TW_SR_DATA_ACK) {
		data = TWDR;
		TWCR |= _BV(TWINT) | _BV(TWEA);
		} else {
		TWCR |= _BV(TWINT) | _BV(TWEA);
	}

	if (data == 'o') {
		PORTC &= ~_BV(PORTC0); // Set the A0 pin low for 'open'
		} else if (data == 'c') {
		PORTC |= _BV(PORTC0); // Set the A0 pin high for 'close'
	}
}

int main(void) {
	DDRC |= _BV(DDC0); // Set PC0 (pin A0) as output

	TWAR = SLAVE_ADDR << 1;
	TWCR |= _BV(TWEA) | _BV(TWEN) | _BV(TWIE);

	sei(); // Enable interrupts

	while (1) {}

	return 0;
}
