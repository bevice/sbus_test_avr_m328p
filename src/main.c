#include "main.h"
#include <avr/interrupt.h>
#include <util/delay.h>


#define TIMER_0_PERIOD_US 128
#define DATA_WAIT_COUNTER (20000 / TIMER_0_PERIOD_US)
#define BLINK_NO_DATA_PERIOD_MS 1000
#define SBUS_PACKAGE_SIZE 25
#define BLINK_PERIOD(BLINK_PERIOD_MS) ((uint16_t)((BLINK_PERIOD_MS) * 1000l / (TIMER_0_PERIOD_US)))

volatile uint16_t blink_period = 0;
volatile uint16_t pwm_value = 0;
uint16_t channels[16];
uint8_t sbus_flags;

void decode_sbus(uint8_t *sbusData) {
    // 16 каналов по 11 бит
    channels[0] = ((sbusData[1] | sbusData[2] << 8) & 0x07FF);
    channels[1] = ((sbusData[2] >> 3 | sbusData[3] << 5) & 0x07FF);
    channels[2] = ((sbusData[3] >> 6 | sbusData[4] << 2 | sbusData[5] << 10) & 0x07FF);
    channels[3] = ((sbusData[5] >> 1 | sbusData[6] << 7) & 0x07FF);
    channels[4] = ((sbusData[6] >> 4 | sbusData[7] << 4) & 0x07FF);
    channels[5] = ((sbusData[7] >> 7 | sbusData[8] << 1 | sbusData[9] << 9) & 0x07FF);
    channels[6] = ((sbusData[9] >> 2 | sbusData[10] << 6) & 0x07FF);
    channels[7] = ((sbusData[10] >> 5 | sbusData[11] << 3) & 0x07FF); // & the other 8 + 2 channels if you need them
    channels[8] = ((sbusData[12] | sbusData[13] << 8) & 0x07FF);
    channels[9] = ((sbusData[13] >> 3 | sbusData[14] << 5) & 0x07FF);
    channels[10] = ((sbusData[14] >> 6 | sbusData[15] << 2 | sbusData[16] << 10) & 0x07FF);
    channels[11] = ((sbusData[16] >> 1 | sbusData[17] << 7) & 0x07FF);
    channels[12] = ((sbusData[17] >> 4 | sbusData[18] << 4) & 0x07FF);
    channels[13] = ((sbusData[18] >> 7 | sbusData[19] << 1 | sbusData[20] << 9) & 0x07FF);
    channels[14] = ((sbusData[20] >> 2 | sbusData[21] << 6) & 0x07FF);
    channels[15] = ((sbusData[21] >> 5 | sbusData[22] << 3) & 0x07FF);

}

uint8_t rx_buffer[SBUS_PACKAGE_SIZE];
volatile uint8_t rx_buffer_index;

void init_uart() {
    UBRR0 = 0x09; // 100k @16MHz
    UCSR0B = _BV(RXEN0) | _BV(RXCIE0) | _BV(TXEN0);
    UCSR0C |= _BV(USBS0) | _BV(UPM01);
}

void timer0_start(void) {

    TCCR0B |= (1 << CS01);
    TIMSK0 |= (1 << TOIE0);
}

void setup() {
    DDRB = _BV(PB5);
    DDRD = _BV(PD2);
    DDRC = 0;
    PORTC = 0xFF;
    init_uart();
    timer0_start();

    sei();

}

int main() {
    setup();

    while (1) {
//        static uint8_t x = 0;
//        _delay_ms(1);
//        UDR0 = x++;;
    }

    return 0;
}


volatile uint8_t last_data_counter;
uint16_t get_value(uint16_t channel_data){
    static uint16_t max_value = 0;
    static uint16_t min_value = 0xFFFF;
    if(channel_data>max_value) max_value = channel_data;
    if(channel_data<min_value) min_value = channel_data;

    return (uint16_t )(256.0 * (channel_data-min_value) / (max_value-min_value));

}
void check_sbus_package(uint8_t *buffer) {
    if (buffer[0] == 0x0F) {
        blink_period = BLINK_PERIOD(0);
        decode_sbus(buffer);
        uint8_t ch = (~PINC) & 0x0F;

        pwm_value = get_value(channels[ch]);
        UDR0 = (uint8_t) ((channels[0] & 0xFF00) >>8);
        UDR0 = (uint8_t) (channels[0] & 0xFF);
//        UDR0 = pwm_value;
    } else {
        blink_period = BLINK_PERIOD(100);
        UDR0 = 0xAA;
        UDR0 = buffer[0];
    }

}

#define BLINK_OFF {PORTB &= ~_BV(PB5);}

ISR(USART_RX_vect) {
    last_data_counter = 0;
    uint8_t data = UDR0;

    rx_buffer[rx_buffer_index++] = data;
    if (rx_buffer_index == SBUS_PACKAGE_SIZE) {

        rx_buffer_index = 0;
        check_sbus_package(rx_buffer);
    }
}

void soft_pwm() {
    if(blink_period)
        return;

    PORTD ^= _BV(PD2);
    static uint16_t counter = 0;
    if (pwm_value > counter) PORTB |= _BV(PB5);
    else
        PORTB &= ~_BV(PB5);
    if (++counter == 256) counter = 0;
}

ISR(TIMER0_OVF_vect) {
    if(last_data_counter> (2000/TIMER_0_PERIOD_US)) rx_buffer_index=0;
    if (last_data_counter < DATA_WAIT_COUNTER) last_data_counter++;
    else {
        rx_buffer_index = 0;
        blink_period = BLINK_PERIOD(BLINK_NO_DATA_PERIOD_MS);
    }

    // BLINKER
    static uint16_t _counter = 0;
    if (blink_period && ++_counter > blink_period) {
        _counter = 0;
        PORTB ^= _BV(PB5);
    }
    soft_pwm();
}
