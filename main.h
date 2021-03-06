#define BUTTON_1_PORT PORTD
#define BUTTON_1_DIR DDRD
#define BUTTON_1_PIN_PORT PIND
#define BUTTON_1_PIN 5
#define BUTTON_2_PORT PORTB
#define BUTTON_2_DIR DDRB
#define BUTTON_2_PIN_PORT PINB
#define BUTTON_2_PIN 5
#define BUTTON_3_PORT PORTC
#define BUTTON_3_DIR DDRC
#define BUTTON_3_PIN_PORT PINC
#define BUTTON_3_PIN 0

#define IS_BUTTON_1_PRESSED  !(BUTTON_1_PIN_PORT & _BV(BUTTON_1_PIN))
#define IS_BUTTON_2_PRESSED  !(BUTTON_2_PIN_PORT & _BV(BUTTON_2_PIN))
#define IS_BUTTON_3_PRESSED  !(BUTTON_3_PIN_PORT & _BV(BUTTON_3_PIN))

#define CURRENTS_PORT PORTC
#define CURRENTS_DIR DDRC
#define CUR1_PIN 2
#define CUR2_PIN 1
#define CUR3_PIN 3

#define ENABLE_CUR1  CURRENTS_PORT |= _BV(CUR1_PIN)
#define ENABLE_CUR2  CURRENTS_PORT |= _BV(CUR2_PIN)
#define ENABLE_CUR3  CURRENTS_PORT |= _BV(CUR3_PIN)
#define DISABLE_CUR1  CURRENTS_PORT &= ~_BV(CUR1_PIN)
#define DISABLE_CUR2  CURRENTS_PORT &= ~_BV(CUR2_PIN)
#define DISABLE_CUR3  CURRENTS_PORT &= ~_BV(CUR3_PIN)
#define DISABLE_ALL_CURRENTS CURRENTS_PORT &= ~_BV(CUR1_PIN) & ~_BV(CUR2_PIN) & ~_BV(CUR3_PIN)
#define IS_CUR1_ENABLE (CURRENTS_PORT & _BV(CUR1_PIN))
#define IS_CUR2_ENABLE (CURRENTS_PORT & _BV(CUR2_PIN))
#define IS_CUR3_ENABLE (CURRENTS_PORT & _BV(CUR3_PIN))

#define SENS_DIR DDRC
#define SENS_PORT PORTC
#define ISENS_PIN 5
#define VSENS_PIN 4

#define VOLTAGE_COEFF 0.0051015228
#define CURRENT_COEFF 1.9718309859

#define OVERFLOW_MAX_COUNTER 511
#define OVERFLOW_DISP_COUNTER 15

