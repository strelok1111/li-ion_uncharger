/*
 * main.c
 *
 *  Created on: 15 мая 2016 г.
 *      Author: strelok
 */

#include "lcd.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <string.h>
#include "main.h"

volatile double voltage,current,volume;
volatile uint8_t ovf_disp = 0,measure_start = 0,discharged = 0,no_batt = 0;
volatile uint16_t ovf_counter = 0;

void EEPROM_writeDouble(uint16_t ee, double value){
     eeprom_write_block((void *)&value,ee,sizeof(value));
}

double EEPROM_readDouble(int ee)
{
	double *r;
	eeprom_read_block(r,ee,sizeof(double));
	return *r;
}

double get_max_charge(double current){
	if(current < 5.0){
		return 4.2;
	}
	if(current >= 5.0 && current < 300){
		return 4.1;
	}
	if(current >= 300 && current < 1300){
		return 4.0;
	}
	return 0;
}

double get_min_charge(double current){
	if(current < 5.0){
		return 3.6;
	}
	if(current >= 5.0 && current < 300){
		return 3.4;
	}
	if(current >= 300 && current < 1300){
		return 3.2;
	}
	return 0;
}

double get_percent(double voltage,double current){
	return ((voltage - get_min_charge(current)) / (get_max_charge(current) - get_min_charge(current))) * 100.0;
}

void init_gpio(void){
	BUTTON_1_DIR &= ~_BV(BUTTON_1_PIN);
	BUTTON_1_PORT |= _BV(BUTTON_1_PIN);
	BUTTON_2_DIR &= ~_BV(BUTTON_2_PIN);
	BUTTON_2_PORT |= _BV(BUTTON_2_PIN);
	BUTTON_3_DIR &= ~_BV(BUTTON_3_PIN);
	BUTTON_3_PORT |= _BV(BUTTON_3_PIN);

	CURRENTS_DIR |= _BV(CUR1_PIN) | _BV(CUR2_PIN) | _BV(CUR3_PIN);
	DISABLE_ALL_CURRENTS;

	SENS_DIR &= ~_BV(ISENS_PIN) & ~_BV(VSENS_PIN);
	PORTC &= ~_BV(ISENS_PIN) & ~_BV(VSENS_PIN);
}

uint16_t get_adc_voltage_int(){
	ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX2);
	ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADPS2) | _BV(ADPS1);
	while(ADCSRA & _BV(ADSC));
	uint16_t adc;
	adc = ADCL;
	adc = (ADCH << 8) | adc;
	return adc;
}

uint16_t get_adc_current_int(){
	ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX2) | _BV(MUX0);
	ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADPS2) | _BV(ADPS1);
	while(ADCSRA & _BV(ADSC));
	uint16_t adc;
	adc = ADCL;
	adc = (ADCH << 8) | adc;
	return adc;
}

double get_voltage(){
	return get_adc_voltage_int() * VOLTAGE_COEFF;
}

double get_current(){
	return get_adc_current_int() * CURRENT_COEFF;
}
void display(){
	lcd_clrscr();
	if(no_batt){
		lcd_puts("NO BATTERY!");
	}else{
		char buffer[16];
		char volt_str[4];
		char cur_str[4];
		dtostrf(voltage,1,2,volt_str);
		itoa((uint16_t)current,cur_str,10);
		sprintf(buffer,"V-%sv,I-%sma",volt_str,cur_str);
		lcd_puts(buffer);
		lcd_gotoxy(0,1);
		char buffer_sec_line[16];
		char vol_buff[7];
		dtostrf(volume,5,1,vol_buff);
		if(discharged){
			sprintf(buffer_sec_line,"Vol-%s,DIS",vol_buff);
		}else{
			if(current > 2){
				char cur_charge[4];
				dtostrf(get_percent(voltage,current),2,1,cur_charge);
				sprintf(buffer_sec_line,"Vol-%s,%s%%",vol_buff,cur_charge);
			}else{
				sprintf(buffer_sec_line,"Vol-%s",vol_buff);
			}
		}
		lcd_puts(buffer_sec_line);
	}
}
void timer0_init(){
	TCCR0 |= _BV(CS00) | _BV(CS02);
	TIMSK |= _BV(TOIE0);
}
void overfow_counter_callback(void){
	if(!no_batt && current > 5){
		volume = volume + (current * 0.0046603378);
	}
}
void overfow_disp_counter_callback(void){
	display();
}
ISR(TIMER0_OVF_vect){
	voltage = get_voltage();
	current = get_current();
	if(ovf_counter <= OVERFLOW_MAX_COUNTER){
		ovf_counter ++;
	}else{
		overfow_counter_callback();
		ovf_counter = 0;
	}
	if(ovf_disp <= OVERFLOW_DISP_COUNTER){
		ovf_disp ++;
	}else{
		overfow_disp_counter_callback();
		ovf_disp = 0;
	}
	if(voltage < 1.0){
		no_batt = 1;
	}else{
		no_batt = 0;
	}
	if(voltage < get_min_charge(current) && measure_start && !no_batt){
		discharged = 1;
		measure_start = 0;
		DISABLE_ALL_CURRENTS;
	}
}
int main (void){

	lcd_init(LCD_DISP_ON);
	lcd_clrscr();
	lcd_puts("Li-ion tester");
	lcd_gotoxy(0,1);
	lcd_puts("Version 1.0");
	_delay_ms(2000);
	init_gpio();
	timer0_init();
	sei();
	char level = 0;
	char button_flags[3] = {0,0,0};
	while(1){
		if(IS_BUTTON_1_PRESSED && !measure_start && !no_batt && !button_flags[0]){
			ENABLE_CUR1;
			button_flags[0] = 1;
			measure_start = 1;
			discharged = 0;
			level = 1;
			_delay_ms(200);
		}
		if(IS_BUTTON_2_PRESSED && measure_start && !no_batt && !button_flags[1]){
			button_flags[1] = 1;
			if(level == 1){
				ENABLE_CUR2;
				level = 2;
			}else if(level == 2){
				ENABLE_CUR3;
				level = 3;
			}else if(level == 3){
				DISABLE_ALL_CURRENTS;
			    ENABLE_CUR1;
				level = 1;

			}
			_delay_ms(200);
		}
		if(IS_BUTTON_1_PRESSED && measure_start && !no_batt && !button_flags[0]){
			button_flags[0] = 1;
			DISABLE_ALL_CURRENTS;
			level = 0;
			measure_start = 0;
			_delay_ms(200);
			ovf_counter = 0;
		}
		if(IS_BUTTON_3_PRESSED && !no_batt && !button_flags[2]){
			button_flags[3] = 1;
			volume = 0;
			discharged = 0;
			_delay_ms(200);
		}
		if(!IS_BUTTON_1_PRESSED){
			button_flags[0] = 0;
		}
		if(!IS_BUTTON_2_PRESSED){
			button_flags[1] = 0;
		}
		if(!IS_BUTTON_3_PRESSED){
			button_flags[2] = 0;
		}
	};
}
