//#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/power.h>
#include <stdlib.h>


#ifndef cbi
#define cbi(sfr, bit) (sfr &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (sfr |= _BV(bit))
#endif

// �������� ����
#define	OTG			PB0
// HUB & Router
#define	HUB			PB1
#define	LED			PB2
//#define	VOLT		PB3
#define	ACC			PB4

//#define	byte	unsigned char

// ��������� ���������
typedef enum  {S_OFF, S_ACC_ON, S_WORK, S_ACC_OFF, S_BAT_LOW} states_t;
states_t curr_state = S_OFF;

// ������� �������, ������� ���������� � ������ ���������
void	f_off();
void	f_acc_on();
void	f_work();
void	f_acc_off();
void	f_bat_low();
void (*state_funcs[])() = {f_off, f_acc_on, f_work, f_acc_off, f_bat_low};

// ������� ��� ������
volatile uint8_t	sec_counter	= 0;
// uptime ������ � ��������
volatile uint32_t uptime = 0;
// �������� ��� ������� ��������
volatile  unsigned int	count_down = 0;

// ������� ������� ��� (���������� ������)
volatile uint8_t	acc_cnt	= 0;
// ������� ��� ����������� ��� (���������� ������)
volatile uint16_t	acc_off_cnt	= 0;
// ����������, ��� ������� �� ����� �������� (x 10)
#define BAT_LOW	100
// ������� ��� � ��� ������� ��������� (���������� ������)
volatile uint8_t	vbat_low	= 0;
// ������� ��� � ��� ������� � ���������� ��������� (���������� ������)
volatile uint8_t	vbat_ok		= 0;

// ������ �������� LED
typedef enum	{LED_OFF = 0, LED_ON, LED_50, LED_25, LED_10, LED_IMP_1, LED_IMP_3} ledmodes_t;
// ����� �������� LED
volatile ledmodes_t	led_mode	= LED_IMP_3;

// �������� ���������� � �������� ����
volatile uint8_t	vbat	= 0;


ISR (TIMER1_OVF_vect)
{
	if (sec_counter == 25)
		// start ADC the conversion
		sbi(ADCSRA, ADSC);

	// ����������� ��� � �������
	if (sec_counter > 122)
	{
		sec_counter = 0;
		uptime++;

		if (PINB & _BV(ACC))
		{
			acc_off_cnt = 0;
			if (acc_cnt < 255)
				acc_cnt++;
		}
		else
		{
			acc_cnt = 0;
			if (acc_off_cnt < 65535)
				acc_off_cnt++;
		}

		// ��������� �������� ����������
	uint8_t low, high;
		low  = ADCL; high = ADCH;
		// combine the two bytes
	uint16_t out = (high << 8) | low;
		// map to real value
		// resistor divider k=4
		// we need return U*10
		// formula Uret = out*5(Vref)*k(4)/1024 * 10
		// after optimization = out * 25/128
		out *= 25;
		vbat = out >> 7; // div 128

		// ���� ������� �����������
		if (vbat < BAT_LOW)
		{
			if (vbat_low < 255)
				vbat_low++;
		}
		else
			vbat_low = 0;

		// ������� ���������� ������� - ���� �� �������� +1�����
		if (vbat > BAT_LOW + 10)
		{
			if (vbat_ok < 255)
				vbat_ok++;
		}
		else
			vbat_ok = 0;

	}

	switch (led_mode)
	{
		case	LED_OFF:	cbi(PORTB, LED); break;
		case	LED_ON:		sbi(PORTB, LED); break;
		case	LED_50:		if (sec_counter % 60 == 0) sbi(PORTB, LED);
							if (sec_counter % 122 == 0) cbi(PORTB, LED);
							break;
		case	LED_25:		if (sec_counter % 30 == 0) sbi(PORTB, LED);
							if (sec_counter % 60 == 0) cbi(PORTB, LED);
							break;
		case	LED_10:		if (sec_counter % 10 == 0) sbi(PORTB, LED);
							if (sec_counter % 20 == 0) cbi(PORTB, LED);
							break;
		case	LED_IMP_1:	if (sec_counter == 0) sbi(PORTB, LED);
							if (sec_counter == 1) cbi(PORTB, LED);
							break;
		case	LED_IMP_3:	if (sec_counter == 0 || sec_counter == 30 || sec_counter == 60) sbi(PORTB, LED);
							if (sec_counter == 1 || sec_counter == 31 || sec_counter == 61) cbi(PORTB, LED);
							break;
	}

	sec_counter++;

	// ��� ������� delay
	if (count_down > 0)
		count_down--;

}

// ������� ��������, �� ����� 10 �� � �� ����� 650 �
void	delay(unsigned long ms)
{
	if (ms < 10)		ms = 10;
	if (ms > 655350)	ms = 655350;

	cli();
	count_down = ms / 10;
	sei();
	while (count_down)
		sleep_mode();
}

void	setup()
{
	// Set pins as output
	DDRB = _BV(OTG) | _BV(LED) | _BV(HUB);

	// Set Pull-Up on ACC pin
	PORTB = _BV(ACC);

	// ��� ���������� ������� ������� � ������ �������� ���������� TIMER1
	// ������������� �������� �� 32 = 1MHz /32 /256 =~ 122 ���������� � ���
	TCCR1 = _BV(CS12) | _BV(CS11);
	// TIMER1 - ��������� ���������� �� ������������
	TIMSK |= _BV(TOIE1);

	set_sleep_mode(SLEEP_MODE_IDLE);
/*
	mode = eeprom_read_byte(E_MODE);
	if (mode > 2)
		mode = 0;

	mode_2_rele_and_nvram();
*/

	// set a2d prescale factor to 8 (1 MHz / 8 = 125 KHz, inside the desired by manual 50-200 KHz range)
	ADCSRA = _BV(ADPS1) | _BV(ADPS0);

	// �������� ������� ������ (5,01�) ��� AREF � 3-� ����� (���� B3)
	ADMUX = _BV(MUX1) | _BV(MUX0);

	// ��������� ������ ADC
	sbi(ADCSRA, ADEN);

	sei();

}

uint16_t	analogRead()
{
uint8_t low, high;


	// start the conversion
	sbi(ADCSRA, ADSC);

	// ADSC is cleared when the conversion finishes
	while (bit_is_set(ADCSRA, ADSC));

	// we have to read ADCL first; doing so locks both ADCL
	// and ADCH until ADCH is read.  reading ADCL second would
	// cause the results of each conversion to be discarded,
	// as ADCL and ADCH would be locked when it completed.
	low  = ADCL;
	high = ADCH;


	// combine the two bytes
uint16_t out = (high << 8) | low;

	// map to real value
	// resistor divider k=4
	// we need return U*10
	// formula Uret = out*5(Vref)*k(4)/1024 * 10
	// after optimization = out * 25/128
	out *= 25;
	return out >> 7; // div 128
}

int	main()
{
	setup();
	// �������� ������� ��������� ����������
	delay(1500);

	//eeprom_write_byte(0, vbat);
	while (1)
	{
		state_funcs[curr_state]();
		delay(250);
	}
}

void	check_bat()
{
	// ���� ����� 1 ������ ���������
	if (vbat_low > 1*60)
		curr_state = S_BAT_LOW;
}

// �� ��������� � ���� ��������� ACC
void	f_off()
{
	led_mode = LED_IMP_1;
	// ��������� ��
	PORTB &= ~(_BV(HUB) | _BV(OTG));

	// ���� ��������� �������� ��� ����� 1-� �������
	if (acc_cnt >= 1)
		curr_state = S_ACC_ON;

	// ������� �� �����������?
	check_bat();
}

// �������� ���/������ � ����� ��������� ������ ������ � ����� ������ (���� ��������� �� ���������)
void	f_acc_on()
{
	led_mode = LED_10;
	// �������� ������� �� ��� � ������
	PORTB |= _BV(HUB);

	// ���� ��������� �������� ��� ����� 2-� �������
	if (acc_cnt >= 2)
		curr_state = S_WORK;

	// ��������� ���������?
	if (acc_cnt == 0)
		curr_state = S_ACC_OFF;
}

// ��������
void	f_work()
{
	led_mode = LED_50;
	// �������� ������� �� ��� � ������
	PORTB |= _BV(OTG);

	// ��������� ���������?
	if (acc_cnt == 0)
		curr_state = S_ACC_OFF;

	// ������� �� �����������?
	check_bat();
}

// ���� �������� � ����������
void	f_acc_off()
{
	led_mode = LED_25;

	// ��������� ���������?
	if (acc_cnt > 0)
		curr_state = S_ACC_ON;

	// ������ 2 ������ ����� ������� ���������?
	if (acc_off_cnt > 2)
		// ��������� OTG
		PORTB &= ~_BV(OTG);

	// ������ 20 ����� ����� ������� ���������?
	if (acc_off_cnt > 20*60)
		curr_state = S_OFF;

	// ������� �� �����������?
	check_bat();
}

void	f_bat_low()
{
	led_mode = LED_IMP_3;

	// ��������� OTG
	PORTB &= ~_BV(OTG);

	// ��������� HUB ����� 10 ������
	if (vbat_low > 10)
		PORTB &= ~_BV(HUB);

	// ��������� - �������� �� �������? ����� ��������� ���� � ������� 30 ������
	if (vbat_ok > 30)
	{
		curr_state = S_OFF;
		acc_cnt = 0; // ���� ��� ��� ��� ������� - ������ ���, ��� ������ ��� ��������
	}
}
