#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#ifndef sbi
#define sbi(port,bit) port|=(1 << bit)
#endif

#ifndef cbi
#define cbi(port,bit) port&=~(1 << bit)
#endif

#define true 1
#define false 0
#define LATCH	4
#define DATA	5
#define SCK		7
#define BTN0	0b11111101
#define BTN1	0b11111011
#define BTN2	0b11110111
#define DIR00	0
#define DIR01	1
#define DIR10	3
#define DIR11   6
#define SERVO_CENTER		2550	//Sai số của cần sensor trên xe (am se dich qua trai, duong se dich qua phai)
#define ANGLE_MAX           150
#define STEP				5

volatile float special_ratio;
volatile float PID_ratio;
volatile uint8_t pulse_v_des=8;
volatile uint8_t pulse_v_crook_des=8;
volatile uint8_t pulse_v_des_base=8;
volatile uint8_t pulse_v_crook_des_base=8;
volatile uint8_t straight_flag=true;
volatile uint8_t crook_flag=false;
volatile uint8_t special_flag=false;

//Variable ADC
uint16_t ADC_average[8];		//ADC trung bình
uint16_t linetrang[8];			//ADC line trắng
uint16_t lineden[8];			//ADC line đen

//special case
// 0 la noline, 1 la cua vuong
uint8_t sum=0;
uint8_t list_case[20];
uint8_t index_list_case=0;

//Variable LED7
struct led7 {
	uint8_t i;
	uint8_t unit;
	uint8_t ten;
	uint8_t hundred;
	uint8_t thousand;
	uint8_t sensor_out;
} led7_data;

//===================BUTTON + SWITCH=====================
uint8_t get_button(uint8_t keyid)
{
	if ( (PINB & 0x0e) != 0x0e)
	{
		_delay_ms(100);
		if ((PINB|keyid) == keyid) return 1;
	}
	return 0;
}

//================RATIO + SERVO + MOTOR ================
void speed(float left, float right, float persent) { //max left la 255, max right la 255
	if(special_flag) {
		left*=((persent/100)*special_ratio);
		right*=((persent/100)*special_ratio);
	}
	else {
		if(crook_flag || straight_flag) {
			if(left==right) {
				left*=(PID_ratio/100)*persent;
				right*=(PID_ratio/100)*persent;
			}
			if(left!=right) {
				if(PID_ratio<0) {
					float c=left;
					left=right;
					right=c;
				}
				left*=(PID_ratio/100)*persent;
				right*=(PID_ratio/100)*persent;
			}
		}
	}
	if (left>=0) {
		sbi(PORTD,DIR10);
		cbi(PORTD,DIR11);
		if (left*157>40000) OCR1B=40000;
		else
			OCR1B=(int)left*157;
	}
	else {
		cbi(PORTD,DIR10);
		sbi(PORTD,DIR11);
		if (left*157<-(40000)) OCR1B=40000;
		else
			OCR1B=(int)-left*157;
	}
	if (right>=0) {
		sbi(PORTD,DIR00);
		cbi(PORTD,DIR01);
		if(right>255) right=255;
		OCR2=(int)right;
	}
	else {
		cbi(PORTD,DIR00);
		sbi(PORTD,DIR01);
		if(right<-255) right=-255;
		OCR2=-(int)right;
	}
}
void handle(float goc)
{
	if (goc>ANGLE_MAX) goc=ANGLE_MAX;
	if (goc<-ANGLE_MAX) goc=-ANGLE_MAX;
	OCR1A=SERVO_CENTER+(int)goc*STEP; //duong phai, am trai
									//45* =1000;
}

inline void fast_brake_left()
{
	sbi(PORTD, DIR00);
	sbi(PORTD, DIR01);
	OCR1B=20000;
}

inline void fast_brake_right()
{
	sbi(PORTD, DIR10);
	sbi(PORTD, DIR11);
	OCR2=255;
}

void fast_brake()
{
	fast_brake_left();
	fast_brake_right();
}

//==========================LED7=========================
void SPI(uint8_t data)			//Truyền dữ  liệu sang led7, sử dụng SPI
{
	SPDR = data;
	while(!(SPSR & (1<<SPIF)));	//Đợi phần cứng truyền xong data
}
void led7(unsigned int num)		// Tính toán dữ liệu cho 4 led 7 đoạn
{
	led7_data.unit		 = (1<<7) |(unsigned int)(num%10);
	led7_data.ten		 = (unsigned int)(num%100 / 10);
	led7_data.hundred	 = (unsigned int)(num%1000 / 100);
	led7_data.thousand	 = (unsigned int)(num/ 1000);
	led7_data.thousand	|= (led7_data.thousand!=0)? 1<<4 : 0;
	led7_data.hundred	|= (led7_data.thousand!=0 || led7_data.hundred!=0)? 1<<5 : 0;
	led7_data.ten		|= (led7_data.thousand!=0 || led7_data.hundred !=0 || led7_data.ten!=0)? 1<<6 : 0;
}
void print()					//Luôn thực thi mỗi vài ms để quét LED
{
	uint8_t value=0;
	if(led7_data.i++ == 4) led7_data.i=0;
	switch(led7_data.i)
	{
		case 0: value=led7_data.thousand;	break;
		case 1: value=led7_data.hundred;	break;
		case 2: value=led7_data.ten;		break;
		case 3: value=led7_data.unit;		break;
		default: break;
	}
	SPI(~led7_data.sensor_out);
	SPI(value);
	sbi(PORTB,LATCH);
	cbi(PORTB,LATCH);
}

//==========================ADC==========================
void read_adc_eeprom()
{
	for(uint8_t j=0; j<8; j++)
	{
		while(!eeprom_is_ready());
		linetrang[j] = eeprom_read_word((uint16_t*)(j*2));
		while(!eeprom_is_ready());
		lineden[j] = eeprom_read_word((uint16_t*)((j+8)*2));
	}
	for(uint8_t i=0; i<8; i++)
	{
		ADC_average[i]=(linetrang[i]+lineden[i])/2;
	}
}
void write_adc_eeprom()
{
	for(uint8_t j=0; j<8; j++)
	{
		while(!eeprom_is_ready());
		eeprom_write_word((uint16_t*)(j*2), (uint16_t)linetrang[j]);
		while(!eeprom_is_ready());
		eeprom_write_word((uint16_t*)((j+8)*2), (uint16_t)lineden[j]);
	}
}
uint16_t adc_read(uint8_t ch)
{
	ADMUX = (1<< REFS0)|ch;									// selecting channel
	ADCSRA|=(1<<ADSC);										// start conversion
	while(!(ADCSRA & (1<<ADIF)));							// waiting for ADIF, conversion complete
	return ADCW;											// Giá trị trả về từ [0 -> 1024] tương ứng [0V -> 5V]	
}
uint8_t sensor_cmp(uint8_t mask)							//Sensor compare: đọc về và so sánh với trung bình 
{															//Thêm tính năng che mặt nạ: mask mặc định là: 0xff (0b11111111)
	uint8_t ADC_value=0;												
	for(uint8_t i=0; i<8; i++)
	{
		if(adc_read(i)<ADC_average[i]) sbi(ADC_value,i);	//Nhỏ hơn trung bình -> gần về 0V -> led thu hồng ngoại dẫn -> có nhiều hồng ngoại -> vạch trắng
		//else    cbi(ADC_value,i);
	}
	return (ADC_value & mask);
}
void learn_color() // learn color + volocity
{
	uint16_t ADC_temp=0;
	for (uint8_t i=0; i<8; i++)
	{
		linetrang[i]=1024;
		lineden[i]=0;
	}
	while(1)
	{
		led7(9999);
		print();
		if(get_button(BTN1)) break;
		for (uint8_t i=0; i<8; i++)
		{
			ADC_temp=adc_read(i);
			if (ADC_temp < linetrang[i]) linetrang[i]=ADC_temp;
			if(ADC_temp>lineden[i]) lineden[i]=ADC_temp;
		}
	}
	for (uint8_t i=0; i<8; i++)
	{
		ADC_average[i]=(linetrang[i]+lineden[i])/2;
	}
	write_adc_eeprom();										//Ghi vào eeprom để cho các lần sau			
}

//=======================INITIAL=========================
void INIT()
{
	//ADC
	ADMUX=(1<<REFS0);										// 0b0100000000 Chọn điện áp tham chiếu từ chân AVCC, thêm tụ ở AREF
	ADCSRA=(1<<ADEN) | (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);	// 0b10000111 Enable ADC and set Prescaler = 128
	read_adc_eeprom();										// Tự động đọc Eeprom ra khi bật nguồn chip
	
	//PORT
	DDRB  = 0b11110001;
	PORTB = 0b11111111;
	DDRC  = 0b00000000;
	PORTC = 0b11111111;
	DDRD  = 0b11111011;
	PORTD = (1 << DIR00) | (1 << DIR10);										// DIR00 = 1, DIR01 = 0, DIR10 = 1, DIR11 = 0
	
	//SPI
	SPCR	= (1<<SPE)|(1<<MSTR);							//Enable spi, Master
	SPSR	= (1<<SPI2X);									//SCK Mode 2X: Fosc/2
	
	//TIMER
	TCCR0=(1<<WGM01) | (1<<CS02);							// Mode 2 CTC,  Prescaler = 256
	OCR0=62;												// 1ms
	TIMSK=(1<<OCIE0);
		
	TCCR1A = (1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);			// SET OCR1A & OCR1B at BOTTOM, CLEAR at Compare Match (Non-invert), Mode 14 Fast PWM
	TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);				// Prescaler = 8
	ICR1 = 40000;											// Time Period = 20ms
	
	TCCR2=(1<<WGM20)|(1<<WGM21)|(1<<COM21)|(1<<CS22)|(1<<CS21)|(1<<CS20);  //SET OC2 at BOTTOM, CLEAR OC2 on compare match,(non-invert), Mode 3 Fast PWM,  Prescaler = 1024
	OCR2=0;
	sei();
	
	//ENCODER
	MCUCR |= (1<<ISC11)|(1<<ISC01);
	GICR |= (1<<INT0);
	//thong so
	special_ratio=1.0;
	PID_ratio=1.0;
	sum=0;
	index_list_case=0;
	for(int i=0; i<20; i++) {
		list_case[i]=0;
	}
}

void test_hardware()
{
	uint8_t _index=0;
	while(1)
	{
		if(get_button(BTN1)) break;
		led7(adc_read(_index));
		led7_data.sensor_out = 0 | (1<<_index);
		print();
		if(get_button(BTN0)) {
			_index=(_index+1)%8;
		}
	}
}
//========================START==========================
void sel_mode()
{
	handle(0);
	speed(0,0,0);
	while(1)
	{
		led7(1111);
		led7_data.sensor_out=sensor_cmp(0xff);
		print();
		if(get_button(BTN0)) {
			led7(2222);
			_delay_ms(500);
			while (1) // lay toc do pulse encoder mong muon
			{
				pulse_v_des=pulse_v_des_base;
				led7(pulse_v_des);
				if(get_button(BTN1)) {
					if ((pulse_v_des_base+1) > 30) pulse_v_des_base=8;
					else pulse_v_des_base++;
				}
				if(get_button(BTN0)) {
					if ((pulse_v_des_base-1) < 0) pulse_v_des_base=30;
					else pulse_v_des_base--;
				}
				if(get_button(BTN2)) break;
			}
			led7(3333);
			_delay_ms(500);
			while (1) // lay toc do pulse encoder mong muon
			{
				pulse_v_crook_des=pulse_v_crook_des_base;
				led7(pulse_v_crook_des);
				if(get_button(BTN1)) {
					if ((pulse_v_crook_des_base+1) > 50) pulse_v_crook_des_base=8;
					else pulse_v_crook_des_base++;
				}
				if(get_button(BTN0)) {
					if ((pulse_v_crook_des_base-1) < 0) pulse_v_crook_des_base=50;
					else pulse_v_crook_des_base--;
				}
				if(get_button(BTN2)) break;
			}
			led7(4444);
			_delay_ms(500);
			while (1) // lay tong so case cua vuong va no line
			{
				led7(sum);
				if(get_button(BTN1)) {
					if ((sum+1) > 20) sum=0;
					else sum++;
				}
				if(get_button(BTN0)) {
					if ((sum-1) < 0) sum=20;
					else sum--;
				}
				if(get_button(BTN2)) break;
			}
			if(sum>0) {
				led7(5555);
				_delay_ms(200);
			}
			while (index_list_case<sum) {
				led7(list_case[index_list_case]);
				if(get_button(BTN1)) {
					if ((list_case[index_list_case]+1) > 1) list_case[index_list_case]=0;
					else list_case[index_list_case]++;
				}
				if(get_button(BTN0)) {
					if ((list_case[index_list_case]-1) < 0) list_case[index_list_case]=1;
					else list_case[index_list_case]--;
				}
				if(get_button(BTN2)) {
					if(index_list_case+1<sum) {
						led7(6666);
						_delay_ms(400);
					}
					index_list_case++;
				}
			}
			return;
		}
		else if (get_button(BTN1))	test_hardware();
		else if (get_button(BTN2))	learn_color();
	}
}