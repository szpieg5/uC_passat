/*
						uC samochodowy, start 16.01.2011
						kwarc 8MHz
						program wywolywany co 111,1ms
						czujniki przyspieszenia 2 osiowy MMA6260Q
						5 przyciskow
						czujnik temp oleju NTC
						3 czujniki temperatury DS18B20 na PC7
						pomiar cisnienia w mbar
						sterowanie PWM N75 - OCR2
						automatyczne wylaczenie uC jesli auto jest zamkniete lub silnik jest wylaczony 230 sekund
						2 wyswietlacze lcd RGB
						V 1.5
						13.10.2011 - 11336 bytes
						27.10.2011 - 11296 bytes
						10.11.2011 - 11360 bytes
						04.12.2011 - 11088 bytes
						01.01.2012 - 11134 bytes
						04.01.2012 - 11236 bytes
						23.01.2012 - 11740 bytes
						03.03.2012 - 12300 bytes
						11.03.2012 - 11746 bytes
						03.12.2012 - 12988 bytes
						04.04.2014 - 11724 bytes, nowy toolchain
						02.08.2014 - zmienienie sposobu obslugi przerwania - dodanie zmiennej przerwanie, przeniesnienie zmiennych
						11.08.2014 - dodanie automatycznej zmiany czasu zimowy, letni, obliczanie godziny wschodu, zachodu slonca
						19.03.2015 - dodanie stabilizatora 3,3V do podtrzymania napiecia zegara prosto z akumulatora
						08.04.2015 - dodanie dodatkowego obliczania ilosci paliwa z wykorzystaniem cz przyspieszenia
						25.07.2017 - wylaczenie niepotrzebnych funkcji, wyswietlanie temperatury w pokoju
						

dlaczego jest reset uP podczas wachania napiecia? - dodac kondensator
zmiana ilosci paliwa w zaleznosci od sredniej predkosci lub obrotow z ostatnich 5 minut
dodac automatyczne zerowanie cz przyspieszenia - co bylo nie tak z tym?
odczyt ilosc paliwa tylko jesli odczyt z cz przys jest bliski zeru - srednia paliwa ciagnieta 64 pomiary
jest blad 1000 odczytu moze od pierwszego odczytu dht
dlaczego czasami powoli uruchamia sie uC? - zmiana opoznienia podczas uruchamiania
poprawic ostatni dzien czasu letniego
poprawic czas wylaczenia uC, spr czy jest sygnal od zamkniecia samochodu - nie ma sygnalu
przerobic przyciski aby dzialaly na puszczenie
zmiana godziny z telefonu, przez bluetooth - brak pamieci uP
sprawdzic czasy
1wire bledy w transmisji, odczycie temperatury - sprawdzic timingi tranasmisji
poprawic predkosc, zapisywanie kilometrow trip
napiecie baterii 3V
pomiar poboru pradu z akumulatora - mierzyc spadek napiecia na przewodzie od aku
poscic sygnal z generatora aby zasymulowac predkosc
sprawdzic dlugosc wykonywania sie poszczegolnych czesci programu
*/

#define F_CPU 8000000
#define XTAL 8000000              /**< clock frequency in Hz, used to calculate delay timer */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>
#include <math.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include "HD44780.h"
#include <avr/wdt.h>
#include <avr/eeprom.h>                // dostêp do pamiêci EEPROM


#define DQ 7
#define SET_DQ DDRC &= ~_BV(DQ)
#define CLR_DQ DDRC |= _BV(DQ)
#define IN_DQ PINC & _BV(DQ)

// definicja dla DHT11
#define DT 3
#define SET_DT DDRC &= ~_BV(DT) // wejscie
#define CLR_DT DDRC |= _BV(DT)  // wyjscie
#define IN_DT PINC & _BV(DT)

// wolne rejestry ktore mozna uzyc jako zmienne 8bit
// EEARL - EEPROM address register
// EEDR - EEPROM data register
// UBRRL
//#define i EEARL
//#define pomiarh EEDR 
#define i UBRRL

#define delay(us)  _delayFourCycles( ( ( 1*(XTAL/3850) )*us)/1000 )
//#define delay(us)  _delayFourCycles( ( ( 1*(XTAL/3900) )*us)/1000 ) // kalibracja analizatorem
//#define delay(us)  _delayFourCycles( ( ( 1*(XTAL/3500) )*us)/1000 ) // wyzsze temperatury, lato
//#define delay(us)  _delayFourCycles( ( ( 1*(XTAL/3810) )*us)/1000 ) // zima

//volatile 
volatile unsigned char przerwanie = 0;
//volatile unsigned int pomiarh, pomiarl;
volatile unsigned char godz = 0, min = 0, sek = 0; 

unsigned char eeprom_adres __attribute__((section(".eeprom")));




	char ch[5], lcd1 = 1, lcd2 = 2, menu = 4, rgb = 1, DHT11_data[2], odczyt_10sekund = 0;

	//char linia_0_lcd[16] = "                ", linia_1_lcd[16] = "                ";
	// tablica wartosci przetwornika adc dla odpowiedniej temperatury co 1C: MIN -39C, -38C, -37C ... MAX
	const unsigned int temp[]= {0,10,11,12,13,14,15,16,17,19,20,21,23,24,26,28,29,31,33,36,38,40,43,45,48,51,54,57,60,63,67,70,74,78,82,86,91,96,100,105,110,116,121,127,133,139,145,151,158,165,172,179,186,193,201,209,217,225,233,241,250,259,268,277,286,295,304,314,323,333,342,352,362,372,382,392,402,412,422,432,442,452,462,472,482,492,502,512,521,531,541,550,560,569,578,588,597,605,614,623,632,640,648,656,665,673,681,688,696,704,711,718,725,732,738,745,752,758,764,770,776,782,788,793,799,804,809,814,819,824,829,833,838,842,847,851,855,859,863,867,870,874,877,881,884,887,891,894,897,900,903,905,908,911,913,916,918,921,923,925,928,930,932,934,936,938,940,942,944,945,947,949,950,952,953,955,956,958,959,960,962,963,965,966,967,968,969,970,972,973,974,975,976,977,978,1023};
	unsigned int adc0 = 0, Ubl = 600, uC_time = 0, uC_min = 0, e_temp = 0;
	unsigned char oil_temp = 40;
	unsigned char t_oil[5];
	unsigned char B_5 = 0, B_6 = 0, C_5 = 0, engine_on_off_l, engine_on_off_h, sec = 0;
	unsigned int Uacc = 0;
	unsigned char on_off_l, on_off_h, info = 0, off = 0;
	unsigned char engine_pp = 0, engine_time[5];
	unsigned char max_boost[5], avg_speed[5], travel[5], max_obr[5];
	unsigned char engine_off_sec = 0, during_off = 0, czas_letni = 0;


	//unsigned int licznik0=0, licznik1=0, licznik2=0, licznik3=0, licznik4=0, licznik5=0;
	unsigned int pwm_podsw = 0, Ubl_blue = 0;
	
	unsigned long int ee[3];
	//unsigned char ee0;
	
	int b=0;
	int adc2=0, adc3=0;
	unsigned long int km_total = 0, avg_SPEED = 0;
	unsigned int engine_on_sec = 0; // zmienione z long uint 22.12.2012
	long int speed1 = 0, speed2 = 0; // predkosc z dokladnoscia 0,1km/h
	//long int v_old = 0;
	unsigned long int aktualny_dystans = 0;
	unsigned int dziesiec_metrow = 0;
	unsigned char v1 = 0, v2 = 0, rrr = 0;
	
	unsigned char  L1 = 0, H1 = 0, L2 = 0, H2 = 0;
	char msb, lsb;
	int temp1=0, temp1r=0, temp2=0, temp3=0, boost = 0, doladowanie = 600, avg_boost = 0, max_BOOST = 0, boost_b = 0;
	unsigned int rpm = 0, obr = 0, max_OBR = 0;
	unsigned int paliwo = 0, poczatkowe_paliwo = 400, avg_paliwo;	
	long int avg_fuel = 0;
	int fuel = 0, fuel_old = 0;
	

	const char add_znaki[64] PROGMEM = { // definicja 8 dodatkowych znakow LCD
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, // 0 belek
	0x01, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x01, // 1 belka = 20mbat = 0,02bar
	0x01, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x01, // 2 belki
	0x01, 0x1c, 0x1c, 0x1c, 0x1c, 0x1c, 0x1c, 0x01, // 3 belki
	0x01, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x01, // 4 belki
	0x01, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x01, // 5 belek
	//0x1E, 0x06, 0x18, 0x1E, 0x00, 0x0C, 0x12, 0x0C, // "on"
	0x1C, 0x14, 0x1F, 0x00, 0x1F, 0x01, 0x1F, 0x00, // "UP"
	0x0C, 0x12, 0x12, 0x0C, 0x00, 0x00, 0x00, 0x00  // stopien
	};


// procedura obs³ugi przerwania od T1 co 11,111ms
ISR(TIMER1_COMPA_vect)
	{
	przerwanie++;
	}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//         DS18B20

/*************************************************************************
 delay loop for small accurate delays: 16-bit counter, 4 cycles/loop
*************************************************************************/
static inline void _delayFourCycles(unsigned int __count)
	{
    if ( __count == 0 )    
        __asm__ __volatile__( "rjmp 1f\n 1:" );    // 2 cycles
    else
        __asm__ __volatile__ (
    	    "1: sbiw %0,1" "\n\t"                  
    	    "brne 1b"                              // 4 cycles/loop
    	    : "=w" (__count)
    	    : "0" (__count)
    	   );
	}

// procedura reset 
void ow_reset(void) // ~1ms
	{
	CLR_DQ; // stan niski na linii 1wire
	delay(480); // opóŸnienie ok 480us
	SET_DQ;// stan wysoki na linii 1wire
	delay(480); // opóŸnienie ok 480us
	}

// procedura zapisu bitu na liniê 1wire
void ow_write_bit(char b)  // okolo 110us
	{
	//cli(); // zablokowanie przerwañ
	CLR_DQ; // stan niski na linii 1wire
	delay(5); // opóŸnienie 5us
	if(b) SET_DQ; // jeœli parametr jest niezerowy to ustaw stan wysoki na linii
	//delay(100); // opóŸnienie 100us
	delay(75); // opóŸnienie 75us
	SET_DQ; // stan wysoki na linii 1wire
	//sei(); // odblokowanie przerwan
	}

char ow_read_bit(void) // okolo 15us
	{
	//cli();
	CLR_DQ; // stan niski na linii 1wire
	delay(2);
	SET_DQ; // ustaw stan wysoki na linii
	delay(15);
	//sei();
	if(IN_DQ) return 1; else return 0;
	}

unsigned char ow_read_byte(void) // ~120us
	{
	unsigned char ia;
	unsigned char value = 0;
	for (ia=0;ia<8;ia++)
	{
	if(ow_read_bit()) value|=0x01<<ia;
	delay(8);
	}
	return(value);
	}

void ow_write_byte(char val)  // ~900us
	{
	unsigned char ib;
	unsigned char tmp1;
	for (ib=0; ib<8; ib++)
	{
	tmp1 = val >> ib;
	tmp1 &= 0x01;
	ow_write_bit(tmp1);
	}
	delay(7);
	}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%555



// procedura obslugi I2C
void twistart(void)
	{
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
	}

// procedura transmisji sygna³u STOP
void twistop(void)
	{
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
	while ((TWCR & (1<<TWSTO)));
	}

// procedura transmisji bajtu danych
void twiwrite(char data)
	{
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
	}

//procedura odczytu bajtu danych
char twiread(char ack)
	{
	TWCR = ack
	? ((1 << TWINT) | (1 << TWEN) | (1 << TWEA))
	: ((1 << TWINT) | (1 << TWEN)) ;
	while (!(TWCR & (1<<TWINT)));
	return TWDR;
	}




// definicja dla DHT11
//#define DT 3
//#define SET_DT DDRC &= ~_BV(DT)
//#define CLR_DT DDRC |= _BV(DT)
//#define IN_DT PINC & _BV(DT)

char DHT_read_bit(void) // okolo 15us
	{
	//CLR_DT; // stan niski na linii 1wire
	//delay(2);
	//SET_DT; // ustaw jako wejscie
	//while(IN_DT){} //czeka na 0
	//delay(2);
	while(!(IN_DT)){} //czeka na 1
	delay(36);
	if(IN_DT){while(IN_DT){}; return 1;} else return 0;
	}


void DHT11_start(void)
	{
	unsigned char ia;
	// odczyt danych z DHT11 - temperatura i wilgotnosc
	// start
	CLR_DT; // stan niski na linii 1wire 21ms
	for (ia=0;ia<100;ia++)delay(200); // opóŸnienie ok 21ms
	SET_DT;// ustaw jako wejscie
	//delay(20); // opóŸnienie ok 20us
	//while(IN_DT){ } // jesli jeden to czekaj
	delay(40);
	while(!(IN_DT)); // jesli zero to czekaj 
	delay(75);
	while(IN_DT){ } // jesli jeden to czekaj
	delay(62);
	
	
	}

unsigned char DHT11(void)
	{
	unsigned char ia;
	unsigned char value = 0;

	
	for (ia=0;ia<8;ia++)
		{
		if(DHT_read_bit()) value|=0x01<<(7-ia);
		//if(DHT_read_bit() == 1) while(IN_DT){} //czeka na 0
		//delay(8);
		}
	return(value);
	}



//for (i=0;i<64;i++) LCD_WriteData (add_znaki[i]);


void time(void)
	{
	// I2C
	// procedura odczytu czasu z ds1307
	twistart();
	// procedura transmisji bajtu danych
	twiwrite(0xD0); // adres sprzetowy ds1307
	twiwrite(0x00); // adres pierwszej komorki pamieci ds1307
	twistart();
	twiwrite(0xD1); // adres sprzetowy ds1307 do odczytu danych
	sek = twiread(1); //  sekund
	min = twiread(1); //  minut
	godz = twiread(0); //  godzin
	// procedura transmisji sygna³u STOP
	twistop();
	
	//godz += czas_letni;
	if (czas_letni == 1)
		{
		if (godz == 0x23) godz = 0x00;
		else 
			{
			godz+=((godz & 0x0f)==9) ? 7 : 1;
			godz+=((godz & 0xf0) > 0x90) ? 0x60 : 0; 
			}
		}
	}
	

	
	
	
	
	
void adc(void){ // odczyt danych z adc
	ADCSRA |= _BV(ADSC);	// Start conversion
	while(bit_is_set(ADCSRA,ADSC)){};	 // Oczekiwanie na zakoñczenie przetwarzania max 25 (13) cykli zegara
	//pomiarl = ADCL;
	//pomiarh = ADCH;	
	}



	




  // #######################      Program g³ówny       ####################### 
int main(void)
{
// konfiguracja portów I/O
//       0-wejœcie, 1-wyjœcie
 
DDRA = 0x00; //  portA jako  0000 0000
DDRB = 0x8C; //  portB jako  1000 1100
DDRC = 0x88; //  portC jako  1000 1000
DDRD = 0xFF; //  portD jako  1111 1111

PORTA = 0x00; // bez podciagania napiecia
PORTC = 0x70; // 0111 0000
PORTB = 0x68; // 0110 1000

PORTB |= _BV(2); // ustaw wyjœcie 2 portu B w stan wysoki - wlaczenie przekaznika zasilania

wdt_reset(); // Reset the watchdog timer
wdt_enable(7); // konfiguracja watchdoga na okolo 2 sekundy


//TWSR =0b00000000; 
//TWBR=0x20;

// void aktualna_data(void) {}



unsigned char dzien = 1, miesiac = 1, rok = 15, dzien_tygodnia = 4;
unsigned char wschod_min, wschod_godz, zachod_min, zachod_godz;
double obl_double = 0.0;

int zeroGx = 520, zeroGy = 520;

	

	
// ------ gamma  GREEN = 2,5
uint8_t gamma_dzien[]  = {
  0,  1,  10,  40,  50,  70,  90,  100,  120, 120, 120, 120, 120, 120, 120, 120,
 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120,120,90,50,30, 10
};
	
// ------ gamma  GREEN = 2,5
uint8_t gamma_noc[]  = {
  0,  1,  1,  1,  1,  2,  2,  2,  3,  3,  4,  5,  6,  7,  8,  9, 10, 11,
 13, 15, 16, 18, 21, 23, 25, 28, 31, 34, 38, 45, 55, 70
};	
	
	
/*  int mbar[obr/min][oil temp] mapa cisnienia doladowania w zaleznosci od obrotow i temperatury silnika  */
	int mbar[2][10]= {
{-200 , -100,-50,  -50, 200, 300, 300,200,100  ,100  },
{ -200,-90 , 200, 1000,1100, 900, 800,600,200  ,100 }
	};
// 0  , 500, 1000,1500,2000,2500, 3000,3500,4000,4500 obr/min
//     obr/500
	
/*   PWM[kPa][obr/min] wysterowanie podstawowym % PWM zaworka w zaleznosci od cisnienia i obrotow przy gazie na 100%  */
/*
	int PWM[5][8] = {
//-50,300,600,900, 1200 mbar
  { 0, 0 , 0 , 0 , 90 },  // 900 
  { 0, 30, 40, 50, 90 },  // 1500 
  { 0, 30, 40, 50, 90 },  // 2000 
  { 0, 20, 30, 40, 90 },  // 2500 
  { 0, 20, 30, 40, 90 },  // 3000 
  { 0, 15, 20, 30, 90 },  // 3500 
  { 0, 15, 20, 30, 90 },  // 4000 
  { 0, 10, 10, 10, 90 }   // 4500 
};
*/

i = 10;
while(i > 0)
	{
	delay(10000); // opóŸnienie 10ms x 10 = 100ms
	i--;
	}
	
//******************** lcd1 *********************//
PORTB &= ~_BV(7); // ustaw wyjœcie 7 portu B w stan niski
LCD_Initalize();
// dodatkowe znaki do lcd
LCD_WriteCommand(0x40);
//for (i=0;i<64;i++) LCD_WriteData (add_znaki[i]);
for (i=0;i<64;i++) LCD_WriteData (pgm_read_byte (&(add_znaki[i])));


LCD_Clear();							
LCD_GoTo(0,0);
LCD_WriteText(" v 1.5  ");


utoa(MCUCSR, ch, 2); 
LCD_WriteText(ch);
MCUCSR = 0;

//**************** I2C *******************//
// procedura odczytu daty z ds1307
twistart();
// procedura transmisji bajtu danych
twiwrite(0xD0); // adres sprzetowy ds1307
twiwrite(0x03); // adres pierwszej komorki pamieci ds1307 do odczytu
twistart();
twiwrite(0xD1); // adres sprzetowy ds1307 do odczytu danych
dzien_tygodnia = twiread(1); 
dzien = twiread(1); 
miesiac = twiread(1); 
rok = twiread(0); 
// procedura transmisji sygna³u STOP
twistop();

//konwersja BCD na DEC
//0x09->9 9/16 = 0   9%16 = 9
//0x10->10 16/16 = 1   16%16 = 0
//0x11->11 17/16 = 1   17%16 = 1
dzien =  (dzien >> 4) * 10 + (dzien & 0x0F);
miesiac =  (miesiac >> 4) * 10 + (miesiac & 0x0F);

//obliczenie czasu wschodu slonca
//=(4,6-1,6*SIN((30,4*D6+D7-111)/57,3)*1,2)+1
obl_double = sin((30.4 * miesiac + dzien - 111)/57.3);
obl_double = 5.6-1.9*obl_double;
//obl_double = 6600-1900*obl_double;

wschod_godz = (char)(obl_double); 
wschod_min = (char)((obl_double - wschod_godz)*60);
// zmienia bajt w kodzie binarnym na bajt zakodowany w BCD 
// 6 = 0110 => 0110
wschod_godz =  wschod_godz/10<<4 | wschod_godz%10;



//obliczenie godziny zachodu slonca
obl_double = sin((30.4 * miesiac + dzien - 111)/57.3);
obl_double = 17.8+2*obl_double;

zachod_godz = (char)(obl_double); 
zachod_min = (char)((obl_double - zachod_godz)*60);
// zmienia bajt w kodzie binarnym na bajt zakodowany w BCD 
// 18 = 0001 0010 => 0001 
zachod_godz =  zachod_godz/10<<4 | zachod_godz%10;

//read_history();
//void read_history(void) 
	//{// I2C
	// procedura odczytu histori parametrow silnika z ds1307
	twistart();
	// procedura transmisji bajtu danych
	twiwrite(0xD0); // adres sprzetowy ds1307
	twiwrite(0x08); // adres komorki pamieci ds1307
	twistart();
	twiwrite(0xD1); // adres sprzetowy ds1307 do odczytu danych
	on_off_h = twiread(1); //  0x08
	on_off_l = twiread(1); //  0x09
	engine_on_off_h = twiread(1); //  0x0A
	engine_on_off_l = twiread(1); //  0x0B
	
	engine_time[0] = twiread(1); //  0x0C
	max_boost[0] = twiread(1); //  0x0D
	t_oil[0] = twiread(1); //  0x0E
	max_obr[0] = twiread(1); //  0x0F
	travel[0] = twiread(1);
	avg_speed[0] = twiread(1);
	ee[0] = twiread(1);
	ee[1] = twiread(1);
	ee[2] = twiread(1);
	fuel_old = twiread(1); // 0x15 ilosc paliwa w baku zapisany z dokladnoscia 0,5L
	
	engine_time[1] = twiread(1); //  0x16
	engine_time[2] = twiread(1); 
	engine_time[3] = twiread(1); 
	engine_time[4] = twiread(1);

	max_boost[1] = twiread(1); //  0x1A
	max_boost[2] = twiread(1); 
	max_boost[3] = twiread(1); 
	max_boost[4] = twiread(1); 

	t_oil[1] = twiread(1); //  0x1E
	t_oil[2] = twiread(1); 
	t_oil[3] = twiread(1); 
	t_oil[4] = twiread(1);

	max_obr[1] = twiread(1); //  0x22
	max_obr[2] = twiread(1); 
	max_obr[3] = twiread(1); 
	max_obr[4] = twiread(1); 
	
	travel[1] = twiread(1); //  0x26
	travel[2] = twiread(1); 
	travel[3] = twiread(1); 
	travel[4] = twiread(1); 
	
	avg_speed[1] = twiread(1); //  0x2A
	avg_speed[2] = twiread(1); 
	avg_speed[3] = twiread(1); 
	avg_speed[4] = twiread(1); 
	
	// sprawdzenie poprawnosci odczytu pamieci
	i = twiread(0); 
	
	twistop();// procedura transmisji sygna³u STOP
	
	if (engine_on_off_h > 100) {engine_on_off_h = 0; engine_on_off_l = 0;}
	
	if (i == 250)
		{
		fuel_old *= 5; // x5
		poczatkowe_paliwo = fuel_old;
		avg_paliwo = poczatkowe_paliwo * 64;
		km_total = (ee[0]<<16) | (ee[1]<<8) | ee[2];
		}
	else 
		{
		fuel_old = 50; 
		avg_paliwo = 3200;
		on_off_h = 0; 
		on_off_l = 0; 
		engine_on_off_h = 0; 
		engine_on_off_l = 0; 
		
		twistart();
		// procedura transmisji bajtu danych
		twiwrite(0xD0); // adres sprzetowy ds1307
		twiwrite(0x00); // adres komorki pamieci ds1307
		twiwrite(0x05); // ustawienie sekund
		twiwrite(0x00); // ustawienie minut
		twiwrite(0x00); // ustawienie godzin
		twistop();
		}
	//}


LCD_GoTo(3,1);
if(dzien < 10)LCD_WriteText("0");
itoa(dzien, ch, 10); 
LCD_WriteText(ch);

LCD_WriteText("-");

if(miesiac < 10)LCD_WriteText("0");
itoa(miesiac, ch, 10); 
LCD_WriteText(ch);

LCD_WriteText("-");

if(rok < 10)LCD_WriteText("0");
LCD_WriteText("20");
itoa(rok, ch, 16); 
LCD_WriteText(ch);


// ************* lcd2 ******************//
PORTB |= _BV(7); // ustaw wyjœcie 7 portu B w stan wysoki
LCD_Initalize();
// dodatkowe znaki do lcd
LCD_WriteCommand(0x40);
//for (i=0;i<64;i++) LCD_WriteData (add_znaki[i]);
for (i=0;i<64;i++) LCD_WriteData (pgm_read_byte (&(add_znaki[i])));
LCD_Clear();




//********* ilosc startow uC, silnika ***********//
LCD_GoTo(0,0);
LCD_WriteText("uC on/off: ");
itoa(on_off_h*256 + on_off_l, ch, 10); 
LCD_WriteText(ch);

LCD_GoTo(0,1);
LCD_WriteText("engine: ");
utoa(engine_on_off_h*256 + engine_on_off_l, ch, 10); 
LCD_WriteText(ch);
LCD_WriteText("x");



wdt_reset(); // Reset the watchdog timer

i = 70;
while(i > 0)
	{
	delay(10000); // opóŸnienie 10ms x 70 = 700ms
	i--;
	}

wdt_reset(); // Reset the watchdog timer


i = 80;
while(i > 0)
	{
	delay(10000); // opóŸnienie 10ms x 80 = 800ms
	i--;
	}

wdt_reset(); // Reset the watchdog timer




/*
twistart();
// procedura transmisji bajtu danych
twiwrite(0xD0); // adres sprzetowy ds1307
twiwrite(0x0A); // adres komorki pamieci ds1307
twiwrite(0); // 0x0A
twiwrite(0); // 0x0B
twistop();
*/

// procedura zapisu danych do ds1307

/*
twistart();
// procedura transmisji bajtu danych
twiwrite(0xD0); // adres sprzetowy ds1307
twiwrite(0x00); // adres komorki pamieci ds1307
twiwrite(0x30); // ustawienie sekund
twiwrite(0x27); // ustawienie minut
twiwrite(0x10); // ustawienie godzin wg czasu zimowego
twistop();
*/

/*
twiwrite(0x07); //  dzien tygodnia
twiwrite(0x26); //  dzien miesiaca
twiwrite(0x09); //  miesiac
twiwrite(0x15); // rok
twiwrite(0x00); // rejestr 07h
// procedura transmisji sygna³u STOP
twistop();
*/

/*
twistart();
// procedura transmisji bajtu danych
twiwrite(0xD0); // adres sprzetowy ds1307
twiwrite(0x2e); // adres komorki pamieci ds1307
twiwrite(250); // sta³a do sprawdzenian poprawnosci zapisu pamieci
twistop();
*/



/*
// zapis wartosci poczatkowych do pamieci
twistart();
// procedura transmisji bajtu danych
twiwrite(0xD0); // adres sprzetowy ds1307
twiwrite(0x10); // adres komorki pamieci ds1307
 
		twiwrite(0); // travel
		twiwrite(0); // avg speed
		twiwrite(9);
		twiwrite(133);
		twiwrite(158);
		twiwrite(11);
		
		twiwrite(195); 
		twiwrite(16); 
		twiwrite(12); 
		twiwrite(0); 
		
		twiwrite(94); 
		twiwrite(78); 
		twiwrite(62); 
		twiwrite(0); 
		
		twiwrite(102); 
		twiwrite(122); 
		twiwrite(126); 
		twiwrite(40); 
		
		twiwrite(38); 
		twiwrite(27); 
		twiwrite(26); 
		twiwrite(0); 
		
		twiwrite(200); 
		twiwrite(0); 
		twiwrite(0); 
		twiwrite(0); 
		
		twiwrite(0); 
		twiwrite(0); 
		twiwrite(0); 
		twiwrite(0); 
		
		twistop();// procedura transmisji sygna³u STOP  





		// zapis wartosci poczatkowych do pamieci
		twistart();
		// procedura transmisji bajtu danych
		twiwrite(0xD0); // adres sprzetowy ds1307
		twiwrite(0x15); // adres komorki pamieci ds1307
 
		twiwrite(72); 
		//twiwrite(0); 
		//twiwrite(0);
		//twiwrite(0);


		twistop();// procedura transmisji sygna³u STOP 
*/


/*
fuel_old = 400; // 40 l
twistart(); // zapis do pamieci aktualnej ilosci paliwa 
// procedura transmisji bajtu danych
twiwrite(0xD0); // adres sprzetowy ds1307
twiwrite(0x15); // adres komorki pamieci ds1307
twiwrite(fuel_old/5); // 0x15
twistop();// procedura transmisji sygna³u STOP
*/

/*
// aktualizacja przejechanych kilometrow
		//km_total = 3000; // 300.0
		
		km_total = 9475; // 947,5
		
		ee[0] = km_total >> 16;
		//eeprom_write_byte (&eeprom_adres, ee[0]); // zapis do eeprom
		
		ee[1] = km_total >> 8;
		ee[1] &= 0x0000FF;
		//eeprom_write_byte (&eeprom_adres+1, ee[1]); // zapis do eeprom
		
		ee[2] = km_total & 0x0000FF;
		//eeprom_write_byte (&eeprom_adres+2, ee[2]); // zapis do eeprom
		
		twistart();
		// procedura transmisji bajtu danych
		twiwrite(0xD0); // adres sprzetowy ds1307
		twiwrite(0x12); // adres komorki pamieci ds1307
		twiwrite(ee[0]);
		twiwrite(ee[1]);
		twiwrite(ee[2]);
		twistop();// procedura transmisji sygna³u STOP
*/

/*
		s = 4000000;
		twistart();
		// procedura transmisji bajtu danych
		twiwrite(0xD0); // adres sprzetowy ds1307
		twiwrite(0x26); // adres komorki pamieci ds1307
		twiwrite(s/1000000);

		twistop();// procedura transmisji sygna³u STOP
*/



// zapis do pamieci liczby startow uC
// procedura zapisu danych do pamieci RAM ds1307
if(on_off_l == 255){ on_off_h++; on_off_l = 0;} else on_off_l++;

//on_off_h = 0; on_off_l = 0;

twistart();
// procedura transmisji bajtu danych
twiwrite(0xD0); // adres sprzetowy ds1307
twiwrite(0x08); // adres komorki pamieci ds1307
twiwrite(on_off_h);
twiwrite(on_off_l);
//for(i=56;i--;i==0)twiwrite(0x00); // zerowanie calej pamieci
twistop();// procedura transmisji sygna³u STOP

/*
//kasowaie ilosc startow silnika
twistart();
// procedura transmisji bajtu danych
twiwrite(0xD0); // adres sprzetowy ds1307
twiwrite(0x0A); // adres komorki pamieci ds1307
twiwrite(0);
twiwrite(0);
//for(i=56;i--;i==0)twiwrite(0x00); // zerowanie calej pamieci
twistop();// procedura transmisji sygna³u STOP
*/

time();

sec = sek; // zapamiêtanie sekundy wlaczenia uC


wdt_reset();

 
if( (miesiac > 3 && miesiac < 10)	// miesiace od kwietnia do wrzesnia wlacznie
	|| (miesiac == 03 && dzien > ( 31 - ( ( 30 - dzien + dzien_tygodnia ) % 7 ) ) )	// dni marca po ostaniej niedzieli
	|| (miesiac == 03 && dzien == ( 31 - ( ( 30 - dzien + dzien_tygodnia ) % 7 ) ) && godz > 0 )	// ostatnia niedziela marca, po godzinie 1.00
	|| (miesiac == 10 && ( dzien < ( 31 - ( ( 30 - dzien + dzien_tygodnia ) % 7 ) ) ) )	// dni pazdziernika przed ostatnia niedziela
	//                       1 < 31 - 4
	//                       2 < 31 - 4
	//                       6 < 31 - 4
	//                       13 < 31 - 4
	//                       27 == 31 - 4
	//                       26 < 31 - 4
	|| (miesiac == 10 && dzien == ( 31 - ( ( 30 - dzien + dzien_tygodnia ) % 7 ) ) && godz < 0x01 ) )	// po ostatniej niedzieli pazdziernika, przed godzina 1.00
{
	czas_letni = 0x01;	// czas letni
	wschod_godz+=((wschod_godz & 0x0f)==9) ? 7 : 1;
	wschod_godz+=((wschod_godz & 0xf0) > 0x90) ? 0x60 : 0; 
	zachod_godz+=((zachod_godz & 0x0f)==9) ? 7 : 1;
	zachod_godz+=((zachod_godz & 0xf0) > 0x90) ? 0x60 : 0; 
	if (godz > 0x22) { dzien ++; dzien_tygodnia ++; }
}
else
{
	czas_letni = 0x00;	// czas zimowy
}

wdt_reset();

//wschod_godz += czas_letni;
//zachod_godz += czas_letni;


//pierwszy odczyt z dht
DHT11_start();
wdt_reset();
DHT11_data[0] = DHT11();
wdt_reset();
DHT11();
wdt_reset();
DHT11_data[1] = DHT11();




// ustawienie dzielnika na 64 czyli f= 125kHz
// Enable A/D 
//ADCSR 
ADCSRA = 134;

//konfiguracja timer0 PWM - Fast PWM Mode,  pin OC0 (portB.3) jako wyjscie pwm
// sterowanie podswietleniem zielonym wyswietlacza
//TCCR0 = 108; // 0110 1100  // 8 MHz / (256*256) = 122Hz
//OCR0 = 0; // 0/255 = 0% - poczatkowa wartosc PWM 


//konfiguracja timer2 PWM - Fast PWM Mode,  pin OC2 (portD.7) jako wyjscie pwm
// sterowanie cisnieniem doladowania turbiny PWN N75
//TCCR2 = 102; // 0110 0110 // 8 MHz / (510*256) = 61Hz
TCCR2 = 111; // 0110 1111 // 8 MHz / (1024*256) = 30,5Hz
OCR2 = 0; // 0/255 = 0% - poczatkowa wartosc PWM N75
  
// konfiguracja timer0
TCCR0 = 7; // T0 ON, sygnal z T0(PB0) - obroty silnika




	


//PORTB |= _BV(1); // ustaw wyjœcie 1 portu B w stan wysoki 
//PORTB &= ~_BV(1); // ustaw wyjœcie 1 portu B w stan niski 

//******************** lcd1 *********************//
PORTB &= ~_BV(7); // ustaw wyjœcie 7 portu B w stan niski
LCD_Clear();


wdt_reset(); // Reset the watchdog timer

//konfiguracja timer1
TCCR1B = 0x0A;	//   1010 Timer1 pracuje z preskalerem 8 w trybie CTC
// 0x0D90 = 1101 1001 0000 = 3472
// 31250Hz -

//1000000Hz
//
OCR1AH = 0x2B;	// Czas miêdzy przerwaniami 11,111ms
OCR1AL = 0x67;		
TIMSK |= _BV(OCIE1A);   // Zezwolenie na przerwania od licznikow
TIFR |= _BV(TOV1);		// Zezwolenie na przerwanie od przepe³nienia
sei();



//wdt_disable();
// pusta pêtla nieskonczona

// zapis i odczyt EEPROM ATMega
//eeprom_read_word ( *adres ) - czyta 16 bitow¹ zawartoœæ pamiêci spod adresu adres.
//eeprom_write_word ( *buf, *adres) - zapisuje wartoœci 16bit ze zmiennej buf pod adres
//
//eeprom_read_block ( *buf, *adres, n) - czyta n wartoœci od adresu adres i zapisuje do pamiêci SRAM w *buf.
//eeprom_write_block ( *buf, *adres, n) - zapisuje n wartoœci od adresu adres ze zmiennej *buf.
//
//

/*
ee0 = eeprom_read_byte(&eeprom_adres); // odczyt z eeprom

if (eeprom[0] == 0){
	eeprom[1] = eeprom_read_byte(&eeprom_adres+1); // odczyt z eeprom
	eeprom[2] = eeprom_read_byte(&eeprom_adres+2); // odczyt z eeprom
	eeprom[3] = eeprom_read_byte(&eeprom_adres+3); // odczyt z eeprom
	}
 else {
	// 62 403 0 hm
	// 
	ee0 = 0; // zmienna kontrolna nie wykasowania pamieci EEPROM
	ee[0] = 9;
	ee[1] = 133;
	ee[2] = 158;
	eeprom_write_byte (&eeprom_adres+1, ee[0]); // zapis do eeprom
	eeprom_write_byte (&eeprom_adres+2, ee[1]); // zapis do eeprom
	eeprom_write_byte (&eeprom_adres+3, ee[2]); // zapis do eeprom
	eeprom_write_byte (&eeprom_adres, ee0); // zapis do eeprom
	}


 km_total = (ee[0]<<16) | (ee[1]<<8) | ee[2];

*/

//************************************************//
		/*
			unsigned int gamma1, gamma2, gamma3, gamma4;
			//odczyt pamieci radia gamma po I2C
			
			twistart();
			// procedura transmisji bajtu danych
			twiwrite(0x0A); // adres sprzetowy
			twiwrite(0x0E); // adres pierwszej komorki pamieci 
			twistart();
			twiwrite(0x0B); // adres sprzetowy do odczytu danych
			gamma1 = twiread(1); 
			gamma2 = twiread(1); 
			gamma3 = twiread(1); 
			gamma4 = twiread(0); 
			// procedura transmisji sygna³u STOP
			twistop();
		
			LCD_GoTo(0,0);
			utoa(gamma1, ch, 16); 
			LCD_WriteText(ch);
			LCD_WriteText("-");
			utoa(gamma2, ch, 16); 
			LCD_WriteText(ch);
			LCD_WriteText("-");
			utoa(gamma3, ch, 16); 
			LCD_WriteText(ch);
			LCD_WriteText("-");
			utoa(gamma4, ch, 16); 
			LCD_WriteText(ch);
			*/
			//************************************************//
//wdt_reset(); // Reset the watchdog timer




for(;;)
	{

if(przerwanie > 9)
	{
	
	i++;
	
	//zmiana wejscia na x1 000 - temperatura oleju
	ADMUX = 0;
	adc();
	//adc0 = (pomiarh * 256) + pomiarl;
	//adc0 += 1023 - ( (pomiarh << 8) | pomiarl );
	adc0 += 1023 - ADCW;
	
	// zmiana wejscia na adc1 001 - cisnienie doladowania
	ADMUX |= _BV(0);	// 0->1
	//	ADMUX &= ~_BV(1);  // 1->0
	//	ADMUX &= ~_BV(2);  // 1->0
	
	//adc0 = 1023 - adc0;
	adc0 = adc0 >> 1;
	
	adc();	
	boost = ADCW;
	
	// zmiana wejscia na 01100 - ADC2 x10 
	//ADMUX = 13;
	ADMUX = 2;
	
	boost = (boost - 41) * 2.58;   // obliczenia doladowania w mbar
	boost_b += boost;
	avg_boost += boost; 
		
	
	// pomiar ADC2 
	adc();
	//ay = (pomiarh << 8) | pomiarl;
	adc2 += ADCW;
	adc2 /= 2;
	
	
	//zmiana wejscia na 011
	ADMUX = 3;  // 0->1

	
	// pomiar we ADC3
	adc();
	adc3 += ADCW;
	adc3 /= 2;
	
	// zmiana wejscia na  100
	//ADMUX &= ~_BV(0); // 1->0	
	//ADMUX &= ~_BV(1); // 1->0
	//ADMUX |= _BV(2);  // 0->1
	ADMUX = 4;

	if (i==6)
		{
		adc();
		Ubl = ADCW;
		}
	

	
	// zmiana wejscia na  101
	ADMUX |= _BV(0);  // 0->1
	//	ADMUX &= ~_BV(1); // 1->0
	// ADMUX |= _BV(2);  // 0->1
	adc();
	//adc5=(pomiarh * 256) + pomiarl;
	//Uacc += adc5;
	Uacc += ADCW;
	
	// zmiana wejscia na 110 - poziom paliwa
	//	ADMUX &= ~_BV(0); // 1->0
	//	ADMUX |= _BV(1);  // 0->1
	//	ADMUX |= _BV(2);  // 0->1
	ADMUX = 6;
	adc();
	//adc6 = (pomiarh * 256) + pomiarl;
	//fuel += adc6;
	fuel += ADCW;
	paliwo = ADCW;
	
	
	// zmiana wejscia na 111 - temp silnika
	ADMUX |= _BV(0);  // 0->1
	//	ADMUX |= _BV(1);  // 0->1
	//	ADMUX |= _BV(2);  // 0->1
	adc();
	e_temp += ADCW;



/*
	// 0-1024 *4 = 0-4096    100  = 1m/s^2
	// niwelowanie szpilek przyœpieszenia
	ax = 4*(ax - zero_x);
	if (ax > 800) ax = 800;
	if (ax < -800) ax = -800;
	x += ax;
	y += 4*(ay - zero_y);
*/
	
	
	//************ pomiar obrotów co 333,3ms ************//

	rpm += TCNT0;
	TCNT0 = 0;
	
	if(i == 2){ 
	if (rpm < 100) { obr += rpm * 10;  obr /= 2;  } // jesli obroty sa mniejsze od 1000 to male usrednianie jest wykonywane
	else obr = rpm * 10;
	rpm = 0;
	}
	
	if(i == 5){ 
	if (rpm < 100) { obr += rpm * 10;  obr /= 2;  } // jesli obroty sa mniejsze od 1000 to male usrednianie jest wykonywane
	else obr = rpm * 10;
	rpm = 0;
	}
	
	if(i == 8){ 
	if (rpm < 100) { obr += rpm * 10;  obr /= 2;  } // jesli obroty sa mniejsze od 1000 to male usrednianie jest wykonywane
	else obr = rpm * 10;
	rpm = 0;
	
	}




	
	
	
	
	// PWM 0 - 255 = 0% - 100%
	// PWM = 0% jesli silnik jest wylaczony lub czas dzialania silnika jest krotszy niz 60 sekund
	OCR2 = 0; // wylaczony PWM N75
	
	

	
	// odczyt temp1 z DS18B20 co 1 sekunde
		if(i == 6  && odczyt_10sekund == 2)
		{
			ow_reset();
			// ow_write_byte(0xCC); // Wyslanie komendy pominiecia porownywania numeru seryjnego - 0xCC 
			ow_write_byte(0x55); // wyslanie komendy - MATCH ROM - 0x55 
			ow_write_byte(0x28);// wyslanie 8 bajtow numeru seryjnego czujnika numer 1
			ow_write_byte(0xC9);
			ow_write_byte(0x8E);
			ow_write_byte(0x25);
			ow_write_byte(0x02);
			ow_write_byte(0x00);
			ow_write_byte(0x00);
			ow_write_byte(0xC0);
			
			ow_write_byte(0xBE); // wyslanie komendy do odczytu danych - 0xBE 
			lsb = ow_read_byte();
			msb = ow_read_byte();
			
			// konwersja wyniku na liczbe calkowita ze znakiem 
			temp1r = lsb;  // czesc ulamkowa 4 cyfrowa 0,xxxx
			temp1r &= 0x0F; // x 0000 1111
			temp1r *= 625;
			lsb = lsb >> 4;
			//msb &= 0x07;
			msb = msb << 4;
			temp1 = lsb | msb;
			if(temp1 > 127) // jesli temp jest ujemna
				{
				temp1 = temp1 - 256;
				//if(temp1 > 127) temp1 = -(~temp1 + 1); // jesli temp jest ujemna
				temp1r = 10000 - temp1r;
				}
			
			ow_reset();
			ow_write_byte(0x55); // wyslanie komendy - MATCH ROM - 0x55 
			ow_write_byte(0x28);// wyslanie 8 bajtow numeru seryjnego czujnika numer 2
			ow_write_byte(0x79);
			ow_write_byte(0xAA);
			ow_write_byte(0x25);
			ow_write_byte(0x02);
			ow_write_byte(0x00);
			ow_write_byte(0x00);
			ow_write_byte(0x20);
			
			ow_write_byte(0xBE); // wyslanie komendy do odczytu danych - 0xBE 
			lsb = ow_read_byte();
			msb = ow_read_byte();
			
			// konwersja wyniki na liczbe calkowita ze znakiem 
			lsb = lsb >> 4;
			//msb &= 0x07; // msb &= 0x0F;
			msb = msb << 4;
			temp2 = lsb | msb;
			if(temp2 > 127) temp2 = temp2 - 256; // jesli temp jest ujemna
		}
		
		if(i == 7  && odczyt_10sekund == 2)
		{
			ow_reset();
			ow_write_byte(0x55); // wyslanie komendy - MATCH ROM - 0x55 
			ow_write_byte(0x28);// wyslanie 8 bajtow numeru seryjnego czujnika numer 3
			ow_write_byte(0x39);
			ow_write_byte(0x8E);
			ow_write_byte(0x25);
			ow_write_byte(0x02);
			ow_write_byte(0x00);
			ow_write_byte(0x00);
			ow_write_byte(0xB2);
			
			ow_write_byte(0xBE); // wyslanie komendy do odczytu danych - 0xBE 
			lsb = ow_read_byte();
			msb = ow_read_byte();
			
			// konwersja wyniki na liczbe calkowita ze znakiem 
			lsb = lsb >> 4;
			//msb &= 0x07;
			msb = msb << 4;
			temp3 = lsb | msb;
			if(temp3 > 127) temp3 = temp3 - 256; // jesli temp jest ujemna
		}
		
		if(i == 8  && odczyt_10sekund == 1)
			{
			ow_reset();
			ow_write_byte(0xCC); // Wyslanie komendy pominiecia porownywania numeru seryjnego - 0xCC 
			ow_write_byte(0x44); // Wyslanie komendy konwersji temperatury - 0x44 
			// Po tej kombinacji wszystkie termometry zaczna konwersje temperatury. 
			
			}
		
		if(i == 6  && odczyt_10sekund == 0)
			{ 
			// odczyt z dht
			DHT11_start();
			DHT11_data[0] = DHT11();
			DHT11();
			DHT11_data[1] = DHT11();
			//DHT11_data[3] = DHT11();
			}
		
		
	/*
	if (engine_on_sec < 22)
		{
		if( (i == 7) && ( (lcd2 == 3) || (lcd2 == 4) ) ) time();
		}
	else 
		{
		if( (engine_on_sec % 16) && (i == 7) && (lcd2 == 4) ) time();
		}
	*/
	
	if( (i == 7) && (during_off == 0)  ) time();
	
	
	if ( (obr == 0) && (engine_pp == 1) && (engine_off_sec > 9) ) 
		{
		// 10 sekund po zgaszeniu silnika
		// zapis do eeprom calkowitej ilosci kilometrow
		
		
		
		km_total += ( (aktualny_dystans+5)/10 ); // ilosc kilometrow z dokladnoscia 0,1km
		
		ee[0] = km_total >> 16;
		//eeprom_write_byte (&eeprom_adres, ee[0]); // zapis do eeprom
		
		ee[1] = km_total >> 8;
		ee[1] &= 0x0000FF;
		//eeprom_write_byte (&eeprom_adres+1, ee[1]); // zapis do eeprom
		ee[2] = km_total & 0x0000FF;
		//eeprom_write_byte (&eeprom_adres+2, ee[2]); // zapis do eeprom
		
		rrr = aktualny_dystans/100;
		
		twistart();
		// procedura transmisji bajtu danych
		twiwrite(0xD0); // adres sprzetowy ds1307
		twiwrite(0x0F); // adres komorki pamieci ds1307
		twiwrite(max_OBR/100);       // 0x0F
		twiwrite(rrr);       // 0x10 - aktualny dystans
		twiwrite(avg_SPEED);       // 0x11
		twiwrite(ee[0]);
		twiwrite(ee[1]);
		twiwrite(ee[2]);
		twistop();// procedura transmisji sygna³u STOP
		
		twistart();
		// zapis stalej liczby do pamieci
		twiwrite(0xD0); // adres sprzetowy ds1307
		twiwrite(0x2E); // adres komorki pamieci ds1307
		twiwrite(250);       // 0x2E
		twistop();// procedura transmisji sygna³u STOP
		
		// przesuniecie historii tylko na zmiennych, bez niepotrzebnego czytania z pamieci
		engine_time[4] = engine_time[3];
		max_boost[4] = max_boost[3];
		t_oil[4] = t_oil[3];
		max_obr[4] = max_obr[3];
		travel[4] = travel[3];
		avg_speed[4] = avg_speed[3];
		
		engine_time[3] = engine_time[2];
		max_boost[3] = max_boost[2];
		t_oil[3] = t_oil[2];
		max_obr[3] = max_obr[2];
		travel[3] = travel[2];
		avg_speed[3] = avg_speed[2];
		
		engine_time[2] = engine_time[1];
		max_boost[2] = max_boost[1];
		t_oil[2] = t_oil[1];
		max_obr[2] = max_obr[1];
		travel[2] = travel[1];
		avg_speed[2] = avg_speed[1];
		
		engine_time[1] = engine_time[0];
		max_boost[1] = max_boost[0];
		t_oil[1] = t_oil[0];
		max_obr[1] = max_obr[0];
		travel[1] = travel[0];
		avg_speed[1] = avg_speed[0];
		
		engine_time[0] = engine_on_sec/60;
		max_boost[0] = max_BOOST/10;
		t_oil[0] = oil_temp;
		max_obr[0] = max_OBR/100;
		travel[0] = rrr;
		avg_speed[0] = avg_SPEED;

		engine_pp = 0;
		engine_on_sec = 0;
		//aktualny_dystans = 0;
		
		}
	
	if( obr > 600 && engine_pp == 0 && engine_on_sec > 4) // zwiekszenie liczby startow silnika, zapis przesunietej historii
		{ 
		// 5 sekund po uruchomieniu silnika
		max_BOOST = 0;               
		max_OBR = 600;
		//aktualny_dystans = 0;
		//rrr = 0;
		avg_SPEED = 0;
		engine_pp = 1;
		// procedura odczytu i zapisu danych do pamieci RAM ds1307
		// zwiekszenie licznika startow silnika
		if(engine_on_off_l == 255){ engine_on_off_h++; engine_on_off_l = 0;} else engine_on_off_l++;
		
		
		
		twistart();
		// procedura transmisji bajtu danych
		twiwrite(0xD0); // adres sprzetowy ds1307
		twiwrite(0x0A); // adres komorki pamieci ds1307
		twiwrite(engine_on_off_h); // 0x0A
		twiwrite(engine_on_off_l); // 0x0B
		twiwrite(engine_on_sec/60); // 0x0C
		twiwrite(0);        // 0x0D
		twiwrite(oil_temp);        // 0x0E
		twiwrite(6);        // 0x0F
		
		twiwrite(rrr); // aktualny dystans
		twiwrite(avg_SPEED);
		twiwrite(ee[0]);
		twiwrite(ee[1]);
		twiwrite(ee[2]);
		twiwrite(fuel_old/5);
		
		twiwrite(engine_time[0]); 
		twiwrite(engine_time[1]); 
		twiwrite(engine_time[2]); 
		twiwrite(engine_time[3]); 
		//twiwrite(engine_time[4]); 
		
		twiwrite(max_boost[0]); 
		twiwrite(max_boost[1]); 
		twiwrite(max_boost[2]); 
		twiwrite(max_boost[3]); 
		//twiwrite(max_boost[4]); 
		
		twiwrite(t_oil[0]); 
		twiwrite(t_oil[1]); 
		twiwrite(t_oil[2]); 
		twiwrite(t_oil[3]); 
		//twiwrite(t_oil[4]); 
		
		twiwrite(max_obr[0]); 
		twiwrite(max_obr[1]); 
		twiwrite(max_obr[2]); 
		twiwrite(max_obr[3]); 
		//twiwrite(max_obr[4]); 
		
		twiwrite(travel[0]); 
		twiwrite(travel[1]); 
		twiwrite(travel[2]); 
		twiwrite(travel[3]); 
		//twiwrite(travel[4]); 
		
		twiwrite(avg_speed[0]); 
		twiwrite(avg_speed[1]); 
		twiwrite(avg_speed[2]); 
		twiwrite(avg_speed[3]); 
		//twiwrite(avg_speed[4]); 
		
		twistop();// procedura transmisji sygna³u STOP        
		}
		
		
	if ( (obr > 600) && (engine_on_sec%30 == 0) && (engine_on_sec > 8) && (i == 3) ) // zapis do pamieci co 30 sekund pracy silnika aktualnego czasu pracy silnika, max doladowania, max obr, temp oleju, przejechanego dystansu, sredniej predkosci
		{
		rrr = aktualny_dystans/100;
		twistart();
		// procedura transmisji bajtu danych
		twiwrite(0xD0); // adres sprzetowy ds1307
		twiwrite(0x0C); // adres komorki pamieci ds1307
		twiwrite(engine_on_sec/60); // 0x0C
		twiwrite(max_BOOST/10);       // 0x0D
		twiwrite(oil_temp);          // 0x0E
		twiwrite(max_OBR/100);       // 0x0F
		twiwrite(rrr);       // 0x10 - aktualny dystans
		twiwrite(avg_SPEED);       // 0x11
		twistop();// procedura transmisji sygna³u STOP
		}
	
	if ( (i == 6) && (during_off != 255))
		{
		if (Ubl > 564) Ubl -= 564; else Ubl = 0; 
		//pwm_podsw = Ubl/12;
		//if (pwm_podsw < 1) pwm_podsw = 0;
		//if (pwm_podsw > 31) pwm_podsw = 31;
		
		//if (obr == 0 && pwm_podsw < 3) pwm_podsw = 18; // wlaczenie podswietlenia na wylaczonych swiatlach
		
		pwm_podsw = 18; // wlaczenie podswietlenia 
		
		if ( (godz > wschod_godz) && (godz < zachod_godz))
			{
			pwm_podsw = gamma_dzien[pwm_podsw]; // wartosc pwm atmega8 - wieksza maksymalna jasnosc w dzien
			}
		else 
			{
			pwm_podsw = gamma_noc[pwm_podsw]; //  wartosc pwm atmega8 w nocy
			}
		
		if ( ((godz == wschod_godz) || (godz == zachod_godz)) && min < 0x30) pwm_podsw = 50 + (gamma_noc[pwm_podsw]); // pwm podczas wschodu i zachodu slonca
		if ( ((godz == wschod_godz) || (godz == zachod_godz)) && min > 0x29) pwm_podsw = 25 + (gamma_noc[pwm_podsw]); // pwm podczas wschodu i zachodu slonca
		
		if ((info > 57) && (i == 1)) Ubl_blue = 128; // zmien kolor wyswietlacza co 1 sekunde
		else Ubl_blue = 0;
		
		
		if (temp1 < 27)
			{
			Ubl_blue = (26 - temp1) * pwm_podsw;
			Ubl_blue = Ubl_blue >> 6; // dzielenie przez 64
			if (pwm_podsw > Ubl_blue) pwm_podsw = pwm_podsw - Ubl_blue; else pwm_podsw = 0;
			Ubl_blue *= 2;
			if (Ubl_blue > 240) Ubl_blue = 240;
			// 
			// 
			}
		// zmiana reagowania koloru podswietlenia
		// temp1 =  25C   (26-25)*10=10   Ubl_blue = 5   pwm_podsw = 40     5%
		// temp1 =  23C   (26-23)*10=30    Ubl_blue = 35  pwm_podsw = 30    30%
		// temp1 =  21C   (26-21)*10=50    Ubl_blue = 55  pwm_podsw = 10    70%  100%
		// temp1 =  19C   (26-19)*10=70     Ubl_blue = 85  pwm_podsw = 0    100%	
			
		// temp =  40C   temp = 80   Ubl_blue = 5   pwm_podsw = 40     5%
		// temp =  10C   temp = 50   Ubl_blue = 35  pwm_podsw = 30    30%
		// temp = -10C   temp = 30   Ubl_blue = 55  pwm_podsw = 10    70%  100%
		// temp = -40C   temp = 0    Ubl_blue = 85  pwm_podsw = 0    100%
		
		
		
		//if (during_off == 255) { Ubl = 0; Ubl_blue = 0; }
		// komunikacja z atmega8, przeslanie 1+2 zmiennych 8bit
		twistart();
		twiwrite(0xFC); // adres atmega8
		twiwrite(255); // wyslanie liczby okreslajacej poczatek tablicy 
		twiwrite(pwm_podsw); // kolor zielony
		twiwrite(Ubl_blue); // kolor niebieski
		twistop();
		}
	
	if ( (i == 8) && (during_off != 255))
		{
		// odczyt z atmega8
		twistart();
		// procedura transmisji bajtu danych
		twiwrite(0xFC); // adres sprzetowy ds1307
		twiwrite(0x00); // adres pierwszej komorki pamieci ds1307 do odczytu
		twistart();
		twiwrite(0xFD); // adres sprzetowy ds1307 do odczytu danych
		v1 = twiread(1); 
		v2 = twiread(0); 
		// procedura transmisji sygna³u STOP
		twistop();
		}
	
		

	wdt_reset(); // Reset the watchdog timer
	
	if (B_5 > 0) B_5 --;
	if (B_6 > 0) B_6 --;
	if (C_5 > 0) C_5 --;
	
	przerwanie = 0;
	}

	// _-_-_-_-_
	if ( (bit_is_set(PINB, 1)) && (H1 < 250) ) H1 ++; 
	else if ( (bit_is_clear(PINB, 1)) && (L1 < 250) ) L1 ++; 
	if ( (L1 > 25) && (H1 > 20) ) {speed1 += 5; L1 = 0; H1 = 0; dziesiec_metrow += 68;}
	if (dziesiec_metrow > 9999) 
		{
		dziesiec_metrow -= 10000;
		aktualny_dystans ++; // aktualna droga w metrach /10  1 = 10 metrow
		}
	
	if ( (bit_is_set(PINB, 1)) && (H2 < 250) ) H2 ++; 
	else if ( (bit_is_clear(PINB, 1)) && (L2 < 250) ) L2 ++; 
	if ( (L2 > 15) && (H2 > 11) ) {speed2 += 5; L2 = 0; H2 = 0; dziesiec_metrow += 68;} 
	
	// 60 - 
	// 50 - 45-55
	// 40 - zaklocenia pomiedzy 50-60km/h - 9 + 9 ms
	// 30 - zaklocenia pomiedzy 60-70km/h - 7,5 + 7,5ms
	// 25 - zaklocenia pomiedzy 70-80km/h - 6,5 + 6,5ms
	// 10 - zawyzanie wartosci w calym zakresie predkosci
	
	
	
	
	//if (   ((engine_off_sec > 200) && (temp2 != -1) && (temp3 != -1) )  )
	//if ( ((bit_is_set(PINB, 4)) && (engine_off_sec > 30)) || ((engine_off_sec > 200) && (temp2 != -1) && (temp3 != -1)) )
	if ( ( (bit_is_set(PINB, 4)) && (engine_off_sec > 12) ) || ( (engine_off_sec > 50) && (bit_is_clear(PINC, 2)) && (Uacc < 13000) ) )
		{
		PORTB &= ~_BV(2); // ustaw wyjœcie 2 portu B w stan niski - wylaczenie uC - jesli auto jest zamkniete - swieci sie dioda od alarmu
							//lub jesli silnik jest wylaczony dluzej niz 50 sekund i nie ma zasilania po stacyjce
		wdt_reset(); // Reset the watchdog timer
		during_off = 255; // w trakcie wylaczania
		} 
	else
		{
		PORTB |= _BV(2); // ustaw wyjœcie 2 portu B w stan wysoki - wlaczenie przekaznika zasilania
		if (off > 0) during_off = 255; // w trakcie wylaczania
		else during_off = 0;
		}
		
		
	// ********************** wylaczanie uC dwoma przyciskami ********************** //
		if ( (bit_is_clear(PINC, 6)) || (bit_is_clear(PINC, 4)) )
			{
			twistart();
			twiwrite(0xFC); // adres atmega8
			twiwrite(254); // wyslanie liczby okreslajacej poczatek tablicy, informujacej o wylaczeniu zasilania
			twistop();
			TWCR = (0<<TWEN); // wylaczenie I2C
			off = 20; // sekund pauzy aby uP nie wykonywal zapisow do pamieci
			} 
	// **************************************************************************** //




	if (engine_off_sec == 5) lcd1 = 1; // 1
	if (engine_off_sec == 5) lcd2 = 3;	// 3
	
	
	
	if ( (bit_is_clear(PINB, 5)) && (B_5 == 0) )
		{
		// if (engine_off_sec > 10) engine_off_sec = 10; // wydluzenie czasu do wylaczenia uC
		if (obr > 600) lcd2 = 1; else lcd1 ++;
		if (lcd1 == lcd2) lcd1++;
		if (lcd1 > 4) lcd1 = 1;
		B_5 = 11;
		} 
		
	
	if ( (bit_is_clear(PINC, 5)) && (C_5 == 0) )
		{
		// if (engine_off_sec > 10) engine_off_sec = 10; // wydluzenie czasu do wylaczenia uC
		if (obr > 600) lcd2 = 1; else lcd1 --;
		if (lcd1 == lcd2) lcd1--;
		if (lcd1 < 1) lcd1 = 4;
		C_5 = 11;
		} 
	
	if ( (bit_is_clear(PINB, 6)) && (B_6 == 0) ) // przycisk w samochodzie
		{
		if (engine_off_sec > 0) {
			lcd1++;
			if (lcd1 > 4) lcd1 = 1; 
			if (lcd1 == lcd2) lcd1++;
			if (engine_off_sec > 10) engine_off_sec = 10; // wydluzenie czasu do wylaczenia uC
			}
		else {
			lcd2++;
			if (lcd1 == lcd2) lcd2++;
			if (lcd2 > 4) lcd2 = 1;
			}
		B_6 = 11;
		} 
		





	
	// jesli temperatura oleju jest wieksza od 60C i mniejsza od 115C oraz swiatla sa wlaczone to zwieksz cisnienie turbiny do wartosci zapisanej w mapie
	// jesli przyspieszenie jest male, jest hamowanie albo jadê na dó³ to zmniejsz wymagane doladowanie
	//if ((oil_temp > 100) && (oil_temp < 150) && (bit_is_set(PINC, 4))){ doladowanie = 900; } 
	if ((oil_temp > 99) && (oil_temp < 155) && (adc3-zeroGy > 0) ){ doladowanie = mbar[1][(obr/500)];} 
	else 
		{
		doladowanie = mbar[0][(obr/500)]; 
		} 
	
	// maksymalne obroty
	if (obr > max_OBR) max_OBR = obr;
	
	
	
	
 	
	//if(bit_is_set(PINC, 3) && heat_on == 0) heat_on = 300; // jesli jest wlaczone podgrzewanie lusterek
	//if(heat_on > 0) heat_on--;
	
	/*
	if (uC_time < 2)
		{
		// szybkie szukanie wartosci przetwornika adc temperatury oleju po wlaczeniu zasilania
		if((adc0 < temp[oil_temp]) && (oil_temp > 0)) oil_temp --;				
		if((adc0 > temp[oil_temp]) && (oil_temp < 195)) oil_temp ++;
		}
	*/
	
	
	if(i == 9)
		{       //odswiezanie wynikow co 1 sekunde, 9 pomiarow, menu wybierane przyciskami
		if (++odczyt_10sekund > 10) odczyt_10sekund = 0;
		
		//if ( (sec == sek) && (lock == 0) )  // jesli minê³a kolejna minuta od uruchomienia
		//	{
		//	//uC_time++;
		//	lock = 1;
		//	} else lock = 0;
		
		if ( sec == sek) uC_time++;
		
	// szukanie wartosci przetwornika adc temperatury oleju
	if((adc0 < temp[oil_temp]) && (oil_temp > 0)) oil_temp --;				
	if((adc0 > temp[oil_temp]) && (oil_temp < 195)) oil_temp ++;
		
		//oil_temp = sek;
		
		// Uacc mV
		Uacc *= 1.9;
		// 13444 850
		//Uacc *= 30;
		//Uacc /= 16;
		
		if (off > 0) off--;
		
		
		// obliczanie zeroG - srednia ciagnieta kilka minut - dostosowanie zero G do aktualnego montazu sterownika
		zeroGx = 540;
		zeroGy = 390;
		//zeroGy -= (speed1+speed2)/4; // odjecie predkosci
		//zeroGy = adc3;
		
			
			
 	if (obr > 650) { 
			engine_on_sec++;
			engine_off_sec = 0;
			lcd1 = 2;
			avg_SPEED = (aktualny_dystans*36)/(engine_on_sec);
			} 
		else {
			engine_off_sec ++;
			if (bit_is_set(PINC, 2) && (uC_time > 1) ) engine_off_sec = 10; // jesli jest napiecie po stacyjce
			if(engine_off_sec > 252) engine_off_sec = 10;
			}
		
		
		avg_boost /= i;
		if(avg_boost > max_BOOST) max_BOOST = avg_boost;
		
		
		
		// sygnalizacja przekroczonych temperatur
		
		if (info > 0)info --;
		else
			{
			if (oil_temp == 85 || oil_temp > 150) info = 60;
			}
		
		for (rgb=1; rgb<3; rgb++)
			{
			// wybor wyswietlacza, zmiana co sekunde
			if ( rgb == 1 ) {
			// lcd1
			PORTB &= ~_BV(7); // ustaw wyjœcie 7 portu B w stan niski
			menu = lcd1;
			}
			else {
			// lcd2
			PORTB |= _BV(7); // ustaw wyjœcie 7 portu B w stan wysoki
			menu = lcd2;
			}	
	
		
		
		LCD_GoTo(0,0);
		LCD_WriteText("                ");
		
		if(menu == 2)LCD_GoTo(10,1); else LCD_GoTo(0,1);
		LCD_WriteText("                ");
		
		LCD_GoTo(0,0);
		
		
		switch(menu){
		case 1:   //     
			{    
			/*
			// zmiana czasu
			if ( ((nowa_godz != 0) || (nowa_min != 0)) ) {
					// procedura zapisu danych do ds1307	
					twistart();
					// procedura transmisji bajtu danych
					twiwrite(0xD0); // adres sprzetowy ds1307
					twiwrite(0x00); // adres komorki pamieci ds1307
					twiwrite(0x30); // ustawienie sekund
					twiwrite(nowa_min); // ustawienie minut
					twiwrite(nowa_godz); // ustawienie godzin
					twistop();
					nowa_godz = 0;
					nowa_min = 0;
					}
			*/

			utoa(DHT11_data[0], ch, 10);
			LCD_WriteText(ch);
			LCD_WriteText("% ");
			utoa(DHT11_data[1], ch, 10);
			LCD_WriteText(ch);
			if (DHT11_data[1] > -10) LCD_WriteText("C "); else LCD_WriteText(" ");
			
			
			
			LCD_GoTo(11,0);
			// temperatura wewnatrz samochodu z DS18B20
			if(temp1 < 10 && temp1 > -1)LCD_GoTo(12,0);
			if(temp1 < -9)LCD_GoTo(10,0);
			itoa(temp1, ch, 10); 
			LCD_WriteText(ch);
			LCD_WriteText(".");
			if(temp1r < 1000)LCD_WriteText("0");
			itoa(temp1r, ch, 10);
			LCD_WriteText(ch);
			LCD_GoTo(15,0);
			LCD_WriteData(0x07); // stopien
			//LCD_WriteText("C");
			
			
			
			
			
			LCD_GoTo(0,1);
			// napiecie akumulatora
			if (Uacc > 9999) LCD_GoTo(1,1); else LCD_GoTo(2,1);
			utoa(Uacc, ch, 10);  // Konwersja liczby  int do asci
			LCD_WriteText(ch);
			if (Uacc > 9999) LCD_GoTo(0,1); else LCD_GoTo(1,1);
			utoa(Uacc/1000, ch, 10);  // Konwersja liczby  int do asci
			LCD_WriteText(ch);
			LCD_GoTo(2,1);
			LCD_WriteText(".");
			LCD_GoTo(5,1);
			LCD_WriteText("V");
			
			
			
			
			
			// wypelnienie pwm,
			LCD_GoTo(7,1);
			LCD_WriteText("PWM ");	
			itoa(OCR2*11/28, ch, 10); 
			LCD_WriteText(ch);
			LCD_WriteText("%");	
			
			
			
			/*
			//LCD_GoTo(0,1);
			if(nowa_godz < 10)LCD_WriteText("0");
			utoa(nowa_godz, ch, 16); 
			LCD_WriteText(ch);
			LCD_WriteText(":");
			if(nowa_min < 10)LCD_WriteText("0");
			utoa(nowa_min, ch, 16); 
			LCD_WriteText(ch);
			
			if (bit_is_clear(PINC, 6)) nowa_godz ++;
			if (nowa_godz > 0x09) nowa_godz = 0x10;
			if (nowa_godz > 0x19) nowa_godz = 0x20;
			if (nowa_godz > 0x24) nowa_godz = 0;
			
			if (bit_is_clear(PINC, 4)) nowa_min ++;
			if (nowa_min > 0x09) nowa_min = 0x10;
			if (nowa_min > 0x19) nowa_min = 0x20;
			if (nowa_min > 0x29) nowa_min = 0x30;
			if (nowa_min > 0x39) nowa_min = 0x40;
			if (nowa_min > 0x49) nowa_min = 0x50;
			if (nowa_min > 0x59) nowa_min = 0;
			*/

			
			
			


			/*
			LCD_GoTo(11,1);	
			// zasieg w kilometrach
			utoa( ((fuel_old-50)*8)/5, ch, 10); // ilosc paliwa -5l x 16km/litr
			LCD_WriteText(ch);
			LCD_WriteData(0xDC); // strzalka
			*/
			
			
			
			break; // opuœæ instrukcjê switch
			}
		
		case 2:  
			{  
			
			LCD_GoTo(0,1);
			LCD_WriteText("                ");
			LCD_GoTo(0,1);
			utoa(pwm_podsw, ch, 10); 
			LCD_WriteText(ch);
			LCD_WriteText("-");
			utoa(Ubl_blue, ch, 10); 
			LCD_WriteText(ch);
			
			
			
			//LCD_WriteText(" ");
			//utoa(dzien_tygodnia, ch, 10);
			//LCD_WriteText(ch);
			
			
			LCD_GoTo(8,1);
			// przebyte km + zasiêg -> ilosc km na calym baku paliwa
			//utoa( ((fuel_old-50)*8)/5 + km_total/10 + aktualny_dystans/100, ch, 10); 
			
			//LCD_WriteText(ch);
			//LCD_WriteText("km");
			//if  (bit_is_set(PINB, 4)) LCD_WriteText("close"); else LCD_WriteText("open");
			
			
			itoa(adc2, ch, 10); 
			LCD_WriteText(ch);
			LCD_WriteText(" ");
			itoa(adc3, ch, 10); 
			LCD_WriteText(ch);
			
			
			/*
			// spalanie na calym baku
			LCD_GoTo(0,0);
			utoa( (610000 / (fuel_old*17 + km_total/10 +aktualny_dystans/100 )), ch, 10);
			LCD_WriteText(ch);
			LCD_GoTo(1,0);
			LCD_WriteText(ch);
			LCD_WriteText("L/100km");
			LCD_GoTo(1,0);
			LCD_WriteText(".");
			*/
			
			
			
			
			
			
			break; // opuœæ instrukcjê switch
			}
		
		
		
		case 3:   
			{
			
			//LCD_GoTo(0,0);
			//0123456789012345
			//w/z: 6:00 19:00
			LCD_WriteText("w/z: ");
			//dtostrf(obl_double, 5, 3, ch);
			//LCD_WriteText(ch);
			utoa(wschod_godz, ch, 16);
			LCD_WriteText(ch);
			LCD_WriteText(":");
			if(wschod_min < 10) LCD_WriteText("0");
			utoa(wschod_min, ch, 10);
			LCD_WriteText(ch);
			//LCD_GoTo(0,1);
			LCD_WriteText(" ");
			utoa(zachod_godz, ch, 16);
			LCD_WriteText(ch);
			LCD_WriteText(":");
			if(zachod_min < 10) LCD_WriteText("0");
			utoa(zachod_min, ch, 10);
			LCD_WriteText(ch);
			
			
			
			LCD_GoTo(0,1);
			// czas dzialania silnika
			if (engine_on_sec > 19)
				{
				utoa(engine_on_sec/60, ch, 10); 
				LCD_WriteText(ch);
				LCD_WriteText("/");
				} 
				else {
					LCD_WriteData(0x06);
					LCD_WriteText(" ");
					}
				
			// czas dzialania uC
			uC_min = uC_time - ((uC_time/60)*60);
			utoa(uC_time/60, ch, 10); // ilosc godzin
			LCD_WriteText(ch);
			LCD_WriteText(":");
			if (uC_min < 10) LCD_WriteText("0");
			utoa(uC_min, ch, 10);
			LCD_WriteText(ch);
			
			
			if ( (temp2 > -10) && (engine_on_sec < 20)  && (during_off != 255) )  LCD_GoTo(8,1);
			else LCD_GoTo(11,1);
			// godzina z DS1307
			if(godz < 10)LCD_WriteText("0");
			utoa(godz, ch, 16); 
			LCD_WriteText(ch);
			
			LCD_WriteText(":");
			
			if(min < 10)LCD_WriteText("0");
			utoa(min, ch, 16); 
			LCD_WriteText(ch);
			
			LCD_WriteText(":");
			if(sek < 10)LCD_WriteText("0");
			utoa(sek, ch, 16); 
			LCD_WriteText(ch);
			
			
			
			break; // opuœæ instrukcjê switch
			}
		
		case 4:   
			{
			LCD_WriteText("lcd 4");

			
			break; // opuœæ instrukcjê switch
			}
		
		
		case 5:   //  
			{    
			LCD_WriteText("lcd 5");
			
			break; // opuœæ instrukcjê switch
			}
		
		case 6:  
			{    
			LCD_WriteText("lcd 6");
			
			break; // opuœæ instrukcjê switch
			}
			
	
		}
		
		/*
		// informacja na wyswietlaczu o wlaczonym podgrzewaniu lusterek przez 4 sek
		if(heat_on > 295 && rgb == 2)
			{
			LCD_GoTo(0,1);
			LCD_WriteText("    heat on     ");
			}
		*/
		
		}
//		x = 0;
//		y = 0;


		//adc2exh_temp = 0;
		//v_old = speed1;
		speed1 = 0;
		speed2 = 0;
		i = 0;
		avg_boost = 0;
		fuel = 0;	
		Uacc = 0;
		e_temp = 0;
		
		
		
		}
	}
return 0;
}