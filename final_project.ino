// Lucas Black
// Swamp cooler system

// Servo library
#include <Servo.h>

// RTC library
#include "Wire.h"
#include "RTClib.h"

#define RDA 0x80
#define TBE 0x20

// RTC module
RTC_DS1307 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// servo
Servo myservo;
int servo_pin = 5;

// potentiometer pin
int potpin = 0;
unsigned int potval = 0, prev_potval = 0;

// fan motor
int fan_enable = 5;
int fan_input1 = 5;
int fan_input2 = 4;

// Define UART Register Pointers
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

void U0init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

unsigned char U0kbhit()
{
  return *myUCSR0A & RDA;
}

unsigned char U0getchar()
{
  return *myUDR0;
}

void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

void serialPrintInt(unsigned int input, bool newline)
{
  // get order of magnitude
  unsigned int copy = 0, magnitude = 1;
  while (input > 0) {
    copy += (input % 10) * magnitude;
    magnitude *= 10;
    input /= 10;
  }
  // goes too high
  magnitude /= 10;

  while (magnitude > 0) {
    // print highest order digit
    U0putchar(copy / magnitude + '0');
    // grab lower order digits
    copy %= magnitude;
    // decrease magnitude
    magnitude /= 10;
  }

  if (newline) {
    // end with new line
    U0putchar('\n');
  }
}

void serialPrint(const char *str, bool newline)
{
  int i = 0;
  while (str[i] != '\0') {
    // print each character in string
    U0putchar(str[i]);
    i++;
  }
  
  if (newline) {
    // end with new line
    U0putchar('\n');
  }
}

void serialPrintDateTime(RTC_DS1307 &rtc)
{
  DateTime now = rtc.now();
  serialPrintInt(now.month(), 0);
  serialPrint("/", 0);
  serialPrintInt(now.day(), 0);
  serialPrint("/", 0);
  serialPrintInt(now.year(), 0);
  serialPrint(" (", 0);
  serialPrint(daysOfTheWeek[now.dayOfTheWeek()], 0);
  serialPrint(") ", 0);
  serialPrintInt(now.hour(), 0);
  serialPrint(":", 0);
  serialPrintInt(now.minute(), 0);
  serialPrint(":", 0);
  serialPrintInt(now.second(), 1);
}

// Define Timer Register Pointers
volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;

// todo: modify this
void timerDelay(double period)
{
  // double period = 1.0/(double)freq;
  // 50% duty cycle
  double half_period = period/ 2.0f;
  // clock period def
  double clk_period = 0.0000000625;
  // calc ticks
  unsigned int ticks = half_period / clk_period;
  // stop the timer
  *myTCCR1B &= 0xF8;
  // set the counts
  *myTCNT1 = (unsigned int) (65536 - ticks);
  // start the timer
  *myTCCR1B |= 0b00000011;
  // wait for overflow
  while((*myTIFR1 & 0x01)==0); // 0b 0000 0000
  // stop the timer
  *myTCCR1B &= 0xF8;   // 0b 0000 0000
  // reset TOV           
  *myTIFR1 |= 0x01;
}


// Define ADC Register pointers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

// Define Port B Register Pointers
volatile unsigned char* port_b = (unsigned char*) 0x25; 
volatile unsigned char* ddr_b  = (unsigned char*) 0x24;
// Define Port E Register Pointers
volatile unsigned char* port_e = (unsigned char*) 0x2E; 
volatile unsigned char* ddr_e  = (unsigned char*) 0x2D;
// Define Port G Register Pointers
volatile unsigned char* port_g = (unsigned char*) 0x34; 
volatile unsigned char* ddr_g  = (unsigned char*) 0x33;

// allows any port/pin to be set as output
void set_port_as_output(volatile unsigned char* ddr, unsigned char pin_num)
{
  *ddr |= 0x01 << pin_num;
}

// allows any port/pin to be written to
void write_port(volatile unsigned char* port, unsigned char pin_num, unsigned char state)
{
  if(state == 0)
  {
    *port &= ~(0x01 << pin_num);
  }
  else
  {
    *port |= 0x01 << pin_num;
  }
}

void setup() {
  // For RTC...
  Wire.begin();
  // setup the UART
  U0init(9600);

  while (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
  }

  if (!rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
  }

  // set RTC time to current time of compliation
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // set servo output
  myservo.attach(servo_pin);

  // set fan output pins
  set_port_as_output(ddr_g, fan_enable);
  set_port_as_output(ddr_e, fan_input1);
  set_port_as_output(ddr_e, fan_input2);

  // initialize fan inputs
  write_port(port_e, fan_input1, LOW);
  write_port(port_e, fan_input2, LOW);

  // setup the ADC
  adc_init();

  // read potentiometer from A0
  potval = adc_read(0);
  // map value from potentiometer to range of servo values
  potval = map(potval, 0, 1023, 0, 180);
  myservo.write(potval);
  prev_potval = potval;
}

void loop() {
  prev_potval = potval;
  // read potentiometer from A0
  potval = adc_read(0);
  // map value from potentiometer to range of servo values
  potval = map(potval, 0, 1023, 0, 180);
  myservo.write(potval);

  if (prev_potval != potval) {
    // print change to vent position
    serialPrint("Vent position: ", 0);
    serialPrintInt(potval, 1);

    serialPrintDateTime(rtc);
  }

  // set motor speed to max
  write_port(port_g, fan_enable, HIGH);
  // spin motor in one direction
  write_port(port_e, fan_input1, LOW);
  write_port(port_e, fan_input2, HIGH);

  // delay (change this)
  delay(100);
}
