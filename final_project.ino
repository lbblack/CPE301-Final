// Lucas Black
// Swamp cooler system
#include <Servo.h>

// servo
Servo myservo;
int servo_pin = 5;

// potentiometer pin
int potpin = 0;

// fan motor
int fan_enable = 5;
int fan_input1 = 5;
int fan_input2 = 4;

// Define Timer Register Pointers
volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;

// Define ADC Register pointers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

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

void timerDelay(unsigned int freq)
{
  double period = 1.0/(double)freq;
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

void setup() {
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
}

void loop() {
  // read potentiometer from A0
  unsigned int val = adc_read(0);
  // map value from potentiometer to range of servo values
  val = map(val, 0, 1023, 0, 180);
  myservo.write(val);

  // set motor speed to max
  write_port(port_g, fan_enable, HIGH);
  // spin motor in one direction
  write_port(port_e, fan_input1, LOW);
  write_port(port_e, fan_input2, HIGH);
  // delay
  timerDelay(1);
}
