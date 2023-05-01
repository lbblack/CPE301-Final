// Lucas Black
// Swamp cooler system

// Stepper library
#include <Stepper.h>
const int stepsPerRevolution = 64;
Stepper myStepper(stepsPerRevolution, 34, 38, 35, 39);

// RTC library
#include "Wire.h"
#include "RTClib.h"

// Humidity/temperature library
#include <SimpleDHT.h>

// LCD library
#include <LiquidCrystal.h>

// initialize the LCD (digital pins)
const int rs = 30, en = 31, d4 = 22;
const int d5 = 23, d6 = 26, d7 = 27;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// initialize DHT11
byte sys_temperature = 100;
byte sys_humidity    = 0;
int pinDHT11 = 6; // digital pin 6 for object initialisation
SimpleDHT11 dht11(pinDHT11);

#define RDA 0x80
#define TBE 0x20

// RTC module
RTC_DS1307 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// potentiometer pin
int potpin = 8; // A8
unsigned int potval = 0, prev_potval = 0;

// fan motor
int fan_enable = 5; // PG5
int fan_input1 = 5; // PE5
int fan_input2 = 4; // PE4

// led pins
int yellow_led = 4; // PH4 (DISABLE)
int green_led = 5;  // PH5 (IDLE)
int blue_led = 6;   // PH6 (RUNNING)
int red_led = 4;    // PB4 (ERROR)

// push buttons
int push_starp = 5; // PB5 (On/off button)

// variables for button input
volatile bool SYSTEM_TOGGLE = false; // Start/stop

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

void serialPrintInt(unsigned int input)
{
  if (input == 0) {
    U0putchar('0');
    return;
  }
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
}

void serialPrint(const char *str)
{
  int i = 0;
  while (str[i] != '\0') {
    // print each character in string
    U0putchar(str[i]);
    i++;
  }
}

void serialPrintDateTime(RTC_DS1307 &rtc)
{
  DateTime now = rtc.now();
  serialPrintInt(now.month());
  serialPrint("/");
  serialPrintInt(now.day());
  serialPrint("/");
  serialPrintInt(now.year());
  serialPrint(" (");
  serialPrint(daysOfTheWeek[now.dayOfTheWeek()]);
  serialPrint(") ");
  serialPrintInt(now.hour());
  serialPrint(":");
  serialPrintInt(now.minute());
  serialPrint(":");
  serialPrintInt(now.second());
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
  // double half_period = period/ 2.0f;
  // clock period def
  double clk_period = 0.0000000625;
  // calc ticks
  unsigned int ticks = period / clk_period;
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

unsigned int adc_read(unsigned char adc_channel_num, bool prescale)
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
  *my_ADMUX  |= adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  if (prescale) {
    *my_ADCSRA |= 0x43;
  } else {
    *my_ADCSRA |= 0x40;
  }
  
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

// Define Port B Register Pointers
volatile unsigned char* port_b = (unsigned char*) 0x25;
volatile unsigned char* ddr_b  = (unsigned char*) 0x24;
// Define Port B Register Pointers
volatile unsigned char* port_c = (unsigned char*) 0x28;
volatile unsigned char* ddr_c  = (unsigned char*) 0x27;
// Define Port E Register Pointers
volatile unsigned char* port_e = (unsigned char*) 0x2E;
volatile unsigned char* ddr_e  = (unsigned char*) 0x2D;
// Define Port G Register Pointers
volatile unsigned char* port_g = (unsigned char*) 0x34;
volatile unsigned char* ddr_g  = (unsigned char*) 0x33;
// Define Port H Register Pointers
volatile unsigned char* port_h = (unsigned char*) 0x102;
volatile unsigned char* ddr_h  = (unsigned char*) 0x101;

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

volatile unsigned char read_port(volatile unsigned char* port, unsigned char pin_num)
{
  return *port & (0x01 << pin_num);
}

// setup the RTC module
void initialize_serial_and_rtc()
{
  // Needed for RTC
  Wire.begin();
  // setup the UART
  U0init(9600);

  // initialize RTC
  while (!rtc.begin()) { }

  // set RTC time to current time of compliation
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

// setup the fan
void initialize_fan()
{
  // set fan output pins
  set_port_as_output(ddr_g, fan_enable);
  set_port_as_output(ddr_e, fan_input1);
  set_port_as_output(ddr_e, fan_input2);

  // initialize fan inputs
  write_port(port_e, fan_input1, LOW);
  write_port(port_e, fan_input2, LOW);
}

// setup the servo
void initialize_servo()
{
  // set servo output
  myStepper.setSpeed(200);
}

void initialize_leds()
{
  // set up led pins as outputs
  set_port_as_output(ddr_h, yellow_led);
  set_port_as_output(ddr_h, green_led);
  set_port_as_output(ddr_h, blue_led);
  set_port_as_output(ddr_b, red_led);
}

void initialize_buttons()
{
  // set up push buttons
  *ddr_b &= 0b10011111;
}

unsigned long previous_DHT_timer = 0;
unsigned long previous_water_timer = 0;

void setup() {
  // set columns and rows
  lcd.begin(16, 2);
  
  initialize_serial_and_rtc();
  initialize_fan();
  initialize_servo();
  initialize_leds();
  initialize_buttons();

  // setup the ADC
  adc_init();

  // ensure the fan starts as off
  turn_fan_off();
  read_DHT11(&sys_temperature, &sys_humidity);

  *ddr_b &= 0b10011111; // enable pullup
  *port_b |= 0b01100000;

  PCICR |= B00000001; // Enable interrupts on PB & PC
  PCMSK0 |= B01100000; // Trigger interrupts on pins D11 and D12
  EICRA |= B00000011;
}

// volatile unsigned int presses = 0;
volatile unsigned int current_button_state = 0;

char *previous_state = "disabled";
char *state = "disabled";
unsigned int water_level = 0, water_threshold = 130;
unsigned int temp_threshold = 20;
volatile unsigned long button_timer, previous_button_timer;
unsigned long idle_timer, previous_idle_timer;
volatile bool begin_lag = 0;
void loop() {
  button_timer = millis();

  if (!begin_lag) {
    previous_button_timer = button_timer;
  }

  if (button_timer - previous_button_timer > 50) {
    SYSTEM_TOGGLE = 1;
    begin_lag = 0;
  } else {
    SYSTEM_TOGGLE = 0;
  }

  // report timestamp and motor position
  if (previous_state != state) {
    serialPrintDateTime(rtc);
    serialPrint(" ( ");
    serialPrint(previous_state);
    serialPrint(" -> ");
    serialPrint(state);
    serialPrint(" )");
    U0putchar('\n');

    lcd.clear();
    if (prev_potval != potval) {
      // print change to vent position
      serialPrint("Vent position: ");
      serialPrintInt(potval);
      U0putchar('\n');
    }

    if (state == "disabled" && previous_state == "error") {
      SYSTEM_TOGGLE = 0;      
    } else if (state == "idle" && previous_state == "disabled") {
      water_level = read_water_level();
    }

    previous_state = state;
  }

  // handle state transitions
  if (state == "disabled") {
    write_led(0); // yellow

    // transition on start button
    if (SYSTEM_TOGGLE) {
      state = "idle";
      return;
    }
  } else if (state == "idle") {
    write_led(1); // green

    // go back to disabled on stop
    if (SYSTEM_TOGGLE && (button_timer - previous_idle_timer > 200)) {
      state = "disabled";
      return;
    }

    
    // go to error on low water levels
    if (water_level < water_threshold && (button_timer - previous_idle_timer > 1500)) {
      state = "error";
      return;
    }

    // go to running on hot temperature
    if (sys_temperature > temp_threshold && (button_timer - previous_idle_timer > 2000)) {
      state = "running";
      return;
    }
  } else if (state == "running") {
    write_led(2); // blue
    turn_fan_on();

    // go to disabled on stop
    if (SYSTEM_TOGGLE) {
      state = "disabled";
      turn_fan_off();
    }

    // go to error on low water levels
    if (water_level < water_threshold) {
      state = "error";
      turn_fan_off();
      return;
    }

    // go back to idle on lower temperatures
    if (sys_temperature <= temp_threshold) {
      state = "idle";
      turn_fan_off();

      return;
    }
  } else if (state == "error") {
    write_led(3); // red
    write_error_to_lcd();
    
    if (SYSTEM_TOGGLE && water_level >= water_threshold) {
      SYSTEM_TOGGLE = 0;
      state = "idle";
      return;
    } else if (SYSTEM_TOGGLE) {
      state = "disabled";
      return;
    }
  }

  if (state != "idle") {
    previous_idle_timer = button_timer;
  }
  
  // update water sensor and temp/humidity
  if (state != "disabled") {
    if ((button_timer - previous_DHT_timer) >= 10000) {
      read_DHT11(&sys_temperature, &sys_humidity);
      previous_DHT_timer = button_timer;
    }

    if ((button_timer - previous_water_timer) >= 10000) {
      water_level = read_water_level();
      previous_water_timer = button_timer;
    }
  }

  // vent position should be adjustable in all positions except error and disabled
  // (according to state diagram and state descriptions)
  if (state == "running" || state == "idle") {
    read_pot_write_stepper(myStepper);
    write_data_to_lcd();
  }
}

// read the potentiometer and write the value to the servo
void read_pot_write_stepper(Stepper &myStepper) {
  prev_potval = potval;
  // read potentiometer from A0
  potval = adc_read(potpin, false);
  // map value from potentiometer to range of servo values
  if (potval > prev_potval + 25) {
    myStepper.step(potval - prev_potval);
    prev_potval = potval;
  }
  
  if (potval + 25 < prev_potval) {
    myStepper.step(-(prev_potval - potval));
    prev_potval = potval;
  }
}

unsigned int read_water_level() {
  return adc_read(1, true);
}

void turn_fan_on()
{
  // set motor speed to max
  write_port(port_g, fan_enable, HIGH);
  // spin motor in one direction
  write_port(port_e, fan_input1, LOW);
  write_port(port_e, fan_input2, HIGH);
}

void turn_fan_off()
{
  // set motor speed to max
  write_port(port_g, fan_enable, LOW);
  // spin motor in one direction
  write_port(port_e, fan_input1, LOW);
  write_port(port_e, fan_input2, LOW);
}

// Read the temperature/humidity sensor
void read_DHT11(byte *temperature, byte *humidity)
{
  byte temp = 0, hum = 0;
  int err = SimpleDHTErrSuccess;
  if ((err = dht11.read(&temp, &hum, NULL)) != SimpleDHTErrSuccess) {
    return;
  }
  *temperature = temp;
  *humidity = hum;
}

// Output current air temperature/humidity to LCD display
void write_data_to_lcd()
{
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(sys_temperature);
  lcd.print(" C      ");
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(sys_humidity);
  lcd.print("%        ");
}

// Output error to LCD display
void write_error_to_lcd()
{
  lcd.setCursor(0, 0);
  lcd.print("Water level     ");
  lcd.setCursor(0, 1);
  lcd.print("is too low      ");
}

void write_led(int i)
{
  // reset LEDs
  if (i != 0) {
    write_port(port_h, yellow_led, LOW);
  }
  if (i != 1) {
    write_port(port_h, green_led, LOW);
  }
  if (i != 2) {
    write_port(port_h, blue_led, LOW);
  }
  if (i != 3) {
    write_port(port_b, red_led, LOW);
  }

  // enable selected LED
  switch(i) {
  case 0:
    write_port(port_h, yellow_led, HIGH);
    break;
  case 1:
    write_port(port_h, green_led, HIGH);
    break;
  case 2:
    write_port(port_h, blue_led, HIGH);
    break;
  case 3:
    write_port(port_b, red_led, HIGH);
    break;
  }
}

ISR(PCINT0_vect)
{
  current_button_state = (*(port_b - 2) & 0x20) >> 5;
  if (current_button_state) {
    previous_button_timer = button_timer;
    begin_lag = 1;
  }
}
