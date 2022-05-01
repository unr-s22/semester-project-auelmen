#include "DHT.h"
#include <Stepper.h>
#include <LiquidCrystal.h>
#define TEMPERATURE_THRESHOLD 65
#define WATER_LEVEL_THRESHOLD 100 
#define DHTPIN 2 //Whichever pin is used
#define DHTTYPE DHT11
#define STEPS 30

Stepper stepper (STEPS, 3, 5, 4, 6);
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal lcd(7, 8, 9, 10, 11, 12); //may change pins, we'll see
unsigned char WATER_LEVEL_PORT = 0;
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;
volatile unsigned char *port_c = (unsigned char *) 0x28;
volatile unsigned char *ddr_c = (unsigned char *) 0x27;
volatile unsigned char *pin_c = (unsigned char *) 0x26;
volatile unsigned char* port_b = (unsigned char*) 0x25; 
volatile unsigned char* ddr_b  = (unsigned char*) 0x24; 
volatile unsigned char* pin_b  = (unsigned char*) 0x23; 
int potVal = 0;
int previous =0;


enum state {
   off = 0,
   idle = 1,
   temp = 2,
   water = 3
};

enum state stat = off;
volatile bool ISRReset = 0;

void setup() {
//  adc_init();
  stepper.setSpeed(200);
  dht.begin();
  Serial.begin(9600);
  lcd.begin(16, 2); //sixteen columns, 2 rows
  *ddr_c = 0xFF;
  *port_c = 0x00;
  //EICRA |= (1 << ISC10);
  //EIMSK |= (1 << INT0);
  *ddr_b = 0x01;
  PCICR |= (1 << PCIE2);
  
  PCMSK2 |= (1 << PCINT16);
}

void loop() {
  delay(100);
  if (ISRReset == 1) {
        Serial.print("Interrupt ");
        Serial.println(ISRReset);
        ISRReset=0;
  }
  unsigned int w = read_adc(WATER_LEVEL_PORT);
  float temperature = tempRead();
  float humid = humidRead();// put your main code here, to run repeatedly:
  Serial.print(F("Humidity: "));
  Serial.print(humid);
  Serial.print(F("%  Temperature: "));
  Serial.print(temperature);
  Serial.print(F(" Water: "));
  Serial.print(w);
  Serial.print('\n');
  lcd_th(temperature, humid);
//  int val = read_adc(1);

  // move a number of steps equal to the change in the
  // sensor reading
  //stepper.step(val - previous);

  // remember the previous value of the sensor
 // previous = val;
  //Serial.print(val); 

  // Switch uses enumerated stat variable defined above, starting at off
  switch(stat) {
    case off:
      Serial.println("Disabled State");
      disabled_state();
      break;
    case idle:
      Serial.println("Idle State");
      idle_state();
      break;
    case water:
      Serial.println("Error State");
      error_state();
      break;
    case temp:
      Serial.println("Running State");
      running_state();
      break;
    default:
      break;
  }

  /* 57 PC4 ( A12 ) Digital pin 33
58  PC5 ( A13 ) Digital pin 32
59  PC6 ( A14 ) Digital pin 31
60  PC7 ( A15 ) Digital pin 30
*port_c = 0b00010000
*port_c = 0b00100000
*port_c = 0b01000000
*port_c = 0b10000000

*/
}
//Template for idle, error, and running states (to be tweaked)
//idle_state()
    //while(temperature > threshold && water level > too low) {
      // GREEN LED is ON 
      // check temperature 
      // check water level
      // print transition times }
    //if (temperature < threshold && water level > too low) {
      // switch to running (stat = temp)
      //} else {
      // switch to error (stat = water)}

//error_state()
    // while(reset = not pressed && water < too low){
      // motor is OFF
      // RED LED is ON (other LEDS are OFF)
      // print on LCD ("Error")}
    //switch to idle (stat = idle)
  
//running_state()
    //while(temperature < threshold && water > too low) {
      // motor is ON
      // BLUE LED is ON (other LEDS are OFF)}
    //if (temperature > threshold && water level > too low) {
      // switch to idle (stat = idle)
    //} else {
      // switch to error (stat = water)}

void disabled_state() { // Or off state
  lcd.clear();
  lcd.noDisplay();

  *port_c &= 0b00000000; // Turn off all LEDs
  *port_c |= 0x01 << 4;// Turn on Yellow LED
  Serial.print(stat);
  while (ISRReset != 1) { }

  // Start button pressed and initialize idle state.
  stat = idle;
  lcd.display();
}

void idle_state() {
  *port_c |= 0b01000000; // Turn on green LED
  *port_c &= 0b01000000; // Turn off other LEDs & fan
  
  // Get water level, temperature, and humidity
  unsigned int w = water_level();
  float t = tempRead();
  float h = humidRead();

  // Display temperature and humidity to screen
  lcd_th(t, h);

  // Check water level.
  if (w < WATER_LEVEL_THRESHOLD) stat = water;
  
  // Check temperature.
  else if (t > TEMPERATURE_THRESHOLD) stat = temp;
}

void error_state() {
  *port_c |= 0b000100000; // Turn on red LED
  *port_c &= 0b000100000; // Turn off other LEDs
  
  lcd.clear();
  lcd.print("Low Water");

  unsigned int w = water_level();

  // Wait for water level to increase
  while (w < WATER_LEVEL_THRESHOLD) {
    delay(1000);
    w = water_level();
    lcd.setCursor(0, 1);
    lcd.print("Level:");
    lcd.setCursor(7, 1);
    lcd.print(w);
}
  
  // Water level is now okay
  stat = idle;
  lcd.clear();
}

void running_state()
{
  *port_c |= 0b10001000; // Enable fan and running LED
  *port_c &= 0b10001000; // Disable other LEDs
  float f = tempRead();
  float h = humidRead();

  // Check water level and temperature
  if (water_level() < WATER_LEVEL_THRESHOLD) stat = water;
  else if ( f > TEMPERATURE_THRESHOLD && ISRReset == 0) {
    delay(1000);
    lcd.clear();
    Serial.print("Temp: ");
    Serial.print(f);
    Serial.print('\n');
    //lcd_th(f, h);
    return running_state();
  }
  else if (ISRReset == 0){
    lcd.clear();
    stat = idle;
  }
  else
  {
    lcd.clear();
    stat = off;
  }
}

ISR(PCINT2_vect) {
  if (PINK & (1<<PK0)){
  ISRReset = 1;
  if (stat != off) { // We are in a non-off state.
    Serial.println("Turning Off");
    stat = off;
  }
  else { // We are off so turn on.
    Serial.println("Turning On");
    stat = idle;
  }
  }
}



unsigned int water_level() {
  return read_adc(WATER_LEVEL_PORT);
}
void adc_init() {
  *my_ADCSRA |= 0b10000000; // sets bit 7 which is ADEN
  // clear bit 5 and bit 3 and bits 2:0 to 0 to disable the ADC trigger mode, the ADC interrupt, and to set prescaler mode to slow reading
  *my_ADCSRA &= 0b11010000;
  *my_ADCSRB &= 0b11110000;
  *my_ADMUX &= 0b01111111;
  // set bit   6 to 1 for AVCC analog reference'
  *my_ADMUX |= 0b01000000;
  // clear bit 5 to 0 for right adjust result
  *my_ADMUX &= 0b11011111;
  *my_ADMUX &= 0b11011111;
  // clear bit 5 to 0 for right adjust result

  // clear bit 4-0 to 0 to reset the channel and gain bits
  *my_ADMUX &= 0b11100000;
}

unsigned int read_adc(unsigned char adc_channel_num) {
  *my_ADMUX &= 0b11100000;
  // clear the channel selection bits (MUX 4:0)

  // clear the channel selection bits (MUX 5) bit 3 in ADCSRB
  *my_ADCSRB &= 0b11110111; 
  
  // set the channel number
  if (adc_channel_num > 7) {
    *my_ADCSRB |= 0b00001000;
    adc_channel_num -= 8;
  }
  
  // set the channel selection bits, but remove the most significant bit (bit 3)
  //my_ADMUX &= 0xF7;


  // set the channel selection bits
  *my_ADMUX += adc_channel_num;
  
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0b01000000;

  // wait for the conversion to complete
  while ((*my_ADCSRA & 0x40) != 0);
  
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

// This function reads temperature from DHT Sensor
float tempRead() 
{
  float t;
  t = dht.readTemperature(true);   //Fahrenheit
  if (isnan(t)) Serial.println(F("Failed to read temperature."));
  return t;
}

// This function reads humidity from DHT sensor
float humidRead()
{
  float h = dht.readHumidity();
  if (isnan(h)){
    lcd.setCursor(0, 0);
    lcd.println(F("Failed to read "));
    lcd.setCursor(0, 1);
    lcd.print("humidity.");
  }
  return h;
}

//This function displays the info to LCD
void lcd_th(float t, float h) {
  lcd.setCursor(0, 0);
  lcd.print("Temp:  Humidity:");
  lcd.setCursor(0, 1);
  lcd.print(t);
  lcd.setCursor(7, 1);
  lcd.print(h);
  lcd.print("%");
}
