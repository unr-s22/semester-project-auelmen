#include "DHT.h"
#include <Stepper.h>
#include <LiquidCrystal.h>
#define TERMPERATURE_THRESHOLD 80
#define WATER_LEVEL_THRESHOLD 100 
#define DHTPIN 2//Whichever pin is used
#define DHTTYPE DHT11
#define STEPS 32
Stepper stepper (STEPS, 3, 5, 4, 6);
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal lcd(7, 8, 9, 10, 11, 12); //may change pins, we'll see
unsigned char WATER_LEVEL_PORT = 0;
 volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
 volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
 volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
 volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
 volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;
int potVal = 0;
int Pval = 0;


enum state {
   off = 0,
   idle = 1,
   temp = 2,
   water = 3
};

enum state stat = off;

void setup() {
//  adc_init();
  stepper.setSpeed(200);
  U0init(9600);
  lcd.begin(16, 2); //sixteen columns, 2 rows

}

void loop() {
  delay(1000);
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

  potVal = map(analogRead(A1),0,1024,0,500);
  if (potVal>Pval && stat != water)
  {
    stepper.step(5);
  }
  if (potVal<Pval && stat != water)
  {
    stepper.step(-5);
  }

  // Switch uses enumerated stat variable defined above, starting at off
 /* switch(stat) {
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
  } */
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



void U0init(unsigned long U0baud)
{
// Students are responsible for understanding
// this initialization code for the ATmega2560 USART0
// and will be expected to be able to intialize
// the USART in differrent modes.
//
unsigned long FCPU = 16000000;
unsigned int tbaud;
tbaud = (FCPU / 16 / U0baud - 1);
// Same as (FCPU / (16 * U0baud)) - 1;
*myUCSR0A = 0x20;
*myUCSR0B = 0x18;
*myUCSR0C = 0x06;
*myUBRR0 = tbaud;
}

void adc_init(void)
{
 
//16MHz/128 = 125kHz the ADC reference clock
 
ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));
 
ADMUX |= (1<<REFS0);       //Set Voltage reference to Avcc (5v)
 
ADCSRA |= (1<<ADEN);       //Turn on ADC
 
ADCSRA |= (1<<ADSC); 
} 
uint16_t read_adc(uint8_t channel)
{
  ADMUX &= 0xE0; //Clear bits MUX0-4
  ADMUX |= channel&0x07; //Defines the new ADC channel to be read by setting bits MUX0-2
  ADCSRB = channel&(1<<3); //Set MUX5
  ADCSRA |= (1<<ADSC); //Starts a new conversion
  while(ADCSRA & (1<<ADSC)); //Wait until the conversion is done
  return ADCW;
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
}
