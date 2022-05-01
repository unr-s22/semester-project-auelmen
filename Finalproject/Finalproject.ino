#include "DHT.h"
#include <Stepper.h>
#include <LiquidCrystal.h>
#define TEMPERATURE_THRESHOLD 65
#define WATER_LEVEL_THRESHOLD 300 
#define DHTPIN 2 //Whichever pin is used
#define DHTTYPE DHT11
const int stepsPerRevolution = 300;  // change this to fit the number of steps per revolution
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 3, 5, 4, 6);
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal lcd(7, 8, 9, 10, 11, 12); //may change pins, we'll see
unsigned char WATER_LEVEL_PORT = 1;
volatile unsigned char *port_c = (unsigned char *) 0x28;
volatile unsigned char *ddr_c = (unsigned char *) 0x27;
volatile unsigned char *pin_c = (unsigned char *) 0x26;
volatile unsigned char *port_e = (unsigned char *) 0x2E;
volatile unsigned char *ddr_e = (unsigned char *) 0x2D;
volatile unsigned char *pin_e = (unsigned char *) 0x2C;
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
volatile bool loopStep;

void setup() {
//  adc_init();
  myStepper.setSpeed(60);
  dht.begin();
  Serial.begin(9600);
  lcd.begin(16, 2); //sixteen columns, 2 rows
  *ddr_c = 0xFF;
  *ddr_e = 0x00;
  *port_e = 0x00;
  *port_c = 0x00;
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);

  PCICR |= (1 << PCIE2);
  
  PCMSK2 |= (1 << PCINT16);
  analogRead(WATER_LEVEL_PORT);
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
  if(loopStep)
  {
    loopStep = 0; 
    myStepper.step(2*stepsPerRevolution);
  }



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

}

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
  while (w < WATER_LEVEL_THRESHOLD && ISRReset == 0) {
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
  if(ISRReset){
    stat = off;
  }
  
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
    Serial.print("Temp: ");
    Serial.print(f);
    Serial.print('\n');
    lcd_th(f, h);
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
ISR(PCINT0_vect){
  if(PINB & (1 << PB0)) {
    loopStep = 1;
}
}



unsigned int water_level() {
  return read_adc(WATER_LEVEL_PORT);
}
void adc_init(void){
 
//16MHz/128 = 125kHz the ADC reference clock
 
ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));
 
ADMUX |= (1<<REFS0);       //Set Voltage reference to Avcc (5v)
 
ADCSRA |= (1<<ADEN);       //Turn on ADC
 
ADCSRA |= (1<<ADSC);
while(ADCSRA & (1<<ADSC)); 
} 

uint16_t read_adc(uint8_t channel)
{
ADMUX &= 0xE0;           //Clear bits MUX0-4
ADMUX |= channel&0x07;   //Defines the new ADC channel to be read by setting bits MUX0-2
ADCSRB = channel&(1<<3); //Set MUX5
ADCSRA |= (1<<ADSC);      //Starts a new conversion
while(ADCSRA & (1<<ADSC));  //Wait until the conversion is done
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
  lcd.print("%");
}
