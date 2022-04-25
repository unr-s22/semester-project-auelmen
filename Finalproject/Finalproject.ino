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

 volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
 volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
 volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
 volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
 volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;



enum state {
   off = 0,
   idle = 1,
   temp = 2,
   water = 3
};

enum state stat = off;

void setup() {
  adc_init();
  stepper.setSpeed(200);
  myservo.attach(6)
  Serial.begin(9600);
  lcd.begin(16, 2) //sixteen columns, 2 rows

}

void loop() {
  delay(1000);
  unsigned int w = adc_read(WATER_LEVEL_PORT);
  float temperature = tempRead(true);
  float humid = humidRead();// put your main code here, to run repeatedly:

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(f);
  Serial.print(F(" Water: "));
  Serial.print(w);
  Serial.print('\n');

  potVal = map(analogRead(A0),0,1024,0,500);
  if (potVal>Pval && stat != water)
    stepper.step(5);
  if (potVal<Pval %% stat != water)
    stepper.step(-5);

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

}




void adc_init(void){
 
//16MHz/128 = 125kHz the ADC reference clock
 
ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));
 
ADMUX |= (1<<REFS0);       //Set Voltage reference to Avcc (5v)
 
ADCSRA |= (1<<ADEN);       //Turn on ADC
 
ADCSRA |= (1<<ADSC); 
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
  if (isnan(h)) Serial.println(F("Failed to read humidity."));
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
