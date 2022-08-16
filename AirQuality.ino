//----------SD------------//
#include <SPI.h>
#include <SD.h>
int pinCS = 10;
//----------------DHT11--------------------------------//
#include <dht.h>
dht DHT;
#define DHT11_PIN 6
File myFile;
//----------------MQ135--------------------------------//
#include<MQ135.h>
int sensorPinMq135 = A1; 
//----------------Timer-----------------------------//
#include <Wire.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
//----------------Display on LCD-----------------------//
#include<LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,20,4);
//----------------MQ2--------------------------------//
#include<MQ2.h>
int Analog_Input = A0;
float lpg, co,smoke;
MQ2 mq2(Analog_Input);
//----------LED------------//
#define redLED 5
#define greenLED 2
#define whiteLED 3

//--------------------PM 2.5---------------------------//
int measurePin = A2; 
int samplingTime = 280; 
int deltaTime = 40;  
int sleepTime = 9680;
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  pinMode(pinCS, OUTPUT);
  digitalWrite(10,HIGH);
 while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.print("Initializing SD card...");

  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
myFile = SD.open("test.txt", FILE_WRITE);

 lcd.init();
  lcd.backlight();
  lcd.setBacklight(HIGH);
  lcd.setCursor(0,0);
  lcd.print("Sensors warming up!");
  lcd.setCursor(0,1);
  lcd.print("MQ2");
  lcd.setCursor(0,2);
  lcd.print("MQ135");
  lcd.setCursor(0,3);
  lcd.print("GP2Y1014");
  delay(5000);
  lcd.clear();
  
  //----Setting up LEDs----//
  pinMode(greenLED,OUTPUT);
  pinMode(whiteLED,OUTPUT);
  pinMode(redLED,OUTPUT);
}

void loop() {
  // nothing happens after setup
   //--------------DHT11 readings-------------------------//

    tmElements_t tm;
      //---------------print on lcd------------------------//
      //--------------Time and date-------------------------//
 int chk = DHT.read11(DHT11_PIN);
  if (RTC.read(tm)) {
    lcd.setCursor(0,0);
    lcd.print("Date:");
    lcd.print(tm.Day);
    lcd.print('/');
    lcd.print(tm.Month);
    lcd.write('/');
    lcd.print(tmYearToCalendar(tm.Year));
    lcd.setCursor(0,1);
    lcd.print("Time - ");
    lcd.print(tm.Hour);
    lcd.print(':');
    lcd.print(tm.Minute);
    lcd.print(':');
    lcd.print(tm.Second);
    lcd.setCursor(0,2);
    delay(2000);
    lcd.clear();
  
  }
 //--------------DHT11 readings-------------------------//
 
 lcd.setCursor(0,0);
 lcd.print("DHT11 readings");
  lcd.setCursor(0,1);
  lcd.print("Temperature:");
  lcd.print(DHT.temperature);
  lcd.print(" C");
  lcd.setCursor(0,2);
  lcd.print("Humidity:");
  lcd.print(DHT.humidity);
  lcd.print("%");
  delay(2000);
  lcd.clear();

  //--------------MQ2 readings-------------------------//
    co = (mq2.readCO());

  lpg =( mq2.readLPG());
  smoke = mq2.readSmoke();
  lcd.setCursor(0,0);
  lcd.print("MQ2 readings ");
  lcd.setCursor(0,1);
  lcd.print("CO:");
  lcd.print(co);
  lcd.setCursor(0,1);
  lcd.print("CO:");
  lcd.print(co);
  lcd.print(" ppm");
  lcd.setCursor(0,2);
  lcd.print("LPG:");
  lcd.print(lpg);
  lcd.print(" ppm");
  lcd.setCursor(0,3);
  lcd.print("Smoke:");
  lcd.print(smoke);
  lcd.print(" ppm");
  delay(2000);
  lcd.clear();
    //-----------------MQ135 readings--------------------//
  int aqi = analogRead(sensorPinMq135);

  int co2 = aqi*1.75;
  lcd.setCursor(0,0);
  lcd.print("MQ135 values");
  lcd.setCursor(0,1);
  lcd.print("AQI:");
  lcd.print(aqi);
  lcd.setCursor(0,2);
  lcd.print("CO2:");
  lcd.print(co2);
  lcd.print(" ppm");
  
  //----------------PM 2.5------------------------//
  delayMicroseconds(samplingTime);
  voMeasured = analogRead(measurePin);
  delayMicroseconds(deltaTime);
  delayMicroseconds(sleepTime);
  calcVoltage = voMeasured * (5.0 / 1024.0);
  dustDensity = (170 * calcVoltage )/40;
  Serial.println(dustDensity); // unit: ug/m3
  lcd.setCursor(0,3); 
  lcd.print("PM 2.5:");
  lcd.print(dustDensity); 
  delay(2000);
  lcd.clear();
//--------------------LED -----------------------------//
  if(aqi <= 300){
    digitalWrite(greenLED,HIGH);
    digitalWrite(whiteLED, LOW);
    digitalWrite(redLED,LOW);
  }
  else if (aqi > 300 && aqi <=500){
    digitalWrite(greenLED,LOW);
    digitalWrite(whiteLED, HIGH);
    digitalWrite(redLED,LOW);
  }
  else{
    digitalWrite(greenLED,LOW);
    digitalWrite(whiteLED, LOW);
    digitalWrite(redLED,HIGH);
  }
if (myFile) {
        
    //-----------Print on file---------------//
   
   //------------time and date-----//
 myFile.print(tm.Day);
 myFile.print("-");
    myFile.print(tm.Month);
    myFile.print("-");
    myFile.print(tmYearToCalendar(tm.Year));
    myFile.print("  ");
    myFile.print(tm.Hour);
    myFile.print(":");
    myFile.print(tm.Minute);
    myFile.print(":");
    myFile.print(tm.Second);
    myFile.println(" ");
    //---------DHT11 readings---------//
   myFile.print(" DHT readings :- ");
    myFile.print("  Temperature = ");
    myFile.print(DHT.temperature);
    myFile.print("°C    Humidity = ");
    myFile.print(DHT.humidity);
    myFile.println("%");
     //---------MQ 135 readings---------//
    myFile.print("MQ135 readings :- ");
    myFile.print("AQI: ");
    myFile.print(aqi);
    myFile.print(",    CO2: ");
    myFile.print(co2);
    myFile.println(" ppm");
    
     //----------MQ2 readings---------//
    myFile.print("MQ2 readings :- ");
    myFile.print("CO:");
    myFile.print(co);
    myFile.print(" ppm   ,");
    myFile.print("LPG:");
    myFile.print(lpg);
    myFile.print(" ppm   ,");
    myFile.print("Smoke");
    myFile.print(smoke);
    myFile.println(" ppm");
       //----------PM 2.5 values------------//
    myFile.print("Particulate Matter PM(2.5) :");
    myFile.print(dustDensity);
    myFile.println(" ug/cubic m");
    myFile.println("");
   // myFile.close();
}
}
