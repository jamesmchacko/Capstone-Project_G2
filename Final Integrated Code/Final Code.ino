
// TTGO T-Call pin definitions
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22




#include <TinyGPS++.h> //https://github.com/mikalhart/TinyGPSPlus
#include <AceButton.h> // https://github.com/bxparks/AceButton

#define BLYNK_PRINT Serial
#define BLYNK_HEARTBEAT 30
#define TINY_GSM_MODEM_SIM800

#include <TinyGsmClient.h> // https://github.com/vshymanskyy/TinyGSM
#include <BlynkSimpleSIM800.h> //https://github.com/blynkkk/blynk-library

#include <Wire.h>
// #include <TinyGsmClient.h>
#include "utilities.h"


using namespace ace_button;

//Buttons
#define SMS_Button 34
#define Call_Button 35

// Emergency Number and Message
String message = "It's an Emergency. I'm at this location ";
String mobile_number = "+16475400635";

String message_with_data;

// Variables for storing GPS Data
float latitude;
float longitude;
float speed;
float satellites;
String direction;

// Switch
ButtonConfig config1;
AceButton call_button(&config1);
ButtonConfig config2;
AceButton sms_button(&config2);

void handleEvent_call(AceButton*, uint8_t, uint8_t);
void handleEvent_sms(AceButton*, uint8_t, uint8_t);

// Set serial for GPS Module
#define SerialMon Serial1

// Hardware Serial for builtin GSM Module
#define SerialAT Serial2

const char apn[]  = "chatrweb.apn";

const char user[] = "";
const char passs[] = "";

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
//const char auth[] = "84PCXBRNmB_SZpGl9c_MUrrDtFEveGeK";

//static const int RXPin = 4, TXPin = 5;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
WidgetMap myMap(V0);

//SoftwareSerial ss(RXPin, TXPin);
//BlynkTimer timer;

TinyGsm modem(SerialAT);

unsigned int move_index = 1;


///********************************///


#include <OneWire.h>
#include <DallasTemperature.h>


#include <WiFi.h>
#include <WiFiClient.h>
//#include <BlynkSimpleEsp32.h>

#include <Wire.h>
#include "MAX30105.h" //sparkfun MAX3010X library
#include "heartRate.h"
MAX30105 particleSensor;
TaskHandle_t Task1 = NULL;
TaskHandle_t Task2 = NULL;
//#define MAX30105 //if you have Sparkfun's MAX30105 breakout board , try #define MAX30105

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "5vdJ192fLhBpL3TKDg6p3O9bVtO7oG8m"; // Token you received on your email, Replace with stars
BlynkTimer sendTimer;
const int oneWireBus = 4;     
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "2101_rogers"; // Replace stars with Wifi Name/SSID
char pass[] = "QWERTYUIOP"; //Replace stars with Wifi Password
bool notifyBPM = false;
double avered = 0; double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
int i = 0;
int Num = 100;//calculate SpO2 by this sampling interval

double ESpO2 = 95.0;//initial value of estimated SpO2
double FSpO2 = 0.7; //filter factor for estimated SpO2
double frate = 0.95; //low pass filter for IR/red LED value to eliminate AC component
#define TIMETOBOOT 3000 // wait for this time(msec) to output SpO2
#define SCALE 88.0 //adjust to display heart beat and SpO2 in the same scale
#define SAMPLING 5 //if you want to see heart beat more precisely , set SAMPLING to 1
#define FINGER_ON 30000 // if red signal is lower than this , it indicates your finger is not on the sensor
#define MINIMUM_SPO2 80.0

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;


#define USEFIFO
void setup()
{

Serial.begin(115200);
  Serial.println("Initializing...");

 // Blynk.begin(auth, ssid, pass);
   //sendTimer.setInterval(250L, sendToBlynkServer);

  // Initialize sensor
  while (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30102 was not found. Please check wiring/power/solder jumper at MH-ET LIVE MAX30102 board. ");
    //while (1);
  }
  //Setup to sense a nice looking saw tooth on the plotter
  byte ledBrightness = 0x7F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  //Options: 1 = IR only, 2 = Red + IR on MH-ET LIVE MAX30102 board
  int sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 16384; //Options: 2048, 4096, 8192, 16384
  // Set up the wanted parameters
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  particleSensor.enableDIETEMPRDY();
  xTaskCreatePinnedToCore(Task1code,"Task1",70000,NULL,9,&Task1,0);
  delay(1000);
  xTaskCreatePinnedToCore(Task2code,"Task2",70000,NULL,0,&Task2,1);
delay(1000); 
  
  // Set console baud rate
  Serial.begin(9600);
  delay(10);

  // Keep power when running from battery
  Wire.begin(I2C_SDA, I2C_SCL);
  bool   isOk = setPowerBoostKeepOn(1);
  SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));

  // Set-up modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);


  pinMode(SMS_Button, INPUT);
  pinMode(Call_Button, INPUT);


  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem: ");
  SerialMon.println(modemInfo);

  // Unlock your SIM card with a PIN
  //modem.simUnlock("1234");

  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork(240000L)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" OK");

  if (modem.isNetworkConnected()) {
    SerialMon.println("Network connected");
  }

  SerialMon.print(F("Connecting to APN: "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, user, passs)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" OK");
  //  ss.begin(GPSBaud);
  Blynk.begin(auth, modem, apn, ssid, pass);
  sendTimer.setInterval(5000L, sendToBlynkServer);

  config1.setEventHandler(handleEvent_call);
  config2.setEventHandler(handleEvent_sms);

  call_button.init(Call_Button);
  sms_button.init(SMS_Button);


///////*************************////


 
}

/////////////****************************//////////

void Task1code(void*pvParameters){
   Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  for(;;){

  uint32_t ir, red ;
  double fred, fir;
  double SpO2 = 0; //raw SpO2 before low pass filtered
  long irValue = particleSensor.getIR();


#ifdef USEFIFO
  particleSensor.check(); //Check the sensor, read up to 3 samples

  while (particleSensor.available()) {//do we have new data
#ifdef MAX30105
    red = particleSensor.getFIFORed(); //Sparkfun's MAX30105
    ir = particleSensor.getFIFOIR();  //Sparkfun's MAX30105
#else
    red = particleSensor.getFIFOIR(); //why getFOFOIR output Red data by MAX30102 on MH-ET LIVE breakout board
    ir = particleSensor.getFIFORed(); //why getFIFORed output IR data by MAX30102 on MH-ET LIVE breakout board
#endif



    i++;
    fred = (double)red;
    fir = (double)ir;
    avered = avered * frate + (double)red * (1.0 - frate);//average red level by low pass filter
    aveir = aveir * frate + (double)ir * (1.0 - frate); //average IR level by low pass filter
    sumredrms += (fred - avered) * (fred - avered); //square sum of alternate component of red level
    sumirrms += (fir - aveir) * (fir - aveir);//square sum of alternate component of IR level
    if ((i % SAMPLING) == 0) {//slow down graph plotting speed for arduino Serial plotter by thin out
      if ( millis() > TIMETOBOOT) {
        float ir_forGraph = (2.0 * fir - aveir) / aveir * SCALE;
        float red_forGraph = (2.0 * fred - avered) / avered * SCALE;
        //trancation for Serial plotter's autoscaling
        if ( ir_forGraph > 100.0) ir_forGraph = 100.0;
        if ( ir_forGraph < 80.0) ir_forGraph = 80.0;
        if ( red_forGraph > 100.0 ) red_forGraph = 100.0;
        if ( red_forGraph < 80.0 ) red_forGraph = 80.0;
        //        Serial.print(red); Serial.print(","); Serial.print(ir);Serial.print(".");
        if (ir < FINGER_ON) ESpO2 = MINIMUM_SPO2; //indicator for finger detached
        float temperature = particleSensor.readTemperatureF();
        Blynk.run();
        Blynk.virtualWrite(V4, ESpO2 );
        Serial.print(" Oxygen % = ");
        Serial.println(ESpO2);
        Blynk.virtualWrite(V3, beatAvg );
        Serial.print(" EKG % = ");
        Serial.println(beatAvg);

      }
    }
    if ((i % Num) == 0) {
      double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
      //Serial.println(R);
      SpO2 = -23.3 * (R - 0.4) + 100; //http://ww1.microchip.com/downloads/jp/AppNotes/00001525B_JP.pdf
      ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;//low pass filter
      //  Serial.print(SpO2);Serial.print(",");Serial.println(ESpO2);
      sumredrms = 0.0; sumirrms = 0.0; i = 0;
      break;
    }
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
    //Serial.println(SpO2);
  }
  if (checkForBeat(irValue) == true)
  {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }


#endif
}
Task1 = NULL;
vTaskDelete(NULL); 
}
void Task2code(void*pvParameters){
   Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());
  for(;;){
  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);
  Serial.print(temperatureC);
  Serial.println("ºC");
  delay(5000);
  Blynk.virtualWrite(V2, temperatureC);
}
Task2 = NULL;
vTaskDelete(NULL); 
}
void sendToBlynkServer(){
  notifyBPM = notifyBPM | (beatAvg > 50);
  Blynk.virtualWrite(V3, beatAvg);
    Blynk.virtualWrite(V4, ESpO2);
  if(notifyBPM){
    if(beatAvg < 40 || beatAvg > 100)
      Blynk.notify("Your patient's BPM is "+(String)beatAvg+"!");
    if(ESpO2 < 80)
      Blynk.notify("Your patient's Blood Oxygen Level is "+(String)ESpO2+"!");

  }
  if (gps.charsProcessed() < 10)
  {
    //Serial.println(F("No GPS detected: check wiring."));
    Blynk.virtualWrite(V5, "GPS ERROR");
  }
}








/*
void checkGPS()
{
  if (gps.charsProcessed() < 10)
  {
    //Serial.println(F("No GPS detected: check wiring."));
    Blynk.virtualWrite(V4, "GPS ERROR");
  }
}
*/

void loop()
{
  while (Serial.available() > 0)
  {
    if (gps.encode(Serial.read()))
      displayInfo();
  }

  Blynk.run();
  sendTimer.run();
  sms_button.check();
  call_button.check();
}

void displayInfo()
{

  if (gps.location.isValid() )
  {

    latitude = (gps.location.lat());     //Storing the Lat. and Lon.
    longitude = (gps.location.lng());

    //Serial.print("LAT:  ");
    //Serial.println(latitude, 6);  // float to x decimal places
    //Serial.print("LONG: ");
    //Serial.println(longitude, 6);
    Blynk.virtualWrite(V5, String(latitude, 6));
    Blynk.virtualWrite(V6, String(longitude, 6));
    myMap.location(move_index, latitude, longitude, "GPS_Location");
    speed = gps.speed.kmph();               //get speed
    Blynk.virtualWrite(V7, speed);


    direction = TinyGPSPlus::cardinal(gps.course.value()); // get the direction
    Blynk.virtualWrite(V8, direction);
    
    satellites = gps.satellites.value();    //get number of satellites
    Blynk.virtualWrite(V9, satellites);


  }


  //Serial.println();
}

void handleEvent_sms(AceButton* /* button */, uint8_t eventType,
                     uint8_t /* buttonState */) {
  switch (eventType) {
    case AceButton::kEventPressed:
      // Serial.println("kEventPressed");
      message_with_data = message + "Latitude = " + (String)latitude + "Longitude = " + (String)longitude;
      modem.sendSMS(mobile_number, message_with_data);
      message_with_data = "";
      break;
    case AceButton::kEventReleased:
      //Serial.println("kEventReleased");
      break;
  }
}
void handleEvent_call(AceButton* /* button */, uint8_t eventType,
                      uint8_t /* buttonState */) {
  switch (eventType) {
    case AceButton::kEventPressed:
      // Serial.println("kEventPressed");
      modem.callNumber(mobile_number);
      break;
    case AceButton::kEventReleased:
      //Serial.println("kEventReleased");
      break;
  }
}
