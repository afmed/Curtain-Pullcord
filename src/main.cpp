#include <Arduino.h>
#include <AccelStepper.h>
#include <TMCStepper.h>
#include <Preferences.h>
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"


#if __has_include("config.hpp")
    #include "config.hpp"
#endif

#define STEPPER_SERIAL Serial2 // TMC2209 Serial Port 
#define STEP_PIN  13 // Step Pin
#define DIR_PIN  14 // Direction Pin
#define DIAG_PIN 2 // Stallguard
#define EN_PIN 27 // Enable Pin

#define R_SENSE 0.15f // Sense Resistor Value
#define DRIVER_ADDRESS 0b00  // Stepper Driver Address

int RMSCURRENT ; //= 2000; 
int MICROSTEPPING ; //= 16; // Default Microstepping Value
int STEPPER_ACCELERATION ;//= 50000; 
int STEPPER_SPEED ; //= 8000;
int STALLGUARD_THRESHOLD ; //= 100; // [0..255] 
int TCOOL_THRESHOLD ; //= ((3089838.00*pow(STEPPER_SPEED,-1.00161534))*1.5);
int MOVE_SMALL ; //= 1000;
int MOVE_FULL ; //= 100000;

#define BUTTON1 23
#define BUTTON2 34
#define BUTTON3 25
#define BUTTON4 32
#define BUTTON5 33

#define IDLE 1
#define STOP 2
#define MOVE 3
#define INIT 4

int invokeAction = STOP;
int stallCount = 0;

// webserver vars
String header;
unsigned long currentTime = millis();
unsigned long previousTime = 0; 
const long timeoutTime = 2000;



AccelStepper   stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
TMC2209Stepper driver(&STEPPER_SERIAL, R_SENSE, DRIVER_ADDRESS);
Preferences    preferences;
AsyncWebServer server(80);

String processor(const String& var){
  Serial.println(var);
  if(var == "TITLE"){
    return "Lounge Front Right";
  }
  return String();
}

void spiffsSetup() {
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
}

void wirelessSetup() {
  // Connect to Wi-Fi network with SSID and password
  Serial.printf("Connecting to %s", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void webserverSetup() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.htm", String(), false, processor);
  });
  server.begin();
}

void preferencesSetup() {
  preferences.begin("curtainsettings", false);

  TCOOL_THRESHOLD       = preferences.getInt("TCOOL_THRESHOLD", 570);
  STALLGUARD_THRESHOLD  = preferences.getInt("STALLGUARD_THRESHOLD", 100);
  STEPPER_SPEED         = preferences.getInt("STEPPER_SPEED", 8000);
  STEPPER_ACCELERATION  = preferences.getInt("STEPPER_ACCELERATION", 50000);
  RMSCURRENT            = preferences.getInt("RMSCURRENT", 2000);
  MICROSTEPPING         = preferences.getInt("MICROSTEPPING", 16);
  MOVE_SMALL            = preferences.getInt("MOVE_SMALL", 1000);
  MOVE_FULL             = preferences.getInt("MOVE_FULL", 100000);

}

void showDriverSettings() {
  Serial.println("------------------------------");
  Serial.println("      CONFIGURED SETTINGS");
  Serial.println("------------------------------");
  Serial.printf(" TCOOL_THRESHOLD:      %i \n", TCOOL_THRESHOLD);
  Serial.printf(" STALLGUARD_THRESHOLD: %i \n", STALLGUARD_THRESHOLD);
  Serial.printf(" STEPPER_SPEED:        %i \n", STEPPER_SPEED);
  Serial.printf(" STEPPER_ACCELERATION: %i \n", STEPPER_ACCELERATION);
  Serial.printf(" RMSCURRENT:           %i \n", RMSCURRENT);
  Serial.printf(" MICROSTEPPING:        %i \n", MICROSTEPPING);
  Serial.println("------------------------------");
}

void showDriverData() {
  Serial.println("------------------------------");
  Serial.println("       CURRENT SETTINGS");
  Serial.println("------------------------------");
  Serial.printf(" TCOOL_THRESHOLD:      %i \n", driver.TCOOLTHRS());
  Serial.printf(" STALLGUARD_THRESHOLD: %i \n", driver.SGTHRS());
  Serial.printf(" RMSCURRENT:           %i \n", driver.rms_current());
  Serial.printf(" MICROSTEPPING:        %i \n", driver.microsteps());
  Serial.println("------------------------------");
}

void driverSetup() {
  STEPPER_SERIAL.begin(115200); 
  driver.pdn_disable(true);
  driver.begin();
  driver.TPWMTHRS(0);
  driver.semin(0);
  driver.semax(2);
  driver.sedn(0b00);
  driver.toff(4);
  driver.blank_time(24);
  driver.microsteps(MICROSTEPPING);
  driver.en_spreadCycle(false);

  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, true);
  stepper.disableOutputs();
}

void driverInitialise() {
  stepper.enableOutputs();
  
  stepper.setAcceleration(STEPPER_ACCELERATION);
  stepper.setMaxSpeed(STEPPER_SPEED);
  driver.TCOOLTHRS(TCOOL_THRESHOLD);
  driver.SGTHRS(STALLGUARD_THRESHOLD);
  driver.rms_current(RMSCURRENT); 
}

void IRAM_ATTR stallMotor() {
  stallCount++;
  if (stallCount > 2 ){
    stepper.move(0);
    invokeAction = STOP;
    stallCount = 0;
  } 
  Serial.printf("STALL DETECTED - TCOOL: %i - StallCount %i \n", TCOOL_THRESHOLD, stallCount);
}

void setup() {

  Serial.begin(115200);
  while(!Serial);
  Serial.println("Start...");
  
  spiffsSetup();
  preferencesSetup();
  wirelessSetup();
  webserverSetup();
 
  pinMode(BUTTON1,INPUT);
  pinMode(BUTTON2,INPUT);
  pinMode(BUTTON3,INPUT);
  pinMode(BUTTON4,INPUT);
  pinMode(BUTTON5,INPUT);

  pinMode(EN_PIN, OUTPUT);
  pinMode(DIAG_PIN, INPUT); 
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);

  attachInterrupt(DIAG_PIN, stallMotor, RISING);

  driverSetup();
  showDriverSettings();

}

void loop() {
  
  if (digitalRead(BUTTON1)==HIGH) {
    stepper.move(MOVE_FULL);
    if (invokeAction == IDLE) {invokeAction = INIT;}
  }

  if (digitalRead(BUTTON2)==HIGH) {
    stepper.move(MOVE_SMALL);
    if (invokeAction == IDLE) {invokeAction = INIT;}
  }

  if (digitalRead(BUTTON3)==HIGH && invokeAction==MOVE) {
    stepper.move(0);
  }

  if (digitalRead(BUTTON3)==HIGH && invokeAction==IDLE) {
    showDriverData();
  }

  if (digitalRead(BUTTON4)==HIGH) {
    stepper.move(-MOVE_SMALL);
    if (invokeAction == IDLE) {invokeAction = INIT;}
  }

  if (digitalRead(BUTTON5)==HIGH) {
    stepper.move(-MOVE_FULL);
    if (invokeAction == IDLE) {invokeAction = INIT;}
  }

  if (invokeAction==STOP) {
    stepper.disableOutputs();
    invokeAction = IDLE;
    stallCount = 0;
  } 

  if (invokeAction==INIT) {
    driverInitialise();
    invokeAction = MOVE;
  } 

  if (invokeAction==MOVE) {
    stepper.run();

    if (stepper.distanceToGo() == 0) {
      invokeAction = STOP;
    }
  }
}
