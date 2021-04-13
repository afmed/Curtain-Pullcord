#include <Arduino.h>
#include <AccelStepper.h>
#include <TMCStepper.h>

/*

#define RA_STEPPER_TYPE STEPPER_TYPE_NEMA17
#define RA_DRIVER_TYPE DRIVER_TYPE_TMC2209_UART

#define RA_PULLEY_TEETH 16

#define USE_AUTOHOME 1        // Autohome with TMC2209 stall detection:  ON = 1  |  OFF = 0   
#define RA_AUDIO_FEEDBACK  0 // If one of these are set to 1, the respective driver will shut off the stealthchop mode, resulting in a audible whine
#define USE_VREF 0      //By default Vref is ignored when using UART to specify rms current. Only enable if you know what you are doing.

*/
#define STEPPER_SERIAL Serial2
#define SW_RX  16
#define SW_TX  17

#define STEP_PIN  13
#define DIR_PIN  14
#define DIAG_PIN 2 // Stallguard
#define EN_PIN 27
#define DRIVER_ADDRESS 0b00
#define R_SENSE 0.11f 
#define MOTOR_CURRENT_RATING      2000 // mA
#define OPERATING_CURRENT_SETTING 80 // %
#define RMSCURRENT 2000 // MOTOR_CURRENT_RATING * (OPERATING_CURRENT_SETTING / 100.0f) / 1.414f
#define UART_STEALTH_MODE   0  // set to 1 to make some noise
#define MICROSTEPPING 16
#define STEPPER_ACCELERATION 5000
#define STEPPER_SPEED 5000

#define STALL_VALUE 10       // 0..255 adjust this value if the RA autohoming sequence often false triggers, or triggers too late
#define STALL_TCOOLTHRS (3089838.00*pow(float(STEPPER_SPEED),-1.00161534))*1.5 
// 1048575 // 0xFFFFF

#define BUTTON1 23
#define BUTTON2 34
#define BUTTON3 25
#define BUTTON4 32
#define BUTTON5 33

#define IDLE 1
#define MOVE 2
#define STOP 3

int invokeAction = 1;

using namespace TMC2209_n;

TMC2209Stepper driver(&STEPPER_SERIAL, R_SENSE, DRIVER_ADDRESS);
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);

// STALLGUARD PIN STOP MOTOR
void IRAM_ATTR stalled_position()
{ 
  Serial.print("STALL DETECTED ");
  Serial.println(digitalRead(DIAG_PIN));
  stepper.move(0);
  invokeAction = STOP;
}


void setup() {

  SPI.begin();

  Serial.begin(115200);
  while(!Serial);
  Serial.println("Start...");
  
  pinMode(BUTTON1,INPUT);
  pinMode(BUTTON2,INPUT);
  pinMode(BUTTON3,INPUT);
  pinMode(BUTTON4,INPUT);
  pinMode(BUTTON5,INPUT);

  pinMode(EN_PIN, OUTPUT);
  pinMode(DIAG_PIN, INPUT); 

  attachInterrupt(DIAG_PIN, stalled_position, FALLING);

  //digitalWrite(EN_PIN, LOW);

  STEPPER_SERIAL.begin(115200);  // Start HardwareSerial comms with driver
  driver.begin();
  driver.mstep_reg_select(true);
  driver.pdn_disable(true);

  driver.toff(0);     
  //driver.I_scale_analog(0);
  driver.rms_current(RMSCURRENT, 1.0f); //holdMultiplier = 1 to set ihold = irun
  driver.toff(1);
  driver.en_spreadCycle(UART_STEALTH_MODE); 
  driver.blank_time(24);
  driver.semin(0); //disable CoolStep so that current is consistent
  driver.microsteps(MICROSTEPPING);
  driver.fclktrim(4);

  driver.TCOOLTHRS(STALL_TCOOLTHRS);  //xFFFFF);
  driver.SGTHRS(STALL_VALUE);

/*
  Serial.print("Mount: Actual RA motor rms_current: %d mA ");
  Serial.println(driver.rms_current() );
  Serial.print("Mount: Actual RA CS value: %d ");
  Serial.println(driver.cs_actual());
  Serial.print("Mount: Actual RA vsense: %d ");
  Serial.println(driver.vsense());
*/

  stepper.setEnablePin(EN_PIN);
  stepper.disableOutputs();

  stepper.setMaxSpeed(STEPPER_SPEED);
  stepper.setAcceleration(STEPPER_ACCELERATION);
  stepper.setPinsInverted(false, false, true);


}

void loop() {
    
    if(digitalRead(BUTTON1)==HIGH){
      stepper.move(10000);
      stepper.enableOutputs();
      invokeAction = MOVE; 
    }
    
    if(digitalRead(BUTTON2)==HIGH){
      stepper.move(2000);
      stepper.enableOutputs();
      invokeAction = MOVE; 
    }

    if(digitalRead(BUTTON3)==HIGH){
      if (stepper.distanceToGo() > 0){
        stepper.move(100);
        stepper.enableOutputs();
        invokeAction = MOVE; 
      } 
      else if (stepper.distanceToGo() < 0) {
        stepper.move(-100);
        stepper.enableOutputs();
        invokeAction = MOVE; 
      }
    }

    if(digitalRead(BUTTON4)==HIGH){
      stepper.move(-2000);
      stepper.enableOutputs();
      invokeAction = MOVE; 
    }

    if(digitalRead(BUTTON5)==HIGH){
      stepper.move(-10000);
      stepper.enableOutputs();
      invokeAction = MOVE; 
    }
       
    if (invokeAction==STOP){
      stepper.disableOutputs();
      invokeAction = IDLE;
    } 
    
    if (invokeAction==MOVE){
      // action any steps waiting to be moved
      stepper.run();

      // when no steps remain, invokeAction STOP
      if (stepper.distanceToGo() == 0) {
        invokeAction = STOP;
      }
      //Serial.print("Remaining Steps: ");
      //Serial.print(stepper.distanceToGo());
      //Serial.print(" Diag Value: ");
      //Serial.println(analogRead(DIAG_PIN));


    }

}