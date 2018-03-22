
// An implementation that rely on Curie lib (not portable)
#include "CurieTimerOne.h"
#include <CurieBLE.h>

// BLE BT2PWM Services & Characteristic
BLEPeripheral blePeripheral; // create peripheral instance

// BLE Device Name
#define BLENAME "BT2PWM"

//Get PWM Service
#define GETPWMSERVICEUUID "f5964316-dc14-43c0-8c62-a5ed67fb5fd3"
#define GETTHROTTLECHARUUID "f5964317-dc14-43c0-8c62-a5ed67fb5fd3"
#define GETSTEERINGCHARUUID "f5964318-dc14-43c0-8c62-a5ed67fb5fd3"

// Set PWM Service
#define SETPWMSERVICEUUID "df6987fd-6d1d-4136-8f89-b87cf48d70a6"
#define SETTHROTTLECHARUUID "df6987fe-6d1d-4136-8f89-b87cf48d70a6"
#define SETSTEERINGCHARUUID "df6987ff-6d1d-4136-8f89-b87cf48d70a6"

// Set Mode Service
#define SETMODESERVICEUUID "14373251-3da2-474b-ab31-248cba4c2503"
#define SETMODECHARUUID "14373252-3da2-474b-ab31-248cba4c2503"

// create services
BLEService getPwmService(GETPWMSERVICEUUID); 
BLEService setPwmService(SETPWMSERVICEUUID); 
BLEService setModeService(SETMODESERVICEUUID); 

// Create Characteristics
BLEIntCharacteristic getThrottleCharacteristic(GETTHROTTLECHARUUID, BLERead | BLENotify);
BLEIntCharacteristic getSteeringCharacteristic(GETSTEERINGCHARUUID, BLERead | BLENotify);
BLEIntCharacteristic setThrottleCharacteristic(SETTHROTTLECHARUUID, BLEWrite);
BLEIntCharacteristic setSteeringCharacteristic(SETSTEERINGCHARUUID, BLEWrite);
BLEIntCharacteristic setModeCharacteristic(SETMODECHARUUID, BLEWrite);

// PWM Arduino Well Know freq
#define PWM_OUTPUT_FREQ 490

// Input Pin assignment (from RC)
#define  PWM_RC_STEERING_INPUT_PIN 11
#define  PWM_RC_THROTTLE_INPUT_PIN 5

// Output Pin assignment (to RC)
#define  PWM_CMD_STEERING_OUTPUT_PIN 9
#define  PWM_CMD_THROTTLE_OUTPUT_PIN 3

//Working mode and default value
/* Mode 0 is loopback mode, all command received from inputs are used to drive outputs) */
#define MODE_LOOPBACK 0
/* Mode 1 is open circuit mode, outputs are driven by bluetooth SETPWMSERVICEUUID service) */
#define MODE_OPENCIRCUIT 1

int mode = MODE_LOOPBACK; 

//Each 50ms, check and update value to ble part
#define OUTPUTLOOP 50000


// DEBUG

#define DEBUG_ON_SERIAL
#ifdef  DEBUG_ON_SERIAL
#define LOG(x) {Serial.print(x);}
#define LOGln(x) {Serial.println(x);}
#else
#define LOG(x) {}
#define LOGln(x) {}
#endif


// Some global value
unsigned int pwm_throttle_value = 0;
unsigned int pwm_steering_value = 0;
unsigned int freq_value = 0;
unsigned int prev_throttle_time = 0;
unsigned int prev_steering_time = 0;
unsigned int prev_freq_time = 0;

// Use to detect if Rx signal is good
int throttle_toggle = 0;
int steering_toggle = 0;

void timedCheckOutput()
{
  boolean validData = false;
  
  if ((throttle_toggle == 0) || (steering_toggle == 0)) {
    // No activity detected on RC, output special value -1 to ble.
    getThrottleCharacteristic.setValue(-1);
    getSteeringCharacteristic.setValue(-1);
  } else {
    getThrottleCharacteristic.setValue(pwm_throttle_value);
    getSteeringCharacteristic.setValue (pwm_steering_value);
    if (mode == MODE_LOOPBACK) {
      setPwmOutput(PWM_CMD_THROTTLE_OUTPUT_PIN, pwm_throttle_value);
      setPwmOutput(PWM_CMD_STEERING_OUTPUT_PIN, pwm_steering_value);
    }
  /*   
    LOG (pwm_throttle_value);
    LOG ("-");
    LOGln (pwm_steering_value);
    *    
  */
  }

  throttle_toggle = 0;
  steering_toggle = 0;  
}

void throttle_rising() {
  attachInterrupt(PWM_RC_THROTTLE_INPUT_PIN, throttle_falling, FALLING);
  prev_throttle_time = micros();
  throttle_toggle ++;
}

void steering_rising() {
  attachInterrupt(PWM_RC_STEERING_INPUT_PIN, steering_falling, FALLING);
  prev_steering_time = micros();
  steering_toggle ++;
}

void throttle_falling() {
  attachInterrupt(PWM_RC_THROTTLE_INPUT_PIN, throttle_rising, RISING);
  pwm_throttle_value = micros()-prev_throttle_time;
  freq_value = 1000000 / (micros()-prev_freq_time); 
  prev_freq_time = micros();
  throttle_toggle ++;
}

void steering_falling() {
  attachInterrupt(PWM_RC_STEERING_INPUT_PIN, steering_rising, RISING);
  pwm_steering_value = micros()-prev_steering_time;
  steering_toggle ++;
}


// Set Throttle PWM out
void setPwmOutput(int pin, int duration_us)
{
  //remap duration to std duration accordingly to arduino pwm freq 
  int std_duration_us = map (duration_us, 0, 2000, 0, (1000000/PWM_OUTPUT_FREQ));
  int ratio = map (std_duration_us, 0, (1000000/PWM_OUTPUT_FREQ), 0, 255);
  ratio = constrain(ratio, 1, 254);
  analogWrite(pin, ratio);
}

void setup() {

  Serial.begin(115200);

  pinMode(PWM_CMD_THROTTLE_OUTPUT_PIN, OUTPUT);   // sets the pin as output
  pinMode(PWM_CMD_STEERING_OUTPUT_PIN, OUTPUT);   // sets the pin as output

  attachInterrupt(PWM_RC_THROTTLE_INPUT_PIN, throttle_rising, RISING);
  attachInterrupt(PWM_RC_STEERING_INPUT_PIN, steering_rising, RISING);
  CurieTimerOne.start(OUTPUTLOOP, &timedCheckOutput);  // set timer and callback

  blePeripheral.setLocalName(BLENAME);
  blePeripheral.setAdvertisedServiceUuid(getPwmService.uuid());
  blePeripheral.setAdvertisedServiceUuid(setPwmService.uuid());

  // add service and characteristic:
  blePeripheral.addAttribute(getPwmService);
  blePeripheral.addAttribute(getThrottleCharacteristic);
  blePeripheral.addAttribute(getSteeringCharacteristic);
  blePeripheral.addAttribute(setPwmService);
  blePeripheral.addAttribute(setThrottleCharacteristic);
  blePeripheral.addAttribute(setSteeringCharacteristic);
  blePeripheral.addAttribute(setModeService);
  blePeripheral.addAttribute(setModeCharacteristic);
  
  // set the initial value for the characeristic:
  getThrottleCharacteristic.setValue(-1);
  getSteeringCharacteristic.setValue(-1);
  
  // begin advertising BLE service:
  blePeripheral.begin();

  LOGln("BLE PWM Peripheral started");

}

void loop() {
    // listen for BLE peripherals to connect:
  BLECentral central = blePeripheral.central();

  // if a central is connected to peripheral:
  if (central) {
    LOG("Connected to central: ");
    // print the central's MAC address:
    LOGln(central.address());

    // while the central is still connected to peripheral:
    while (central.connected()) {
      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      if (setThrottleCharacteristic.written()) {
        setPwmOutput(PWM_CMD_THROTTLE_OUTPUT_PIN, setThrottleCharacteristic.value());
      }
      if (setSteeringCharacteristic.written()) {
        setPwmOutput(PWM_RC_STEERING_INPUT_PIN, setSteeringCharacteristic.value());
      }
      if (setModeCharacteristic.written()) {
        switch(setModeCharacteristic.value()) {
          case MODE_LOOPBACK:
            mode = MODE_LOOPBACK;
            LOGln("Mode : Switch to loopback mode ");
          break;
          
          default:
            mode = MODE_OPENCIRCUIT;
            LOGln("Mode : Switch to open circuit mode ");
          break;
        }
      }
    }
    // when the central disconnects, print it out:
    LOG("Disconnected from central: ");
    LOGln(central.address());
  }
}



