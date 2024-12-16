#include <Bluepad32.h>
#include <ESP32Servo.h>


int servo_1 = 2;
int servo_2 = 15;
Servo Base_fire_extinguisher;
Servo Gas;

// first H-bridge Driver
int joystick_left_up_forward    = 32;  //  0
int joystick_left_down_reverse  = 33;  //  1
int joystick_right_up_forward = 26;  //  2
int joystick_right_down_reverse  = 25;  //  3

// second H-bridge Driver
int joystick_left_up_forward1 =  27; // 0
int joystick_left_down_reverse1 = 14;  // 1
int joystick_right_up_forward1 = 12; // 2
int joystick_right_down_reverse1 = 13; // 3

int max_speed = 0;
int speed = 0;
int speed_num = 1;

unsigned long currentT;
unsigned long lastUpdateT = 0;


ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }
    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}


void processGamepad(ControllerPtr ctl) {
  if (ctl->b()) {
      static int led = 0;
      led++;
      ctl->setPlayerLEDs(led & 0x0f);
  }
  buttons_keys(ctl);
  joyStick_left_speed(ctl);
  joyStick_right_speed(ctl);
  base_position(ctl);
  Gas.write(90);

  static unsigned long printT = 0;
  if(currentT - printT >= 300){
      //Serial.println(speed);
    //dumpGamepad(ctl);
  printT = currentT;
  }

}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
        
            } else {
                Serial.println("Unsupported controller");
            }
        }
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);

    Base_fire_extinguisher.attach(servo_1);
    Gas.attach(servo_2);
    
    Base_fire_extinguisher.write(180);
    
    // First Driver
    pinMode(joystick_left_up_forward, OUTPUT);     
    pinMode(joystick_left_down_reverse, OUTPUT);
    pinMode(joystick_right_up_forward, OUTPUT);
    pinMode(joystick_right_down_reverse, OUTPUT);

    // Second Driver
    pinMode(joystick_left_up_forward1, OUTPUT);     
    pinMode(joystick_left_down_reverse1, OUTPUT);
    pinMode(joystick_right_up_forward1, OUTPUT);
    pinMode(joystick_right_down_reverse1, OUTPUT);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
   currentT = millis();
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

}

void joyStick_left_speed(ControllerPtr ctl) {
  int axisValue = ctl->axisY();
  const int deadZone = 30;
  if (axisValue <= -deadZone) {
    speed = map(axisValue, -deadZone, -511, 0, max_speed);
    analogWrite(joystick_left_up_forward, speed); // Forward
    analogWrite(joystick_left_down_reverse, 0);
    analogWrite(joystick_left_up_forward1, speed); // Forward
    analogWrite(joystick_left_down_reverse1, 0);
  } else if (axisValue >= deadZone) {
    speed = map(axisValue, deadZone, 520, 0, max_speed);
    analogWrite(joystick_left_up_forward, 0);
    analogWrite(joystick_left_down_reverse, speed);
    analogWrite(joystick_left_up_forward1, 0);
    analogWrite(joystick_left_down_reverse1, speed);
  } else {
    speed = 0;
    // Joystick in dead zone, stop movement
    analogWrite(joystick_left_up_forward, 0); 
    analogWrite(joystick_left_down_reverse, 0);
    analogWrite(joystick_left_up_forward1, 0);
    analogWrite(joystick_left_down_reverse1, 0);
  }

}
void joyStick_right_speed(ControllerPtr ctl){
  int axisValue = ctl->axisRY();
  const int deadZone = 30;
  if (axisValue <= -deadZone) {
    speed = map(axisValue, -deadZone, -511, 0, max_speed);
    analogWrite(joystick_right_up_forward, speed); 
    analogWrite(joystick_right_down_reverse, 0);
    analogWrite(joystick_right_up_forward1, speed); 
    analogWrite(joystick_right_down_reverse1, 0);
  } else if (axisValue >= deadZone) {
    speed = map(axisValue, deadZone, 520, 0, max_speed);
    analogWrite(joystick_right_up_forward, 0);
    analogWrite(joystick_right_down_reverse, speed);  
    analogWrite(joystick_right_up_forward1, 0);
    analogWrite(joystick_right_down_reverse1, speed);                                 
  } else {
    speed = 0;
  // Joystick in dead zone, stop movement
    analogWrite(joystick_right_up_forward, 0); 
    analogWrite(joystick_right_down_reverse, 0);
    analogWrite(joystick_right_up_forward1, 0);
    analogWrite(joystick_right_down_reverse1, 0);
  }
}

void buttons_keys(ControllerPtr ctl){
    if(ctl->buttons() ){
      static unsigned long buttonsT = 0;
      if(ctl->y() && currentT - buttonsT >=  500  ){
        static int colorIdx = 0;
        switch (colorIdx % 3) {
            case 0:
                // Red
                ctl->setColorLED(255, 0, 0);
                break;
            case 1:
                // Green
                ctl->setColorLED(0, 255, 0);
                break;
            case 2:
                // Blue
                ctl->setColorLED(0, 0, 255);
                break;
        }
        colorIdx++;
        speed_num++;
        if(speed_num >= 4){
        speed_num = 1;
        }
      buttonsT = currentT;
      }

      if (ctl->a() && currentT - buttonsT >= 500) {
        static bool pos = true;
        pos = !pos;  
        if  (pos == true){
          Gas.write(0);
          delay(700);
        }else{
          Gas.write(180);
          delay(700);
        }
              
        buttonsT = currentT; 
      }
        
        
      switch(speed_num){
        case 1:
          max_speed = 255/3;
        break;
        case 2:
          max_speed = 255/2;
        break;
        case 3:
          max_speed = 255;
        break;
        default:
        break;
    }  
  }
}

void base_position(ControllerPtr ctl){
  static int position_Base_fire_extinguisher = 180;
  if(ctl->brake() > 30){
    position_Base_fire_extinguisher += 2;
  }
  if(ctl->throttle() > 30){
    position_Base_fire_extinguisher -= 2;
  }
  Base_fire_extinguisher.write(position_Base_fire_extinguisher);

  if(position_Base_fire_extinguisher <= 50){
  position_Base_fire_extinguisher = 50;
  } 
  if(position_Base_fire_extinguisher >= 180){
  position_Base_fire_extinguisher = 180;
  }

  if(ctl->dpad() == 0x01){
    for( ; position_Base_fire_extinguisher <= 180; position_Base_fire_extinguisher += 5 ){
      Base_fire_extinguisher.write(position_Base_fire_extinguisher);
      delay(15);
    }
  }
}
