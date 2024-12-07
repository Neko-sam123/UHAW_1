#include <Bluepad32.h>
#include <ESP32Servo.h>

int servo = 12;
Servo Steer;

int joystick_left_up_forward    = 32;  //  0
int joystick_left_down_reverse  = 33;  //  1
int joystick_left_right_forward = 25;  //  2
int joystick_left_left_reverse  = 26;  //  3

#define TRIG_PIN 14
#define ECHO_PIN 27 
float distance = 0;

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
    if (ctl->a()) {
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
    }

    if (ctl->b()) {
        static int led = 0;
        led++;
        ctl->setPlayerLEDs(led & 0x0f);
    }
    if(ctl->axisRX()){
      int angle = map(ctl->axisRX(),-510,511,45,135);
      Steer.write(angle); 
    }
    joyStick_left(ctl);
    distance_trigger();
    //dumpGamepad(ctl);
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

    Steer.attach(servo);
    Steer.write(90);

    pinMode(joystick_left_up_forward, OUTPUT);     
    pinMode(joystick_left_down_reverse, OUTPUT);
    pinMode(joystick_left_right_forward, OUTPUT);
    pinMode(joystick_left_left_reverse, OUTPUT);

    pinMode(TRIG_PIN, OUTPUT); // Set TRIG as output
    pinMode(ECHO_PIN, INPUT);  // Set ECHO as input
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

}

void joyStick_left(ControllerPtr ctl) {
    int axisValue = ctl->axisY();
    int speed = 0;

    const int deadZone = 20;
    if(distance < 10){
       if (axisValue <= -deadZone) {
          speed = map(axisValue, -deadZone, -511, 20, 255);
          analogWrite(joystick_left_up_forward, 0); // Forward
          analogWrite(joystick_left_down_reverse, 0);
          analogWrite(joystick_left_right_forward, 0); // Forward
          analogWrite(joystick_left_left_reverse, 0);
      } else if (axisValue >= deadZone) {
          speed = map(axisValue, deadZone, 520, 20, 255);
          analogWrite(joystick_left_up_forward, 0);
          analogWrite(joystick_left_down_reverse, speed);
          analogWrite(joystick_left_right_forward, 0); 
          analogWrite(joystick_left_left_reverse, speed);
      } else {
          // Joystick in dead zone, stop movement
          analogWrite(joystick_left_up_forward, 0); 
          analogWrite(joystick_left_down_reverse, 0);
          analogWrite(joystick_left_right_forward, 0); 
          analogWrite(joystick_left_left_reverse, 0);
      }
    }
    else if(distance < 25){
       if (axisValue <= -deadZone) {
          speed = map(axisValue, -deadZone, -511, 20, 255);
          analogWrite(joystick_left_up_forward, speed/2); // Forward
          analogWrite(joystick_left_down_reverse, 0);
          analogWrite(joystick_left_right_forward, speed/3); // Forward
          analogWrite(joystick_left_left_reverse, 0);
      } else if (axisValue >= deadZone) {
          speed = map(axisValue, deadZone, 520, 20, 255);
          analogWrite(joystick_left_up_forward, 0); 
          analogWrite(joystick_left_down_reverse, speed);
          analogWrite(joystick_left_right_forward, 0); 
          analogWrite(joystick_left_left_reverse, speed/3);
      } else {
          // Joystick in dead zone, stop movement
          analogWrite(joystick_left_up_forward, 0); 
          analogWrite(joystick_left_down_reverse, 0);
          analogWrite(joystick_left_right_forward, 0); 
          analogWrite(joystick_left_left_reverse, 0);
      }
    }
    else{
      if (axisValue <= -deadZone) {
          speed = map(axisValue, -deadZone, -511, 20, 255);
          analogWrite(joystick_left_up_forward, speed); // Forward
          analogWrite(joystick_left_down_reverse, 0);
          analogWrite(joystick_left_right_forward, speed); // Forward
          analogWrite(joystick_left_left_reverse, 0);
      } else if (axisValue >= deadZone) {
          speed = map(axisValue, deadZone, 520, 20, 255);
          analogWrite(joystick_left_up_forward, 0); // Forward
          analogWrite(joystick_left_down_reverse, speed);
          analogWrite(joystick_left_right_forward, 0); // Forward
          analogWrite(joystick_left_left_reverse, speed);
      } else {
          // Joystick in dead zone, stop movement
          analogWrite(joystick_left_up_forward, 0); 
          analogWrite(joystick_left_down_reverse, 0);
          analogWrite(joystick_left_right_forward, 0); 
          analogWrite(joystick_left_left_reverse, 0);
      }
    }

}

void distance_trigger(){
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2); 
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10); 
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);

  distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");


}
