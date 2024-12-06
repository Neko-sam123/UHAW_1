#include <Bluepad32.h>

#include <ESP32Servo.h>

Servo Steer;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

unsigned long currentT;
unsigned long previousT = 0;



#define bluetooth_led 18

#define arrow_up 23   
#define arrow_down 22  
#define arrow_left 21   
#define arrow_right 19

#define cross 5
#define circle 17
#define square  16
#define triangle 4

// Pins for joystick left (previously for arrow)
#define joystick_left_up_forward    32  //  0
#define joystick_left_down_reverse  33  //  1
#define joystick_left_right_forward 25  //  2
#define joystick_left_left_reverse  26  //  3

// Pins for joystick right (previously for cross, circle, square, triangle)
//#define joystick_right_up   27
//#define joystick_right_down  14
//#define joystick_right_right  12
//#define joystick_right_left 13

#define break_led 2
#define speed_up_led 15

const int pwmFreq = 5000;     
const int pwmResolution = 10;
const int bit_resolution = 1024; 


void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;

            digitalWrite(bluetooth_led,HIGH);

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
  
  //  if (ctl->x()) {
  //      ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0x80 /* weakMagnitude */,
  //                          0x40 /* strongMagnitude */);
  //  }
  joyStick_right(ctl);
  joyStick_left(ctl);
  arrow_buttons(ctl);
  shape_buttons(ctl);
  accel_throttle(ctl);
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




void setup() {
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);

    Steer.attach(27);

    ledcSetup(0, pwmFreq, pwmResolution);  
    ledcSetup(1, pwmFreq, pwmResolution); 
    ledcSetup(2, pwmFreq, pwmResolution);   
    ledcSetup(3, pwmFreq, pwmResolution);  
    //ledcSetup(4, pwmFreq, pwmResolution);
    //ledcSetup(5, pwmFreq, pwmResolution);    
    //ledcSetup(6, pwmFreq, pwmResolution);  
    //ledcSetup(7, pwmFreq, pwmResolution);  
    ledcSetup(8, pwmFreq, pwmResolution);  
    ledcSetup(9, pwmFreq, pwmResolution);  

    ledcAttachPin(joystick_left_up_forward, 0);     
    ledcAttachPin(joystick_left_down_reverse, 1);
    ledcAttachPin(joystick_left_right_forward, 2);
    ledcAttachPin(joystick_left_left_reverse, 3);

    //ledcAttachPin(joystick_right_up, 4);
    //ledcAttachPin(joystick_right_down, 5);
    //ledcAttachPin(joystick_right_right, 6);
    //ledcAttachPin(joystick_right_left, 7);

    ledcAttachPin(break_led,8);
    ledcAttachPin(speed_up_led,9);
    
    pinMode(bluetooth_led,OUTPUT);


    pinMode(arrow_up,OUTPUT);
    pinMode(arrow_down,OUTPUT);
    pinMode(arrow_left,OUTPUT);
    pinMode(arrow_right,OUTPUT);
    pinMode(cross,OUTPUT);
    pinMode(circle,OUTPUT);
    pinMode(square,OUTPUT);
    pinMode(triangle,OUTPUT);

  }

void loop() {
  currentT = millis();
  if(currentT - previousT >= 5){
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();
  previousT = currentT;
  }
}
void joyStick_left(ControllerPtr ctl) {
    int axisValue = ctl->axisY();
    int speed = 0;

    // Define dead zone
    const int deadZone = 20;

    // Check if joystick is out of the dead zone
    if (axisValue <= -deadZone) {
        speed = map(axisValue, -deadZone, -511, 70, bit_resolution);
        ledcWrite(0, speed); // Forward
        ledcWrite(1, 0);
        ledcWrite(2, speed); // Forward
        ledcWrite(3, 0);
    } else if (axisValue >= deadZone) {
        speed = map(axisValue, deadZone, 520, 70, bit_resolution);
        ledcWrite(1, speed); // Reverse
        ledcWrite(0, 0);
        ledcWrite(3, speed); // Reverse
        ledcWrite(2, 0);
    } else {
        // Joystick in dead zone, stop movement
        ledcWrite(0, 0);
        ledcWrite(1, 0);
        ledcWrite(2, 0);
        ledcWrite(3, 0);
    }
}


void joyStick_right(ControllerPtr ctl){
  int steerValue = ctl->axisRX();
  int angle = 0;
  const int deadZone = 20;

  // for right-left movement of Right joystick
  if(steerValue >= deadZone){
      angle = map(steerValue, 20, 512, 91, 135);
      Steer.write(angle);
      Serial.println("Right");

  }else if(steerValue <= -deadZone ){
      angle = map(steerValue, -20, -511, 89, 45);
      Steer.write(angle);
      Serial.println("Left");
  }else{
    Steer.write(90);
    Serial.println("No steer");
  }
  Serial.println(angle);
}






void arrow_buttons(ControllerPtr ctl){
  int dpadState = ctl->dpad(); // Store the result of ctl->dpad()

  if (dpadState == 0x01) { // Check if dpadState is non-zero
    digitalWrite(arrow_up, HIGH);
  }else{
    digitalWrite(arrow_up, LOW);
  }
  if(dpadState == 0x02){
    digitalWrite(arrow_down, HIGH);
  }else{
    digitalWrite(arrow_down, LOW);
  }
  if(dpadState == 0x04){
    digitalWrite(arrow_left, HIGH);
  }else{
    digitalWrite(arrow_left, LOW);
  }
  if(dpadState == 0x08){
    digitalWrite(arrow_right, HIGH);
  }else{
    digitalWrite(arrow_right, LOW);
  }
}

void shape_buttons(ControllerPtr ctl){
  int buttonState = ctl->buttons(); // Store the result of ctl->dpad()

  if (buttonState == 0x0001) { 
      digitalWrite(cross, HIGH);
  }else{
   digitalWrite(cross, LOW);
  }
  if (buttonState == 0x0002){
    digitalWrite(circle, HIGH);
  }else{
    digitalWrite(circle, LOW);
  }
  if(buttonState == 0x0004){
    digitalWrite(square, HIGH);
  }else{
    digitalWrite(square, LOW);
  }
  if(buttonState == 0x0008){
    digitalWrite(triangle, HIGH);
  }else{
    digitalWrite(triangle, LOW);
  }
}

void set_led_brightness(int pin, int value) {
    if (value >= 20) {
        int brightness = map(value, 20, 1020, 70, bit_resolution);
        ledcWrite(pin, brightness);
    } else {
        ledcWrite(pin, 0);
    }
}

void accel_throttle(ControllerPtr ctl) {
    set_led_brightness(8, ctl->brake());     
    set_led_brightness(9, ctl->throttle()); 
}

