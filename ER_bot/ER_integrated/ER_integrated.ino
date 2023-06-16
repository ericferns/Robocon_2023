//ER Integrated digital pins communication

#include <PS4BT.h>   //PS4 Bluetooth
#include <usbhub.h>  //Comment this for PS4 USB
//#include <PS4USB.h> //PS4 Usb

#include <Servo.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

//cytron - shooter
const int m1_dir = 24;  //right motor with behind the shooter - pov
const int m1_pwm = 8;
const int m2_dir = 22;  //left motor with behind the shooter - pov
const int m2_pwm = 9;

long long currT = 0;

//shooter - motor's pwm
int pwm1 = 0;
int pwm2 = 0;
int pwm_shooter = 85;

//piston - shooter
const int relay_shooter = 32;

// servos for the claw
Servo servo_left;
Servo servo_right;
const int s1 = 12;  //left servo
const int s2 = 13;  // right servo

float servo_right_var = 67.20;
float servo_left_var = 173.90;

float hold_servo_right = 67.20;
float hold_servo_left = 173.90;

float final_right_servo = 146.10;
float final_left_servo = 70;

float release_servo_right = 146.10;
float release_servo_left = 105.90;

//Y-axis of shooter
int Y_enA = 2;
int Y_in1 = 34;
int Y_in2 = 28;

// rotating 180 motor using l298n
const int rotate_in3 = 30;
const int rotate_in4 = 36;
const int rotate_enB = 3;

//x-axis of shooter
const int X_in3 = 36;
const int X_in4 = 38;
const int X_enB = 25;

// cascading lift motor
const int cas_dir1 = 40;
// int cas_in4 = 33;
const int cas_pwm1 = 5;




// limit switch for flipping
// const int limit_switch1 = 18;
// const int limit_switch2 = 19;
// const int limit_switch3 = 27;
// const int limit_switch4 = 29;

//limit switch for cascading lift
// const int limit_switch3 = ;

// relay claw
const int relay_claw = 47;

//communication pins
//ER
const int com_pin4 = 6;
const int com_pin3 = 43;
const int com_pin2 = 41;
const int com_pin1 = 39;
const int com_pin0 = 37;

/**** PS4 USB ****/
//Use this for PS4 USB and comment PS4 BT
//USB Usb;
//PS4USB PS4(&Usb);
/**** PS4 USB ****/

/**** PS4 BT ****/
// Use this for PS4 BT and comment PS4 USB
USB Usb;
BTD Btd(&Usb);  // You have to create the Bluetooth Dongle instance like so
PS4BT PS4(&Btd, PAIR);
/**** PS4 BT ****/

bool printAngle, printTouch;
uint8_t oldL2Value, oldR2Value;

/**** claw *****/
void rotate_in() {
  // analogWrite(enbf, 255);
  // digitalWrite(rotate_in3, HIGH);
  // digitalWrite(rotate_in4, LOW);
}
void rotate_out() {
  // analogWrite(enbf, 255);
  // digitalWrite(rotate_in3, LOW);
  // digitalWrite(rotate_in4, HIGH);
}
/**** claw *****/

void setup() {

  //Servo
  servo_left.attach(s1);
  servo_right.attach(s2);
  // servo_left.write(servo_left_var);
  // servo_right.write(servo_right_var);
  servo_left.write(hold_servo_left);
  servo_right.write(hold_servo_right);

  //cytron-shooter
  pinMode(m1_dir, OUTPUT);
  pinMode(m1_pwm, OUTPUT);
  pinMode(m2_dir, OUTPUT);
  pinMode(m2_pwm, OUTPUT);

  digitalWrite(m1_dir, HIGH);  // right
  digitalWrite(m2_dir, HIGH);  // left

  analogWrite(m1_pwm, 0);
  analogWrite(m2_pwm, 0);

  //piston shooter
  pinMode(relay_shooter, OUTPUT);

  //L298N - yaxis
  // Set all the motor control pins to outputs
  pinMode(Y_enA, OUTPUT);
  pinMode(Y_in1, OUTPUT);
  pinMode(Y_in2, OUTPUT);

  // Turn off motors - Initial state
  // digitalWrite(Y_enA, HIGH);
  digitalWrite(Y_in1, LOW);
  digitalWrite(Y_in2, LOW);
  analogWrite(Y_enA, 150);


  // rotating 180 motor using l298n
  pinMode(rotate_enB, OUTPUT);
  pinMode(rotate_in3, OUTPUT);
  pinMode(rotate_in4, OUTPUT);
  digitalWrite(rotate_in3, LOW);
  digitalWrite(rotate_in4, LOW);
  analogWrite(rotate_enB, 50);

  //x-axis of shooter
  // pinMode(X_enB, OUTPUT);
  // pinMode(X_in3, OUTPUT);
  // pinMode(X_in4, OUTPUT);
  // digitalWrite(X_in3, LOW);
  // digitalWrite(X_in4, LOW);
  // analogWrite(X_enB, 150);

  // cascading lift motor
  // pinMode(cas_enB, OUTPUT);
  // pinMode(cas_in3, OUTPUT);
  // pinMode(cas_in4, OUTPUT);
  // digitalWrite(cas_in3, LOW);
  // digitalWrite(cas_in4, LOW);
  // analogWrite(cas_enB, 255);

  pinMode(cas_pwm1, OUTPUT);
  pinMode(cas_dir1, OUTPUT);
  digitalWrite(cas_dir1, LOW);
  analogWrite(cas_pwm1, 0);

  // limit switches for flipping
  // pinMode(limit_switch1, INPUT);
  // pinMode(limit_switch2, INPUT);
  // limit switch for cascading lift
  // pinMode(limit_switch3, INPUT);

  //relay for claw
  pinMode(relay_claw, OUTPUT);

  //Communication pins
  pinMode(com_pin4, OUTPUT);
  pinMode(com_pin3, OUTPUT);
  pinMode(com_pin2, OUTPUT);
  pinMode(com_pin1, OUTPUT);
  pinMode(com_pin0, OUTPUT);

  Serial.begin(115200);

#if !defined(MIPSEL)
  while (!Serial)
    ;  // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1)
      ;  // Halt
  }
  Serial.println(F("\r\nPS4 Bluetooth Library Started"));
}

void loop() {
  Usb.Task();

  if (PS4.connected()) {
    Serial.println(pwm_shooter);
    int right_x = map(PS4.getAnalogHat(RightHatX), 0, 255, -255, 255);
    int right_y = map(PS4.getAnalogHat(RightHatY), 0, 255, 255, -255);

    /**** flywheel *****/
    if (PS4.getButtonPress(SHARE) && PS4.getButtonPress(L3) && !(servo_left_var > 179) && !(servo_left_var < 0)) {
      servo_left_var += 0.1;
      if (servo_left_var >= 180) {
        servo_left_var = 180;
      }
      Serial.print("Servo left Var: ");
      Serial.println(servo_left_var);
      servo_left.write(servo_left_var);
    }

    else if (PS4.getButtonPress(OPTIONS) && PS4.getButtonPress(L3) && !(servo_left_var > 179) && !(servo_left_var < 0)) {
      servo_left_var -= 0.1;
      if (servo_left_var <= 0) {
        servo_left_var = 0;
      }
      Serial.print("Servo left Var: ");
      Serial.println(servo_left_var);
      servo_left.write(servo_left_var);
    }

    if (PS4.getButtonPress(SHARE) && PS4.getButtonPress(R3) && !(servo_right_var > 179) && !(servo_right_var < 0)) {
      servo_right_var += 0.1;
      if (servo_right_var >= 180) {
        servo_right_var = 0;
      }
      Serial.print("Servo right Var: ");
      Serial.println(servo_right_var);
      servo_right.write(servo_right_var);

    }

    else if (PS4.getButtonPress(OPTIONS) && PS4.getButtonPress(R3) && !(servo_right_var > 179) && !(servo_right_var < 0)) {
      servo_right_var -= 0.1;
      if (servo_right_var <= 0) {
        servo_right_var = 0;
      }
      Serial.print("Servo right Var: ");
      Serial.println(servo_right_var);
      servo_right.write(servo_right_var);
    }

    if (PS4.getButtonPress(L3) && PS4.getButtonClick(R2)) {
      if (pwm_shooter >= 255) {
        pwm_shooter = 255;
      } else {
        pwm_shooter += 5;
      }
      // Serial.print("Shooter PWM");
      // Serial.println(pwm_shooter);
      analogWrite(m1_pwm, pwm_shooter);
      analogWrite(m2_pwm, pwm_shooter);
    }

    else if (PS4.getButtonPress(L3) && PS4.getButtonClick(L2)) {
      if (pwm_shooter <= 0) {
        pwm_shooter = 0;
      } else {
        pwm_shooter -= 5;
      }
      // Serial.print("Shooter PWM");
      // Serial.println(pwm_shooter);
      analogWrite(m1_pwm, pwm_shooter);
      analogWrite(m2_pwm, pwm_shooter);
    }

    else if (PS4.getButtonPress(L3) && PS4.getButtonClick(SQUARE)) {
      // pwm_shooter = 85;  // Near type 2 pole
      pwm_shooter = 255;  // Near type 2 pole
      Serial.print("Shooter PWM");
      Serial.println(pwm_shooter);
      analogWrite(m1_pwm, pwm_shooter);
      analogWrite(m2_pwm, pwm_shooter);
    }

    else if (PS4.getButtonPress(L3) && PS4.getButtonClick(CROSS))  //stop the flywheel
    {
      // Serial.println("Shooter stop");
      analogWrite(m1_pwm, 0);
      analogWrite(m2_pwm, 0);
    }

    if (PS4.getButtonClick(R3))  //shoot the ring using piston
    {
      // Serial.print("Shoot Ring");
      digitalWrite(relay_shooter, HIGH);
      delay(500);
      // Serial.println("SHOT!!");
      digitalWrite(relay_shooter, LOW);

      digitalWrite(relay_claw, HIGH);
      // digitalWrite(relay_claw, LOW);

      delay(200);
      // final_right_servo = servo_right_var - 90;
      // final_left_servo = servo_left_var + 90;
      // servo_right.write(final_right_servo);
      // servo_left.write(final_left_servo);
      servo_right.write(release_servo_right);
      servo_left.write(release_servo_left);
      delay(1000);

      // final_right_servo = servo_right_var;
      // final_left_servo = servo_left_var;
      // servo_right.write(final_right_servo);
      // servo_left.write(final_left_servo);
      servo_right.write(hold_servo_right);
      servo_left.write(hold_servo_left);
      delay(500);

      // digitalWrite(relay_claw, HIGH);
      digitalWrite(relay_claw, LOW);
    } else  //retract piston
    {
      digitalWrite(relay_shooter, LOW);
      // Serial.println("Retract Shooter Piston");
    }

    // if (PS4.getButtonPress(R2))  //servo
    // {
    //   servo1.attach(s1);
    //   servo2.attach(s2);
    //   for (int i = 0; i <= 65; i++) {
    //     Serial.println(i);
    //     servo1.writeMicroseconds(2000);
    //     Serial.println(i);
    //     servo2.writeMicroseconds(1000);
    //   }
    //   servo1.detach();
    //   servo2.detach();
    // }

    /**** flywheel *****/

    /**** claw *****/
    if (PS4.getButtonPress(CIRCLE) && !(PS4.getButtonPress(L2)) && !(PS4.getButtonPress(R2))) {
      //open claw
      Serial.print("Open Claw");
      digitalWrite(relay_claw, LOW);
    } else if (PS4.getButtonPress(SQUARE)) {
      //close claw
      Serial.println("Close Claw");
      digitalWrite(relay_claw, HIGH);
    }


    if (PS4.getButtonPress(L2) && PS4.getButtonPress(TRIANGLE))  //lift down
    {
      Serial.println("lift down");
      // digitalWrite(cas_in3, HIGH);
      // digitalWrite(cas_in4, LOW);
      digitalWrite(cas_dir1, HIGH);
      analogWrite(cas_pwm1, 255);


    } else if (PS4.getButtonPress(R2) && PS4.getButtonPress(TRIANGLE))  //lift up
    {
      Serial.print("Lift up");
      // digitalWrite(cas_in3, LOW);
      // digitalWrite(cas_in4, HIGH);
      digitalWrite(cas_dir1, HIGH);
      analogWrite(cas_pwm1, 255);
      //      if (digitalRead(limit_switch3) == HIGH)  // shut the motor when cascading lift reaches  max hieght
      //      {
      //        digitalWrite(inc1, LOW);
      //        digitalWrite(inc2, LOW);
      //      }
    } else {
      // Serial.println("Stop Lift");
      // digitalWrite(cas_in3, LOW);
      // digitalWrite(cas_in4, LOW);
      analogWrite(cas_pwm1, 0);
    }


    //flip the claw
    if (PS4.getButtonPress(L2) && PS4.getButtonPress(SQUARE)) {
      Serial.println("Rotate in");
      rotate_in();
    }

    else if (PS4.getButtonPress(R2) && PS4.getButtonPress(SQUARE))  //anticlock
    {
      Serial.println("Rotate out");
      rotate_out();
      // if (digitalRead(limit_switch2) == HIGH and digitalRead(limit_switch1 == HIGH))  // run the following when claw completes 180 degrees
      // {
      //   digitalWrite(inf1, LOW);  // shut the flipping motor
      //   digitalWrite(inf2, LOW);
      // }
    } else {
      // Serial.println("Stop Rotate");
      digitalWrite(rotate_in3, LOW);
      digitalWrite(rotate_in4, LOW);
    }
    /**** claw *****/

    /**** Y-axis *****/
    if (PS4.getButtonPress(TRIANGLE) && !(PS4.getButtonPress(L2)) && !(PS4.getButtonPress(R2)))  //shooter up
    {
      Serial.println("Shooter up");
      digitalWrite(Y_in1, HIGH);
      digitalWrite(Y_in2, LOW);
    } else if (PS4.getButtonPress(CROSS))  //shooter down
    {
      Serial.println("Shooter Down");
      digitalWrite(Y_in1, LOW);
      digitalWrite(Y_in2, HIGH);
    } else {
      Serial.println("Stop Y-axis");
      digitalWrite(Y_in1, LOW);
      digitalWrite(Y_in2, LOW);
    }
    /**** Y-axis *****/

    /**** X-axis *****/
    // if (PS4.getButtonPress(R2) && PS4.getButtonPress(CIRCLE))  //right
    // {
    //   Serial.println("Shooter Right");
    //   digitalWrite(X_in3, HIGH);
    //   digitalWrite(X_in4, LOW);
    // } else if (PS4.getButtonPress(L2) && PS4.getButtonPress(CIRCLE))  //left
    // {
    //   Serial.println("Shooter Left");
    //   digitalWrite(X_in3, LOW);
    //   digitalWrite(X_in4, HIGH);
    // } else {
    //   Serial.println("Stop X-axis");
    //   digitalWrite(X_in3, LOW);
    //   digitalWrite(X_in4, LOW);
    // }
    /**** X-axis *****/

    /**** Drive *****/

    if (PS4.getButtonPress(R2) && PS4.getButtonPress(UP)) {
      Serial.println("Fast forward (9): PWM 130");
      digitalWrite(com_pin4, LOW);
      digitalWrite(com_pin3, HIGH);
      digitalWrite(com_pin2, LOW);
      digitalWrite(com_pin1, LOW);
      digitalWrite(com_pin0, HIGH);
    }

    else if (PS4.getButtonPress(R2) && PS4.getButtonPress(DOWN)) {
      Serial.println("Fast backward (10): PWM 130");
      digitalWrite(com_pin4, LOW);
      digitalWrite(com_pin3, HIGH);
      digitalWrite(com_pin2, LOW);
      digitalWrite(com_pin1, HIGH);
      digitalWrite(com_pin0, LOW);
    }

    else if (PS4.getButtonPress(R2) && PS4.getButtonPress(LEFT)) {
      Serial.println("Fast Left (11): PWM 130");
      digitalWrite(com_pin4, LOW);
      digitalWrite(com_pin3, HIGH);
      digitalWrite(com_pin2, LOW);
      digitalWrite(com_pin1, HIGH);
      digitalWrite(com_pin0, HIGH);
    }

    else if (PS4.getButtonPress(R2) && PS4.getButtonPress(RIGHT)) {
      Serial.println("Fast right (12): PWM 130");
      digitalWrite(com_pin0, LOW);
      digitalWrite(com_pin1, LOW);
      digitalWrite(com_pin2, HIGH);
      digitalWrite(com_pin3, HIGH);
      digitalWrite(com_pin4, LOW);
    }

    else if (PS4.getButtonPress(L1) && PS4.getButtonPress(TRIANGLE)) {
      Serial.println("Forward wheels (17): PWM 130");
      digitalWrite(com_pin4, HIGH);
      digitalWrite(com_pin3, LOW);
      digitalWrite(com_pin2, LOW);
      digitalWrite(com_pin1, LOW);
      digitalWrite(com_pin0, HIGH);
    }

    else if (PS4.getButtonPress(L1) && PS4.getButtonPress(CROSS)) {
      Serial.println("Backward wheels (18): PWM 130");
      digitalWrite(com_pin4, HIGH);
      digitalWrite(com_pin3, LOW);
      digitalWrite(com_pin2, LOW);
      digitalWrite(com_pin1, HIGH);
      digitalWrite(com_pin0, LOW);
    }

    else if (PS4.getButtonPress(L2) && PS4.getButtonPress(UP)) {
      Serial.println("Slow forward (1): PWM 45");
      digitalWrite(com_pin4, LOW);
      digitalWrite(com_pin3, LOW);
      digitalWrite(com_pin2, LOW);
      digitalWrite(com_pin1, LOW);
      digitalWrite(com_pin0, HIGH);
    }

    else if (PS4.getButtonPress(L2) && PS4.getButtonPress(DOWN)) {
      Serial.println("Slow backward (2): PWM 45");
      digitalWrite(com_pin4, LOW);
      digitalWrite(com_pin3, LOW);
      digitalWrite(com_pin2, LOW);
      digitalWrite(com_pin1, HIGH);
      digitalWrite(com_pin0, LOW);
    }

    else if (PS4.getButtonPress(L2) && PS4.getButtonPress(LEFT)) {
      Serial.println("Slow left (3): PWM 45");
      digitalWrite(com_pin4, LOW);
      digitalWrite(com_pin3, LOW);
      digitalWrite(com_pin2, LOW);
      digitalWrite(com_pin1, HIGH);
      digitalWrite(com_pin0, HIGH);
    }

    else if (PS4.getButtonPress(L2) && PS4.getButtonPress(RIGHT)) {
      Serial.println("Slow right (4): PWM 45");
      digitalWrite(com_pin4, LOW);
      digitalWrite(com_pin3, LOW);
      digitalWrite(com_pin2, HIGH);
      digitalWrite(com_pin1, LOW);
      digitalWrite(com_pin0, LOW);
    }

    else if (PS4.getButtonPress(L2) && PS4.getButtonPress(L1)) {
      Serial.println("Slow anti-clock (14): PWM 20");
      digitalWrite(com_pin4, LOW);
      digitalWrite(com_pin3, HIGH);
      digitalWrite(com_pin2, HIGH);
      digitalWrite(com_pin1, HIGH);
      digitalWrite(com_pin0, LOW);
    }

    else if (PS4.getButtonPress(L2) && PS4.getButtonPress(R1)) {
      Serial.println("Slow clock (13): PWM 20");
      digitalWrite(com_pin4, LOW);
      digitalWrite(com_pin3, HIGH);
      digitalWrite(com_pin2, HIGH);
      digitalWrite(com_pin1, LOW);
      digitalWrite(com_pin0, HIGH);
    }

    else if (PS4.getButtonPress(UP)) {
      Serial.println("Forward (5): PWM 85");
      digitalWrite(com_pin4, LOW);
      digitalWrite(com_pin3, LOW);
      digitalWrite(com_pin2, HIGH);
      digitalWrite(com_pin1, LOW);
      digitalWrite(com_pin0, HIGH);
    }

    else if (PS4.getButtonPress(DOWN)) {
      Serial.println("Backward (6): PWM 85");
      digitalWrite(com_pin4, LOW);
      digitalWrite(com_pin3, LOW);
      digitalWrite(com_pin2, HIGH);
      digitalWrite(com_pin1, HIGH);
      digitalWrite(com_pin0, LOW);
    }

    else if (PS4.getButtonPress(LEFT)) {
      Serial.println("LEFT (7): PWM 85");
      digitalWrite(com_pin4, LOW);
      digitalWrite(com_pin3, LOW);
      digitalWrite(com_pin2, HIGH);
      digitalWrite(com_pin1, HIGH);
      digitalWrite(com_pin0, HIGH);
    }

    else if (PS4.getButtonPress(RIGHT)) {
      Serial.println("RIGHT (8): PWM 85");
      digitalWrite(com_pin4, LOW);
      digitalWrite(com_pin3, HIGH);
      digitalWrite(com_pin2, LOW);
      digitalWrite(com_pin1, LOW);
      digitalWrite(com_pin0, LOW);
    }

    else if (PS4.getButtonPress(R1)) {
      Serial.println("Clock (15): PWM 35");
      digitalWrite(com_pin4, LOW);
      digitalWrite(com_pin3, HIGH);
      digitalWrite(com_pin2, HIGH);
      digitalWrite(com_pin1, HIGH);
      digitalWrite(com_pin0, HIGH);
    }

    else if (PS4.getButtonPress(L1)) {
      Serial.println("Anti-Clock (16): PWM 35");
      digitalWrite(com_pin4, HIGH);
      digitalWrite(com_pin3, LOW);
      digitalWrite(com_pin2, LOW);
      digitalWrite(com_pin1, LOW);
      digitalWrite(com_pin0, LOW);
    }

    else {
      Serial.println("Stop (31)");
      digitalWrite(com_pin4, HIGH);
      digitalWrite(com_pin3, HIGH);
      digitalWrite(com_pin2, HIGH);
      digitalWrite(com_pin1, HIGH);
      digitalWrite(com_pin0, HIGH);
    }
    /**** Drive *****/
  }
}