//Meccanum Forward reciever code digital pins communication
//FR->m1 FL->m2
// Pin definition ***** /
int PIN_INPUT1 = 3;
int PIN_INPUT2 = 2;

//IR
volatile long int ctr1 = 0, ctr2 = 0;
long int newcount1 = 0, newcount2 = 0;
int s1 = 0, s2 = 0;

long currT;
float deltaT;

//CYTRON
const int m1_dir = 5;
const int m1_pwm = 6;

const int m2_dir = 7;
const int m2_pwm = 9;

int target1 = 0, target2 = 0;

long prevT = 0;

//motors
int pwr1, pwr2;

//communication pins

int com_pin4 = 13;
int com_pin3 = 11;
int com_pin2 = 12;
int com_pin1 = 10;
int com_pin0 = 8;

int com_var = 0;

int target_slow_left = 45;
int target_slow_right = 45;

int target_normal_left = 130;
int target_normal_right = 130;

int target_fast_left = 170;
int target_fast_right = 170;

int target_stop_left = 0;
int target_stop_right = 0;

int current_value_left = 0;
int current_value_right = 0;

int step_accel_left = 1.0001;
int step_accel_right = 1.0001;

int step_decel_left = 2.2;
int step_decel_right = 2.2;


class SimplePID {
private:
  float kp, kd, ki, umax;  // Parameters
  float eprev, eintegral;  // Storage
  float e, u;
public:
  // Constructor
  SimplePID()
    : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0) {}

  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn) {
    kp = kpIn;
    kd = kdIn;
    ki = kiIn;
    umax = umaxIn;
  }

  // A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr, int pwm_pin) {
    // error
    e = target - value;
    //Serial.print("value = ");
    //Serial.print(value);
    //Serial.print(" e = ");
    //Serial.print(e);

    // derivative
    float dedt = (e - eprev) / (deltaT);
    //Serial.println("e dt");
    //Serial.println(dedt);
    // integral
    eintegral = eintegral + e * deltaT;
    //Serial.println("e int");
    //Serial.println(eintegral);
    //Serial.println("k1");
    //Serial.println(ki);
    // control signal
    if (target == 0) {
      u = (0.1 * e + kd * dedt);
      e = 0;
      dedt = 0;
      eintegral = 0;
    } else {
      u = (kp * e + kd * dedt + ki * eintegral);
    }

    //Serial.print(" u = ");
    //Serial.print(u);

    // motor power
    pwr = (int)fabs(u);
    //pwr = abs(pwr);

    if (pwr > umax) {
      pwr = umax;
    }

    // Serial.print(" pwr = ");
    // Serial.println(pwr);
    analogWrite(pwm_pin, pwr);

    // store previous error
    eprev = e;
  }
};

SimplePID pid1, pid2;

void setup() {
  Serial.begin(115200);

  pinMode(PIN_INPUT1, INPUT);
  pinMode(PIN_INPUT2, INPUT);


  attachInterrupt(digitalPinToInterrupt(PIN_INPUT1), interrupt_routine1, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_INPUT2), interrupt_routine2, FALLING);

  //Cytron
  pinMode(m1_dir, OUTPUT);
  pinMode(m1_pwm, OUTPUT);
  pinMode(m2_dir, OUTPUT);
  pinMode(m2_pwm, OUTPUT);

  //Communication pins
  pinMode(com_pin4, INPUT);
  pinMode(com_pin3, INPUT);
  pinMode(com_pin2, INPUT);
  pinMode(com_pin1, INPUT);
  pinMode(com_pin0, INPUT);

  //FR->m1 FL->m2

  pid1.setParams(1.7, 0, 0.0000005, 255);  //Change kP,kD,kI,umax only here
  pid2.setParams(1.9, 0, 0.0000005, 255);  //Change kP,kD,kI,umax only here
  // pid1.evalu(s1, target1, deltaT, pwr1, m1_pwm);
  // pid2.evalu(s2, target2, deltaT, pwr2, m2_pwm);
}

void loop() {
  currT = micros();
  deltaT = ((float)(currT - prevT));
  if (currT - prevT > 300000) {
    noInterrupts();
    newcount1 = ctr1;
    newcount2 = ctr2;
    readspeed();
    prevT = currT;
    ctr1 = 0;
    ctr2 = 0;
    interrupts();
  }
  // analogWrite(m1_pwm, 50);
  // analogWrite(m2_pwm, 50);
  // pid1.evalu(s1, target1, deltaT, pwr1, m1_pwm);
  // pid2.evalu(s2, target2, deltaT, pwr2, m2_pwm);

  com_var = digitalRead(com_pin4) * pow(2, 4) + digitalRead(com_pin3) * pow(2, 3) + digitalRead(com_pin2) * pow(2, 2) + digitalRead(com_pin1) * pow(2, 1) + digitalRead(com_pin0) * pow(2, 0);

  Serial.print("Pin4: ");
  Serial.print(digitalRead(com_pin4));

  Serial.print("Pin3: ");
  Serial.print(digitalRead(com_pin3));

  Serial.print(" Pin2: ");
  Serial.print(digitalRead(com_pin2));

  Serial.print(" Pin1: ");
  Serial.print(digitalRead(com_pin1));

  Serial.print(" Pin0: ");
  Serial.println(digitalRead(com_pin0));

  Serial.print("Com Variable: ");
  Serial.println(com_var);

  if (com_var == 1) {  // Slow Forward
    target1 = 20;
    target2 = 20;
    fwd();
    // analogWrite(m1_pwm, 45);
    // analogWrite(m2_pwm, 45);

    if (current_value_left < target_slow_left) {
      current_value_left += step_accel_left;
      Serial.print("Current Value Left");
      Serial.println(current_value_left);
      analogWrite(m2_pwm, current_value_left);
    }

    if (current_value_right < target_slow_right) {
      current_value_right += step_accel_right;
      Serial.print("Current Value Right");
      Serial.println(current_value_right);
      analogWrite(m1_pwm, current_value_right);
    }

    if (current_value_left > target_slow_left) {
      current_value_left = target_slow_left;
      Serial.print("Current Value Left");
      Serial.println(current_value_left);
      analogWrite(m2_pwm, current_value_left);
    }

    if (current_value_right > target_slow_right) {
      current_value_right = target_slow_right;
      Serial.print("Current Value Right");
      Serial.println(current_value_right);
      analogWrite(m1_pwm, current_value_right);
    }


  } else if (com_var == 2) {  // Slow Backward
    target1 = 20;
    target2 = 20;
    bkw();
    // analogWrite(m1_pwm, 45);
    // analogWrite(m2_pwm, 45);

    if (current_value_left < target_slow_left) {
      current_value_left += step_accel_left;
      Serial.print("Current Value Left");
      Serial.println(current_value_left);
      analogWrite(m2_pwm, current_value_left);
    }

    if (current_value_right < target_slow_right) {
      current_value_right += step_accel_right;
      Serial.print("Current Value Right");
      Serial.println(current_value_right);
      analogWrite(m1_pwm, current_value_right);
    }

    if (current_value_left > target_slow_left) {
      current_value_left = target_slow_left;
      Serial.print("Current Value Left");
      Serial.println(current_value_left);
      analogWrite(m2_pwm, current_value_left);
    }

    if (current_value_right > target_slow_right) {
      current_value_right = target_slow_right;
      Serial.print("Current Value Right");
      Serial.println(current_value_right);
      analogWrite(m1_pwm, current_value_right);
    }

  } else if (com_var == 3) {  // Slow Left
    target1 = 20;
    target2 = 20;
    lt();
    // analogWrite(m1_pwm, 45);
    // analogWrite(m2_pwm, 45);

    if (current_value_left < target_slow_left) {
      current_value_left += step_accel_left;
      Serial.print("Current Value Left");
      Serial.println(current_value_left);
      analogWrite(m2_pwm, current_value_left);
    }

    if (current_value_right < target_slow_right) {
      current_value_right += step_accel_right;
      Serial.print("Current Value Right");
      Serial.println(current_value_right);
      analogWrite(m1_pwm, current_value_right);
    }

    if (current_value_left > target_slow_left) {
      current_value_left = target_slow_left;
      Serial.print("Current Value Left");
      Serial.println(current_value_left);
      analogWrite(m2_pwm, current_value_left);
    }

    if (current_value_right > target_slow_right) {
      current_value_right = target_slow_right;
      Serial.print("Current Value Right");
      Serial.println(current_value_right);
      analogWrite(m1_pwm, current_value_right);
    }

  } else if (com_var == 4) {  // Slow Right
    target1 = 20;
    target2 = 20;
    rt();
    // analogWrite(m1_pwm, 45);
    // analogWrite(m2_pwm, 45);

    if (current_value_left < target_slow_left) {
      current_value_left += step_accel_left;
      Serial.print("Current Value Left");
      Serial.println(current_value_left);
      analogWrite(m2_pwm, current_value_left);
    }

    if (current_value_right < target_slow_right) {
      current_value_right += step_accel_right;
      Serial.print("Current Value Right");
      Serial.println(current_value_right);
      analogWrite(m1_pwm, current_value_right);
    }

    if (current_value_left > target_slow_left) {
      current_value_left = target_slow_left;
      Serial.print("Current Value Left");
      Serial.println(current_value_left);
      analogWrite(m2_pwm, current_value_left);
    }

    if (current_value_right > target_slow_right) {
      current_value_right = target_slow_right;
      Serial.print("Current Value Right");
      Serial.println(current_value_right);
      analogWrite(m1_pwm, current_value_right);
    }

  } else if (com_var == 5) {  // Forward
    target1 = 20;
    target2 = 20;
    fwd();
    analogWrite(m1_pwm, 85);
    analogWrite(m2_pwm, 85);

    // if (current_value_left < target_normal_left) {
    //   current_value_left += step_accel_left;
    //   Serial.print("Current Value Left");
    //   Serial.println(current_value_left);
    //   analogWrite(m2_pwm, current_value_left);
    // }

    // if (current_value_right < target_normal_right) {
    //   current_value_right += step_accel_right;
    //   Serial.print("Current Value Right");
    //   Serial.println(current_value_right);
    //   analogWrite(m1_pwm, current_value_right);
    // }

    // if (current_value_left > target_normal_left) {
    //   current_value_left = target_normal_left;
    //   Serial.print("Current Value Left");
    //   Serial.println(current_value_left);
    //   analogWrite(m2_pwm, current_value_left);
    // }

    // if (current_value_right > target_normal_right) {
    //   current_value_right = target_normal_right;
    //   Serial.print("Current Value Right");
    //   Serial.println(current_value_right);
    //   analogWrite(m1_pwm, current_value_right);
    // }


  } else if (com_var == 6) {  // Backward
    target1 = 20;
    target2 = 20;
    bkw();
    analogWrite(m1_pwm, 85);
    analogWrite(m2_pwm, 85);

    // if (current_value_left < target_normal_left) {
    //   current_value_left += step_accel_left;
    //   Serial.print("Current Value Left");
    //   Serial.println(current_value_left);
    //   analogWrite(m2_pwm, current_value_left);
    // }

    // if (current_value_right < target_normal_right) {
    //   current_value_right += step_accel_right;
    //   Serial.print("Current Value Right");
    //   Serial.println(current_value_right);
    //   analogWrite(m1_pwm, current_value_right);
    // }

    // if (current_value_left > target_normal_left) {
    //   current_value_left = target_normal_left;
    //   Serial.print("Current Value Left");
    //   Serial.println(current_value_left);
    //   analogWrite(m2_pwm, current_value_left);
    // }

    // if (current_value_right > target_normal_right) {
    //   current_value_right = target_normal_right;
    //   Serial.print("Current Value Right");
    //   Serial.println(current_value_right);
    //   analogWrite(m1_pwm, current_value_right);
    // }

  } else if (com_var == 7) {  // Left
    target1 = 20;
    target2 = 20;
    lt();
    // analogWrite(m1_pwm, 95);
    // analogWrite(m2_pwm, 85);

    if (current_value_left < target_normal_left) {
      current_value_left += step_accel_left;
      Serial.print("Current Value Left");
      Serial.println(current_value_left);
      analogWrite(m2_pwm, current_value_left);
    }

    if (current_value_right < target_normal_right) {
      current_value_right += step_accel_right;
      Serial.print("Current Value Right");
      Serial.println(current_value_right);
      analogWrite(m1_pwm, current_value_right);
    }

    if (current_value_left > target_normal_left) {
      current_value_left = target_normal_left;
      Serial.print("Current Value Left");
      Serial.println(current_value_left);
      analogWrite(m2_pwm, current_value_left);
    }

    if (current_value_right > target_normal_right) {
      current_value_right = target_normal_right;
      Serial.print("Current Value Right");
      Serial.println(current_value_right);
      analogWrite(m1_pwm, current_value_right);
    }

  } else if (com_var == 8) {  // Right
    target1 = 20;
    target2 = 20;
    rt();
    // analogWrite(m1_pwm, 85);
    // analogWrite(m2_pwm, 85);

    if (current_value_left < target_normal_left) {
      current_value_left += step_accel_left;
      Serial.print("Current Value Left");
      Serial.println(current_value_left);
      analogWrite(m2_pwm, current_value_left);
    }

    if (current_value_right < target_normal_right) {
      current_value_right += step_accel_right;
      Serial.print("Current Value Right");
      Serial.println(current_value_right);
      analogWrite(m1_pwm, current_value_right);
    }

    if (current_value_left > target_normal_left) {
      current_value_left = target_normal_left;
      Serial.print("Current Value Left");
      Serial.println(current_value_left);
      analogWrite(m2_pwm, current_value_left);
    }

    if (current_value_right > target_normal_right) {
      current_value_right = target_normal_right;
      Serial.print("Current Value Right");
      Serial.println(current_value_right);
      analogWrite(m1_pwm, current_value_right);
    }

  } else if (com_var == 9) {  // Fast Forward
    target1 = 20;
    target2 = 20;
    fwd();
    // analogWrite(m1_pwm, 130);
    // analogWrite(m2_pwm, 130);

    if (current_value_left < target_fast_left) {
      current_value_left += step_accel_left;
      Serial.print("Current Value Left");
      Serial.println(current_value_left);
      analogWrite(m2_pwm, current_value_left);
    }

    if (current_value_right < target_fast_right) {
      current_value_right += step_accel_right;
      Serial.print("Current Value Right");
      Serial.println(current_value_right);
      analogWrite(m1_pwm, current_value_right);
    }

    if (current_value_left > target_fast_left) {
      current_value_left = target_fast_left;
      Serial.print("Current Value Left");
      Serial.println(current_value_left);
      analogWrite(m2_pwm, current_value_left);
    }

    if (current_value_right > target_fast_right) {
      current_value_right = target_fast_right;
      Serial.print("Current Value Right");
      Serial.println(current_value_right);
      analogWrite(m1_pwm, current_value_right);
    }

  } else if (com_var == 10) {  // Fast Backward
    target1 = 20;
    target2 = 20;
    bkw();
    // analogWrite(m1_pwm, 130);
    // analogWrite(m2_pwm, 130);

    if (current_value_left < target_fast_left) {
      current_value_left += step_accel_left;
      Serial.print("Current Value Left");
      Serial.println(current_value_left);
      analogWrite(m2_pwm, current_value_left);
    }

    if (current_value_right < target_fast_right) {
      current_value_right += step_accel_right;
      Serial.print("Current Value Right");
      Serial.println(current_value_right);
      analogWrite(m1_pwm, current_value_right);
    }

    if (current_value_left > target_fast_left) {
      current_value_left = target_fast_left;
      Serial.print("Current Value Left");
      Serial.println(current_value_left);
      analogWrite(m2_pwm, current_value_left);
    }

    if (current_value_right > target_fast_right) {
      current_value_right = target_fast_right;
      Serial.print("Current Value Right");
      Serial.println(current_value_right);
      analogWrite(m1_pwm, current_value_right);
    }

  } else if (com_var == 11) {  // Fast Left
    target1 = 20;
    target2 = 20;
    lt();
    // analogWrite(m1_pwm, 130);
    // analogWrite(m2_pwm, 130);

    if (current_value_left < target_fast_left) {
      current_value_left += step_accel_left;
      Serial.print("Current Value Left");
      Serial.println(current_value_left);
      analogWrite(m2_pwm, current_value_left);
    }

    if (current_value_right < target_fast_right) {
      current_value_right += step_accel_right;
      Serial.print("Current Value Right");
      Serial.println(current_value_right);
      analogWrite(m1_pwm, current_value_right);
    }

    if (current_value_left > target_fast_left) {
      current_value_left = target_fast_left;
      Serial.print("Current Value Left");
      Serial.println(current_value_left);
      analogWrite(m2_pwm, current_value_left);
    }

    if (current_value_right > target_fast_right) {
      current_value_right = target_fast_right;
      Serial.print("Current Value Right");
      Serial.println(current_value_right);
      analogWrite(m1_pwm, current_value_right);
    }

  } else if (com_var == 12) {  // Fast Right
    target1 = 20;
    target2 = 20;
    rt();
    // analogWrite(m1_pwm, 130);
    // analogWrite(m2_pwm, 130);

    if (current_value_left < target_fast_left) {
      current_value_left += step_accel_left;
      Serial.print("Current Value Left");
      Serial.println(current_value_left);
      analogWrite(m2_pwm, current_value_left);
    }

    if (current_value_right < target_fast_right) {
      current_value_right += step_accel_right;
      Serial.print("Current Value Right");
      Serial.println(current_value_right);
      analogWrite(m1_pwm, current_value_right);
    }

    if (current_value_left > target_fast_left) {
      current_value_left = target_fast_left;
      Serial.print("Current Value Left");
      Serial.println(current_value_left);
      analogWrite(m2_pwm, current_value_left);
    }

    if (current_value_right > target_fast_right) {
      current_value_right = target_fast_right;
      Serial.print("Current Value Right");
      Serial.println(current_value_right);
      analogWrite(m1_pwm, current_value_right);
    }

  } else if (com_var == 13) {  // Slow Clock
    target1 = 20;
    target2 = 20;
    current_value_left = 18;
    current_value_right = 18;
    cw();
    analogWrite(m2_pwm, 18);
    analogWrite(m1_pwm, 18);
  } else if (com_var == 14) {  // Slow Anti Clock
    target1 = 20;
    target2 = 20;
    current_value_left = 18;
    current_value_right = 18;
    ccw();
    analogWrite(m2_pwm, 18);
    analogWrite(m1_pwm, 18);
  } else if (com_var == 15) {  // Clock
    target1 = 20;
    target2 = 20;
    current_value_left = 35;
    current_value_right = 35;
    cw();
    analogWrite(m2_pwm, 35);
    analogWrite(m1_pwm, 35);
  } else if (com_var == 16) {  // Anti Clock
    target1 = 20;
    target2 = 20;
    current_value_left = 35;
    current_value_right = 35;
    ccw();
    analogWrite(m2_pwm, 35);
    analogWrite(m1_pwm, 35);
  }

  else if (com_var == 17) {  // Forward wheels
    target1 = 20;
    target2 = 20;
    current_value_left = 130;
    current_value_right = 130;
    fwd();
    // analogWrite(m1_pwm, 100);
    // analogWrite(m2_pwm, 100);

    if (current_value_left < target_fast_left) {
      current_value_left += step_accel_left;
      Serial.print("Current Value Left");
      Serial.println(current_value_left);
      analogWrite(m2_pwm, current_value_left);
    }

    if (current_value_right < target_fast_right) {
      current_value_right += step_accel_right;
      Serial.print("Current Value Right");
      Serial.println(current_value_right);
      analogWrite(m1_pwm, current_value_right);
    }

    if (current_value_left > target_fast_left) {
      current_value_left = target_fast_left;
      Serial.print("Current Value Left");
      Serial.println(current_value_left);
      analogWrite(m2_pwm, current_value_left);
    }

    if (current_value_right > target_fast_right) {
      current_value_right = target_fast_right;
      Serial.print("Current Value Right");
      Serial.println(current_value_right);
      analogWrite(m1_pwm, current_value_right);
    }

  } else if (com_var == 18) {  // Backward wheels
    target1 = 20;
    target2 = 20;
    current_value_left = 0;
    current_value_right = 0;
    fwd();
    analogWrite(m2_pwm, 0);
    analogWrite(m1_pwm, 0);
  }

  else if (com_var == 0) {  //  Stop
    target1 = 0;
    target2 = 0;
    stp();

    if (current_value_left > target_stop_left) {
      current_value_left -= step_decel_left;
      Serial.print("Current Value Left");
      Serial.println(current_value_left);
      analogWrite(m2_pwm, current_value_left);
    }

    if (current_value_right > target_stop_right) {
      current_value_right -= step_decel_right;
      Serial.print("Current Value Right");
      Serial.println(current_value_right);
      analogWrite(m1_pwm, current_value_right);
    }

    if (current_value_left <= target_stop_left) {
      current_value_left = target_stop_left;
      Serial.print("Current Value Left");
      Serial.println(current_value_left);
      analogWrite(m2_pwm, current_value_left);
    }

    if (current_value_right <= target_stop_right) {
      current_value_right = target_stop_right;
      Serial.print("Current Value Right");
      Serial.println(current_value_right);
      analogWrite(m1_pwm, current_value_right);
    }
  }

  else if (com_var == 31) {  //  Stop
    target1 = 0;
    target2 = 0;
    stp();

    if (current_value_left > target_stop_left) {
      current_value_left -= step_decel_left;
      Serial.print("Current Value Left");
      Serial.println(current_value_left);
      analogWrite(m2_pwm, current_value_left);
    }

    if (current_value_right > target_stop_right) {
      current_value_right -= step_decel_right;
      Serial.print("Current Value Right");
      Serial.println(current_value_right);
      analogWrite(m1_pwm, current_value_right);
    }

    if (current_value_left <= target_stop_left) {
      current_value_left = target_stop_left;
      Serial.print("Current Value Left");
      Serial.println(current_value_left);
      analogWrite(m2_pwm, current_value_left);
    }

    if (current_value_right <= target_stop_right) {
      current_value_right = target_stop_right;
      Serial.print("Current Value Right");
      Serial.println(current_value_right);
      analogWrite(m1_pwm, current_value_right);
    }
  }
}

void interrupt_routine1() {
  ctr1++;
}

void interrupt_routine2() {
  ctr2++;
}

void readspeed() {
  s1 = newcount1;
  Serial.print("FL = ");
  Serial.print(s1);
  // Serial.print(",");
  Serial.print(" PWM1 = ");
  Serial.print(pwr1);
  Serial.print(" Target = ");
  Serial.println(target1);

  s2 = newcount2;
  Serial.print("FR = ");
  Serial.print(s2);
  Serial.print(" PWM2 = ");
  Serial.print(pwr2);
  Serial.print(" Target = ");
  Serial.println(target2);
}

//HIGH LEVEL MOTOR FUNCTIONS (fwd, bkw, rt, lt, cw, ccw)
//FR->m1 FL->m2
void fwd() {
  // Serial.println("Bot moving forward");
  m1_cwMotor();
  m2_cwMotor();
}
void bkw() {
  // Serial.println("Bot moving backwards");
  m1_ccwMotor();
  m2_ccwMotor();
}
void rt() {
  // Serial.println("Bot moving right");
  m1_ccwMotor();
  m2_cwMotor();
}
void lt() {
  // Serial.println("Bot moving left");
  m1_cwMotor();
  m2_ccwMotor();
}
void cw() {
  // Serial.println("Bot moving clockwise");
  m1_ccwMotor();
  m2_cwMotor();
}
void ccw() {
  // Serial.println("Bot moving counter-clockwise");
  m1_cwMotor();
  m2_ccwMotor();
}
void stp() {
  // Serial.println("Bot stop");
  m1_stopMotor();
  m2_stopMotor();
}

//LOW LEVEL MOTOR FUNCTIONS

void m1_cwMotor() {
  // Serial.println("Motor A clockwise");
  digitalWrite(m1_dir, HIGH);
  // analogWrite(m1_pwm, 50);
}
void m1_ccwMotor() {
  // Serial.println("Motor A counter-clockwise");
  digitalWrite(m1_dir, LOW);
  // analogWrite(m1_pwm, 50);
}
void m2_cwMotor() {
  // Serial.println("Motor B clockwise");
  digitalWrite(m2_dir, HIGH);
  // analogWrite(m2_pwm, 50);
}
void m2_ccwMotor() {
  // Serial.println("Motor B counter-clockwise");
  digitalWrite(m2_dir, LOW);
  // analogWrite(m2_pwm, 50);
}

void m1_stopMotor() {
  // Serial.println("Motor A stop");
  analogWrite(m1_pwm, 0);
  // analogWrite(m1_pwm, pwr1);
}
void m2_stopMotor() {
  // Serial.println("Motor B stop");
  analogWrite(m2_pwm, 0);
  // analogWrite(m1_pwm, pwr2);
}
