// @TrevorGlove https://github.com/TrevorGlove/motor_control
// Developed for ESP32

// Variable Declarations

volatile int counter = 0;   // Pulse counter

unsigned long previousMillis = 0;  // Previous pulse time
float pot;  // Potentiometer reading
float motor; // Motor PWM
float rpm;  // Revolutions per minute
float sp;   // Setpoint value
int ppr = 486; // Pulses per revolution of encoder 
int dir; // Motor direction
int max_rpm = 45; // Maximum motor rpm at 5V

float c; // c(n)
float c1; // c(n-1)
float c2; // c(n-2)
float e; // e(n)
float e1; // e(n-1)
float e2; // e(n-2)


// Kp, Ki, Kd Values
// float Kp = 6;
// float Ki = 1;
// float Kd = 1;
float Ts = 0.1;

// PID
// float q0 = Kp + Kd/Ts;
// float q1 = -Kp + Ki*Ts - 2*Kd/Ts;
// float q2 = Kd/Ts;

// PD
// float q0 = Kp + Kd/Ts;
// float q1 = -Kd/Ts;

// PI
// float q0 = Kp;
// float q1 = -Kp + Ki*Ts;

// Auxiliary values for PID transformation from A/D
float q0 = 1.458;
float q1 = 0.5521;
float q2 = -0.9039;

const int PinENA = 19;  // L298N ENA Pin
const int PinIN1 = 18; // L298N IN1 Pin
const int PinIN2 = 5; // L298N IN2 Pin
const int PinA = 15; // Encoder Pin A
const int PinB = 2; // Encoder Pin B
const int PinPOT = 34; // Potentiometer output

void setup() {
  Serial.begin(115200);

  pinMode(PinA, INPUT);
  pinMode(PinB, INPUT);
  pinMode(PinIN1, OUTPUT);
  pinMode(PinIN2, OUTPUT);
  pinMode(PinENA, OUTPUT);
  pinMode(PinPOT, INPUT);
  // External interrupt on Pin 15 on every rising edge
  attachInterrupt(digitalPinToInterrupt(PinA), interrupt, RISING);
}

void loop() {
  unsigned long currentMillis = millis();
  if ((currentMillis - previousMillis) >= 1000 * Ts) {
    previousMillis = currentMillis;

    rpm = counter * 60 / (ppr * Ts);
    counter = 0;

    // ----- Defining the setpoint -----//
    
    pot = analogRead(PinPOT);
    sp = map(pot, 0, 4095, -45, 45); // 0 - 4095 to -45 - 45 rpm 
    e = sp - rpm;   // e(n) = r(n) - y(n)

    //----- PID Control ------ A/D// 
    c = c2 + q0 * e + q1 * e1 + q2 * e2;

    // PID
    c = c1 + q0 * e + q1 * e1 + q2 * e2;
    // PD 
    // c = q0 * e + q1 * e1;
    // PI
    // c = c1 + q0 * e + q1 * e1;

    c1 = c;
    c2 = c1;
    e1 = e;
    e2 = e1;
  }
  
  //---- Defining controller limits ---- //
  if (c > 500) {c = 500;}
  if (c < -500) {c = -500;}

  motor = map(c, -500, 500, -255, 255);

  motor_control(motor);    // PWM output from 0 to 255
  Serial.print(sp);
  Serial.print(",");
  Serial.println(rpm);
  delay(1);
}


void interrupt() {
  // if (digitalRead(PinB) == 1) {
  if (dir == HIGH) {counter++;}
  else {counter--;}
}

// Motor control function
void motor_control(float speed) {
  if (speed > 0) {
    digitalWrite(PinIN1, LOW);
    digitalWrite(PinIN2, HIGH);
    dir = 1;
  }
  else if (speed < 0) {
    digitalWrite(PinIN1, HIGH);
    digitalWrite(PinIN2, LOW);
    dir = -1;
  }
  else {
    digitalWrite(PinIN1, LOW);
    digitalWrite(PinIN2, LOW);
    dir = 0;
  }
  analogWrite(PinENA, abs(speed));   
}
