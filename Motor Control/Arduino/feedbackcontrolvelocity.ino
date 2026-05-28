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
int current_read; // Current reading
int x1; // State variable x1
int x2; // State variable x2

int e;  // r(n) = x(n) - y(n)
float v; // v(n)
float v1; // v(n-1)
float c; // c(n)

float K = 1;
float K1 = -0.8245;
float K2 = -8.4882;
float Ts = 0.1;


const int PinENA = 19;  // L298N ENA Pin
const int PinIN1 = 18; // L298N IN1 Pin
const int PinIN2 = 5; // L298N IN2 Pin
const int PinA = 15; // Encoder Pin A
const int PinB = 4; // Encoder Pin B
const int PinPOT = 34; // Potentiometer output (setpoint)
const int PinVol = 35; // ADC Output (Voltage)
const int PinI = 32; // ADC Output (Current)


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
  if ((currentMillis - previousMillis) >= 100) {
    previousMillis = currentMillis;
    // ----- State Sensing -----//
    rpm = 10 * counter * 60 / ppr;
    counter = 0;
    current_read = analogRead(PinI); 


      // ----- Defining the setpoint -----//
    pot = analogRead(PinPOT); 
    sp = map(pot, 0, 4095, -45, 45); // 0 - 4095 to -45 - 45 rpm
    e = sp - rpm;   // e(n) = r(n) - y(n)

    //----- Integral Control ------//  
    v = e + v1;
    v1 = v;

    //----- Feedback ------//
    c = v * K - x1 * K1 - x2 * K2;
       
    x1 = rpm;
    x2 = current_read;
    x2 = x2 / 4000;
  }  

  //---- Defining controller limits ---- //
  if (c > 500) {c = 500;}
  if (c < -500) {c = -500;}
  
  motor = map(c, -500, 500, -255, 255);
  
  motor_control(motor);    // PWM output from 0 to 255
  Serial.print(sp);
  Serial.print(",");
  Serial.println(rpm);
  delay(10);
}

void interrupt() {
  // if (digitalRead(PinB) == 1) {
  if (dir == 1) {
    counter++;    
  }
  else {
    counter--;    
  }
}

// Motor control function
void motor_control(float speed) {
  if (speed > 0) {
    digitalWrite(PinIN1, LOW);
    digitalWrite(PinIN2, HIGH);
    // dir = 1;
  }
  else if (speed < 0) {
    digitalWrite(PinIN1, HIGH);
    digitalWrite(PinIN2, LOW);
    // dir = -1;
  }
  else {
    digitalWrite(PinIN1, LOW);
    digitalWrite(PinIN2, LOW);
    // dir = 0;
  }
  analogWrite(PinENA, abs(speed));   
}
