/************************************* 
 Press Switch to Start Program 
 Test Tracking Performance(Sine Wave)
 Double Loop Position Controller 
 Tuning Position PID 
*************************************/

// For Motor Controller
int E1 = 3;            // Motor Controller E1
int M1 = 4;            // Motor Controller M1
float motorSpeed = 0;  // PWM Signal Into Motor
int dir = 0;           // Motor Direction

// For Encorder
const byte ChA = 18;      // Encoder Chanel A
const byte ChB = 19;      // Encoder Chanel B
volatile int pos_i = 0;   // Counts Read(in function)
int pos = 0;              // Counts Read(in loop)
int posPre = 0;           // Store Previous Pos
int increment = 0;        // Increment for Counts
int b = 0;                // Read Chanel B

// Velocity from Encoder
long currT = 0;           // Current Time
float deltaT = 0;         // Time Difference
long preT = 0;            // Previous Time
float velocity = 0;       // Velocity Counts/s
float v = 0;              // Velocity RPM
float vFilt = 0;          // Velocity Filt by Low Pass Filter(LPF)
float vPre = 0;           // Store Previous Velocity

// PI Parameters for Velocity Control(Inner Loop) 
float kp = 0;             // Initial Propotional Control Gain
float ki = 0;             // Initial Integral Control Gain
float e = 0;              // Error between Target and Velocity 
float ePre = 0;           // Store Previous Error
float eIntegral = 0;      // ki*eIntegral
float u = 0;              // Control Signal

// PID Parameters for position Control(Outer Loop)
float kp_p = 0;             // Initial Propotional Control Gain
float ki_p = 0;             // Initial Integral Control Gain
float kd_p = 0;             // Initial Derivative Control Gain
float e_p = 0;              // Error between Target and Velocity 
float ePre_p = 0;           // Store Previous Error
float eIntegral_p = 0;      // ki_p*eIntegral
float dedt_p = 0;           // Derivative of Error
float u_p = 0;              // Control Signal

// Variable set for Joystick
float vsw = 0;            // Switch Read by Joystick 
float vswPre = 0;         // Store Previous Switch
int testSW = 0;           // Switch Status

// Target Signal & Feed Forward
float sqTarg = 0;         // Square Target Signal
float sqTarw = 0;         // Target Signal with Switch
float sqTarwPre = 0;      // Previous Target
float dsqTdt = 0;         // Derivative of Target


void setup(){
  Serial.begin(115200);
  // Pin Mode For Encoder and Motor Controller
  pinMode(ChA,INPUT_PULLUP);                                      // Pin Mode for Chanel A
  pinMode(ChB,INPUT);                                             // Pin Mode for Chanel B
  attachInterrupt(digitalPinToInterrupt(ChA),readEncoder,RISING); // Interrupt to Read Encoder
  pinMode(M1, OUTPUT);                                            // Pin Mode for Motor Controller

  // Pin Mode For Joystick
  pinMode(A2, INPUT);    // Pin MOde for Switch
}

void loop(){
  vsw = analogRead(A2);  // Read Joystick Switch

  // Tuning Control Gains
  checkVSW();            // Start Tuning
  vswPre = vsw;          // Store Previous Switch Status

  // Calculate Motor Velocity  
  pos = pos_i;                                         // Read Motor Counts
  currT = micros();                                    // Start Current Time Microsecond
  deltaT = ((float)(currT-preT)) / (1e6);              // Calculate Delta Time
  velocity = (pos - posPre)/deltaT;                    // Calculate Velocity(Counts/s)
  preT = currT;                                        // Store Previous Time
  posPre = pos;                                        // Store Previous Pos
  v = 60 * velocity / (750.0*4);                       // Velocity in RPM
  vFilt = 0.854 * vFilt + 0.0728 * v + 0.0728 * vPre;  // Velocity after LPF(25Hz)
  vPre = v;                                            // Store Previous Velocity

  // Generate Target Reference (Sine Wave)
  sqTarg = 150 * sin(currT/1e6);          // Sine Wave with Amplitude 150 
  sqTarw = sqTarg * testSW;               // Target Start by Switch

  // PID Position Controller
  kp_p = 0.05;
  ki_p = 0.01;
  kd_p = 0.05;
  e_p = sqTarw - pos / 8.3333;                           // Calculate Position Error
  dedt_p = (e_p-ePre_p) / (deltaT);                      // Calculate e Derivative
  eIntegral_p = eIntegral_p + e_p * deltaT;              // Calculate e Integral
  u_p = kp_p * e_p + ki_p * eIntegral_p + kd_p * dedt_p; 

  // Feed Forward
  dsqTdt = (sqTarw - sqTarwPre) / (deltaT);              // Calculate Reference Velocity        

  // PI Velocity Controller
  e = dsqTdt + u_p - vFilt;               // Error Between Target and Velocity
  eIntegral = eIntegral + e * deltaT;     // Error Integrate
  u = kp * e + ki * eIntegral;            // Control Signal into Motor Controller 
  motorSpeed = fabs(u);                   // Calculate Motor Speed(PWM)
  if(motorSpeed > 255){               
    motorSpeed = 255;
    }
  dir = 1;                                // Define Motor Direction
  if(u < 0){
    dir = -1;
    }
  motorSetup(dir , motorSpeed , E1 , M1); // Control Motor
  ePre = e;                               // Store Previous Error(velocity loop)
  ePre_p = e_p;                           // Store Previous Error(position loop)      
  sqTarwPre = sqTarw;                     // Store Previous Target

  // Print Data
  Serial.print(sqTarw);
  Serial.print(",    ");
  Serial.println(testSW);
  //Serial.println("Tar ,   ");
  }

// Joystick Check Function
void checkVSW(){                 // Start Target
  if (vsw == 0){
    if (vswPre > 0){
      testSW = 1 - testSW;
      }
    }
  }

// Motor Control Function
void motorSetup(int dir, int motorSpeed, int in1, int in2){
   analogWrite(in1, motorSpeed);
   if(dir == 1){
    digitalWrite(in2,LOW);
   }
   else if(dir == -1){
    digitalWrite(in2,HIGH);
   }
}

// Read Encoder Function
void readEncoder(){
  b = digitalRead(ChB);
  increment = 0;
  if(b>0){
    increment = -1;
  }
  else{
    increment = 1;
  }
   pos_i = pos_i + increment;
}
