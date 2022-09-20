/**************************************
 Press Switch to Start Program 
 You can adjust the kp ki in real time by rotating the joystick
 in the x or y direction 
**************************************/

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

// PI Parameters for Velocity Control 
float kp = 1.6;             // Initial Propotional Control Gain
float ki = 6.5;             // Initial Integral Control Gain
float kd = 0.018;             // derivative gain
float e = 0;              // Error between Target and Velocity 
float ePre = 0;           // Store Previous Error
float eIntegral = 0;      // ki*eIntegral
float dedt = 0;           // kd*dedt
float u = 0;              // Control Signal

// Variable set for Joystick
float vx = 0;             // X Read by Joystick
float vy = 0;             // Y Read by Joystick
float vsw = 0;            // Switch Read by Joystick 
float vxPre = 0;          // Store Previous X
float vyPre = 0;          // Store Previous Y
float vswPre = 0;         // Store Previous Switch
int testSW = 0;           // Switch Status

// Target Signal
float sqTarg = 0;         // Square Target Signal
float sqTarw = 0;         // Target Signal with Switch

void setup(){
  Serial.begin(115200);
  // Pin Mode For Encoder and Motor Controller
  pinMode(ChA,INPUT_PULLUP);                                      // Pin Mode for Chanel A
  pinMode(ChB,INPUT);                                             // Pin Mode for Chanel B
  attachInterrupt(digitalPinToInterrupt(ChA),readEncoder,RISING); // Interrupt to Read Encoder
  pinMode(M1, OUTPUT);                                            // Pin Mode for Motor Controller

  // Pin Mode For Joystick
  pinMode(A0, INPUT);    // Pin Mode for x Axis
  pinMode(A1, INPUT);    // Pin Mode for y Axis
  pinMode(A2, INPUT);    // Pin MOde for Switch
}

void loop(){
  vx = analogRead(A0);   // Read Joystick x Axis
  vy = analogRead(A1);   // Read Joystick y Axis
  vsw = analogRead(A2);  // Read Joystick Switch

  // Tuning Control Gains
  checkVX();             // Renew kp
  checkVY();             // Renew ki
  checkVSW();            // Start Tuning
  vxPre = vx;            // Store Previous kp
  vyPre = vy;            // Store Previous ki
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

  // Generate Target Reference (Square Wave)
  sqTarg = 150 * (sin(6.28 * 0.4 * currT/1e6) > 0);// Square Wave with Amplitude 150 
  sqTarw = sqTarg * testSW;                        // Target Start by Switch

  // PI Velocity Controller
  e = sqTarw - vFilt;                     // Error Between Target and Velocity
  eIntegral = eIntegral + e * deltaT;     // Error Integrate
  dedt = (e - ePre) / deltaT;
  u = kp * e + ki * eIntegral + kd * dedt;            // Control Signal into Motor Controller 
  motorSpeed = fabs(u);                   // Calculate Motor Speed(PWM)
  if(motorSpeed > 255){               
    motorSpeed = 255;
    }
  dir = 1;                                // Define Motor Direction
  if(u < 0){
    dir = -1;
    }
  motorSetup(dir , motorSpeed , E1 , M1); // Control Motor
  ePre = e;                               // Store Previous Error

  // Print Data
  /*
  Serial.print(kp);
  Serial.print("kp ,  ");
  Serial.print(ki);
  Serial.print("ki ,   ");
  Serial.print(testSW);
  Serial.print("testSW ,   ");
  Serial.print(sqTarw);
  Serial.println("Tar ,   ");
  */
  Serial.print(sqTarw);
  Serial.print(",     ");
  Serial.print(vFilt);
  Serial.println(",    ");
  
  }

// Joystick Check Function
void checkVX(){                  // Renew kp
  if (vx > 750){
    if (vxPre <= 750){
      kp = kp + 0.1;
      }
    }
    else if (vx < 250){
      if (vxPre >= 250){
        kp = kp - 0.1;
        }
      }
}
void checkVY(){                  // Renew ki
  if (vy > 750){
    if (vyPre <= 750){
      ki = ki + 0.1;
      }
    }
    else if (vy < 250){
      if (vyPre >= 250){
        ki = ki - 0.1;
        }
      }
}
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
