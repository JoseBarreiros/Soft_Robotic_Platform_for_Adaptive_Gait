//// Soft Robotic Platform for adaptive gait  ////
//// Methods for low level control ////
//// Jose Barreiros, 2018  ////


class DCMotor
{
  // Class Member Variables for DRV8838 PololuBrushled DC Motor Driver 
  // These are initialized at startup
  int EnablePin;      // the number of the ENABLE pin (PWM: speed) 
  int PhasePin;      // the number of the PHASE pin (Digital: direction)
  long OnTime;     // milliseconds of on-time
  long OffTime;    // milliseconds of off-time
 
  // These maintain the current state
  int motorState;                 // motorState used to set the direction of the MOTOR
  unsigned long previousMillis;   // will store last time MOTOR was updated
 
  // Constructor - creates a DCMotor
  // and initializes the member variables and state
  public:
  DCMotor(int pin_pwm, int pin_dir, long on, long off)
  {
  EnablePin = pin_pwm;
  PhasePin = pin_dir;
  pinMode(EnablePin, OUTPUT);    
   pinMode(PhasePin, OUTPUT);    
    
  OnTime = on;
  OffTime = off;
  
  motorState = LOW; 
  previousMillis = 0;
  }
 
  void Update(int mSpeed)
  {
    // check to see if it's time to change the state of the Motor
    unsigned long currentMillis = millis();
     
    if((motorState == HIGH) && (currentMillis - previousMillis >= OnTime))
    {
      motorState = LOW;  // Release the tendon
      previousMillis = currentMillis;  // Remember the time
      digitalWrite(PhasePin, motorState);  // Update the direction of the Motor
      analogWrite(EnablePin, mSpeed);  // Update the speed of the Motor
      
    }
    else if ((motorState == LOW) && (currentMillis - previousMillis >= OffTime))
    {
      motorState = HIGH;  // turn it on
      previousMillis = currentMillis;   // Remember the time
      digitalWrite(PhasePin, motorState);  // Update the direction of the Motor
      analogWrite(EnablePin, mSpeed);  // Update the speed of the Motor
    }
  }
};



class Waveguide
{
  // Class Member Variables for a waveguide based on IR LED (VSLB3940: 1.15-1.6v) & Phototransistor (PT204-6B)
  // These are initialized at startup
  
  int PhotoPin;      // the number of the Phototransistor pin (Analog: intensity)
  int LEDPin;  // the number of the LED pin
  long SamplingTime;     // milliseconds of sampling time
 
  // These maintain the current state
  unsigned long previousMillis;   // will store last time MOTOR was updated
 
  // Constructor - creates a Waveguide
  // and initializes the member variables and state
  public:
  Waveguide(int pin, int led, long sampling)
  {
  PhotoPin = pin;
  LEDPin=led;
  pinMode(PhotoPin, INPUT);    
  pinMode(LEDPin, OUTPUT);    
    
  SamplingTime = sampling;
  previousMillis = 0;
  }

  int Update()
  {
    // check to see if it's time to change the state of the Motor
    unsigned long currentMillis = millis();
    int sensorRaw; 
    int outputValue; 
    
    analogWrite(LEDPin, 255);
    if((currentMillis - previousMillis >= SamplingTime))
    {
      previousMillis = currentMillis;  // Remember the time
      sensorRaw=analogRead(PhotoPin);  // Update the reading of the Phototransistor 
      // map it to the range of the analog out:
      outputValue = map(sensorRaw, 800, 1023, 0, 1000);
      //Serial.print("sensor_raw = ");
      //Serial.print(sensorRaw);
      //Serial.print("  sensor = ");
      //Serial.println(outputValue);      
      return sensorRaw;
    }

  }
};



class CurrentSensor
{
  // Class Member Variables for a waveguide based on IR LED (VSLB3940: 1.15-1.6v) & Phototransistor (PT204-6B)
  // These are initialized at startup
  
  int SensingPin;      // the number of the Phototransistor pin (Analog: intensity)
  long SamplingTime;     // milliseconds of sampling time
 
  // These maintain the current state
  unsigned long previousMillis;   // will store last time MOTOR was updated
 
  // Constructor - creates a Waveguide
  // and initializes the member variables and state
  public:
  CurrentSensor(int pin, long sampling)
  {
  SensingPin = pin;
  pinMode(SensingPin, INPUT);      
    
  SamplingTime = sampling;
  previousMillis = 0;
  }
 
int Update()
  {
    // check to see if it's time to change the state of the Motor
    unsigned long currentMillis = millis();
    int sensorRaw; 
    int outputValue; 
    
    if((currentMillis - previousMillis >= SamplingTime))
    {
      previousMillis = currentMillis;  // Remember the time
      sensorRaw=analogRead(SensingPin);  // Update the reading of the Phototransistor 
      // map it to the range of the analog out:
      outputValue = map(sensorRaw, 800, 1023, 0, 1000);
      //Serial.print("\tCurrent_raw = ");
      //Serial.println(sensorRaw);

      return sensorRaw;
      //Serial.print("  sensor = ");
      //Serial.print(outputValue);      
      
    }

  }
};



 
//DCMotor muscle1(9, 8, 2800, 1400); //
 
DCMotor muscle1(6, 8, 3200, 1250); // for 2 tendons tex138
DCMotor muscle2(9, 12, 3000, 1250); // for 2 tendons tex138
DCMotor muscle3(5, 7, 3200, 900); // for 2 tendons tex138

Waveguide guide1(A3,4,50);
Waveguide guide2(A1,3,50);
Waveguide guide3(A2,2,50);

CurrentSensor current1(A0,50);
CurrentSensor current2(A6,50);
CurrentSensor current3(A7,50);
 
void setup() 
{ 
  Serial.begin(9600);

} 
 
 
void loop() 
{ 
    int c_sensorRaw1=-1; 
    int w_sensorRaw1=-1; 
    int c_sensorRaw2=-1; 
    int w_sensorRaw2=-1; 
    int c_sensorRaw3=-1; 
    int w_sensorRaw3=-1; 
     
    muscle1.Update(255);
    w_sensorRaw1=guide1.Update();
    c_sensorRaw1=current1.Update();

    muscle2.Update(255);
    w_sensorRaw2=guide2.Update();
    c_sensorRaw2=current2.Update();
    
    muscle3.Update(255);
    w_sensorRaw3=guide3.Update();
    c_sensorRaw3=current3.Update();

  //  && w_sensorRaw2!=-1
  //  if (w_sensorRaw3!=-1 && w_sensorRaw1!=-1 ){
    Serial.print("current_sensor_raw 1 = ");
    Serial.print(c_sensorRaw1);
    Serial.print("\twaveguide_raw 1 = ");
    Serial.print(w_sensorRaw1);

    Serial.print("\tcurrent_sensor_raw 2 = ");
    Serial.print(c_sensorRaw2);
    Serial.print("\twaveguide_raw 2 = ");
    Serial.print(w_sensorRaw2);
    
    Serial.print("\tcurrent_sensor_raw 3 = ");
    Serial.print(c_sensorRaw3);
    Serial.print("\twaveguide_raw 3 = ");
    Serial.println(w_sensorRaw3);
 //   }




    

} 
