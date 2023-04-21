#define USE_ARDUINO_INTERRUPTS true 
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PulseSensorPlayground.h>
#include <Wire.h>
#include <math.h>

//Declare the Temperature Sensor
#define ONE_WIRE_BUS 2   
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

#define ADXL345_ADDRESS 0x53

// Declare the Pulse Sensor
const int PulseWire = 0; 
const int LED13 = 13;
int Threshold = 550; 
PulseSensorPlayground pulseSensor; 

// Declare Acceleration | Calibration values for the X, Y, and Z axes
float xBias = 0.0;
float yBias = 0.0;
float zBias = 0.0;

// Initialie wanted variables
int time = 100;
int disaster = 0;
int nextStep = 0;

// User inputs | Keyboard inputs
float weight = 75; 
float length = 40;
float enviornmentPressure = 0.12;
int heartBeat = 234;


// Timer is counting in the Water & Ground
int timerW = 0;
int timerG = 0;

// Define the LED pin
int LEDPin = 9;

class Data {
    public:
        int status;
        float speed;
        float acceleration;
        float temperature;
        float pressure;
        int beat;

    public:
        Data(int st, float sp, float ac, float tm, float pr, int bt) {
            status = st;
            speed = sp;
            acceleration = ac;                                                                                                                             
            temperature = tm;
            pressure = pr;
            beat = bt;
        }

        int show_Status() {
          return status;
        }

        void show_Values() {
          Serial.print("Status : ");
          Serial.println(status);

          Serial.print("Speed : ");
          Serial.println(speed);

          Serial.print("Acceleration : ");
          Serial.println(acceleration);

          Serial.print("temperature : ");
          Serial.println(temperature);

          Serial.print("pressure : ");
          Serial.println(pressure);

          Serial.print("beat : ");
          Serial.println(beat);

          Serial.println("----------------------------");
          Serial.println("");
        }
};

// Declare Current & Previous objects
Data current = Data(0, 0, 0, 0, 0, 0);
Data previous = Data(0, 0, 0, 0, 0, 0);

void setup() {

    Serial.begin(9600);
    sensors.begin(); 
    Wire.begin();
    
    pulseSensor.analogInput(PulseWire);
    pulseSensor.blinkOnPulse(LED13);
    pulseSensor.setThreshold(Threshold);
 
    if (pulseSensor.begin()) {
      Serial.println("Measuring user's Heart Beat"); 
    }

    // Put the ADXL345 into measurement mode
    Wire.beginTransmission(ADXL345_ADDRESS);
    Wire.write(0x2D); // Power control register
    Wire.write(0x08); // Enable measurement mode
    Wire.endTransmission();
  
    // Calibrate the sensor
    calibrateAccelerometer();

    // Set the LED pin as an output
    pinMode(LEDPin, OUTPUT);
}

void loop() {
   
    if (disaster == 1) {
        // NEXT STEP | Connect Serial.println("Measuring user's Heart Beat"); 
         Serial.println("Abnormal situation happened....!"); 
         digitalWrite(LEDPin, HIGH); 
    }

    float xAccel = readAccelerometer(0) - xBias;
    float yAccel = readAccelerometer(1) - yBias;
    float zAccel = readAccelerometer(2) - zBias;

    sensors.requestTemperatures(); 

    float _TEMPERATURE = sensors.getTempCByIndex(0); // Enviornment temperature
    int _BEAT = pulseSensor.getBeatsPerMinute(); // User's Heart Beat

    // Convert raw data to g-forces (m/s^2)
    float scale = 9.81 / 256.0; // 256 LSB/g
    float xAccel_mss = xAccel * scale;
    float yAccel_mss = yAccel * scale;
    float zAccel_mss = zAccel * scale;

    float two_d_AXCEL = sqrt(pow(xAccel_mss, 2) + pow(yAccel_mss, 2));
    float _ACCELERATION = sqrt(pow(two_d_AXCEL, 2) + pow(zAccel_mss, 2))/10;

    current = Data(1, 0, _ACCELERATION, _TEMPERATURE, 9, _BEAT);
    float _SPEED = (current.acceleration - previous.acceleration) * time / 10;

    if (_SPEED <= 0.05) {
      _SPEED = 0.00;
    }

    current = Data(1, _SPEED, _ACCELERATION, _TEMPERATURE, 0.12, _BEAT);

    

    if (previous.show_Status() == 1) {
      // Serial.println(_SPEED);
      current.show_Values();

      // Calling Situation Trackers
      if (current.pressure > enviornmentPressure ) {
        timerW = timerW + 1;
        check_NextSTEP(0);

        if (timerW >= 20) {
          track_Situation(1);
        }

        else {
          disaster_In_WATER(current.pressure, previous.pressure, current.speed, previous.speed, current.acceleration, current.beat);
        }
      }

      else {
        timerW = 0; // reset timer in water
        check_NextSTEP(0); // reset next step
        disaster_On_GROUND(current.acceleration, current.speed, previous.speed, current.beat);
      }  
    }

    if (disaster == 0) {
      if (nextStep == 1) {
        disaster_Of_INCIDENTS(current.temperature);
      }
    }

    previous = current;
    delay(time);
}


/*--------------Disaster Tracking Algorithems---------------*/

// Traking abnormal situations in water
// https://pressbooks-dev.oer.hawaii.edu/collegephysics/chapter/11-4-variation-of-pressure-with-depth-in-a-fluid/#:~:text=We%20begin%20by%20solving%20the,%CF%81%20g%20for%20depthh%3A&text=h%3D%20P%CF%81g.,water%20that%20creates%20the%20pressure.
void disaster_In_WATER(float p, float q, float v, float u, float a, float b) {
  if(p > q) {
    if (a > 0) {
      if (b > fear_HeartRATE(b)) {
        track_Situation(1);
      }

      else {
        track_Situation(0);
      }
    }

    else {
      track_Situation(0);
    }
  }

  else if(p == q) {
    if(v == 0) {
      if (v == u) {
        track_Situation(1);
      }

      else {
        track_Situation(0);
      }
    }

    else {
      if(a > 0) {
        if(b > fear_HeartRATE(b)) {
          track_Situation(1);
        }

        else {
          track_Situation(0);
        }
      }

      else {
        if(v == 0) {
          if(b > fear_HeartRATE(b)) {
            track_Situation(1);
          }

          else {
            track_Situation(0);
          }
        }

        else {
          track_Situation(0);
        }
      }
    }
  }
}

// Traking abnormal situations on ground
void disaster_On_GROUND(float a, float v, float u, float b) {
  //Stage - 01
  if(a > 9.81) {
    if(v >= max_Velocity(weight, a, length)) {
      track_Situation(1);
    }

    else {
      track_Situation(0);
    }

    reset_Timer();
  }

  //Stage - 02
  else if(a < 9.81 && a > 5.00) {
    if(v >= max_Velocity(weight, a, length)) {
      track_Situation(1);
    }

    else {
      track_Situation(0);
    }

    reset_Timer();
  }

  //Stage - 03
  else if(a <= 5.00 && a > 3.00) {
    if(v >= max_Velocity(weight, a, length)) {
      track_Situation(1);
      reset_Timer();
    }

    else {
      if(b > fear_HeartRATE(b)) { 
        track_Situation(1);
        reset_Timer();
      }

      else {
        track_Situation(0);
        check_NextSTEP(1);
      }
    }
  }

  //Stage - 04
  else if(a <= 3 && a > 0) {
    if(b > fear_HeartRATE(b)) {
      track_Situation(1);
      reset_Timer();
    }

    else{
      track_Situation(0);
      check_NextSTEP(1);
    }
  }

  //Stage - 05
  else if(a >= 0 && a < 1) {
    track_Situation(0);
    check_NextSTEP(1);
  }

  //Stage - 06
  else if(a < 0) {
    if(v == 0) {
      if(u >= max_Velocity(weight, a, length)) {
        track_Situation(1);
      }

      else {
        track_Situation(0);
      }
    }

    else {
      track_Situation(0);
    }

    reset_Timer();
  }
}

// Check other enviornmental datasets
void disaster_Of_INCIDENTS(float temp) {
  if(temp > 45) { // https://www.livescience.com/hottest-temperature-people-can-tolerate.html
    timerG = timerG + 1;

    if(timerG > 8) {
      track_Situation(1);
    }

    else {
      track_Situation(0);
    }
  }

  else {
    track_Situation(0);
  }
}


/*--------------Calculation Functions---------------*/

// Calculating current acceleration
float get_Acceleration(float c_Speed, float p_Speed) {
  return (c_Speed - p_Speed)*1000/time;
}

// Situation Informer
void track_Situation(int status)  {
  disaster = status;
}

// Check another Step
void check_NextSTEP(int handler) {
  nextStep = handler;
}

// Reset time Counter
void reset_Timer() {
  timerG = 0;
}

// Function to read accelerometer data for a specified axis (0 = X, 1 = Y, 2 = Z)
int readAccelerometer(int axis) {
  int address = 0x32 + axis * 2; // Starting register address for the specified axis
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(address);
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345_ADDRESS, 2);
  int accel = Wire.read() | Wire.read() << 8; // Combine two bytes into a 16-bit integer
  return accel;
}

// Function to calibrate the accelerometer
void calibrateAccelerometer() {
  const int numSamples = 1000; // Number of samples to use for calibration
  float xSum = 0.0, ySum = 0.0, zSum = 0.0;

  Serial.println("Calibrating accelerometer...");
  
  // Take a series of readings to calculate the bias for each axis
  for (int i = 0; i < numSamples; i++) {
    xSum += readAccelerometer(0);
    ySum += readAccelerometer(1);
    zSum += readAccelerometer(2);
    delay(0);
  }

  // Calculate the biases
  xBias = xSum / numSamples;
  yBias = ySum / numSamples;
  zBias = zSum / numSamples;
}

int fear_HeartRATE(int beat) {
  int fear = sqrt((beat * beat) * 1.3); // https://www.frontiersin.org/articles/10.3389/fnins.2019.01131/full
  return fear;
}

float max_Velocity(float weight, float acceleration, float length) { // https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6043578/
  float Vmax = acceleration/length + (weight * acceleration / pow(length , 2));
  return Vmax;
}