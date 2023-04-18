#define USE_ARDUINO_INTERRUPTS true 
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PulseSensorPlayground.h>

//Declare the Temperature Sensor
#define ONE_WIRE_BUS 2   
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Declare the Pulse Sensor
const int PulseWire = 0; 
const int LED13 = 13;
int Threshold = 550; 
PulseSensorPlayground pulseSensor; 

// Initialie wanted variables
int time = 1000;
int disaster = 0;
int nextStep = 0;

// User inputs | Keyboard inputs
float weight = 75;
float enviornmentPressure = 10;
int heartBeat = 234;

// Calculations for user Input
float Vmax = weight*2;

// Timer is counting in the Water & Ground
int timerW = 0;
int timerG = 0;

class Data {
    public:
        int status;
        float speed;
        float temperature;
        float pressure;
        int beat;

    public:
        Data(int st, float sp, float tm, float pr, int bt) {
            status = st;
            speed = sp;                                                                                                                             
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
Data current = Data(0, 0, 0, 0, 0);
Data previous = Data(0, 0, 0, 0, 0);

void setup() {
    Serial.begin(9600);
    sensors.begin(); 
    
    pulseSensor.analogInput(PulseWire);
    pulseSensor.blinkOnPulse(LED13);
    pulseSensor.setThreshold(Threshold);
 
    if (pulseSensor.begin()) {
      Serial.println("Measuring user's Heart Beat"); 
    }
}

void loop() {
    if (disaster == 1) {
        // NEXT STEP | Connect
    }

    sensors.requestTemperatures(); 

    float _TEMPERATURE = sensors.getTempCByIndex(0); // Enviornment temperature
    int _BEAT = pulseSensor.getBeatsPerMinute(); // User's Heart Beat

    current = Data(1, 3, _TEMPERATURE, 9, _BEAT);

    if (previous.show_Status() == 1) {
      // current.show_Values();
      float acceleration = get_Acceleration(current.speed, previous.speed);

      // Calling Situation Trackers
      if (current.pressure > enviornmentPressure ) {
        timerW = timerW + 1;
        check_NextSTEP(0);

        if (timerW >= 20) {
          track_Situation(1);
        }

        else {
          disaster_In_WATER(current.pressure, previous.pressure, current.speed, previous.speed, acceleration, current.beat);
        }
      }

      else {
        timerW = 0; // reset timer in water
        check_NextSTEP(0); // reset next step
        disaster_On_GROUND(acceleration, current.speed, previous.speed, current.beat);
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

// Traking abnormal situations in water
void disaster_In_WATER(p, q, v, u, a, b) {
  if(p > q) {
    if (a > 0) {
      if (b > heartBeat) {
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
        if(b > heartBeat) {
          track_Situation(1);
        }

        else {
          track_Situation(0);
        }
      }

      else {
        if(v == 0) {
          if(b > heartBeat) {
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
    if(v >= Vmax) {
      track_Situation(1);
    }

    else {
      track_Situation(0);
    }
  }

  //Stage - 02
  else if(a < 9.81 && a > 5.00) {
    if(v >= Vmax) {
      track_Situation(1);
    }

    else {
      track_Situation(0);
    }
  }

  //Stage - 03
  else if(a <= 5.00 && a > 3.00) {
    if(v >= Vmax) {
      track_Situation(1);
    }

    else {
      if(b > heartBeat) {
        track_Situation(1);
      }

      else {
        track_Situation(0);
        check_NextSTEP(1);
      }
    }
  }

  //Stage - 04
  else if(a <= 3 && a > 0) {
    if(b > heartBeat) {
      track_Situation(1);
    }

    else{
      track_Situation(0);
      check_NextSTEP(1);
    }
  }

  //Stage - 05
  else if(a == 0) {
    track_Situation(0);
    check_NextSTEP(1);
  }

  //Stage - 06
  else if(a < 0) {
    if(v == 0) {
      if(u >= Vmax) {
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

// Check other enviornmental datasets
void disaster_Of_INCIDENTS(float temp) {
  if(temp > 40) {
    timerG = timerG + 1;

    if(timerG > 10) {
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