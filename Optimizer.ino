#include <Servo.h>
#include <Wire.h>
#include <Time.h>
#include <DS1307RTC.h>

byte hpotpin = A0;  // analog pin used to connect the horizontal potentiometer
byte vpotpin = A2;  // analog pin used to connect the vertical potentiometer


byte redpin = 12;
byte greenpin = 11;
byte bluepin = 10;

byte inAutomationpin = 9; //INPUT pin that detects whether automation button is ON or OFF
byte outAutomationpin = 4;

byte hval;    // variable to read the value from the analog pin (horizontal potentiometer)
byte vval;    // variable to read the value from the analog pin (vertical potentiometer)
byte sunrisehour[12][31] = {{32,32,32,32,32,32,32,32,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,30,30,30
},{30,30,30,30,30,30,30,30,30,29,29,29,29,29,29,29,29,29,28,28,28,28,28,28,28,28,28,27
},{27,27,27,27,27,27,27,26,26,26,26,26,26,26,25,25,25,25,25,25,25,25,24,24,24,24,24,24,24,24,23
},{23,23,23,23,23,23,23,22,22,22,22,22,22,22,21,21,21,21,21,21,21,21,21,20,20,20,20,20,20,20
},{20,20,19,19,19,19,19,19,19,19,19,19,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,17,17,17
},{17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17
},{17,17,17,17,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,19,19,19,19,19,19,19,19,19,19
},{19,19,19,20,20,20,20,20,20,20,20,20,20,20,20,21,21,21,21,21,21,21,21,21,21,21,21,22,22,22,22
},{22,22,22,22,22,22,22,23,23,23,23,23,23,23,23,23,23,23,23,24,24,24,24,24,24,24,24,24,24,24
},{24,25,25,25,25,25,25,25,25,25,25,25,26,26,26,26,26,26,26,26,26,26,26,27,27,27,27,27,27,27,27
},{27,27,28,28,28,28,28,28,28,28,28,28,28,29,29,29,29,29,29,29,29,29,29,30,30,30,30,30,30,30
},{30,30,30,30,30,30,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,32,32,32,32,32
}};
byte sunhour[12][31] = {{37,37,37,37,37,37,37,37,37,37,37,38,38,38,38,38,38,38,38,39,39,39,39,39,39,40,40,40,40,40,40
},{41,41,41,41,41,42,42,42,42,42,43,43,43,43,43,44,44,44,44,44,45,45,45,45,45,46,46,46
},{46,47,47,47,47,48,48,48,48,48,49,49,49,49,50,50,50,50,50,51,51,51,51,52,52,52,52,52,53,53,53
},{53,53,54,54,54,54,55,55,55,55,55,56,56,56,56,57,57,57,57,57,58,58,58,58,58,59,59,59,59,59
},{60,60,60,60,60,60,61,61,61,61,61,62,62,62,62,62,62,63,63,63,63,63,63,63,64,64,64,64,64,64,64
},{64,64,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65
},{65,65,65,65,65,65,65,65,65,64,64,64,64,64,64,64,64,64,63,63,63,63,63,63,63,62,62,62,62,62,62
},{61,61,61,61,61,61,60,60,60,60,60,59,59,59,59,59,58,58,58,58,58,57,57,57,57,57,56,56,56,56,56
},{55,55,55,55,55,54,54,54,54,53,53,53,53,53,52,52,52,52,51,51,51,51,51,50,50,50,50,50,49,49
},{49,49,48,48,48,48,48,47,47,47,47,46,46,46,46,46,45,45,45,45,45,44,44,44,44,43,43,43,43,43,42
},{42,42,42,42,41,41,41,41,41,41,40,40,40,40,40,40,39,39,39,39,39,39,38,38,38,38,38,38,38,38
},{37,37,37,37,37,37,37,37,37,37,37,37,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,37
}};

byte elevation[13][24] = {{0,0,0,0,0,0,0,0,5,14,20,25,26,25,20,14,5,0,0,0,0,0,0,0
},{0,0,0,0,0,0,0,0,8,17,24,29,31,30,26,19,11,1,0,0,0,0,0,0
},{0,0,0,0,0,0,0,4,14,24,32,38,41,39,34,27,18,7,0,0,0,0,0,0
},{0,0,0,0,0,0,1,12,24,34,43,49,52,49,43,34,24,13,2,0,0,0,0,0
},{0,0,0,0,0,0,9,21,32,43,53,60,62,59,51,41,30,19,7,0,0,0,0,0
},{0,0,0,0,0,4,14,26,37,48,59,67,70,65,56,46,34,23,12,1,0,0,0,0
},{0,0,0,0,0,5,16,27,38,49,60,69,73,68,59,48,37,26,15,4,0,0,0,0
},{0,0,0,0,0,3,13,24,36,47,58,66,70,67,58,48,36,25,14,3,0,0,0,0
},{0,0,0,0,0,0,9,20,32,43,53,60,64,61,53,43,32,21,9,0,0,0,0,0
},{0,0,0,0,0,0,4,16,27,37,46,52,54,51,44,35,24,13,1,0,0,0,0,0
},{0,0,0,0,0,0,0,10,21,30,37,42,43,40,33,25,15,4,0,0,0,0,0,0
},{0,0,0,0,0,0,0,4,14,22,29,32,33,30,25,17,7,0,0,0,0,0,0,0
},{0,0,0,0,0,0,0,0,8,16,22,26,27,25,20,13,4,0,0,0,0,0,0,0
}};



//elevation values will be updated weekly 

Servo ServoHorizontal; // Object
Servo ServoVertical; // Object

void setup() {
  ServoVertical.attach(6, 500, 2500);  // attaches the vertical servo to pin 6 (pulses this servo reacts to)
  ServoHorizontal.attach(5, 500, 2500); // attaches the horizontal servo to pin 5
  pinMode(redpin, OUTPUT);  //use small resistor on cathode 
  pinMode(bluepin, OUTPUT);  //use small resistor on cathode 
  pinMode(greenpin, OUTPUT);  //use small resistor on cathode !!
  pinMode(inAutomationpin, INPUT);
  pinMode(outAutomationpin, OUTPUT);
  Serial.begin(9600);
  while (!Serial){ ; // wait until Arduino Serial Monitor opens
    setSyncProvider(RTC.get);   // the function to get the time from the RTC
    if(timeStatus()!= timeSet){
       Serial.println("Unable to sync with the RTC");
    }else{
       Serial.println("RTC has set the system time");
    } 
  }
  

}

void loop() {
  tmElements_t tm;
  if(RTC.read(tm)){
      byte mins = tm.Minute;               // between 0 and 59
      byte hr = tm.Hour;                   // between 0 and 23
      byte jour = tm.Day;                     // between 1 and 31
      byte mois = tm.Month;              // between 1 and 12
      double percentage = (hr/24.0+mins/1440.0)*100.0;      // percentage of the day that has passed
      byte todaysun = sunhour[mois-1][jour-1];    // hours of sun for today
      byte todaysunrise = sunrisehour[mois-1][jour-1]; //hour at which the sun comes up
      digitalWrite(outAutomationpin, HIGH);
      if(digitalRead(inAutomationpin) == LOW){                         //MANUAL
        digitalWrite(bluepin, HIGH);
        digitalWrite(greenpin, LOW);
        digitalWrite(redpin, LOW);
        hval = analogRead(hpotpin);            // reads the value of the potentiometer (value between 0 and 1023)
        vval = analogRead(vpotpin);
        hval = map(hval, 0, 511, 0, 179);      // scale it to use it with the servo (value between 0 and 180)
        vval = map(vval, 0, 511, 0 , 179);  
                               
        ServoHorizontal.write(hval);          // sets the servo position according to the scaled value
        delay(200);                            // waits for the servo to get there
        ServoVertical.write(vval);
        delay(200);                            // waits for the servo to get there     
     }else{                                    //AUTOMATION
      if(percentage > todaysunrise && percentage < (todaysunrise+todaysun)){
          digitalWrite(greenpin, HIGH);
          digitalWrite(redpin, LOW);
          digitalWrite(bluepin, LOW);
          
          hval = map(percentage-todaysunrise,0, todaysun, 0, 180);
          ServoHorizontal.write(180-hval);
          delay(30);
          
          int roundhour = (int) (percentage*24)/100; //elevation will only update every hour (so 10.87 hours will be 10 hours, circumventing minute conversion) from 0 to 23
          int roundweek = (int) (mois*4+jour/7)/4;  //from 0 to 52, changes every four weeks due to memory limitations
         
          int nowelevation = elevation[roundweek][roundhour];
          vval = map(nowelevation, 0, 180, 0, 180);        
          ServoVertical.write(120-vval); //135 degrees being 0 degrees of elevation on this servo (after testing)
      }else{
          digitalWrite(redpin, HIGH);
          digitalWrite(greenpin, LOW);
          digitalWrite(bluepin, LOW);
      }
    }
  }
}

