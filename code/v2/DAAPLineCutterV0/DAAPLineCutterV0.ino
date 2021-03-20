//Define the states
enum State {
  preliminary = 1,
  tube = 2,
  ejected = 3,
  disreefed1 = 4,
  disreefed2 = 5
};

State state = preliminary; //State variable

//nichrome pins
const int nichrome1 = 5;
const int nichrome2 = 9;

//buzzer pin
const int buzzer = 8;

//Sensor pins
const int voltage = A1;
const int light = A0;

const int lightThreashold = 450; //Anything above this value is considered to be outside of the tube, anything below it is inside the tube
unsigned long lastLightTime = 0; //Last time that it was light
unsigned long lastDarkTime = 0; //Last time it was dark
const int darkTriggerTime = 20000; //The continuous time interval for which it much be constantly dark in order for the board to decide it is in the tube.
const int lightTriggerTime = 2000; //The continuous time interval for which it must be constantly light in order for the board to decide it has been ejected.

//Keeping track of the beeps
unsigned long lastBeepTime = 0;
const int beepDelay = 5000;

const int disreef1Time = 2000; //Delay after ejection is detected before the first line is cut.
const int disreef2Time = 3000; //Delay after the first line is cut before the second line is cut.
const int cutDuration = 3000; //Time the nichrome turns on for.
const int nominalPower = 75; //desired pwm value (out of 255) when the battery is at 3.7V

unsigned long lastStateChangeTime = 0;

void cut(int pin);


void setup() {
  Serial.begin(9600);
  
  pinMode(nichrome1, OUTPUT);
  digitalWrite(nichrome1, 0);
  
  pinMode(nichrome1, OUTPUT);
  digitalWrite(nichrome1, 0);

  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, 1);
  delay(500);
  digitalWrite(buzzer, 0);

  // Start the timers
  lastLightTime = millis();
  lastBeepTime = millis();
  lastStateChangeTime = millis();

  
}

void loop() {
  //Read Sensors
  int l = analogRead(light);
  float v = analogRead(voltage)*0.003247*2;

  //Print info
  Serial.print("state: ");
  Serial.print(state);
  Serial.print("\tlight: ");
  Serial.print(l);
  Serial.print("\tv = ");
  Serial.println(v);
  

  //Keep track of whether it is dark or light and for how long
  if(l > lightThreashold)
    lastLightTime = millis();

  if(l < lightThreashold)
    lastDarkTime = millis();


  //Do the beeping
  if(millis() - lastBeepTime > beepDelay){
    for(int i = 0; i < state; i++){
      digitalWrite(buzzer,1);
      delay(100);
      digitalWrite(buzzer,0);
      delay(50);
    }
    lastBeepTime = millis();
  }

  //Perform State-specific stuff.
  switch(state){
  
    case preliminary:
      if(millis() - lastLightTime > darkTriggerTime){
        state = tube;
        lastStateChangeTime = millis();
      }
    break;

    case tube:
      if(millis() - lastDarkTime > lightTriggerTime){
        state = ejected;
        lastStateChangeTime = millis();
      }
    break;

    case ejected:
      if(millis() - (lastStateChangeTime) > disreef1Time){
        cut(nichrome1, v);
        lastStateChangeTime = millis();
        state = disreefed1;
      }
    break;

    case disreefed1:
      if(millis() - lastStateChangeTime > disreef2Time){
        cut(nichrome2, v);
        lastStateChangeTime = millis();
        state = disreefed2;
      }
    break;
  }
}

void cut(int pin, int v){
  analogWrite(pin, (3.7*nominalPower/v));
  delay(cutDuration);
  analogWrite(pin, 0);
}
