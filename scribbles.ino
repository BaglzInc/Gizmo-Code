//decleration for accelerometer_____________________________________________________

#include <Adafruit_MPU6050.h> //library for accelerometer
#include <Adafruit_Sensor.h> //library for accelerometer
#include <Wire.h> //library for accelerometer
unsigned long first_check = 0; // monitors how long the monkey has been resting
unsigned long latest_check = 0; // monitors how long the monkey has been resting
bool setdown = false; // monitors how long the monkey has been resting

Adafruit_MPU6050 mpu; // initialize the accelerometer

#include <Stepper.h>
const int dirPin = A3;
const int stepPin = A2;
#define motorInterfaceType 1

//decleration for motors_____________________________________________________________
Stepper stepper(200,8,9,10,11); //intialize first stepper
Stepper stepper_2(200,6,7,A2,A3); //initialize second stepper

#define AIN1 4 // pins for dc motor
#define AIN2 5 // pins for dc motor
#define BIN1 7 // pins for dc motor
#define BIN2 8 // pins for dc motor

//variable decleration for logging data phase________________________________________
int pushsensor = A0; // push sensor pin
bool flipflop = false; // boolean to check if the sensor is being pushed
bool first_schlap = false; // variable to initialize logging data
bool last_schlap = false; // variable to end logging data

String datastream = ""; // global variable to log data
unsigned long runner = 0; // variables to detect time between each hit
unsigned long chaser = 0; // variables to detect time between each hit
byte current_highest_power = 0; // variable to log part hardest hit
bool killer = false; // ends the drumming sequence

//variable decleration for button to switch between both user interaction phases_____
int pushbutton = 3; // button to activate drumming sequence
bool button_touched = false; // boolean to determine wether button is still pushed

void setup() {

  Serial.begin(9600); // establish communication with arduino
  
  pinMode(pushbutton, INPUT); // initialize button pin

  // initialize dc motor pins________________________________________________________
  pinMode(AIN1,OUTPUT); 
  pinMode(AIN2,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);
  
  //initialize accelerometer_________________________________________________________
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

}

//Motor functions____________________________________________________________________
//function to activate first drumstick - takes in force of hit as its only argument

int motor_1 (int x) {
  float scalar = 50.0/256.0; // scales hit from sensor data to stepper speed
  Serial.println((x * scalar) + 50); // prints force to serial monitor
  stepper.setSpeed( (x * scalar) + 50); // inputs scaled speed value to stepper
  stepper.step(25); // rotates the stepper an 1/8th of a turn (45 deg)
  stepper.step(-25); // rotates the stepper an 1/8th of a turn (45 deg) opposite way
}

//function to activate second drumstick - takes in force of hit as its only argument
// same code as the previous function but with a diffrent motor
int motor_2 (int x) {
  float scalar = 50.0/256.0;
  Serial.println((x * scalar) + 50);
  stepper_2.setSpeed( (x * scalar) + 50);
  stepper_2.step(25);
  stepper_2.step(-25);
}

// variable decleration for converter________________________________________________
String timing = ""; //initialize string that will contain only time values
String power = ""; // initialize string that will contain only power values
int total_claps = 0; // registers total number of claps 
bool converted = false; // boolean that registers wether or not data has been converted for drumming sequence

// Converter function________________________________________________________________
//takes string of data as only argument and appends it to the respective force and time strings

  int converter(String data) {
  String current_datavalue = ""; // variable that will take on the current data while parsing the loop
  int len = data.length(); // calauates length of total data
  Serial.print("length: ");
  Serial.println(len);
  for (int i = 0; i < len; i++) {
    char current_datapoint = data[i]; // variable that takes on the specific character of the datastream
    if (current_datapoint != '/' and current_datapoint != ':') { // enters the if statment if the character isn't a seperator
      current_datavalue += current_datapoint; //add character to current value
    }
    else {
      if (current_datapoint == '/') { //if the seperator is a / add to time storage
        timing += current_datavalue + '/'; 
        current_datavalue = ""; // reset current value
      }
      else {
        power += current_datavalue + '/'; //if the seperator is a : add to power storage
        total_claps += 1;
        current_datavalue = ""; // reset current value
      }
    }
  }
  // print values
  Serial.print("times: ");
  Serial.println(timing);
  Serial.print("force: ");
  Serial.println(power);
  Serial.print("number of claps: ");
  Serial.println(total_claps);
}

// Drumming function________________________________________________________________
// takes 3 arguments: first the total number of claps in the stream, then a string containing only power values
// seperated by /, lastly a string containing only time values seperated by /. It takes these inputs and plays
// the rythmn out on the drum, alternating between each stick for each hit and delaying between each one while
// accounting for the time of each swing.

int drumming (int claps, String powers, String times){
  int time_runner = 0; // initialize variable that will iterate through time loop
  int power_runner = 0; // initialize variable that will iterate through power loop
  String current_power = ""; // initialize variable that will take on the current power value in the data
  String current_time = ""; // intialize varaiable that will take on current time value in data
  int motor_choice = 1; // variable switches between positve and negative one to indicate motor choice
  for (int i = 0; i < claps; i++) {
    while (powers[power_runner] != '/') { // while statment iterates through power character string and seperates out each individual value
      current_power += powers[power_runner];
      power_runner += 1; 
    }
    Serial.print("current power: ");
    int swingforce = current_power.toInt(); // changes power from string to integer 
    Serial.println(swingforce);

    unsigned long start = millis(); // registers time stamp of beginning of swing

    // if statments used to delegate swinging task to one of the two motors
    if (motor_choice == 1) { 
      motor_1(swingforce);
    }

    if (motor_choice == -1) {
      motor_2(swingforce);
    }
    
    while (times[time_runner] != '/') { // while statment iterates through time character string and seperates out each individual value
      current_time += times[time_runner];
      time_runner += 1; 
    }
    unsigned long finish = millis(); // registers time stamp at end of swing
    int nexthit = current_time.toInt(); // converts time from string to int
    int accountedtime = (nexthit - (finish-start)); // calculates delay adjusted for the time of swing
    if (accountedtime > 0 ) { // if statments delays if the adjusted delay is bigger than 0
      delay(accountedtime);
    }
    Serial.print("delay: ");
    Serial.println(current_time);

    // reset variables for next iteration through loop
    current_power = "";
    current_time = "";
    time_runner += 1;
    power_runner += 1;

    //change swinging arm
    motor_choice = motor_choice * -1;

    // check accelerometer to see if the monkey is still dancing
    dancing();
    if (killer == true) {// if it has been inactive for too long hop out of the loop
      break;
    }
  }
}

// Dancing function________________________________________________________________
// Dancing takes no arguments and simply checks on wether or not the accelerometer is moving.
// If the monkey is inactive for more than 2 seconds (measured by checking if magnitude of speed
// vector falls below 0.5rad/s) then the function changes the sequence killer boolean to true which
// in turn stops all the mechanisms and drumming sequence and refreshes all the global variables 
// for the next pass in the loop.

void dancing () {

  // get information from accelerometer
  sensors_event_t a, g, temp; 
  mpu.getEvent(&a, &g, &temp);
  
  float mag = sqrt(sq(g.gyro.x) + sq(g.gyro.y) + sq(g.gyro.z)); // calculates the magnitude of the accelerometer's speed vector
  if (mag < 0.5) { //if the magnitude falls below a threshold (0.5 rad/s)

     // change inactivity boolean to true and register time stamp of first inactivity
     if (setdown == false) { 
      first_check = millis();
      setdown = true; 
    }

    // periodically refresh the latest instance in which the monkey is inactive
    latest_check = millis();
  }
  Serial.println(latest_check - first_check);

  // if the monkey has been inactive for over 2 seconds (latest check - first check) turn the sequence killing boolean to true
  if (setdown == true && (latest_check - first_check) > 2000) {
    Serial.println("kill me now");
    killer = true;
 }

  if (mag > 0.5) { // if the monkey becomes active again (speed vector exceeds threshold of 0.5 rad/s) set the inactivity boolean to false
    setdown = false;
  }
}

// Void loop________________________________________________________________
// The void loop interacts with the user and executes all the functions we want our systems to perform.
// It's split into two distinct phases, the first is the collection phase where the system monitors the 
// users inputs to the pressue sensor and the button to replicate the beat. The second phase is the playback
// phase when the system repeats the beat on the drum along with making the jungle come alive to dance along 
// with you. Lastly it monitors the accelerometer embedded within the monkey and dies when you are no longer
// dancing with it. The code is also written so that once the monkey is set to rest, everything refreshes so
// you can create another beat without needing to reset the arduino.


void loop() {

  // Collection phase______________________________________________________
  // read pressure sensor
  byte buttonstate = analogRead(pushsensor);

  // if the sensor detects a force above a certain threshold (done to avoid ghost taps since there's felt sown above sensor which applies constant pressure)
  if (buttonstate >= 30) {
    flipflop = true; // changes sensor pressed boolean to true
    Serial.println(buttonstate);

    // registers the hardest impact throughout the push sensor being engaged
    if (current_highest_power < buttonstate) {
      current_highest_power = buttonstate;
      }
    }

  // if the sensor is now no longer being pressed (falls below a certain threshold) and has been pressed (flipflop = true), the button has not yet been pressed
  // (last_schlap = false), and the function killing variable remains false (killer = false).
  if (buttonstate <= 30 and flipflop == true and last_schlap == false and killer == false){
    
    flipflop = false; //turn sensor pressed boolean to false
    runner = millis(); //register time of current hit

    // if the time between hits is above a certain threshold (100 miliseconds) register the hit (implemented to reduce number of ghost taps)
    if (runner - chaser > 100) {
      String current_hit = String(current_highest_power,10) + ":"; // adds seperator to the current datapoint being logged (power)
      current_highest_power = 0; // reset variable for next iteration through the loop
      String datapoint = String(runner - chaser,10) + "/"  ; // adds seperator to the current datapoint being logged (time)

      // if its the first hit, switch the first hit boolean to true and only log the power of the hit since there is no time since the previous hit
      if (first_schlap == false){
        first_schlap = true;
        }
      // log time since last hit to data stream
      else {
        datastream += datapoint;
      }
      datastream += current_hit; // log power of current hit to data stream
      Serial.print(datastream);
      Serial.println(" - full script");
      chaser = runner; // change the time stamp of the current hit to that of the previous hit
    }
  }

  int button_switch = digitalRead(pushbutton); //read current state of the button
  Serial.println(buttonstate);

  // if the button is pressed change the pressed button boolean to true
  if (button_switch == HIGH) { 
    button_touched = true;
    }

  // if the button has been pressed and is no longer being pressed, change the last hit boolean
  // to true and log delay between last hit and button press
  if (button_switch == LOW and button_touched == true) {
    button_touched = false; // reset variable for next iteration
    last_schlap = true; // set last registered hit boolean to true
    runner = millis(); // register time stamp of current hit
    String datapoint = String(runner - chaser,10) + "/"  ; // add seperator to current datavalue
    datastream += datapoint; // log delay between final hit and button press to datastream
    chaser = runner; // change current hit to previous hit
  }

  // Playback phases______________________________________________________
  // if the last hit has been registered but the sequence killing boolean is not true, 
  //begin the second phase of user interaction
  
  if (last_schlap == true and killer == false) {

  // start mechanism motor
  analogWrite(AIN1,220); 
  digitalWrite(AIN2,LOW);
  analogWrite(BIN1,220); 
  digitalWrite(BIN2,LOW);

  // if the datastream has not yet been converted (first iteration through this sequence) convert the data
  if (converted == false) {
    int conversion = converter(datastream);
    converted = true; //change data conversion boolean to true
  }
  // begin drumming sequence with converted data values
  drumming(total_claps,power,timing);
  }

 // if the circuit killer variable is true, stop the mechanism motor and reset all global variables
 if (killer == true) {
  digitalWrite(AIN1,LOW); 
  digitalWrite(AIN2,LOW);
  digitalWrite(BIN1,LOW); 
  digitalWrite(BIN2,LOW);

  last_schlap = false;
  converted = false;
  first_schlap = false;
  datastream = "";
  killer = false;

  flipflop = false;
  timing = "";
  power = "";
  total_claps = 0;

  first_check = 0;
  latest_check = 0;
  setdown = false;

  runner = 0;
  chaser = 0;
 }
  
}
