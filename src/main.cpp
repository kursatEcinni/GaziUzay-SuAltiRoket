#include <Arduino.h>
#include <TaskScheduler.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

int depthPin = 4; // Analog pin for depth sensor (potentiometer)
int initButton = 10; // Digital pin for initialization button

int ThtrusterPin = 11; // Digital pin for thruster control (PWM)
int finServoPin1 = 12; // Digital pin for fin servo control (PWM)
int finServoPin2 = 13; // Digital pin for second fin servo control (PWM)
int finServoPin3 = 14; // Digital pin for third fin servo control (PWM)
int finServoPin4 = 15; // Digital pin for fourth fin servo control (PWM)

bool  isInitialized = false;
float pitch, roll, yaw;
float depth;

float cruiseSpeed = 0.0f;
float lastDistance = 0.0;
int   missionState = 0;

void readIMU();
void readDepth();

void filterSensors();

void calcEstDistance();

void armThrusters();
void disarmThrusters();

void updateState();
void calculatePID();

void updateActuators();

void checkHealth();

void safeZoneProtocol();

Scheduler scheduler;

Task taskReadIMU(20, TASK_FOREVER, &readIMU);
Task taskReadDepth(50, TASK_FOREVER, &readDepth);
Task taskUpdateDistance(100, TASK_FOREVER, &calcEstDistance);
Task taskUpdateState(20, TASK_FOREVER, &updateState);
Task taskArmThrusters(100, TASK_ONCE, &armThrusters);
//Task taskFilter(20, TASK_FOREVER, &filterSensors);
//Task taskControl(20, TASK_FOREVER, &processControl);
Task taskUpdateActuators(20, TASK_FOREVER, &updateActuators);
//Task taskHealthCheck(200, TASK_FOREVER, &checkHealth);

////////////////SET UP////////////////
void setup() {
  scheduler.init();

  scheduler.addTask(taskReadIMU);
  scheduler.addTask(taskReadDepth);
  scheduler.addTask(taskUpdateDistance);
  scheduler.addTask(taskArmThrusters);//ONCE
  scheduler.addTask(taskUpdateState);
  //scheduler.addTask(taskFilter);
  //scheduler.addTask(taskControl);
  //scheduler.addTask(taskUpdateAct);
  //scheduler.addTask(taskHealthCheck);

  taskReadIMU.enable();
  taskReadDepth.enable();
  taskUpdateState.enable();
  //taskFilter.enable();
  //taskControl.enable();
  //taskUpdateAct.enable();
  //taskHealthCheck.enable();

  bno.begin();
  bno.setExtCrystalUse(true);

  pinMode(depthPin, INPUT);
  pinMode(initButton, INPUT_PULLDOWN);

  Serial.begin(115200);
}

////////////////LOOP////////////////
void loop() {
  if (isInitialized == false) {
    if (digitalRead(initButton) == HIGH) {
      isInitialized = true;
      Serial.println("System Initialized");
    }
  }

  scheduler.execute();
}



void readIMU(){
  sensors_event_t event; 
  bno.getEvent(&event);

  yaw   = event.orientation.x; // 0 - 360 derece
  roll  = event.orientation.y; // -90 / +90
  pitch = event.orientation.z; // -180 / +180
  Serial.print("Pitch: "); Serial.print(pitch);
  Serial.print(" Roll: "); Serial.print(roll);
  Serial.print(" Yaw: "); Serial.println(yaw);
}
void readDepth(){
  // Placeholder for depth reading code
  // After deciding on the depth sensor, implement the reading logic here
  // Simulate depth reading
  depth = (analogRead(depthPin) / 4095.0f) * 10.0f;
  depth = map(depth, 0, 1023, 0, 10); // Replace with actual depth reading logic
  Serial.print("Depth: "); Serial.println(depth);
}
void calcEstDistance(){
  float currentDistance = lastDistance; // Placeholder for current distance calculation
  // Placeholder for distance calculation code
  // Implement the logic to calculate distance based on sensor readings
  // Simulate distance calculation

  //ADD TURNING BACK LOGIC AND MINUS DISTANCE WHEN TURNING BACK !!!!!!!!!!!
  cruiseSpeed = 1.5f; // Replace with actual distance calculation logic
  lastDistance = currentDistance + cruiseSpeed * 0.1f; // Update last distance based on cruise speed and time interval
  Serial.print("Cruise Speed: "); Serial.println(cruiseSpeed);
  Serial.print("Estimated Distance: "); Serial.println(lastDistance);
}
void updateState(){
  if(missionState == 0 && isInitialized){
    taskArmThrusters.enable();
    taskUpdateDistance.enable();
    missionState = 1;
    Serial.println("State: 1");
  }
  else if(missionState == 1 && lastDistance >= 10.0f){
    missionState = 2;
    Serial.println("State: 2");
  }
  else if(missionState == 2 && lastDistance >= 60.0f){
    missionState = 3;
    Serial.println("State: 3");
  }
  else if(missionState == 3 && yaw >= 180.0f){
    // make sure it is a 180 degree turn, you can add a tolerance if needed
    missionState = 4;
    Serial.println("State: 4");
  }
  else if(missionState == 4 && lastDistance <= 10){
    missionState = 5;
    Serial.println("State: 5");
  }
  else if(missionState == 5){
    disarmThrusters();
    safeZoneProtocol();
    Serial.println("Mission Completed");
  }
}

void armThrusters(){
  // Placeholder for thrusters arming code
  Serial.println("Thrusters armed");
  delay(100);
}
void disarmThrusters(){
  // Placeholder for thruster disarming code
  Serial.println("Thrusters disarmed");
  delay(100);
}

void safeZoneProtocol(){
  // Placeholder for safe zone protocol code
  Serial.println("Safe zone protocol activated");

  taskReadIMU.disable();
  taskReadDepth.disable();
  taskUpdateDistance.disable();
  //taskFilter.enable();
  //taskControl.enable();
  //taskUpdateAct.enable();
  //taskHealthCheck.enable();

  delay(100);
}

void updateActuators(){
  if(missionState == 1){
    // Placeholder for actuator update code
    // Implement the logic to update actuators based on control signals
    Serial.println("Actuators just armed. Waiting for hitting 10meter distance...");
  }
  else if(missionState == 2){
    // Placeholder for actuator update code
    // Implement the logic to update actuators based on control signals
    Serial.println("Thrusters are running full speed. Waiting for hitting 60meter(total) distance...");
  }
  else if(missionState == 3){
    // Placeholder for actuator update code
    // Implement the logic to update actuators based on control signals
    Serial.println("Thrusters slowed down. 180 degree turn is in progress.");
  }
  else if(missionState == 4){
    // Placeholder for actuator update code
    // Implement the logic to update actuators based on control signals
    Serial.println("Thrusters are running full speed. Waiting for hitting safe zone ... ");
  }
  else if(missionState == 5){
    // Placeholder for actuator update code
    // Implement the logic to update actuators based on control signals
    Serial.println("Thrusters are stopped. Mission completed.");
  }
}

//void calculatePID(){delay(10);}

//void filterSensors(){delay(10);}

//void checkHealth(){delay(10);}
