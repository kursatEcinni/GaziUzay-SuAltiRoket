// =====================================================================
//  Subrocket controller - WORK IN PROGRESS (not a working mission yet)
//
//  Target: ESP32-S3 (Arduino framework / PlatformIO)
//  Libraries: TaskScheduler, Adafruit BNO055, Adafruit Unified Sensor, ESP32Servo
//
//  STATUS
//    - Syntax-checked on a PC with stub headers only. NOT yet built with the
//      real libraries and NOT yet run on hardware.
//    - Gains, mixer signs, ESC pulse range and axis mapping are starting
//      values and must be verified on the bench (propeller removed).
//
//  KNOWN ISSUES
//    1. updateFins(): the servo write is a placeholder. Servos are not attached,
//       so the fins do not move yet.
//    2. readDepth(): the map() line collapses the value to ~0 (integer math and
//       wrong input range). Depth always reads 0 until it is removed or replaced
//       with the real sensor conversion.
//    3. calcEstDistance(): placeholder with a fixed 1.5 m/s. Distance only grows,
//       so state 5 (return to start zone) can never be reached without forcing it.
//    4. updateState(), state 3 -> 4: uses an absolute heading (yaw >= 180). It must
//       become "heading ramp finished and heading error small", relative to yaw0.
//    5. updateState(), state 5: disarmThrusters() and safeZoneProtocol() run every
//       20 ms, so "Mission Completed" repeats and delay(100) blocks the scheduler.
//       The final actions must run once.
//    6. readIMU() / readDepth() / calcEstDistance() print at 5-50 Hz and flood
//       the serial port. Move them to one slow debug task.
//    7. No failsafe yet: no IMU timeout, depth limit or mission timeout. No
//       check that bno.begin() succeeded.
//    8. safeZoneProtocol() contains delay(100).
//
//  Pitch/roll axis assignment of the BNO055 depends on how the board is mounted.
// =====================================================================

#include <Arduino.h>
#include <TaskScheduler.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ESP32Servo.h>   // generates the 50 Hz ESC pulse on the thruster pin

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

//////////////// PID / FIN CONTROL (NEW) ////////////////
// Generic PID: input = error (target - measured) and dt in seconds, output = correction.
struct PID {
  float kp, ki, kd;
  float iLimit;     // max |integral|
  float outLimit;   // max |output|
  float integral = 0.0f;
  float prevErr  = 0.0f;
  bool  first    = true;

  PID(float p, float i, float d, float il, float ol)
    : kp(p), ki(i), kd(d), iLimit(il), outLimit(ol) {}

  void reset() { integral = 0.0f; prevErr = 0.0f; first = true; }

  float update(float err, float dt) {
    integral += err * dt;
    if (integral >  iLimit) integral =  iLimit;   // anti-windup
    if (integral < -iLimit) integral = -iLimit;

    float d = first ? 0.0f : (err - prevErr) / dt;
    first = false;
    prevErr = err;

    float out = kp * err + ki * integral + kd * d;
    if (out >  outLimit) out =  outLimit;
    if (out < -outLimit) out = -outLimit;
    return out;
  }
};

// Angle helpers
static float wrap180(float a) {            // -> (-180, 180]
  a = fmodf(a + 180.0f, 360.0f);
  if (a < 0) a += 360.0f;
  return a - 180.0f;
}
static float wrap360(float a) {            // -> [0, 360)
  a = fmodf(a, 360.0f);
  if (a < 0) a += 360.0f;
  return a;
}

// Targets / limits
const float DEPTH_START_M    = 2.0f;    // depth target during the first 10 m (state 1). ASSUMPTION: dive to 2 m right away
const float DEPTH_CRUISE_M   = 2.0f;    // depth target for the 50 m run, the turn and the return (states 2, 3, 4)
const float PITCH_LIMIT_DEG  = 20.0f;   // max pitch the depth loop may ask for
const float TURN_RATE_DEG_S  = 30.0f;   // heading setpoint ramp speed in state 3
const float FIN_CENTER_DEG   = 90.0f;   // servo neutral
const float FIN_MAX_DEG      = 30.0f;   // max deflection each way from neutral

// Gains are STARTING VALUES only, tune on the bench / pool (pitch first, then depth, then heading, then roll)
PID pidDepth  (15.0f, 0.5f,  0.0f,  5.0f, PITCH_LIMIT_DEG); // depth error [m]   -> pitch setpoint [deg]
PID pidPitch  (0.04f, 0.005f, 0.01f, 10.0f, 1.0f);          // pitch error [deg] -> pitch command  (-1..1)
PID pidHeading(0.02f, 0.0f,  0.01f, 10.0f, 1.0f);           // yaw error [deg]   -> yaw command    (-1..1)
PID pidRoll   (0.03f, 0.0f,  0.005f, 10.0f, 1.0f);          // roll error [deg]  -> roll command   (-1..1)

float yaw0 = 0.0f;          // outbound heading, captured when the mission starts
bool  yaw0Set = false;
float turnRampDeg = 0.0f;   // 0 -> 180 during state 3
float headingSP = 0.0f;     // absolute heading setpoint [deg]
float pitchSP = 0.0f;       // pitch setpoint from the depth loop [deg]

float pitchCmd = 0.0f, yawCmd = 0.0f, rollCmd = 0.0f;   // normalized -1..1
Servo fins[4];                                           // fin servos (ESP32Servo)
int   finPins[4] = {finServoPin1, finServoPin2, finServoPin3, finServoPin4};
const int FIN_PULSE_MIN_US = 500;    // TODO: check your servo datasheet (many use 1000..2000)
const int FIN_PULSE_MAX_US = 2400;
void initFins();
float finAngle[4] = {90.0f, 90.0f, 90.0f, 90.0f};       // final servo angles [deg]

// X-tail mixer. Columns: {pitch, yaw, roll}. Rows: fin 1..4.
// TODO: verify the signs by moving the fins on the bench; flip a sign if a fin moves the wrong way.
const int MIX[4][3] = {
  {+1, +1, +1},
  {+1, -1, -1},
  {-1, +1, -1},
  {-1, -1, +1},
};
//////////////// END NEW ////////////////

//////////////// THRUSTER (NEW) ////////////////
// One motor driven by an ESC through one PWM pin (servo-style pulse, 50 Hz).
// TODO: set these two for YOUR ESC.
//   bidirectional ESC (e.g. T200 Basic ESC): STOP = 1500, MAX = 1900
//   forward-only ESC:                        STOP = 1000, MAX = 2000
const int      ESC_STOP_US = 1000;    // pulse width = zero thrust (Hobbywing Skywalker, forward-only)
const int      ESC_MAX_US  = 2000;    // pulse width = full forward thrust
const uint32_t ESC_ARM_MS  = 3000;    // hold the stop pulse this long so the ESC arms

const float THRUST_START = 0.40f;     // state 1: first 10 m (soft start)
const float THRUST_FULL  = 1.00f;     // states 2 and 4
const float THRUST_TURN  = 0.30f;     // state 3: slow down for the turn
const float THRUST_SLEW_PER_S = 1.0f; // max thrust change per second (limits current spikes)

Servo thruster;
bool     thrustersArmed  = false;     // true only after the arming time has passed
bool     thrusterArming  = false;     // arming in progress
uint32_t armStartMs      = 0;
float    thrustTarget    = 0.0f;      // wanted thrust 0..1
float    thrustApplied   = 0.0f;      // thrust actually sent to the ESC 0..1

void initThruster();
void writeThruster(float cmd01);
//////////////// END NEW ////////////////

void readIMU();
void readDepth();

void filterSensors();

void calcEstDistance();

void armThrusters();
void disarmThrusters();

void calculatePID();
void updateThrusters();
void updateFins();

void updateState();


//void checkHealth();

void safeZoneProtocol();

Scheduler scheduler;

Task taskReadIMU(20, TASK_FOREVER, &readIMU);
Task taskReadDepth(50, TASK_FOREVER, &readDepth);

Task taskUpdateDistance(100, TASK_FOREVER, &calcEstDistance);
//Task taskFilter(20, TASK_FOREVER, &filterSensors);

Task taskUpdateState(20, TASK_FOREVER, &updateState);
Task taskArmThrusters(100, TASK_ONCE, &armThrusters);
Task taskDisarmThrusters(100, TASK_ONCE, &disarmThrusters);

Task taskCalcPID(20, TASK_FOREVER, &calculatePID);
Task taskUpdateFins(20, TASK_FOREVER, &updateFins);
Task taskUpdateThrusters(20, TASK_FOREVER, &updateThrusters);

//Task taskHealthCheck(200, TASK_FOREVER, &checkHealth);

////////////////SET UP////////////////
void setup() {
  initThruster();   // first thing: the ESC must see a stop pulse from power-up
  initFins();       // fins to neutral (90 deg) from power-up

  scheduler.init();

  scheduler.addTask(taskReadIMU);
  scheduler.addTask(taskReadDepth);
  scheduler.addTask(taskUpdateDistance);
   //scheduler.addTask(taskFilter);

  scheduler.addTask(taskArmThrusters);//ONCE
  scheduler.addTask(taskUpdateState);

  scheduler.addTask(taskCalcPID);
  scheduler.addTask(taskUpdateThrusters);
  scheduler.addTask(taskUpdateFins);
  //scheduler.addTask(taskHealthCheck);

  taskReadIMU.enable();
  taskReadDepth.enable();
  taskUpdateState.enable();
  //taskFilter.enable();
  //taskControl.enable();
  taskCalcPID.enable();
  taskUpdateFins.enable();
  taskUpdateThrusters.enable();
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

//////////////// THRUSTER FUNCTIONS (NEW) ////////////////
void initThruster(){
  ESP32PWM::allocateTimer(0);   // timers are shared with the fin servos later
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  thruster.setPeriodHertz(50);
  thruster.attach(ThtrusterPin, 1000, 2000);   // allowed pulse range
  thruster.writeMicroseconds(ESC_STOP_US);     // stop pulse from the very start
}

// cmd01: 0 (stop) .. 1 (full forward). Always clamped.
void writeThruster(float cmd01){
  if (cmd01 < 0.0f) cmd01 = 0.0f;
  if (cmd01 > 1.0f) cmd01 = 1.0f;
  int us = ESC_STOP_US + (int)(cmd01 * (ESC_MAX_US - ESC_STOP_US));
  thruster.writeMicroseconds(us);
}

// Non-blocking: only starts the arming period (stop pulse).
// updateThrusters() sets thrustersArmed once ESC_ARM_MS has passed.
void armThrusters(){
  thrustersArmed = false;
  thrusterArming = true;
  armStartMs = millis();
  thrustTarget = 0.0f; thrustApplied = 0.0f;
  thruster.writeMicroseconds(ESC_STOP_US);
  Serial.println("Thrusters arming...");
}

// Instant stop. Safe to call repeatedly.
void disarmThrusters(){
  thrustTarget = 0.0f; thrustApplied = 0.0f;
  thrusterArming = false;
  thruster.writeMicroseconds(ESC_STOP_US);
  if (thrustersArmed) {                 // print only once
    thrustersArmed = false;
    Serial.println("Thrusters disarmed");
  }
}
//////////////// END NEW ////////////////

void safeZoneProtocol(){
  // Placeholder for safe zone protocol code
  Serial.println("Safe zone protocol activated");

  taskReadIMU.disable();
  taskReadDepth.disable();
  taskUpdateDistance.disable();

  taskUpdateThrusters.disable();
  //taskFilter.enable();
  //taskControl.enable();

  //taskHealthCheck.enable();

  delay(100);
}

void updateThrusters(){
  static uint32_t lastMs = 0;
  uint32_t now = millis();
  float dt = (now - lastMs) / 1000.0f;
  lastMs = now;
  if (dt <= 0.0f || dt > 0.25f) dt = 0.02f;

  // Finish arming (non-blocking)
  if (thrusterArming && (now - armStartMs >= ESC_ARM_MS)) {
    thrusterArming = false;
    thrustersArmed = true;
    Serial.println("Thrusters armed");
  }

  // Thrust wanted in each state
  switch (missionState) {
    case 1:  thrustTarget = THRUST_START; break;   // first 10 m
    case 2:  thrustTarget = THRUST_FULL;  break;   // straight run, full speed
    case 3:  thrustTarget = THRUST_TURN;  break;   // 180 degree turn, slowed
    case 4:  thrustTarget = THRUST_FULL;  break;   // back to the start zone, full speed
    default: thrustTarget = 0.0f;         break;   // state 0 and 5: stopped
  }

  // Never push before the ESC is armed
  if (!thrustersArmed) thrustTarget = 0.0f;

  // Slew limit: smooth speed-up / slow-down, instant stop
  if (thrustTarget <= 0.0f) {
    thrustApplied = 0.0f;
  } else {
    float step = THRUST_SLEW_PER_S * dt;
    float diff = thrustTarget - thrustApplied;
    if (diff >  step) diff =  step;
    if (diff < -step) diff = -step;
    thrustApplied += diff;
  }

  writeThruster(thrustApplied);
}

//////////////// PID + FINS (NEW) ////////////////
// Cascade:  depth error -> pitch setpoint -> pitch command
//           heading error (ramped during state 3) -> yaw command
//           roll hold -> roll command
// Only active in states 1..4. In every other state the commands are zero and the PIDs are reset.
void calculatePID(){
  static uint32_t lastMs = 0;
  static bool wasActive = false;

  uint32_t now = millis();
  float dt = (now - lastMs) / 1000.0f;
  lastMs = now;
  if (dt <= 0.0f || dt > 0.25f) dt = 0.02f;   // first call or a big gap -> use the nominal period

  bool active = (missionState >= 1 && missionState <= 4);

  if (!active) {
    if (wasActive) {                           // just left the active states: clean up once
      pidDepth.reset(); pidPitch.reset(); pidHeading.reset(); pidRoll.reset();
    }
    wasActive = false;
    pitchCmd = 0.0f; yawCmd = 0.0f; rollCmd = 0.0f;
    return;
  }

  if (!wasActive) {                            // just entered state 1: remember the outbound heading
    if (!yaw0Set) { yaw0 = yaw; yaw0Set = true; }
    turnRampDeg = 0.0f;
    pidDepth.reset(); pidPitch.reset(); pidHeading.reset(); pidRoll.reset();
  }
  wasActive = true;

  // --- Depth -> pitch -> pitch command ---
  // TODO: if the vehicle goes the wrong way, flip the sign of the gains (depends on how pitch/fins are mounted)
  float depthSP = (missionState == 1) ? DEPTH_START_M : DEPTH_CRUISE_M;   // setpoint per state
  pitchSP  = pidDepth.update(depthSP - depth, dt);          // depth error [m] -> pitch setpoint [deg]
  pitchCmd = pidPitch.update(pitchSP - pitch, dt);          // pitch error [deg] -> command (-1..1)

  // --- Heading ---
  // States 1,2: hold yaw0.  State 3: ramp the setpoint by 180 deg.  State 4: hold yaw0 + 180.
  if (missionState == 3) {
    turnRampDeg += TURN_RATE_DEG_S * dt;
    if (turnRampDeg > 180.0f) turnRampDeg = 180.0f;
  }
  headingSP = wrap360(yaw0 + turnRampDeg);
  yawCmd = pidHeading.update(wrap180(headingSP - yaw), dt); // shortest-way heading error [deg] -> command

  // --- Roll hold (target 0 deg) ---
  rollCmd = pidRoll.update(0.0f - roll, dt);
}

// Attach the four fin servos and centre them. Called once in setup(), after
// initThruster() (which allocates the PWM timers).
void initFins(){
  for (int i = 0; i < 4; i++) {
    fins[i].setPeriodHertz(50);
    fins[i].attach(finPins[i], FIN_PULSE_MIN_US, FIN_PULSE_MAX_US);
    fins[i].write((int)FIN_CENTER_DEG);
  }
}

// Mixes the three axis commands into four servo angles and sends them to the servos.
void updateFins(){
  float cmd[3] = { pitchCmd, yawCmd, rollCmd };

  for (int i = 0; i < 4; i++) {
    float v = 0.0f;
    for (int j = 0; j < 3; j++) v += MIX[i][j] * cmd[j];

    if (v >  1.0f) v =  1.0f;                    // clamp to -1..1
    if (v < -1.0f) v = -1.0f;

    finAngle[i] = FIN_CENTER_DEG + v * FIN_MAX_DEG;   // 90 +/- 30 deg
  }

  // Send to the servos. Only write when the angle changed (less jitter, fewer updates).
  static int lastWritten[4] = {-1, -1, -1, -1};
  for (int i = 0; i < 4; i++) {
    if (isnan(finAngle[i])) finAngle[i] = FIN_CENTER_DEG;   // never send NaN to a servo
    int a = (int)(finAngle[i] + 0.5f);
    if (a < 0)   a = 0;
    if (a > 180) a = 180;
    if (a != lastWritten[i]) {
      fins[i].write(a);
      lastWritten[i] = a;
    }
  }
}
//////////////// END NEW ////////////////

//void filterSensors(){delay(10);}
//void checkHealth(){delay(10);}