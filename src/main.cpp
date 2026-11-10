#include <Arduino.h>
#include <TaskScheduler.h>

void readIMU();
void readDepth();
void filterSensors();

void calculatePID();
void processControl();

void updateActuators();

void checkHealth();

Scheduler scheduler;

Task taskReadIMU(20, TASK_FOREVER, &readIMU);
Task taskReadDepth(50, TASK_FOREVER, &readDepth);
Task taskFilter(20, TASK_FOREVER, &filterSensors);
Task taskControl(20, TASK_FOREVER, &processControl);
Task taskUpdateAct(20, TASK_FOREVER, &updateActuators);
Task taskHealthCheck(200, TASK_FOREVER, &checkHealth);

////////////////SET UP////////////////
void setup() {
  scheduler.init();

  scheduler.addTask(taskReadIMU);
  scheduler.addTask(taskReadDepth);
  scheduler.addTask(taskFilter);
  scheduler.addTask(taskControl);
  scheduler.addTask(taskUpdateAct);
  scheduler.addTask(taskHealthCheck);

  taskReadIMU.enable();
  taskReadDepth.enable();
  taskFilter.enable();
  taskControl.enable();
  taskUpdateAct.enable();
  taskHealthCheck.enable();

}

////////////////LOOP////////////////
void loop() {
  scheduler.execute();
}



void readIMU(){delay(10);}
void readDepth(){delay(10);}
void filterSensors(){delay(10);}

void calculatePID(){delay(10);}
void processControl(){delay(10);}

void updateActuators(){delay(10);}

void checkHealth(){delay(10);}
