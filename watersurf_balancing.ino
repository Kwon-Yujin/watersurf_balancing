/*
    First developer: Kwon-Yujin
*/

// Header files
#include "CalRotation.h"
#include <AccelStepper.h>
#include <MultiStepper.h>

// Define length of parts
#define BASE_CCR 0.040            //CCR: CircumCircle Radius
#define PLATFORM_CCR 0.050
#define LOWERLEG 0.028
#define UPPERLEG 0.060
#define HZ 0.060
// Define Arduino pin number as identifier
#define ENA 0

int posAbs[3];
int ks = 20;
double speed[3] = { 0, }, speedPrev[3];
double ang2step = 3200 / 360;   //steps per degree
double angOrig = 206.662752199;

// Variables for test
// Input values instead of real IMU sensor data
String inputString = "";
double inputValues[6];

// For update targetAngle
CalStep calStep(BASE_CCR, PLATFORM_CCR, LOWERLEG, UPPERLEG);

// For stepper motor control
AccelStepper stepperA(1, 1, 2);
AccelStepper stepperB(1, 3, 4);
AccelStepper stepperC(1, 5, 6);
MultiStepper steppers;

// Prototype of functions
void moveStepper(void);
void ctrlStepper(void);
void parseInputString(String input);

void setup()
{
    Serial.begin(115200);

    // Add the steppers to the steppersControl instance for multi stepper control
    steppers.addStepper(stepperA);
    steppers.addStepper(stepperB);
    steppers.addStepper(stepperC);

    // Enable pin
    pinMode(ENA, OUTPUT);
    digitalWrite(ENA, HIGH);
    delay(1000);
    digitalWrite(ENA, LOW);
}

void loop()
{
    if (Serial.available()) {
        Serial.println("Enter 6 variables of IMU data. Acceleration first, rotation next.");
        char inChar = Serial.read();
        if (inChar != '\n') {
            inputString += inChar;
        } else {
            parseInputString(inputString);
            inputString = "";
        }
    }
}

void parseInputString(String input)
{
    int numValues = 0;
    double value;
    char* ptr = strtok((char*)input.c_str(), " ");
    while (ptr != NULL) {
        value = atof(ptr);
        inputValues[numValues++] = value;
        ptr = strtok(NULL, " ");
    }
    if (numValues == 6) {
        ctrlStepper();
    }
    else {
        Serial.println("Invalid input. Please enter 6 double values separated by spaces.");
    }
}

void ctrlStepper(void)
{
    // Calculate speed of stepper motor
    for (int i=0; i<3; i++) {
        speedPrev[i] = speed[i];
        speed[i] = (i==A) * stepperA.currentPosition() + (i==B) * stepperB.currentPosition() + (i==C) * stepperC.currentPosition();
        // targetPosition(abs) - currentPosition(abs?) = relativePosition
        speed[i] = abs(speed[i] - posAbs[i]) * ks;
        speed[i] = constrain(speed[i], speedPrev[i]-200, speedPrev[i]+200);
        speed[i] = constrain(speed[i], 0, 1000);
    }
    long timeRec = millis();
    while (millis() - timeRec < 20) {   //20ms
        moveStepper();
    }
}

void moveStepper(void)
{
    calStep.setIMU(inputValues);
    calStep.calNormVec();
    for (int i=0; i<3; i++)
        posAbs[i] = round((angOrig - calStep.targetAngle(i, HZ)) * ang2step);
    
    // Set calculated speed[step/sec]
    stepperA.setMaxSpeed(speed[A]);
    stepperB.setMaxSpeed(speed[B]);
    stepperC.setMaxSpeed(speed[C]);
    // Set acceleration to be proportional to speed
    stepperA.setAcceleration(speed[A]*30);
    stepperB.setAcceleration(speed[B]*30);
    stepperC.setAcceleration(speed[C]*30);
    // Set target position of stepper
    stepperA.moveTo(posAbs[A]);
    stepperB.moveTo(posAbs[B]);
    stepperC.moveTo(posAbs[C]);
    // Run stepper to target position
    stepperA.run();
    stepperB.run();
    stepperC.run();
}
