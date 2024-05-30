#ifndef CalRotation_H
#define CalRotation_H

#include <Arduino_AVRSTL.h>
#include <iostream>
#include <math.h>
using namespace std;

//Step motor numbering
#define A 0     //Toward the X-axis
#define B 1
#define C 2

#define ACCEL_G 9.80665

class CalStep
{
private:
    double globalG[3] = { 0, 0, -ACCEL_G }, baseG[3] = { 0, 0, -ACCEL_G }, \
            rotIMU[3], accIMU[3], normVec[3];
    double rotMat[3][3];
    //baseR = base_ccr = circumcircle radius of base triangle
    double baseR, ptfR, upperLeg, lowerLeg;
public:
    CalStep(double base_ccr, double ptf_ccr, double l1, double l2)
    : baseR(base_ccr), ptfR(ptf_ccr), lowerLeg(l1), upperLeg(l2) { }
    void setIMU(const double *arrIMU);
    void calNormVec(void);
    double targetAngle(int leg, double hz);
};

#endif
