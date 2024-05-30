#include "CalRotation.h"
double R2D = 180/M_PI;
double D2R = M_PI/180;


void CalStep::setIMU(const double *arrIMU)
{
    for (int i=0; i<6; i++) {
        if (i<3)    accIMU[i] = arrIMU[i];
        // Rotation matrix should be a representation of the global coordinate system from the base coordinate system perspective
        else        rotIMU[i-3] = -arrIMU[i]*D2R;
    }
}

void CalStep::calNormVec(void)
{
    // Calculate rotation matrix C_BI (B: Base coordinate, I: Inertial coordinate)
    double qx, qy, qz, qw;

    qx = sin(rotIMU[0]/2) * cos(rotIMU[1]/2) * cos(rotIMU[2]/2) - cos(rotIMU[0]/2) * sin(rotIMU[1]/2) * sin(rotIMU[2]/2);
    qy = cos(rotIMU[0]/2) * sin(rotIMU[1]/2) * cos(rotIMU[2]/2) + sin(rotIMU[0]/2) * cos(rotIMU[1]/2) * sin(rotIMU[2]/2);
    qz = cos(rotIMU[0]/2) * cos(rotIMU[1]/2) * sin(rotIMU[2]/2) - sin(rotIMU[0]/2) * sin(rotIMU[1]/2) * cos(rotIMU[2]/2);
    qw = cos(rotIMU[0]/2) * cos(rotIMU[1]/2) * cos(rotIMU[2]/2) + sin(rotIMU[0]/2) * sin(rotIMU[1]/2) * sin(rotIMU[2]/2);
    //cout << "quaternion = \n" << qx << ' ' << qy << ' ' << qz << ' ' << qw << endl;

    rotMat[0][0] = 2 * (pow(qx,2) + pow(qy,2)) - 1;
    rotMat[0][1] = 2 * (qy*qz - qx*qw);
    rotMat[0][2] = 2 * (qy*qw + qx*qz);
    rotMat[1][0] = 2 * (qy*qz + qx*qw);
    rotMat[1][1] = 2 * (pow(qx,2) + pow(qz,2)) - 1;
    rotMat[1][2] = 2 * (qz*qw - qx*qy);
    rotMat[2][0] = 2 * (qy*qw - qx*qz);
    rotMat[2][1] = 2 * (qz*qw + qx*qy);
    rotMat[2][2] = 2 * (pow(qx,2) + pow(qw,2)) - 1;
    /*
    cout << "rotMat = " << endl;
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++)
            cout << rotMat[i][j] << ' ';
        cout << endl;
    }
    */

    // Calculate normal vector of targeting platform angle
    for (int i=0; i<3; i++) {
        baseG[i] = 0;           //Clear existing value
        for (int j=0; j<3; j++)
            baseG[i] += rotMat[i][j] * globalG[j];
        normVec[i] = accIMU[i] - 2*baseG[i];
    }
    //cout << "normVec = \n" << normVec[0] << '\n' << normVec[1] << '\n' << normVec[2] << endl;
}

double CalStep::targetAngle(int leg, double hz)
{
    // Create unit normal vector
    double nx = normVec[0], ny = normVec[1], nz = normVec[2];
    double nMag = sqrt(pow(nx, 2) + pow(ny, 2) + pow(nz, 2));
    nx /= nMag;
    ny /= nMag;
    nz /= nMag;
    double x, y, z, mag, angle;

    switch (leg) {
        case A:
            y = baseR + (ptfR / 2) * (1 - (pow(nx, 2) + 3 * pow(nz, 2) + 3 * nz) / (nz + 1 - pow(nx, 2) + (pow(nx, 4) - 3 * pow(nx, 2) * pow(ny, 2)) / ((nz + 1) * (nz + 1 - pow(nx, 2)))));
            z = hz + ptfR * ny;
            mag = sqrt(pow(y, 2) + pow(z, 2));
            angle = acos(y / mag) + acos((pow(mag, 2) + pow(lowerLeg, 2) - pow(upperLeg, 2)) / (2 * mag * lowerLeg));
            break;
        case B:
            x = (sqrt(3.0) / 2) * (ptfR * (1 - (pow(nx, 2) + sqrt(3.0) * nx * ny) / (nz + 1)) - baseR);
            y = x / sqrt(3.0);
            z = hz - (ptfR / 2) * (sqrt(3.0) * nx + ny);
            mag = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
            angle = acos((sqrt(3.0) * x + y) / (-2 * mag)) + acos((pow(mag, 2) + pow(lowerLeg, 2) - pow(upperLeg, 2)) / (2 * mag * lowerLeg));
            break;
        case C:
            x = (sqrt(3.0) / 2) * (baseR - ptfR * (1 - (pow(nx, 2) - sqrt(3.0) * nx * ny) / (nz + 1)));
            y = -x / sqrt(3.0);
            z = hz + (ptfR / 2) * (sqrt(3.0) * nx - ny);
            mag = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
            angle = acos((sqrt(3.0) * x - y) / (2 * mag)) + acos((pow(mag, 2) + pow(lowerLeg, 2) - pow(upperLeg, 2)) / (2 * mag * lowerLeg));
            break;
    }
    return (angle*R2D);
}
/*
int main(void) {
    
    Vector3d accIMU, rotIMU, globalG;
    globalG << 0, 0, -9.81;
    accIMU << 4.905, 0, -8.48;
    rotIMU << 0, 30, 0;
    rotIMU *= -D2R;
    Matrix3d rotMat;

    CalStep machine(0.040, 0.050, 0.028, 0.060);
    rotMat = machine.CalRotMat(rotIMU);
    cout << "rotMat = \n" << rotMat << endl;
    cout << endl;
    Vector3d normVec;
    normVec = machine.CalNormVec(accIMU, rotMat);
    cout << "normVec = \n" << normVec << endl;
    cout << endl;
    cout << "Step A angle : " << machine.MotorTheta(A, 0.060, normVec(0), normVec(1), normVec(2)) << endl;
    cout << "Step B angle : " << machine.MotorTheta(B, 0.060, normVec(0), normVec(1), normVec(2)) << endl;
    cout << "Step C angle : " << machine.MotorTheta(C, 0.060, normVec(0), normVec(1), normVec(2)) << endl;
    
    double imu[6] = { -4.905, 0, -8.48, 0, -30, 0 };
    CalStep machine(0.040, 0.050, 0.028, 0.060);
    machine.setIMU(imu);
    machine.calNormVec();
    cout << "Step A angle : " << machine.targetAngle(A, 0.060) << endl;
    cout << "Step B angle : " << machine.targetAngle(B, 0.060) << endl;
    cout << "Step C angle : " << machine.targetAngle(C, 0.060) << endl;
}
*/
