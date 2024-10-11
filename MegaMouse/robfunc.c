#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <algorithm>

#include "robfunc.h"

/* Calculate the power of left and right motors */
void DetermineAction(int *beaconToFollow, float *lPow, float *rPow)
{
    static int counter = 0;

    bool beaconReady;
    static struct beaconMeasure beacon;
    static float left, right, center;
    static int Ground;
    static bool Collision;

    /*Access to values from Sensors - Only ReadSensors() gets new values */
    if (IsObstacleReady(LEFT))
        left = GetObstacleSensor(LEFT);
    if (IsObstacleReady(RIGHT))
        right = GetObstacleSensor(RIGHT);
    if (IsObstacleReady(CENTER))
        center = GetObstacleSensor(CENTER);

    beaconReady = IsBeaconReady(*beaconToFollow);
    if (beaconReady)
    {
        beacon = GetBeaconSensor(*beaconToFollow);
    }
    else
        beaconReady = 0;

    if (IsGroundReady())
        Ground = GetGroundSensor();
    if (IsBumperReady())
        Collision = GetBumperSensor();

    if (center > 4.5 || right > 4.5 || left > 4.5 || Collision)
    { /* Close Obstacle - Rotate */
        if (counter % 400 < 200)
        {
            *lPow = 0.06;
            *rPow = -0.06;
        }
        else
        {
            *lPow = -0.06;
            *rPow = 0.06;
        }
    }
    else if (right > 1.5)
    { /* Obstacle Near - Avoid */
        *lPow = 0.0;
        *rPow = 0.05;
    }
    else if (left > 1.5)
    {
        *lPow = 0.05;
        *rPow = 0.0;
    }
    else
    {
        if (beaconReady && beacon.beaconVisible && beacon.beaconDir > 20.0)
        { /* turn to Beacon */
            *lPow = 0.0;
            *rPow = 0.1;
        }
        else if (beaconReady && beacon.beaconVisible && beacon.beaconDir < -20.0)
        {
            *lPow = 0.1;
            *rPow = 0.0;
        }
        else
        { /* Full Speed Ahead */
            *lPow = 0.1;
            *rPow = 0.1;
        }
    }

    counter++;
}

void print_sensors(float left, float right, float center)
{
    printf("left: %f \n", left);
    printf("right: %f\n", right);
    printf("center: %f\n", center);
}

void DeterminateAction(int *beaconToFollow, float *lPow, float *rPow)
{
    static int counter = 0;
    bool beaconReady;
    static struct beaconMeasure beacon;
    float left, right, center;
    static int Ground;
    static bool Collision;

    const float k = 0.08f;
    const float max_speed = 0.2f;
    const float turn_threashold = 0.9f;

    if (IsObstacleReady(LEFT))
        left = GetObstacleSensor(LEFT);
    if (IsObstacleReady(RIGHT))
        right = GetObstacleSensor(RIGHT);
    if (IsObstacleReady(CENTER))
        center = GetObstacleSensor(CENTER);

    float delta = left - right;
    printf("Sensor values - Left: %f, Right: %f, Center: %f, Delta: %f\n", left, right, center, delta);

    if (center > 3)
    {
        printf("Center close %f \n", center);
        *lPow = (left > right) ? 0.05 : -0.05;
        *rPow = (left > right) ? -0.05 : 0.05;
        printf("Action: Rotate - lPow: %f, rPow: %f\n", *lPow, *rPow);
    }
    else if (delta < turn_threashold && delta > -turn_threashold)
    {
        *lPow = 0.1;
        *rPow = 0.1;
        printf("Action: Move Forward - lPow: %f, rPow: %f\n", *lPow, *rPow);
    }
    else
    {
        if (left < 5.0f || right < 5.0f) {
            printf("Diagonal or sharp turn detected.\n");
            *lPow = std::max(-max_speed, std::min(max_speed, k * delta * 1.5f));
            *rPow = std::max(-max_speed, std::min(max_speed, -k * delta * 1.5f));
            printf("Action: Sharp Turn - lPow: %f, rPow: %f\n", *lPow, *rPow);
        } else {
            print_sensors(left, right, center);
            *lPow = std::max(-max_speed, std::min(max_speed, k * delta));
            *rPow = std::max(-max_speed, std::min(max_speed, -k * delta));
            printf("Action: Adjusting - lPow: %f, rPow: %f\n", *lPow, *rPow);
            printf("| L: %f | R : %f |\n", *lPow, *rPow);
        }
    }
}

//!Unit Test code

// #include <cassert>
// #include <cstdio>

// // Mock functions and variables
// bool IsObstacleReady(int sensor) { return true; }
// float GetObstacleSensor(int sensor) { return 0.0f; }
// void print_sensors(float left, float right, float center) {}

// void testDeterminateAction() {
//     float lPow, rPow;

//     // Test case: Center obstacle detected
//     GetObstacleSensor = [](int sensor) { return (sensor == CENTER) ? 4.0f : 0.0f; };
//     DeterminateAction(nullptr, &lPow, &rPow);
//     assert(lPow == 0.05f || lPow == -0.05f);
//     assert(rPow == -0.05f || rPow == 0.05f);

//     // Test case: Clear path
//     GetObstacleSensor = [](int sensor) { return 0.0f; };
//     DeterminateAction(nullptr, &lPow, &rPow);
//     assert(lPow == 0.1f);
//     assert(rPow == 0.1f);

//     // Test case: Sharp turn required
//     GetObstacleSensor = [](int sensor) { return (sensor == LEFT) ? 3.0f : 0.0f; };
//     DeterminateAction(nullptr, &lPow, &rPow);
//     assert(lPow != 0.1f);
//     assert(rPow != 0.1f);

//     // Test case: Balanced turning
//     GetObstacleSensor = [](int sensor) { return (sensor == LEFT) ? 5.0f : 4.0f; };
//     DeterminateAction(nullptr, &lPow, &rPow);
//     assert(lPow != 0.1f);
//     assert(rPow != 0.1f);

//     // Test case: Sensor not ready
//     IsObstacleReady = [](int sensor) { return false; };
//     DeterminateAction(nullptr, &lPow, &rPow);
//     // No specific assertion, just ensure no crash
// }

// int main() {
//     testDeterminateAction();
//     printf("All tests passed.\n");
//     return 0;
// }
