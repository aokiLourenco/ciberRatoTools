
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
    printf("-------------------------- \n");
}

void print_motors(float lPow, float rPow)
{
    printf("left: %f \n", lPow);
    printf("right: %f\n", rPow);
    printf("++++++++++++++++++++\n");
}

void DeterminateAction(int *beaconToFollow, float *lPow, float *rPow)
{
    static int counter = 0;
    float too_close_threashold = 1.0f;
    float close_threashold = 0.6f;
    float left, right, center;

    if (IsObstacleReady(LEFT))
        left = GetObstacleSensor(LEFT);
    if (IsObstacleReady(RIGHT))
        right = GetObstacleSensor(RIGHT);
    if (IsObstacleReady(CENTER))
        center = GetObstacleSensor(CENTER);

    float k = 0.03f;
    const float max_speed = 0.15f;

    float delta = left - right;

    if (center > too_close_threashold)
    {
        if (++counter >= 10)
        {
            *lPow = 0.15;
            *rPow = -0.5;
            counter = 0;
            return;
        }
        printf("Center too close\n");
        if (left > right)
        {
            *lPow = 0.15;
            *rPow = -0.5;
        }
        else if (right > left)
        {
            *lPow = -0.5;
            *rPow = 0.15;
        }
        else
        {
            *lPow = -0.15;
            *rPow = -0.15;
        }
    }
    else if (left > close_threashold && right > close_threashold)
    {
        counter = 0;
        *lPow = 0.15;
        *rPow = 0.15;
    }
    else if (left > too_close_threashold)
    {
        counter = 0;

        printf("Left too close\n");
        *lPow = 0.15;
        *rPow = -0.05;
    }
    else if (right > too_close_threashold)
    {
        counter = 0;

        printf("Right too close\n");
        *lPow = -0.05;
        *rPow = 0.15;
    }
    else
    {
        *lPow = 0.15;
        *rPow = 0.15;
        // printf("No obstacles\n");
        // *lPow = std::max(-max_speed, std::min(max_speed, k * delta * 1.5f));  // More aggressive turn
        // *rPow = std::max(-max_speed, std::min(max_speed, -k * delta * 1.5f)); // Turn proportionally
    }

    // static int counter = 0;

    // bool beaconReady;
    // static struct beaconMeasure beacon;
    // float left, right, center;
    // static int Ground;
    // static bool Collision;

    // const float k = 0.03f;
    // const float max_speed = 0.15f;
    // const float turn_threashold = 0.8f;

    // /*Access to values from Sensors - Only ReadSensors() gets new values */
    // if (IsObstacleReady(LEFT))
    //     left = GetObstacleSensor(LEFT);
    // if (IsObstacleReady(RIGHT))
    //     right = GetObstacleSensor(RIGHT);
    // if (IsObstacleReady(CENTER))
    //     center = GetObstacleSensor(CENTER);

    // print_sensors(left, right, center);
    // print_motors(*lPow, *rPow);

    // float delta = left - right;
    // if (center > 1.5)
    // {
    //     *lPow = (left > right) ? 0.02 : -0.02;
    //     *rPow = (left > right) ? -0.02 : 0.02;
    // }
    // else if (delta < turn_threashold && delta > -turn_threashold)
    // {
    //     *lPow = 0.15;
    //     *rPow = 0.15;
    // }
    // else
    // {

    //     *lPow +=  k * delta;
    //     *rPow -=  k * delta;

    //     // *lPow += std::max(-max_speed, std::min(max_speed, k * delta));
    //     // *rPow += std::max(-max_speed, std::min(max_speed, -k * delta));

    //     //  if (left < 4.0f || right < 4.0f) {  // Sharp turn or wall hugging detected
    //     //     printf("Diagonal or sharp turn detected.\n");
    //     //     *lPow = std::max(-max_speed, std::min(max_speed, k * delta * 1.5f));  // More aggressive turn
    //     //     *rPow = std::max(-max_speed, std::min(max_speed, -k * delta * 1.5f)); // Turn proportionally
    //     // } else {
    //     //     // Regular balancing between left and right distances
    //     //     print_sensors(left, right, center);
    //     //     *lPow = std::max(-max_speed, std::min(max_speed, k * delta));
    //     //     *rPow = std::max(-max_speed, std::min(max_speed, -k * delta));
    //     //     printf("| L: %f | R : %f |\n", *lPow, *rPow);
    //     // }

    // }

    // // if(center > 2.0f){
    // //     *lPow = (left)
    // //     *rPow = 0.1;
}