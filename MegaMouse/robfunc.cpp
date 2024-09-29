
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

    const float k = 0.02f;
    const float max_speed = 0.1f;
    const float turn_threashold = 0.9f;


    /*Access to values from Sensors - Only ReadSensors() gets new values */
    if (IsObstacleReady(LEFT))
        left = GetObstacleSensor(LEFT);
    if (IsObstacleReady(RIGHT))
        right = GetObstacleSensor(RIGHT);
    if (IsObstacleReady(CENTER))
        center = GetObstacleSensor(CENTER);


    float delta = left - right;
    if (center > 3)
    {
        printf("Center close %f \n", center);
        *lPow = (left > right) ? 0.05 : -0.05;
        *rPow = (left > right) ? -0.05 : 0.05;
    }
    else if (delta < turn_threashold && delta > -turn_threashold)
    {
        *lPow = 0.1;
        *rPow = 0.1;
    }
    else
    {

         if (left < 4.0f || right < 4.0f) {  // Sharp turn or wall hugging detected
            printf("Diagonal or sharp turn detected.\n");
            *lPow = std::max(-max_speed, std::min(max_speed, k * delta * 1.5f));  // More aggressive turn
            *rPow = std::max(-max_speed, std::min(max_speed, -k * delta * 1.5f)); // Turn proportionally
        } else {
            // Regular balancing between left and right distances
            print_sensors(left, right, center);
            *lPow = std::max(-max_speed, std::min(max_speed, k * delta));
            *rPow = std::max(-max_speed, std::min(max_speed, -k * delta));
            printf("| L: %f | R : %f |\n", *lPow, *rPow);
        }

        // print_sensors(left, right, center);
        // *lPow = std::max(-max_speed, std::min(max_speed, k * delta));
        // *rPow = std::max(-max_speed, std::min(max_speed, -k * delta));
        // printf("| L: %f | R : %f |\n", *lPow, *rPow);
    }

    // printf("left: %f \n", left);
    // printf("right: %f\n", right);
    // printf("center: %f\n", center);

    // * Try to remain on the center path
    // * Is goint to hit the wal on the sides
    // if (left > 10 || right > 10){
    //     *lPow = (left > right) ? 0.05 : -0.05;
    //     *rPow = (left > right) ? -0.05 : 0.05;
    // }

    // // * Is going to hit front
    // if (center > 3)
    // {
    //     printf("F*** TURN BACK (center > 3) \n");
    //     print_sensors(left, right, center);
    //     *lPow = (left > right) ? 0.1 : -0.1;
    //     *rPow = (left > right) ? -0.1 : 0.1;
    // }

    // else if(left > 1.5 && right > 1.5){
    //     *lPow = (left > right) ? 0.1 : 0.09;
    //     *rPow = (left > right) ? 0.09 : 0.1;
    // }

    /* If Really close */

    // else if(left > 3){
    //     printf("TURN TURN RIGHT (Left > 3) \n");
    //     print_sensors(left, right, center);

    //     *lPow = 0.05;
    //     *rPow = 0;//-0.02;
    // }
    // else if(right > 3){
    //     printf("TURN TURN LEFT (Right > 3) \n");
    //     print_sensors(left, right, center);

    //     *lPow = 0;//-0.02;
    //     *rPow = 0.05;
    // }

    // /* If it can still dodge*/
    // else if(left > 2){
    //     printf("TURN  RIGHT (Left > 2) \n");
    //     print_sensors(left, right, center);

    //     *lPow = 0.1;
    //     *rPow = 0.08;
    // }
    // else if(right > 2){
    //     printf("TURN  LEFT (Right > 2) \n");
    //     print_sensors(left, right, center);

    //     *lPow = 0.08;
    //     *rPow = 0.1;
    // }

    // else if(left > 1.5){
    //     *lPow = 0.05;
    //     *rPow = 0.0;
    // }
    // else if(right > 1.5){
    //     *lPow = 0.0;
    //     *rPow = 0.05;
    // }

    // else
    // {
    //     *lPow = 0.1;
    //     *rPow = 0.1;
    // }
}