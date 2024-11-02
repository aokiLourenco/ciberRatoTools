
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <algorithm>

#include "robfunc.h"

#define HARD_TURN -0.02f
#define NORMAL_TURN 3.5f
#define CENTER_FIX 10
#define CENTER_SPEED 3
#define ROTATION_SPEED 0.1f


void DeterminateAction(int *beaconToFollow, float *lPow, float *rPow)
{
    static int counter = 0;
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

    if (center > 1)
    {
        if (left < right)
        {
            *lPow = HARD_TURN * center;
            *rPow = NORMAL_TURN * left + center;
        }
        else 
        {
            *lPow = NORMAL_TURN * right + center;
            *rPow = HARD_TURN * center;
        }
        return;
    }
    else if (left > 2.9)
    {
        float detour = left - 2.15;
        *lPow = CENTER_FIX * detour + CENTER_SPEED * center;
        *rPow = ROTATION_SPEED * right;
        return;
    }
    else if (right > 2.9)
    {
        float detour = right - 2.15;
        *lPow = ROTATION_SPEED * left;
        *rPow = CENTER_FIX * detour + CENTER_SPEED * center;
        return;
    }
    else
    {
        *lPow = 1;
        *rPow = 1;
        return;
    }
}