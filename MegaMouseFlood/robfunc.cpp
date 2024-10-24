#include <map>
#include <math.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>

#include "robfunc.h"

#define CENTER_POINT 25;

typedef struct
{

    std::string map[50][50];

} maze_map;

void print_sensors(float left, float right, float center, float back)
{
    printf("left: %f \n", left);
    printf("right: %f\n", right);
    printf("center: %f\n", center);
    printf("back: %f\n", back);
    printf("-------------------------- \n");
}

void print_motors(float lPow, float rPow)
{
    printf("left: %f \n", lPow);
    printf("right: %f\n", rPow);
    printf("++++++++++++++++++++\n");
}

void store_map__in_memory(maze_map *maze, int x, int y, std::string value)
{

    // L = Left, R = Right, U = Up, D = Down
    std::map<std::string, std::string> map_values = {{"L", "1"}, {"U", "2"}, {"R", "3"}, {"D", "4"}, {"DL", "5"}, {"DR", "6"}, {"RU", "7"}, {"LU", "8"}, {"LR", "9"}, {"DU", "10"}, {"DLR", "11"}, {"DRU", "12"}, {"LRU", "13"}, {"DLU", "14"}, {"", "15"}, {"DLRU", "16"}};

    maze->map[x][y] = map_values[value];
}

std::string check_for_walls(float left, float right, float center, float back, float compass)
{
    std::string value = "";

    float there_is_a_wall = 1.5f;

    if (compass > -15.0f && compass < 15.0f)
    { // Looking right

        if (left > there_is_a_wall)
        {
            value.append("U");
        }
        if (right > there_is_a_wall)
        {
            value.append("D");
        }
        if (center > there_is_a_wall)
        {
            value.append("R");
        }
        if (back > there_is_a_wall)
        {
            value.append("L");
        }
    }
    else if (compass > 85.0f && compass < 95.0f)
    { // Looking up

        if (left > there_is_a_wall)
        {
            value.append("L");
        }
        if (right > there_is_a_wall)
        {
            value.append("R");
        }
        if (center > there_is_a_wall)
        {
            value.append("U");
        }
        if (back > there_is_a_wall)
        {
            value.append("D");
        }
    }
    else if (compass < -175.0f || compass > 175.0f)
    { // Looking left
        if (left > there_is_a_wall)
        {
            value.append("D");
        }
        if (right > there_is_a_wall)
        {
            value.append("U");
        }
        if (center > there_is_a_wall)
        {
            value.append("L");
        }
        if (back > there_is_a_wall)
        {
            value.append("R");
        }
    }
    else if (compass < -85.0f && compass > -90.0f)
    { // Looking down
        if (left > there_is_a_wall)
        {
            value.append("R");
        }
        if (right > there_is_a_wall)
        {
            value.append("L");
        }
        if (center > there_is_a_wall)
        {
            value.append("D");
        }
        if (back > there_is_a_wall)
        {
            value.append("U");
        }
    }

    // Sort the string
    std::sort(value.begin(), value.end());

    return value;
}

int retreive_map_in_memory()
{
}

float calculate_next_point(float xStart, float yStart, maze_map *maze)
{
    // From the current position, calculate the possible next move, using the map as reference

    // * Lets try this logic, it will try to move first to the front, if not rotate to an exit it has not visited yet
    


}

void DeterminateAction(int *beaconToFollow, float *lPow, float *rPow)
{

    // * Variables

    static int counter = 0, count = 0;
    static float too_close_threashold = 1.0f;
    static float close_threashold = 0.6f;
    float left, right, center, back;
    static int current_x, current_y = CENTER_POINT;
    static double first_x = 12345.00;
    static double first_y = 12345.00;
    int compass_direction = 0;
    maze_map maze;

    // * Read sensors
    if (IsObstacleReady(LEFT))
        left = GetObstacleSensor(LEFT);
    if (IsObstacleReady(RIGHT))
        right = GetObstacleSensor(RIGHT);
    if (IsObstacleReady(CENTER))
        center = GetObstacleSensor(CENTER);
    if (IsObstacleReady(OTHER1))
        back = GetObstacleSensor(OTHER1);

    // * Calculate values to move
    float k = 0.03f;
    const float max_speed = 0.15f;

    float delta = left - right;

    double x, y;

    // * Get current position from the gps and compass

    if (IsGPSReady())
    {

        x = GetX();
        y = GetY();
        if (first_x == 12345.00)
        {
            // * Set the first position (GPS is not the same every run, so it needs this to move to the next point)
            first_x = GetX();
            first_y = GetY();
        }
        // printf("First X: %f\n", first_x);
        // printf("X: %f, Y: %f\n", x, y);
    }

    // * Compass varies from -180 to 180
    if (IsCompassReady())
    {
        printf("Compass: %f\n", GetCompassSensor());

        compass_direction = GetCompassSensor();
    }

    //printf("Walls %s\n", check_for_walls(left, right, center, back, compass_direction).c_str());

    // float next_x, next_y = calculate_next_point(x, y);

    // * Store the values in the map
    // * As we don't know the starting position, we can consider the map as a 50 x 50 array, and the center point is always 25
    // * That way, the real map can be drawn inside the 50 x 50 array

    store_map__in_memory(&maze, current_x, current_y, check_for_walls(left, right, center, back, compass_direction));

    // * See where can i move next

    float next_x, next_y = calculate_next_point(x, y, &maze);




    if (count > 2)
    {
        *lPow = -0.01;
        *rPow = 0.01;

        // if (compass_direction > 85.0f && compass_direction < 95.0f)
        // {
        //     *lPow = 0.1;
        //     *rPow = 0.1;
        // }
        // print_sensors(left, right, center, back);
        return;
    }

    if ((x > first_x + 1.9f && x < first_x + 2.1f))
    {
        *lPow = 0.0;
        *rPow = 0.0;
        first_x = GetX();
        printf("Count: %d\n", count);
        count++;
        return;
    }
    *lPow = 0.1;
    *rPow = 0.1;
    return;

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