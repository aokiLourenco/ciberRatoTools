#include <map>
#include <math.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>

#include "robfunc.h"

#define CENTER_POINT 25

typedef struct 
{
    double x;
    double y;
    std::string wall;
} maze_data;

typedef struct
{
                //  x  y
    maze_data* map[50][50];

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

    maze->map[x][y]->wall = map_values[value];
}

void check_for_walls(float left, float right, float center, float back, float compass,int current_x,int current_y, maze_map *maze)
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

    std::cout << "Value: " << value  << "Current x :" << current_x << " Current y :" << current_y << std::endl;
    maze->map[current_x][current_y]->wall = value;
}

int retreive_map_in_memory()
{
}

void calculate_next_point(float xStart, float yStart, float current_angle, float *end_x, float *end_y, float *angle, double *distance, int * maze_x, int* maze_y, maze_map *maze)
{
    if(*distance != -10.0f){
        *distance = sqrt(pow(*end_x - xStart, 2) + pow(*end_y - yStart, 2));
    }
    // * Check if reached the next point
    if (*distance > 0.2f){
        return;
    }


    // From the current position, calculate the possible next move, using the map as reference
    // * If my array is a tupple array, with (x,y,Wall) then i could calculate all the points first, and then check if it is possible to move to them...

    std::cout << "Current position x: " << *maze_x << " y: " << *maze_y << std::endl;
    // *Check my current position and see where there is no wall
    std::string wall =  maze->map[*maze_x][*maze_y]->wall;
    std::cout << "Wall: " << wall << std::endl;

    // * Check if wall is on the right
    if(wall.find("R") == std::string::npos){
        *maze_x = *maze_x + 1;
        *end_x = maze->map[*maze_x][*maze_y]->x;
        *end_y = maze->map[*maze_x][*maze_y]->y;

    }else if(wall.find("U") == std::string::npos){
        *maze_y = *maze_y - 1;
        *end_x = maze->map[*maze_x][*maze_y]->x;
        *end_y = maze->map[*maze_x][*maze_y]->y;
    }else if(wall.find("L") == std::string::npos){
        *maze_x = *maze_x - 1;
        *end_x = maze->map[*maze_x][*maze_y]->x;
        *end_y = maze->map[*maze_x][*maze_y]->y;
    }else if(wall.find("D") == std::string::npos){
        *maze_y = *maze_y + 1;
        *end_x = maze->map[*maze_x][*maze_y]->x;
        *end_y = maze->map[*maze_x][*maze_y]->y;
    }

    *distance = sqrt(pow(*end_x - xStart, 2) + pow(*end_y - yStart, 2));
    *angle = atan2(*end_y - yStart, *end_x - xStart) * 180 / M_PI;




}


void calculate_all_map_positions(maze_map *maze, double first_x, double first_y)
{
    // * Calculate all the positions in the map
    // * This will be used to calculate the next point to move
    // * The map is a 50 x 50 array, so the center is always 25,25
    // * The real map will be drawn inside this 50 x 50 array
    // * The map[25][25] is the middle point, it will calculate 14 for left and right, and 7 up and down
    if(maze->map[CENTER_POINT][CENTER_POINT] != NULL){
        return;
    }

    std::cout << "Calculating all map positions" << std::endl;

    for(int j = -7; j<8; j++){
        for(int i = -14; i<15;i++){
            // * Calculate the position and store it in the map

            maze->map[CENTER_POINT + i][CENTER_POINT + j] = new maze_data();
            maze->map[CENTER_POINT + i][CENTER_POINT + j]->x = first_x + i*2.0f;
            maze->map[CENTER_POINT + i][CENTER_POINT + j]->y = first_y + j*2.0f;
            maze->map[CENTER_POINT + i][CENTER_POINT + j]->wall = "";
        }
    }    


}



void DeterminateAction(int *beaconToFollow, float *lPow, float *rPow)
{

    // * Variables
    int compass_direction = 0;                                  // Direction it's facing
    float left, right, center, back;                            // Sensor values

    // * Static Variables
    static int counter = 0, count = 0;      
    static float too_close_threashold = 1.0f;                   // IF the distance is less than this, then it is too close
    static float close_threashold = 0.6f;                       // If the distance is less than this, then it is close
    static int current_map_x = CENTER_POINT, current_map_y = CENTER_POINT;     // First position in the map
    static double first_x = 12345.00;                           // First x position in the GPS (it is not the same every run, so this value is changed)    
    static double first_y = 12345.00;                           // First y position in the GPS (it is not the same every run, so this value is changed)    
    static maze_map maze;                                       // The map
    static float next_x, next_y,angle_to_turn;                  // Values to recieve after calculating the next point
    static double distance_to_next_point = -10.0f;                // Distance to the next point


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
    calculate_all_map_positions(&maze, first_x, first_y);

    // std::cout << "POS: 25,25 x: "  << maze.map[25][25]->x << " y : " << maze.map[25][25]->y  << std::endl;
    // std::cout << "POS: 26,25 x: "  << maze.map[26][25]->x << " y : " << maze.map[26][25]->y  << std::endl;

    //std::cout << "Checking for walls" << std::endl;
    // * Check for walls in the cell, and store it in the map
    check_for_walls(left, right, center, back, compass_direction, current_map_x, current_map_y, &maze);
    
    // std::cout << "Calculating next point" << std::endl;

    // * With the wall checked we can see if we< can move foward or not
    calculate_next_point(x,y,compass_direction ,&next_x, &next_y, &angle_to_turn,&distance_to_next_point, &current_map_x, &current_map_y, &maze);

    std::cout << "Next point x: " << next_x << " y: " << next_y << " angle: " << (int) angle_to_turn << " distance: " << distance_to_next_point << std::endl;


    // * Rotate to the next point

    if (compass_direction > (int) angle_to_turn){
        *lPow = 0.01;
        *rPow = -0.01;
        return;
    }else if (compass_direction < (int) angle_to_turn){
        *lPow = -0.01;
        *rPow = 0.01;
        return;
    }

    // * Move to the next point
    if (distance_to_next_point > 0.2f)
    {
        *lPow = 0.15;
        *rPow = 0.15;
        return;
    }

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

}