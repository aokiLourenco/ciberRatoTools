#include <map>
#include <math.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <vector>
#include <algorithm>

#include <thread>
#include <future>

#include "robfunc.h"
#include <bits/algorithmfwd.h>

#define CENTER_POINT 25
#define BUFFER_SIZE 10


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

struct PIDController {
    float Kp;
    float Kd;
    float previous_error;
};

void initializePID(PIDController &pid, float Kp, float Kd) {
    pid.Kp = Kp;
    pid.Kd = Kd;
    pid.previous_error = 0.0f;
}

// ! Calculate PID for Y axis (supostamente bem xd)
float calculateYPID(PIDController &pid, float error_y, float error_y_start) {
    float rotation = error_y * pid.Kp + (error_y - error_y_start) / 2 * pid.Kd;
    pid.previous_error = error_y;
    return rotation;
}

// ! Calculate PID for X axis (supostamente bem xd)
float calculateXPID(PIDController &pid, float error_x, float error_x_start) {
    float rotation = error_x * pid.Kp + (error_x - error_x_start) / 2 * pid.Kd;
    pid.previous_error = error_x;
    return rotation;
}

// *
// *
// *
// *
// *
// *
// TODO: Maybe implement a Rotate_90_Left() and Rotate_90_Right() functions using PID controller.
// *
// *
// *
// *
// *
// *

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

void check_for_walls(float left, float right, float center, float back, float compass, int current_x, int current_y, maze_map *maze)
{
    char value[5] = {0}; // Use a character array to store wall information
    int index = 0;

    float there_is_a_wall = 1.5f;

    // Lookup table for compass directions
    struct WallCheck {
        float min_angle;
        float max_angle;
        char left_wall;
        char right_wall;
        char center_wall;
        char back_wall;
    };  

    WallCheck wall_checks[] = {
        {-15.0f, 15.0f, 'U', 'D', 'R', 'L'},   // Looking right
        {85.0f, 95.0f, 'L', 'R', 'U', 'D'},    // Looking up
        {175.0f, -175.0f, 'D', 'U', 'L', 'R'}, // Looking left
        {-95.0f, -85.0f, 'R', 'L', 'D', 'U'}   // Looking down
    };

    for (const auto& check : wall_checks) {
        if (compass > check.min_angle && compass < check.max_angle) {
            if (left > there_is_a_wall) value[index++] = check.left_wall;
            if (right > there_is_a_wall) value[index++] = check.right_wall;
            if (center > there_is_a_wall) value[index++] = check.center_wall;
            if (back > there_is_a_wall) value[index++] = check.back_wall;
            break;
        }
    }

    value[index] = '\0'; // Null-terminate the character array

    std::sort(value, value + index); // Sort the character array

    std::cout << "Value: " << value << " left: " << left << " ; Current x: " << current_x << " Current y: " << current_y << std::endl;
    maze->map[current_x][current_y]->wall = value;
}

int retreive_map_in_memory()
{
    return 0;
}

bool check_if_next_point_has_been_visited(maze_map *maze, int x, int y)
{
    if(maze->map[x][y]->wall.compare("") != 0){
        std::cout << "Point has been visited" << std::endl;
        return true;
    }
    return false;
}

void calculate_next_point(float xStart, float yStart, float current_angle, float *end_x, float *end_y, float *angle, double *distance, int *maze_x, int *maze_y, maze_map *maze)
{
    // Calculate distance and angle only if distance is not initialized
    if (*distance != -10.0f) {
        *distance = sqrt(pow(*end_x - xStart, 2) + pow(*end_y - yStart, 2));
        *angle = atan2(*end_y - yStart, *end_x - xStart) * 180 / M_PI;
    }

    // Check if reached the next point
    if (*distance > 0.1f) {
        return;
    }

    // From the current position, calculate the possible next move, using the map as reference
    std::cout << "Current position x: " << *maze_x << " y: " << *maze_y << std::endl;

    // Check current position and see where there is no wall
    std::string wall = maze->map[*maze_x][*maze_y]->wall;
    std::cout << "Wall: " << wall << std::endl;

    bool enter = true;

    // Define possible moves and their corresponding wall checks
    struct Move {
        int dx;
        int dy;
        char wall_char;
    };

    std::vector<Move> moves = {
        {1, 0, 'R'},  // Move right
        {0, -1, 'U'}, // Move up
        {0, 1, 'D'},  // Move down
        {-1, 0, 'L'}  // Move left
    };

    // Prioritize left move if walls 'R' and 'U' are present
    if (wall.find('R') != std::string::npos && wall.find('U') != std::string::npos) {
        if (wall.find('L') == std::string::npos) {
            *maze_x -= 1;
            *end_x = maze->map[*maze_x][*maze_y]->x;
            *end_y = maze->map[*maze_x][*maze_y]->y;
            if (!check_if_next_point_has_been_visited(maze, *maze_x, *maze_y)) {
                enter = false;
            } else {
                *maze_x += 1; // Revert move if already visited
            }
        }
    }

    for (const auto& move : moves) {
        if (wall.find(move.wall_char) == std::string::npos && enter) {
            *maze_x += move.dx;
            *maze_y += move.dy;
            *end_x = maze->map[*maze_x][*maze_y]->x;
            *end_y = maze->map[*maze_x][*maze_y]->y;
            enter = false;
            if (check_if_next_point_has_been_visited(maze, *maze_x, *maze_y)) {
                *maze_x -= move.dx;
                *maze_y -= move.dy;
                enter = true;
            }
        }
    }

    if (enter) {
        std::cout << "No valid move found, consider using a pathfinding algorithm." << std::endl;
        // Time to use A* or another pathfinding algorithm
    }

    // Calculate distance and angle to the next point
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

    //std::cout << "Calculating all map positions" << std::endl;

    for(int j = -7; j<8; j++){
        for(int i = -14; i<15;i++){
            // * Calculate the position and store it in the map

            maze->map[CENTER_POINT + i][CENTER_POINT + j] = new maze_data();
            maze->map[CENTER_POINT + i][CENTER_POINT + j]->x = first_x + i*2.0f;
            maze->map[CENTER_POINT + i][CENTER_POINT + j]->y = first_y - j*2.0f;
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
        //printf("Compass: %f\n", GetCompassSensor());
        compass_direction = GetCompassSensor();
    }

    //printf("Walls %s\n", check_for_walls(left, right, center, back, compass_direction).c_str());
    // float next_x, next_y = calculate_next_point(x, y);

    // * Store the values in the map
    // * As we don't know the starting position, we can consider the map as a 50 x 50 array, and the center point is always 25
    // * That way, the real map can be drawn inside the 50 x 50 array
    calculate_all_map_positions(&maze, first_x, first_y);

    // * Check for walls in the cell, and store it in the map
    check_for_walls(left, right, center, back, compass_direction, current_map_x, current_map_y, &maze);

    // * With the wall checked we can see if we< can move foward or not
    calculate_next_point(x,y,compass_direction ,&next_x, &next_y, &angle_to_turn,&distance_to_next_point, &current_map_x, &current_map_y, &maze);

    std::cout << "Next point x: " << next_x << " y: " << next_y << " angle: " << (int) angle_to_turn << " distance: " << distance_to_next_point << "\nWALL : "<<  maze.map[current_map_x][current_map_y]->wall << "\nMy angle: " << compass_direction  << std::endl;
    std::cout << "\nNext point :" << current_map_x << " " << current_map_y << "\n" << std::endl; 

    static PIDController pid_angle;
    static bool pid_initialized = false;
    if (!pid_initialized) {
        initializePID(pid_angle, 0.01f, 0.1f); // Adjust PID parameters as needed
        pid_initialized = true;
    }

    float error_y = angle_to_turn - compass_direction;
    float error_y_start = pid_angle.previous_error;

    float error_x = angle_to_turn - compass_direction;
    float error_x_start = pid_angle.previous_error;

    // ! This is not working well but its something
    if (abs(compass_direction) <= 45) {
        float rotation = calculateYPID(pid_angle, error_y, error_y_start);
        *lPow = max_speed - rotation;
        *rPow = max_speed + rotation;
        return;
    } else if (abs(compass_direction) >= 135) {
        float rotation = calculateYPID(pid_angle, error_y, error_y_start);
        *lPow = max_speed + rotation;
        *rPow = max_speed - rotation;
        return;
    } else if (compass_direction > 45 && compass_direction <= 135) {
        float rotation = calculateXPID(pid_angle, error_x, error_x_start);
        *lPow = max_speed + rotation;
        *rPow = max_speed - rotation;
        return;
    } else if (compass_direction < -45 && compass_direction > -135) {
        float rotation = calculateXPID(pid_angle, error_x, error_x_start);
        *lPow = max_speed - rotation;
        *rPow = max_speed + rotation;
        return;
    }

    // * Rotate to the next point

    // if((int) angle_to_turn == 180 && compass_direction > -180 && compass_direction < 0){
    //     *lPow = 0.15;
    //     *rPow = -0.15;
    //     return;
    // }else if((int) angle_to_turn == -180 && compass_direction < 180 && compass_direction > 0){
    //     *lPow = -0.15;
    //     *rPow = 0.15;
    //     return;

    // }else if (compass_direction > (int) angle_to_turn){
    //     *lPow = 0.15;
    //     *rPow = -0.15;
    //     return;
    // }else if (compass_direction < (int) angle_to_turn){
    //     *lPow = -0.15;
    //     *rPow = 0.15;
    //     return;
    // }

    // * Move to the next point
    if (distance_to_next_point > 0.1f)
    {
        *lPow = 0.15;
        *rPow = 0.15;
        return;
    }
}
