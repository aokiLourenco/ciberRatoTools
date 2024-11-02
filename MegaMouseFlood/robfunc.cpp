#include <map>
#include <unordered_map>
#include <tuple>
#include <math.h>
#include <string>
#include <math.h>
#include <time.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <queue>

#include "robfunc.h"
#include "RobSock.h"

// MEGAROB CLASS

MegaRob::MegaRob()
{
    printf("MegaRob created\n");
    ReadSensors();
    if (IsGPSReady())
    {

        GPS_x_init = GetX();
        GPS_y_init = GetY();
        GPS_x_start = GetX();
        GPS_y_start = GetY();
    }
    for (int i = 0; i < 27; i++)
    {
        for (int j = 0; j < 55; j++)
        {
            map[i][j] = " ";
        }
    }
}

MegaRob::~MegaRob()
{
    printf("MegaRob destroyed\n");
}

void MegaRob::set_map(std::string map_value[27][55])
{
    // Set map values
    for (int x = 0; x < 27; x++)
    {
        for (int y = 0; y < 55; y++)
        {
            map[x][y] = map_value[x][y];
        }
    }
}

void MegaRob::print_map()
{

    auto cell_to_char = [&](const std::string &cell)
    {
        // std::cout << "cell" << cell <<"." << std::endl;
        // std::cout << "cell.compare" << cell.compare(" ") << std::endl;
        if (cell.compare("I") == 0)
            return 'I'; // 50
        if (cell.compare("NV") == 0)
            return 'X'; // 80
        if (cell.compare("E") == 0)
            return 'X'; // 20
        if (cell.compare("SV") == 0)
            return 'X'; // 60
        if (cell.compare("A") == 0)
            return 'X'; // 90
        if (cell.compare("WLR") == 0)
            return '-'; // 40
        if (cell.compare("WC") == 0)
            return '|'; // 30
        if (cell.compare(" ") == 0)
            return ' '; // 10
    };
    // Print map values
    FILE *file = fopen("map.txt", "w");
    for (int y = 0; y < 27; ++y)
    {
        for (int x = 0; ++x < 55; ++x)
        {
            std::cout << cell_to_char(map[y][x]);
        }
        std::cout << std::endl;
    }
}

int MegaRob::next_cell_to_explore(const std::vector<int> &list, int N)
{
    std::vector<int> next_cell_arr;
    for (int i : list)
    {
        next_cell_arr.push_back(std::abs(N - i));
    }

    // Find the index of the minimum element in next_cell_arr
    // THIS IS STUPID; THIS IS COMPLICATED; IT SHOULD BE SIMPLER; BUT OH WELL
    auto min_element_iter = std::min_element(next_cell_arr.begin(), next_cell_arr.end());
    return std::distance(next_cell_arr.begin(), min_element_iter);
}

int MegaRob::get_current_position(float GPS, int start, int end, int step)
{
    std::vector<int> GPS_current_vet;
    for (int i = start; i < end; i += step)
    {
        GPS_current_vet.push_back(i);
    }
    int position = next_cell_to_explore(GPS_current_vet, (int)std::round(GPS));
    return GPS_current_vet[position];
}

std::vector<bool> MegaRob::DefineQuadrant(int compass_direction)
{
    ReadSensors();

    if (IsCompassReady())
    {
        compass_direction = GetCompassSensor();
    }
    
    std::vector<bool> Quadrant(4, false);

    if (std::abs(compass_direction) <= 45) {
        Quadrant[0] = true;
    } else if (compass_direction > 45 && compass_direction <= 135) {
        Quadrant[1] = true;
    } else if (std::abs(compass_direction) >= 135) {
        Quadrant[2] = true;
    } else if (compass_direction <= -45 && compass_direction >= -135) {
        Quadrant[3] = true;
    }

    return Quadrant;
}

std::vector<std::string> MegaRob::Mapper(float left, float right, float center, std::vector<bool> quadrants, int map_y, int map_x)
{
    ReadSensors();

    // * Read sensors
    if (IsObstacleReady(LEFT))
        left = GetObstacleSensor(LEFT);
    if (IsObstacleReady(RIGHT))
        right = GetObstacleSensor(RIGHT);
    if (IsObstacleReady(CENTER))
        center = GetObstacleSensor(CENTER);

    float threshold = 1.5f;

    // std::cout<< "For cell pos (" << GPS_y_current <<","<< GPS_x_current <<"), sensors are: L - " << left << ", R - " << right << ", C - " <<center ;

    std::string ahead1, left1, right1;
    std::string ahead2, left2, right2;

    auto update_map = [&](float sensor_value, float threshold, std::vector<int> map_coords, std::string wall_value, std::string empty_value, std::string explore_value)
    {
        if (sensor_value >= threshold)
        {
            map[map_coords[0]][map_coords[1]] = wall_value;
        }
        else
        {
            map[map_coords[0]][map_coords[1]] = empty_value;
            if (map[map_coords[2]][map_coords[3]].compare("NV") != 0)
            {
                map[map_coords[2]][map_coords[3]] = explore_value;
            }
        }
    };

    std::cout << "Quadrants: ";
    for (bool quadrant : quadrants)
    {
        std::cout << quadrant << " ";
    }
    std::cout << std::endl;

    if (quadrants[0])
    {
        update_map(center, threshold, {map_y, map_x + 1, map_y, map_x + 2}, "WC", "E", "SV");
        update_map(left, threshold, {map_y - 1, map_x, map_y - 2, map_x}, "WLR", "E", "SV");
        update_map(right, threshold, {map_y + 1, map_x, map_y + 2, map_x}, "WLR", "E", "SV");

        ahead1 = map[map_y][map_x + 1];
        ahead2 = map[map_y][map_x + 2];
        right1 = map[map_y + 1][map_x];
        right2 = map[map_y + 2][map_x];
        left1 = map[map_y - 1][map_x];
        left2 = map[map_y - 2][map_x];
    }
    else if (quadrants[1])
    {
        update_map(center, threshold, {map_y - 1, map_x, map_y - 2, map_x}, "WLR", "E", "SV");
        update_map(left, threshold, {map_y, map_x - 1, map_y, map_x - 2}, "WC", "E", "SV");
        update_map(right, threshold, {map_y, map_x + 1, map_y, map_x + 2}, "WC", "E", "SV");

        ahead1 = map[map_y - 1][map_x];
        ahead2 = map[map_y - 2][map_x];
        right1 = map[map_y][map_x + 1];
        right2 = map[map_y][map_x + 2];
        left1 = map[map_y][map_x - 1];
        left2 = map[map_y][map_x - 2];
    }
    else if (quadrants[2])
    {
        update_map(center, threshold, {map_y, map_x - 1, map_y, map_x - 2}, "WC", "E", "SV");
        update_map(left, threshold, {map_y + 1, map_x, map_y + 2, map_x}, "WLR", "E", "SV");
        update_map(right, threshold, {map_y - 1, map_x, map_y - 2, map_x}, "WLR", "E", "SV");

        ahead1 = map[map_y][map_x - 1];
        ahead2 = map[map_y][map_x - 2];
        right1 = map[map_y - 1][map_x];
        right2 = map[map_y - 2][map_x];
        left1 = map[map_y + 1][map_x];
        left2 = map[map_y + 2][map_x];
    }
    else if (quadrants[3])
    {
        update_map(center, threshold, {map_y + 1, map_x, map_y + 2, map_x}, "WLR", "E", "SV");
        update_map(left, threshold, {map_y, map_x + 1, map_y, map_x + 2}, "WC", "E", "SV");
        update_map(right, threshold, {map_y, map_x - 1, map_y, map_x - 2}, "WC", "E", "SV");

        ahead1 = map[map_y + 1][map_x];
        ahead2 = map[map_y + 2][map_x];
        right1 = map[map_y][map_x - 1];
        right2 = map[map_y][map_x - 2];
        left1 = map[map_y][map_x + 1];
        left2 = map[map_y][map_x + 2];
    }

    // std::cout << "Ahead1: " << ahead1 << ", Ahead2: " << ahead2 << std::endl;
    // std::cout << "Right1: " << right1 << ", Right2: " << right2 << std::endl;
    // std::cout << "Left1: " << left1 << ", Left2: " << left2 << std::endl;

    return {ahead1, ahead2, right1, right2, left1, left2}; // Use this if you're inside a function
}

void MegaRob::save_map()
{
    auto cell_to_char = [&](const std::string &cell)
    {
        // std::cout << "cell" << cell <<"." << std::endl;
        // std::cout << "cell.compare" << cell.compare(" ") << std::endl;
        if (cell.compare("I") == 0)
            return 'I'; // 50
        if (cell.compare("NV") == 0)
            return 'X'; // 80
        if (cell.compare("E") == 0)
            return 'X'; // 20
        if (cell.compare("SV") == 0)
            return 'X'; // 60
        if (cell.compare("A") == 0)
            return 'X'; // 90
        if (cell.compare("WLR") == 0)
            return '-'; // 40
        if (cell.compare("WC") == 0)
            return '|'; // 30
        if (cell.compare(" ") == 0)
            return ' '; // 10
    };

    map[map_y_init][map_x_init] = "I";

    FILE *file = fopen("map.txt", "w");

    // printf("Saving map\n");
    for (int y = 0; y < 27; ++y)
    {
        for (int x = 0; x < 55; ++x)
        {
            // printf("Cords %d, %d = %c\n", y, x,cell_to_char(map[y][x]));
            fprintf(file, "%c", cell_to_char(map[y][x]));
        }
        fprintf(file, "\n");
    }
    //print_map();
    // printf("Saved\n");
    fclose(file);
}

// Function to find all positions in MAP with a specific value
std::vector<std::pair<int, int>> MegaRob::find_positions(const std::string map[27][55], const std::string &value)
{
    std::vector<std::pair<int, int>> positions;
    for (int y = 0; y < 27; ++y)
    {
        for (int x = 0; x < 55; ++x)
        {
            if (map[y][x].compare(value) == 0)
            {
                positions.emplace_back(y, x);
            }
        }
    }
    return positions;
}

bool should_rotate_right(std::string R2, std::string R, std::string F2)
{
    return R2 == "SV" && R == "E" && F2 != "SV";
}

bool should_rotate_left(std::string L2, std::string L, std::string F2)
{
    return L2 == "SV" && L == "E" && F2 != "SV";
}

// Move function
void MegaRob::Move(std::vector<bool> Z, float *lPow, float *rPow)
{
    ReadSensors();
    double GPS_x = GetX() - GPS_x_init;
    double GPS_y = GetY() - GPS_y_init;

    std::vector<int> GPS_x_current_vet;
    for (int i = -26; i < 28; i += 2)
    {
        GPS_x_current_vet.push_back(i);
    }
    double GPS_x_current_x = (double)GPS_x_current_vet[next_cell_to_explore(GPS_x_current_vet, (int)std::round(GPS_x))];

    std::vector<int> GPS_y_current_vet;
    for (int i = -12; i < 14; i += 2)
    {
        GPS_y_current_vet.push_back(i);
    }

    double GPS_y_current_y = (double)GPS_y_current_vet[next_cell_to_explore(GPS_y_current_vet, (int)std::round(GPS_y))];

    double error_x = 100, error_y = 100;
    double error_x_last = 0, error_y_last = 0;

    const double lin = 0.115;
    const double kp = 0.01;
    const double kd = 0.1;
    const double threshold = 0.225;

    auto update_errors = [&](double &error_x, double &error_y, double GPS_x, double GPS_y, double GPS_x_current_x, double GPS_y_current_y, int direction)
    {
        switch (direction)
        {
        case 0:
            error_x = (GPS_x_current_x + 2.0) - GPS_x;
            error_y = GPS_y_current_y - GPS_y;
            break;
        case 1:
            error_x = GPS_x_current_x - GPS_x;
            error_y = (GPS_y_current_y + 2) - GPS_y;
            break;
        case 2:
            error_x = GPS_x - (GPS_x_current_x - 2);
            error_y = GPS_y - GPS_y_current_y;
            break;
        case 3:
            error_x = GPS_x - GPS_x_current_x;
            error_y = GPS_y - (GPS_y_current_y - 2);
            break;
        }
    };

    auto drive_with_errors = [&](double error_x, double error_y, double &error_x_last, double &error_y_last, int direction)
    {
        float rot = (direction % 2 == 0 ? error_y : error_x) * kp + ((direction % 2 == 0 ? error_y : error_x) - (direction % 2 == 0 ? error_y_last : error_x_last)) / 2 * kd;
        float right_rotation = lin + (direction % 2 == 0 ? rot : -rot);
        float left_rotation = lin - (direction % 2 == 0 ? rot : -rot);
        *lPow = left_rotation;
        *rPow = right_rotation;
        DriveMotors(*lPow, *rPow);
        if (direction % 2 == 0)
        {
            error_y_last = error_y;
        }
        else
        {
            error_x_last = error_x;
        }
    };

    while ((Z[0] && error_x > threshold) || (Z[1] && error_y > threshold) || (Z[2] && error_x > threshold) || (Z[3] && error_y > threshold))
    {
        ReadSensors();

        float x, y;

        if (IsGPSReady())
        {
            x = GetX();
            y = GetY();
        }

        GPS_x = x - GPS_x_init;
        GPS_y = y - GPS_y_init;

        for (int i = 0; i < 4; ++i)
        {
            if (Z[i])
            {
                update_errors(error_x, error_y, GPS_x, GPS_y, GPS_x_current_x, GPS_y_current_y, i);
                drive_with_errors(error_x, error_y, error_x_last, error_y_last, i);
            }
        }
    }
}

// Rotate functions
void MegaRob::rotate_left(float *lPow, float *rPow, int compass)
{
    auto get_true_compass = [&](double compass)
    {
        std::vector<int> compass_vector = {0, 90, -180, -90, 180};
        int true_compass = compass_vector[next_cell_to_explore(compass_vector, compass)];
        return true_compass == 180 ? -180 : true_compass;
    };

    auto calculate_rotation_error = [&](double current_compass, double target_compass)
    {
        double rotation_error = target_compass - current_compass;
        if (rotation_error > 120)
        {
            rotation_error -= 360;
        }
        return rotation_error;
    };

    auto drive_with_rotation_error = [&](double rotation_error)
    {
        const double Kd_angulo = 0.005;
        double rotation = Kd_angulo * rotation_error;
        double right_motor_speed = rotation;
        double left_motor_speed = -rotation;
        *lPow = left_motor_speed;
        *rPow = right_motor_speed;
        DriveMotors(*lPow, *rPow);
    };

    double target_compass = get_true_compass(compass) + 90;
    double rotation_error = 100;

    while (abs(rotation_error) >= 1)
    {
        //printf("Rotating error %d\n", rotation_error);
        ReadSensors();
        int compass_direction;
        if (IsCompassReady())
        {
            compass_direction = GetCompassSensor();
        }

        double current_compass = compass_direction;
        rotation_error = calculate_rotation_error(current_compass, target_compass);
        //printf("Rotating error After %f\n", rotation_error);
        drive_with_rotation_error(rotation_error);
    }
    //printf("Rotated left\n");

}

void MegaRob::rotate_right(float *lPow, float *rPow, int compass)
{
    auto get_true_compass = [&](double compass)
    {
        std::vector<int> compass_vector = {0, 90, -180, -90, 180};
        int true_compass = compass_vector[next_cell_to_explore(compass_vector, compass)];
        return true_compass == 180 ? -180 : true_compass;
    };

    auto calculate_rotation_error = [&](double current_compass, double target_compass)
    {
        double rotation_error = target_compass - current_compass;
        if (rotation_error < -120)
        {
            rotation_error += 360;
        }
        return rotation_error;
    };

    auto drive_with_rotation_error = [&](double rotation_error)
    {
        const double Kd_angulo = 0.005;
        double rotation = Kd_angulo * rotation_error;
        double right_motor_speed = rotation;
        double left_motor_speed = -rotation;
        *lPow = left_motor_speed;
        *rPow = right_motor_speed;
        DriveMotors(*lPow, *rPow);
    };

    double target_compass = get_true_compass(compass) - 90;
    double rotation_error = 100;

    while (abs(rotation_error) >= 1)
    {
        ReadSensors();
        int compass_direction;
        if (IsCompassReady())
        {
            compass_direction = GetCompassSensor();
        }

        double current_compass = compass_direction;
        rotation_error = calculate_rotation_error(current_compass, target_compass);
        drive_with_rotation_error(rotation_error);
    }
}

std::vector<std::string> MegaRob::path_finding(std::string map[27][55], std::vector<int> list_not_visited_x, std::vector<int> list_not_visited_y)
{

    std::vector<std::pair<int, int>> not_visited_positions;
    for (int i = 0; i < list_not_visited_x.size(); i++)
    {
        not_visited_positions.push_back(std::make_pair(list_not_visited_y[i], list_not_visited_x[i]));
    }

    // std::cout << "Not visited position: ";
    // for (const auto &pos : not_visited_positions)
    // {
    //     std::cout << "(" << pos.first << ", " << pos.second << ")";
    // }
    // std::cout << std::endl;

    std::vector<int> linear_movements;
    std::vector<std::vector<std::string>> overall_movements;

    // Dijkstra's algorithm
    // printf("Dijkstra's algorithm\n");

    for (int i = 0; i < not_visited_positions.size(); i++)
    {
        int target_x = not_visited_positions[i].second;
        int target_y = not_visited_positions[i].first;

        std::vector<std::vector<int>> map_for_path;
        for (int i = 0; i < 27; i++)
        {
            std::vector<int> row;
            for (int j = 0; j < 55; j++)
            {
                row.push_back(0);
            }
            map_for_path.push_back(row);
        }

        map_for_path[map_y_current][map_x_current] = 1;

        while (map_for_path[target_y][target_x] == 0)
        {
            std::vector<int> max_values;
            for (int index = 0; index < map_for_path.size(); index++)
            {
                max_values.push_back(*std::max_element(map_for_path[index].begin(), map_for_path[index].end()));
            }
            int max_value = *std::max_element(max_values.begin(), max_values.end());

            // printf("Max_value %d\n", max_value);

            std::vector<std::pair<int, int>> possible_positons;

            for (int y = 0; y < 27; y++)
            {
                for (int x = 0; x < 55; x++)
                {
                    if (map_for_path[y][x] == max_value)
                    {
                        possible_positons.push_back(std::make_pair(y, x));
                    }
                }
            }

            // std::cout << "Possible positions: "; 
            // for (const auto &pos : possible_positons)
            // {
            //     std::cout << "(" << pos.first << ", " << pos.second << ")";
            // }
            // std::cout << std::endl;

            std::vector<std::pair<int, int>> sei_la = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

            for (auto [y, x] : possible_positons)
            {
                if (map[y][x] == "E" || map[y][x] == "NV" || map[y][x] == "A")
                {
                    for (auto [dj, di] : sei_la)
                    {
                        if (map_for_path[y + dj][x + di] == 0 && (map[y + dj][x + di] == "E" || map[y + dj][x + di] == "SV" || map[y + dj][x + di] == "NV"))
                        {
                            map_for_path[y + dj][x + di] = max_value + 1;
                        }
                    }
                }
            }
        }
        // printf("Here\n");

        std::vector<std::string> movements;
        std::vector<int> max_values;
        for (int index = 0; index < map_for_path.size(); index++)
        {
            max_values.push_back(*std::max_element(map_for_path[index].begin(), map_for_path[index].end()));
        }
        int max_value = *std::max_element(max_values.begin(), max_values.end());

        // printf("Max_value %d\n", max_value);
        int current_x = target_x;
        int current_y = target_y;

        int max_value_copy = max_value;

        for (int i = 0; i < max_value_copy; i++)
        {
            // printf("Iteration %d with max %d\n", i,max_value);
            std::vector<std::vector<int>> sides_array = {
                {0, map_for_path[current_y - 1][current_x], 0},
                {map_for_path[current_y][current_x - 1], map_for_path[current_y][current_x], map_for_path[current_y][current_x + 1]},
                {0, map_for_path[current_y + 1][current_x], 0}};

            std::vector<int> j_i;
            std::vector<int> i_i;   
            for (int j = 0; j < sides_array.size(); j++)
            {
                for (int k = 0; k < sides_array[j].size(); k++)
                {
                    if (sides_array[j][k] == max_value - 1)
                    {
                        // printf("AAAA : %d %d\n", j, k);
                        j_i.push_back(j);
                        i_i.push_back(k);
                    }
                }
            }

            // std::cout << "Sides array:\n";
            // for (const auto &row : sides_array)
            // {
            //     for (const auto &elem : row)
            //     {
            //         std::cout << elem << " ";
            //     }
            //     std::cout << "\n";
            // }
            // printf("i_i size: %d\n", i_i.size());

            // std::cout << "Positions in sides array (j_i, i_i): ["; 
            // for(int valor : j_i)
            // {
            //     std::cout << valor << " ";
            // }
            
            // std::cout << "], [";
            // for(int valor : i_i)
            // {
            //     std::cout << valor << " ";
            // }
            // std::cout << "]\n";

            if(j_i.size() == 0 || i_i.size() == 0)
            {
                printf("BREAK \n");
                break;
            }

            std::pair<int, int> ji = std::make_pair(j_i[0], i_i[0]);

            if (ji == std::make_pair(0, 1))
            {
                movements.push_back("DOWN");
                current_y -= 1;
            }
            else if (ji == std::make_pair(1, 0))
            {
                movements.push_back("RIGHT");
                current_x -= 1;
            }
            else if (ji == std::make_pair(1, 2))
            {
                movements.push_back("LEFT");
                current_x += 1;
            }
            else if (ji == std::make_pair(2, 1))
            {
                movements.push_back("UP");
                current_y += 1;
            }

            max_value -= 1;
        }

        // std::cout << "Movements:\n";
        for (const auto &move : movements)
        {
            std::cout << move << "\n";
        }

        std::vector<std::string> filtered_movements;
        for (size_t i = 1; i < movements.size(); i += 2)
        {
            filtered_movements.push_back(movements[i]);
        }
        std::reverse(filtered_movements.begin(), filtered_movements.end());

        overall_movements.push_back(filtered_movements);

        // std::cout << "The agent must follow the next movements: ";
        // for (const auto &move : filtered_movements)
        // {
        //     std::cout << move << " ";
        // }
        // std::cout << std::endl;

        // Equivalent of num_rotations = sum(1 for i in range(1, len(movements)) if movements[i] != movements[i - 1])
        int num_rotations = 0;
        for (size_t i = 1; i < filtered_movements.size(); ++i)
        {
            if (filtered_movements[i] != filtered_movements[i - 1])
            {
                ++num_rotations;
            }
        }
        int real_num_movements = num_rotations + filtered_movements.size();

        linear_movements.push_back(real_num_movements);

    }
    // std::cout << "Linear movements: ";
    // for (const auto &movement : linear_movements)
    // {
    //     std::cout << movement << " ";
    // }
    // std::cout << std::endl;

    // std::cout << "Overall movements: ";
    // for (const auto &movements : overall_movements)
    // {
    //     for (const auto &move : movements)
    //     {
    //         std::cout << move << " ";
    //     }
    //     std::cout << std::endl;
    // }

    int min_index = std::distance(linear_movements.begin(), std::min_element(linear_movements.begin(), linear_movements.end()));
    std::cout << "The closest path is = ";
    for (const auto &move : overall_movements[min_index])
    {
        std::cout << move << " ";
    }
    std::cout << std::endl;
    return overall_movements[min_index];
}

struct pair_hash
{
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> &pair) const
    {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};

void MegaRob::pather(const std::vector<std::string> &next_movements, float *lPow, float *rPow, int compass)
{
    printf("PATHING\n");
    auto rotate_until_facing = [&](std::vector<bool> &quadrant, int direction_idx)
    {
        while (!quadrant[direction_idx])
        {
            // Rotate
            //printf("Rotating\n");
            ReadSensors();
            compass = GetCompassSensor();
            rotate_left(lPow, rPow, compass);
            quadrant = DefineQuadrant(compass);
            //printf("Quadrant: ");
            for (const auto &value : quadrant)
            {
                std::cout << value << " ";
            }
        }
        return quadrant;
    };

    auto move_and_update_quadrant = [&](std::vector<bool> &quadrant, float *lPow, float *rPow)
    {
        Move(quadrant, lPow, rPow);
        return DefineQuadrant(compass);
    };

    auto get_rotation_function = [&](const std::string &prev_direction, const std::string &next_direction)
    {
        using RotationFunction = void (MegaRob::*)(float *, float *, int);
        static const std::unordered_map<std::pair<std::string, std::string>, RotationFunction, pair_hash> rotation_map = {
            {{"LEFT", "DOWN"}, &MegaRob::rotate_left},
            {{"LEFT", "UP"}, &MegaRob::rotate_right},
            {{"RIGHT", "DOWN"}, &MegaRob::rotate_right},
            {{"RIGHT", "UP"}, &MegaRob::rotate_left},
            {{"UP", "LEFT"}, &MegaRob::rotate_left},
            {{"UP", "RIGHT"}, &MegaRob::rotate_right},
            {{"DOWN", "LEFT"}, &MegaRob::rotate_right},
            {{"DOWN", "RIGHT"}, &MegaRob::rotate_left}};
        auto it = rotation_map.find({prev_direction, next_direction});
        return it != rotation_map.end() ? it->second : nullptr;
    };

    std::vector<bool> quadrant = DefineQuadrant(compass);
    std::unordered_map<std::string, int> direction_map = {
        {"LEFT", 2},
        {"RIGHT", 0},
        {"UP", 1},
        {"DOWN", 3}};

    if (direction_map.find(next_movements[0]) != direction_map.end())
    {
        quadrant = rotate_until_facing(quadrant, direction_map[next_movements[0]]);
    }

    quadrant = move_and_update_quadrant(quadrant, lPow, rPow);
    // std::cout << "Next movements: ";
    // for (const auto &movement : next_movements)
    // {
    //     std::cout << movement << " ";
    // }
    // std::cout << std::endl;

    for (size_t i = 1; i < next_movements.size(); ++i)
    {
        if (next_movements[i] == next_movements[i - 1])
        {
            quadrant = move_and_update_quadrant(quadrant, lPow, rPow);
            
        }
        else
        {
            auto rotation_function = get_rotation_function(next_movements[i - 1], next_movements[i]);
            std::cout << "Rotating : "<< rotation_function << std::endl;
            if (rotation_function == &MegaRob::rotate_left)
            {
                ReadSensors();
                compass = GetCompassSensor();
                std::cout << "Rotating left" << std::endl;
                rotate_left(lPow, rPow, compass);
            }
            else if (rotation_function == &MegaRob::rotate_right)
            {
                ReadSensors();
                compass = GetCompassSensor();
                std::cout << "Rotating right" << std::endl;
                rotate_right(lPow, rPow, compass);
            }

            if (rotation_function)
            {
                // std::cout << "Entrou" << std::endl;
                quadrant = DefineQuadrant(compass);
                quadrant = move_and_update_quadrant(quadrant, lPow, rPow);
                printf("Rotating and new quadrant is: ");
                for (const auto &value : quadrant)
                {
                    std::cout << value << " ";
                }
                std::cout << std::endl;
            }
        }
    }
}

void DeterminateAction(float *lPow, float *rPow, MegaRob &mouse)
{
    ReadSensors();

    // * Variables
    int compass_direction = 0;       // Direction it's facing
    float left, right, center, back; // Sensor values
    float x, y;                      // GPS values to get
    float Gps_x, Gps_y;              // GPS values to set

    // * Read sensors
    if (IsObstacleReady(LEFT))
        left = GetObstacleSensor(LEFT);
    if (IsObstacleReady(RIGHT))
        right = GetObstacleSensor(RIGHT);
    if (IsObstacleReady(CENTER))
        center = GetObstacleSensor(CENTER);
    if (IsObstacleReady(OTHER1))
        back = GetObstacleSensor(OTHER1);

    if (IsCompassReady())
    {
        compass_direction = GetCompassSensor();
    }

    if (IsGPSReady())
    {
        x = GetX();
        y = GetY();
        // printf("Before GPS x: %f\n", x);
        // printf("Before GPS y: %f\n", y);
        if (mouse.GPS_x_init == 0 && mouse.GPS_y_init == 0)
        {
            // printf("First time\n");
            mouse.GPS_x_init = GetX();
            mouse.GPS_y_init = GetY();
            mouse.GPS_x_start = GetX();
            mouse.GPS_y_start = GetY(); 
        }
    }

    // std::cout << "Entrou na função" << std::endl;
    Gps_x = x - mouse.GPS_x_init;
    Gps_y = y - mouse.GPS_y_init;

    // printf("Gps Innit x: %f\n", mouse.GPS_x_init);
    // printf("Gps Innit y: %f\n", mouse.GPS_y_init);
    // printf(" AAAA Gps y: %f\n", Gps_y);
    // printf("GPS_INIT \n");

    mouse.GPS_x_current = mouse.get_current_position(Gps_x, -26, 28, 2);
    mouse.GPS_y_current = mouse.get_current_position(Gps_y, -12, 14, 2);

    // printf("Got Current Pos \n");

    mouse.map_x_init = 27;
    mouse.map_y_init = 13;

    mouse.map_x_current = mouse.map_x_init + mouse.GPS_x_current;
    mouse.map_y_current = mouse.map_y_init - mouse.GPS_y_current;

    //    printf("Gps current %f, %f\n", mouse.GPS_x_current, mouse.GPS_y_current);

    // printf("Map Current %d, %d\n", mouse.map_y_current, mouse.map_x_current);

    mouse.map_x_current = mouse.map_x_current ? mouse.map_x_current : mouse.map_x_init;
    mouse.map_y_current = mouse.map_y_current ? mouse.map_y_current : mouse.map_y_init;

    // printf("Got Current Map pos \n");
    // printf("Map innit %d, %d\n", mouse.map_x_init, mouse.map_y_init);

    std::vector<bool> quadrant = mouse.DefineQuadrant(compass_direction);

    // Ele vai buscar o quadrante de forma correta
    // std::cout << "Quadrant : " << quadrant[0] << " " << quadrant[1] << " " << quadrant[2] << " " << quadrant[3] << std::endl;
    mouse.map[mouse.map_y_init][mouse.map_x_init] = "NV"; // Not visited

    std::string F, F2, R, R2, L, L2;
    std::vector<std::string> place_holder = mouse.Mapper(left, right, center, quadrant, mouse.map_y_current, mouse.map_x_current);
    F = place_holder[0];
    F2 = place_holder[1];
    R = place_holder[2];
    R2 = place_holder[3];
    L = place_holder[4];
    L2 = place_holder[5];

    // for(int i =0; i<27; i++){
    //     for(int j =0; j<55; j++){
    //         printf("%s", mouse.map[i][j].c_str());
    //     }
    //     printf("\n");
    // }

    // E isto também
    // std::cout << "F: " << F << " F2: " << F2 << " R: " << R << " R2: " << R2 << " L: " << L << " L2: " << L2 << std::endl;

    mouse.map[mouse.map_y_current][mouse.map_x_current] = "A";

    // printf("Cenas \n");

    std::vector<std::pair<int, int>> positions_not_visited;
    positions_not_visited = mouse.find_positions(mouse.map, "SV");

    // std::cout << "Positions not visited : " << positions_not_visited.size() << std::endl;
    // algo de errado n está certo

    std::vector<int> list_not_visited_y, list_not_visited_x;

    for (auto &position : positions_not_visited)
    {
        list_not_visited_y.push_back(position.first);
        list_not_visited_x.push_back(position.second);
    }

    positions_not_visited = mouse.find_positions(mouse.map, "A");

    std::vector<int> current_position_y, current_position_x;

    for (auto &position : positions_not_visited)
    {
        current_position_y.push_back(position.first);
        current_position_y.push_back(position.second);
    }

    mouse.map[mouse.map_y_current][mouse.map_x_current] = "NV";

    mouse.save_map();
    // printf("positions \n");

    if (should_rotate_right(R2, R, F2))
    {
        mouse.rotate_right(lPow, rPow, compass_direction);
    }
    else if (should_rotate_left(L2, L, F2))
    {
        mouse.rotate_left(lPow, rPow, compass_direction);
    }
    else if (F == "E" && F2 != "NV")
    {
        mouse.Move(quadrant, lPow, rPow);
    }
    else
    {
        if (list_not_visited_y.size() == 0 or list_not_visited_x.size() == 0)
        {
            printf("Aqui??? \n");
            mouse.save_map();
            printf("Finished\n");
            exit(0);
        }

        std::vector<std::string> list_of_movements;
        printf("Path finding : \n");
        list_of_movements = mouse.path_finding(mouse.map, list_not_visited_x, list_not_visited_y);
        printf("Got something : \n");
        mouse.pather(list_of_movements, lPow, rPow, compass_direction);

     
    }
    *lPow = 0.0;
    *rPow = 0.0;
    DriveMotors(*lPow, *rPow);
    mouse.save_map();
}