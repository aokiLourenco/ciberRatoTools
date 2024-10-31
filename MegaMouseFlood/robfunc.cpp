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

    if (std::abs(compass_direction) <= 45)
    {
        // printf("Se entra aqui... \n");
        return {true, false, false, false};
    }
    else if (compass_direction > 45 && compass_direction <= 135)
    {
        return {false, true, false, false};
    }
    else if (std::abs(compass_direction) >= 135)
    {
        return {false, false, true, false};
    }
    else if (compass_direction <= -45 && compass_direction >= -135)
    {
        // printf("WHY NOT HERE???? \n");
        return {false, false, false, true};
    }
    else
    {
        return {false, false, false, false};
    }
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

    float threshold = 1.15f;

    std::string ahead1, left1, right1;
    std::string ahead2, left2, right2;

    auto update_map = [&](float sensor_value, float threshold, std::vector<int> map_coords, std::string wall_value, std::string empty_value, std::string explore_value)
    {
        if (sensor_value > threshold)
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

    // printf("Map x: %d, Map y: %d Here %s\n", map_x, map_y, map[map_y][map_x].c_str());
    // ahead1 = map[map_y][map_x + 1];
    // printf(" X %d ; Y %d ; Ahead1 %s\n",map_x +1,map_y, ahead1.c_str());
    // ahead2 = map[map_y][map_x + 2];
    // printf(" X %d ; Y %d ; Ahead2 %s\n",map_x +2,map_y, ahead2.c_str());

    if (quadrants[0])
    {
        update_map(center, threshold, {map_y, map_x + 1, map_y, map_x + 2}, "WC", "E", "SV");
        update_map(left, threshold, {map_y - 1, map_x, map_y - 2, map_x}, "WLR", "E", "SV");
        update_map(right, threshold, {map_y + 1, map_x, map_y + 2, map_x}, "WLR", "E", "SV");

        ahead1 = map[map_y][map_x + 1];
        ahead2 = map[map_y][map_x + 2];
        // printf("Ahead2 %s\n", ahead2.c_str());
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
    print_map();
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

    // printf("GPS_x: %f\n", GPS_x);
    // printf("GPS_y: %f\n", GPS_y);
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

    // printf("GPS_x_current_x: %f\n", GPS_x_current_x);
    // printf("GPS_y_current_y: %f\n", GPS_y_current_y);

    while ((Z[0] && error_x > threshold) || (Z[1] && error_y > threshold) || (Z[2] && error_x > threshold) || (Z[3] && error_y > threshold))
    {
        ReadSensors();

        if (IsCompassReady())
        {
            int compass_direction = GetCompassSensor();
        }

        float x, y;

        if (IsGPSReady())
        {
            x = GetX();
            y = GetY();
        }

        GPS_x = x - GPS_x_init;
        GPS_y = y - GPS_y_init;

        if (Z[0])
        {
            error_x = (GPS_x_current_x + 2.0) - GPS_x;
            error_y = GPS_y_current_y - GPS_y;
            int rot = error_y * kp + (error_y - error_y_last) / 2 * kd;
            // printf("Rot %f \n",rot);
            double right_rotation = lin + rot;
            double left_rotation = lin - rot;
            *lPow = left_rotation;
            *rPow = right_rotation;
            DriveMotors(*lPow, *rPow);
            // printf("Error 0 x: %f\n", error_x);
            // printf("Error 0 y: %f\n", error_y);
            // printf("Mexeu? \n");
            error_y_last = error_y;
        }

        if (Z[1])
        {
            error_x = GPS_x_current_x - GPS_x;
            error_y = (GPS_y_current_y + 2) - GPS_y;
            int rot = error_x * kp + (error_x - error_x_last) / 2 * kd;
            double right_rotation = lin - rot;
            double left_rotation = lin + rot;
            *lPow = left_rotation;
            *rPow = right_rotation;
            DriveMotors(*lPow, *rPow);
            // printf("Error 1 x: %f\n", error_x);
            // printf("Error 1 y: %f\n", error_y);
            error_x_last = error_x;
        }

        if (Z[2])
        {
            error_x = GPS_x - (GPS_x_current_x - 2);
            error_y = GPS_y - GPS_y_current_y;
            int rot = error_y * kp + (error_y - error_y_last) / 2 * kd;
            double right_rotation = lin + rot;
            double left_rotation = lin - rot;
            *lPow = left_rotation;
            *rPow = right_rotation;
            DriveMotors(*lPow, *rPow);

            // printf("Error 2 x: %f\n", error_x);
            // printf("Error 2 y: %f\n", error_y);
            error_y_last = error_y;
        }

        if (Z[3])
        {
            error_x = GPS_x - GPS_x_current_x;
            error_y = GPS_y - (GPS_y_current_y - 2);
            int rot = error_x * kp + (error_x - error_x_last) / 2 * kd;
            double right_rotation = lin - rot;
            double left_rotation = lin + rot;
            *lPow = left_rotation;
            *rPow = right_rotation;
            DriveMotors(*lPow, *rPow);
            // printf("Error 3 x: %f\n", error_x);
            // printf("Error 3 y: %f\n", error_y);
            error_x_last = error_x;
        }
        // printf("Error x: %f\n", error_x);
        // printf("Error y: %f\n", error_y);
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
        // printf("Rotating left\n");
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

std::vector<std::pair<int, int>> MegaRob::path_finding(std::string map[27][55], std::vector<int> list_not_visited_x, std::vector<int> list_not_visited_y)
{

    std::vector<std::pair<int, int>> not_visited_positions;
    for (int i = 0; i < list_not_visited_x.size(); i++)
    {
        not_visited_positions.push_back(std::make_pair(list_not_visited_y[i], list_not_visited_x[i]));
    }

    std::vector<int> linear_movements;
    std::vector<int> overall_movements;

    // Dijkstra's algorithm

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

        map[map_y_current][map_x_current] = 1;

        while (map_for_path[target_y][target_x] == 0)
        {
            std::vector<int> max_values;
            for (int index = 0; i < map_for_path.size(); i++)
            {
                max_values.push_back(*std::max_element(map_for_path[index].begin(), map_for_path[index].end()));
            }
            int max_value = *std::max_element(max_values.begin(), max_values.end());

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

        std::vector<int> movements;
        std::vector<int> max_values;
        for (int index = 0; i < map_for_path.size(); i++)
        {
            max_values.push_back(*std::max_element(map_for_path[index].begin(), map_for_path[index].end()));
        }
        int max_value = *std::max_element(max_values.begin(), max_values.end());

        int current_x = target_x;
        int current_y = target_y;

        for (int i = 0; i < max_value-1; i++)
        {
            if (current_x > 0 && map_for_path[current_y][current_x - 1] == max_value - 1)
            {
                movements.push_back(0);
                current_x--;
            }
            else if (current_x < 54 && map_for_path[current_y][current_x + 1] == max_value - 1)
            {
                movements.push_back(1);
                current_x++;
            }
            else if (current_y > 0 && map_for_path[current_y - 1][current_x] == max_value - 1)
            {
                movements.push_back(2);
                current_y--;
            }
            else if (current_y < 26 && map_for_path[current_y + 1][current_x] == max_value - 1)
            {
                movements.push_back(3);
                current_y++;
            }
        }

        // std::vector<std::vector<int>> dist(50, std::vector<int>(50, 100));
        // std::vector<std::vector<std::pair<int, int>>> prev(50, std::vector<std::pair<int, int>>(50, {-1, -1}));
        // std::priority_queue<std::tuple<int, int, int>, std::vector<std::tuple<int, int, int>>, std::greater<>> pq;

        // int start_x = map_x_current;
        // int start_y = map_y_current;
        // dist[start_y][start_x] = 0;
        // pq.push({0, start_y, start_x});

        // std::vector<std::pair<int, int>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

        // while (!pq.empty())
        // {
        //     // printf("A*, more like ASHITYCODE \n");
        //     auto [d, y, x] = pq.top();
        //     pq.pop();

        //     if (map[y][x] == "NV")
        //     {
        //         std::vector<std::pair<int, int>> path;
        //         for (std::pair<int, int> at = {y, x}; at != std::make_pair(-1, -1); at = prev[at.first][at.second])
        //         {
        //             path.push_back(at);
        //         }
        //         std::reverse(path.begin(), path.end());
        //         printf("Path found\n");
        //         // print the path
        //          for (auto [y, x] : path)
        //          {
        //              printf("(%d, %d) -> ", y, x);
        //          }
        //         return path;
        //     }

        //     for (auto [dy, dx] : directions)
        //     {
        //         int ny = y + dy;
        //         int nx = x + dx;
        //         if (ny >= 0 && ny < 50 && nx >= 0 && nx < 50 && map[ny][nx] != "WC" && map[ny][nx] != "WLR")
        //         {
        //             int new_dist = d + 1;
        //             if (new_dist < dist[ny][nx])
        //             {
        //                 dist[ny][nx] = new_dist;
        //                 prev[ny][nx] = {y, x};
        //                 pq.push({new_dist, ny, nx});
        //             }
        //         }
        //     }
        // }
        // printf("No path found\n");
        // return {}; // Return an empty path if no path is found
    }
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
        auto rotate_until_facing = [&](std::vector<bool> &quadrant, int direction_idx)
        {
            while (!quadrant[direction_idx])
            {
                // Rotate
                rotate_left(lPow, rPow, compass);
                quadrant = DefineQuadrant(compass);
            }
            return quadrant;
        };

        auto move_and_update_quadrant = [&](std::vector<bool> &quadrant, float *lPow, float *rPow)
        {
            Move(quadrant, lPow, rPow);
            quadrant = DefineQuadrant(compass);
            return quadrant;
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
        std::cout << "Next movements: ";
        for (const auto &movement : next_movements)
        {
            std::cout << movement << " ";
        }
        std::cout << std::endl;

        for (size_t i = 1; i < next_movements.size(); ++i)
        {
            if (next_movements[i] == next_movements[i - 1])
            {
                quadrant = move_and_update_quadrant(quadrant, lPow, rPow);
            }
            else
            {
                auto rotation_function = get_rotation_function(next_movements[i - 1], next_movements[i]);
                if (rotation_function)
                {
                    quadrant = DefineQuadrant(compass);
                    quadrant = move_and_update_quadrant(quadrant, lPow, rPow);
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
                mouse.GPS_y_start = GetY(); // pila
            }
        }

        // std::cout << "Entrou na função" << std::endl;
        Gps_x = x - mouse.GPS_x_init;
        Gps_y = y - mouse.GPS_y_init;

        // printf(" AAAA Gps x: %f Gps init %f\n", Gps_x, mouse.GPS_x_init);
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

            std::vector<std::pair<int, int>> list_of_movements;
            printf("Path finding : \n");
            list_of_movements = mouse.path_finding(mouse.map, list_not_visited_x, list_not_visited_y);
            printf("Got something : \n");
            mouse.pather(place_holder, lPow, rPow, compass_direction);

            // print list_of_movements
            for (auto &movement : list_of_movements)
            {
                printf("Movements : x: %d, y: %d\n", movement.first, movement.second);
            }
        }
        *lPow = 0.0;
        *rPow = 0.0;
        DriveMotors(*lPow, *rPow);
        mouse.save_map();
    }