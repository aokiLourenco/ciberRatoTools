#include <map>
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
    for (int i = 0; i < 50; i++)
    {
        for (int j = 0; j < 50; j++)
        {
            map[i][j] = " ";
        }
    }
}

MegaRob::~MegaRob()
{
    printf("MegaRob destroyed\n");
}

void MegaRob::set_map(std::string map_value[50][50])
{
    // Set map values
    for (int x = 0; x < 50; ++x)
    {
        for (int y = 0; ++y < 50; ++y)
        {
            map[x][y] = map_value[x][y];
        }
    }
}

void MegaRob::print_map()
{
    // Print map values
    FILE *file = fopen("map.txt", "w");
    for (int x = 0; x < 50; ++x)
    {
        for (int y = 0; ++y < 50; ++y)
        {
        
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
    int position = next_cell_to_explore(GPS_current_vet, GPS);
    return GPS_current_vet[position];
}

std::vector<bool> MegaRob::DefineQuadrant(int compass_direction)
{
    ReadSensors();

    if(IsCompassReady())
    {
        compass_direction = GetCompassSensor();
    }

    if (std::abs(compass_direction <= 45))
    {
        return {true, false, false, false};
    }
    else if (compass_direction > 45 && compass_direction <= 135)
    {
        return {false, true, false, false};
    }
    else if (std::abs(compass_direction) > 135)
    {
        return {false, false, true, false};
    }
    else if (compass_direction < -45 && compass_direction >= -135)
    {
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

    printf("Map x: %d, Map y: %d\n", map_x, map_y);

    if (quadrants[0])
    {
        update_map(center, threshold, {map_y, map_x + 1, map_y, map_x + 2}, "WC", "E", "SV");
        update_map(left, threshold, {map_y - 1, map_x, map_y - 2, map_x}, "WLR", "E", "SV");
        update_map(right, threshold, {map_y + 1, map_x, map_y + 2, map_x}, "WLR", "E", "SV");

        ahead1 = map[map_y][map_x + 1];
        ahead2 = map[map_y][map_x + 2];
        printf("Ahead2 %s\n", ahead2.c_str());
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
        //std::cout << "cell" << cell <<"." << std::endl;
        //std::cout << "cell.compare" << cell.compare(" ") << std::endl;
        if (cell.compare("I") == 0)
            return 'I'; // 50
        if (cell.compare("NV") == 0)
            return 'X'; // 80
        if (cell.compare("E") == 0)
            return 'X'; // 20
        if (cell.compare("SV") == 0)
            return 'X'; // 60
        if (cell.compare("A"))
            return 'X'; // 90
        if (cell.compare("WLR"))
            return '-'; // 40
        if (cell.compare("WC"))
            return '|'; // 30
        if (cell.compare(" ") == 0)
            return ' '; // 10
    };

    map[map_y_init][map_x_init] = "I";

    FILE *file = fopen("map.txt", "w");

    printf("Saving map\n");
    for (int y = 0; y < 50; ++y)
    {
        for (int x = 0; x < 50; ++x)
        {
            //printf("Cords %d, %d\n", y, x);
            fprintf(file, "%c", cell_to_char(map[y][x]));
        }
        fprintf(file, "\n");
    }
    //print_map();
    //printf("Saved\n");
    fclose(file);
}

// Function to find all positions in MAP with a specific value
std::vector<std::pair<int, int>> MegaRob::find_positions(const std::string map[50][50], const std::string &value)
{
    std::vector<std::pair<int, int>> positions;
    for (int y = 0; y < 50; ++y)
    {
        for (int x = 0; x < 50; ++x)
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
    double GPS_x = GPS_x_current - GPS_x_init;
    double GPS_y = GPS_y_current - GPS_y_init;

    std::vector<int> GPS_x_current_vet;
    for (int i = -26; i < 28; i += 2)
    {
        GPS_x_current_vet.push_back(i);
    }
    int GPS_x_current_x = GPS_x_current_vet[next_cell_to_explore(GPS_x_current_vet, GPS_x)];

    std::vector<int> GPS_y_current_vet;
    for (int i = -12; i < 14; i += 2)
    {
        GPS_y_current_vet.push_back(i);
    }

    int GPS_y_current_y = GPS_y_current_vet[next_cell_to_explore(GPS_y_current_vet, GPS_y)];

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
        
        float x,y;

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
            //printf("Rot %f \n",rot);
            double right_rotation = lin + rot;
            double left_rotation = lin - rot;
            printf("Right Rotation: %f\n", right_rotation);
            printf("Left Rotation: %f\n", left_rotation);
            *lPow = left_rotation;
            *rPow = right_rotation;
            DriveMotors(*lPow, *rPow);
            printf("Mexeu? \n");
            error_y_last = error_y;
        }

        if (Z[1])
        {
            error_x = GPS_x_current_x - GPS_x;
            error_y = (GPS_y_current_y + 2) - GPS_y;
            int rot = error_x * kp + (error_x - error_x_last) / 2 * kd;
            float right_rotation = lin - rot;
            float left_rotation = lin + rot;
            *lPow = left_rotation;
            *rPow = right_rotation;
            DriveMotors(*lPow, *rPow);
            error_x_last = error_x;
        }

        if (Z[2])
        {
            error_x = GPS_x - (GPS_x_current_x - 2);
            error_y = GPS_y - GPS_y_current_y;
            int rot = error_y * kp + (error_y - error_y_last) / 2 * kd;
            float right_rotation = lin + rot;
            float left_rotation = lin - rot;
            *lPow = left_rotation;
            *rPow = right_rotation;
            DriveMotors(*lPow, *rPow);
            error_y_last = error_y;
        }

        if (Z[3])
        {
            error_x = GPS_x - GPS_x_current_x;
            error_y = GPS_y - (GPS_y_current_y - 2);
            int rot = error_x * kp + (error_x - error_x_last) / 2 * kd;
            float right_rotation = lin - rot;
            float left_rotation = lin + rot;
            *lPow = left_rotation;
            *rPow = right_rotation;
            DriveMotors(*lPow, *rPow);
            error_x_last = error_x;
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
        DriveMotors(left_motor_speed, right_motor_speed);
    };

    double target_compass = get_true_compass(compass) + 90;
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
        DriveMotors(left_motor_speed, right_motor_speed);

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
        double current_compass = compass;
        rotation_error = calculate_rotation_error(current_compass, target_compass);
        drive_with_rotation_error(rotation_error);
    }
}



     
std::vector<std::pair<int,int>> MegaRob::path_finding(std::string map[50][50],std::vector<int> list_not_visited_x, std::vector<int> list_not_visited_y){

    std::vector<std::pair<int,int>> not_visited_positions;
    for(int i = 0; i < list_not_visited_x.size(); i++){
        not_visited_positions.push_back(std::make_pair(list_not_visited_y[i],list_not_visited_x[i]));
    }

    std::vector<int> linear_movements;
    std::vector<int> overall_movements;

    // Dijkstra's algorithm
        std::vector<std::vector<int>> dist(50, std::vector<int>(50, 100));
        std::vector<std::vector<std::pair<int, int>>> prev(50, std::vector<std::pair<int, int>>(50, {-1, -1}));
        std::priority_queue<std::tuple<int, int, int>, std::vector<std::tuple<int, int, int>>, std::greater<>> pq;

        int start_x = map_x_current;
        int start_y = map_y_current;
        dist[start_y][start_x] = 0;
        pq.push({0, start_y, start_x});

        std::vector<std::pair<int, int>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

        while (!pq.empty())
        {
            auto [d, y, x] = pq.top();
            pq.pop();

            if (map[y][x] == "NV")
            {
                std::vector<std::pair<int, int>> path;
                for (std::pair<int, int> at = {y, x}; at != std::make_pair(-1, -1); at = prev[at.first][at.second])
                {
                    path.push_back(at);
                }
                std::reverse(path.begin(), path.end());
                printf("Path found\n");
                //print the path
                // for (auto [y, x] : path)
                // {
                //     printf("(%d, %d) -> ", y, x);
                // }
                return path;
            }

            for (auto [dy, dx] : directions)
            {
                int ny = y + dy;
                int nx = x + dx;
                if (ny >= 0 && ny < 50 && nx >= 0 && nx < 50 && map[ny][nx] != "WC" && map[ny][nx] != "WLR")
                {
                    int new_dist = d + 1;
                    if (new_dist < dist[ny][nx])
                    {
                        dist[ny][nx] = new_dist;
                        prev[ny][nx] = {y, x};
                        pq.push({new_dist, ny, nx});
                    }
                }
            }
        }

        return {}; // Return an empty path if no path is found



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
        if(mouse.GPS_x_init == 0 && mouse.GPS_y_init == 0){
            mouse.GPS_x_init = GetX();
            mouse.GPS_y_init = GetY();
            mouse.GPS_x_start = GetX();
            mouse.GPS_y_start = GetY();
        }
    }

    std::cout << "Entrou na função" << std::endl;
    Gps_x = x - mouse.GPS_x_init;
    Gps_y = y - mouse.GPS_y_init;

    //printf("GPS_INIT \n");

    mouse.GPS_x_current = mouse.get_current_position(Gps_x, -26, 28, 2);
    mouse.GPS_y_current = mouse.get_current_position(Gps_y, -12, 14, 2);

    //printf("Got Current Pos \n");


    mouse.map_x_init = CENTER_POINT; 
    mouse.map_y_init = CENTER_POINT;

    mouse.map_x_current = mouse.map_x_init + mouse.GPS_x_current;
    mouse.map_y_current = mouse.map_y_init - mouse.GPS_y_current;

    mouse.map_x_current = mouse.map_x_current ? mouse.map_x_current : mouse.map_x_init;
    mouse.map_y_current = mouse.map_y_current ? mouse.map_y_current : mouse.map_y_init;

    //printf("Got Current Map pos \n");
    printf("Map innit %d, %d\n", mouse.map_x_init, mouse.map_y_init);

    std::vector<bool> quadrant = mouse.DefineQuadrant(compass_direction);

    // Ele vai buscar o quadrante de forma correta
    //std::cout << "Quadrant : " << quadrant[0] << " " << quadrant[1] << " " << quadrant[2] << " " << quadrant[3] << std::endl;
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
    std::cout << "F: " << F << " F2: " << F2 << " R: " << R << " R2: " << R2 << " L: " << L << " L2: " << L2 << std::endl;
    
    mouse.map[mouse.map_y_current][mouse.map_x_current] = "A";
    
    //printf("Cenas \n");

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

        //print list_of_movements
        for (auto &movement : list_of_movements)
        {
            printf("Movements : x: %d, y: %d\n", movement.first, movement.second);
        }
    
    }
    // *lPow =0.0;
    // *rPow =0.0;
    // DriveMotors(*lPow, *rPow);
    mouse.save_map();
}