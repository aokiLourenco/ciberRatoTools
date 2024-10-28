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


#define CELLROWS 7
#define CELLCOLS 14

using namespace std;

class MyRob : public CRobLinkAngs {
public:
    MyRob(string rob_name, int rob_id, vector<double> angles, string host) {
        // Initialize the base class
        // CRobLinkAngs(rob_name, rob_id, angles, host);
        lap_time = 0;
        ReadSensors();

        MAP.resize(27, vector<int>(55, 10));
        MAP_x_current = MAP_y_current = vector<int>();

        GPS_x_initial = measures.x;
        GPS_y_initial = measures.y;
        GPS_x_start = measures.x;
        GPS_y_start = measures.y;
    }

    void setMap(vector<vector<char>> labMap) {
        this->labMap = labMap;
    }

    void printMap() {
        for (auto it = labMap.rbegin(); it != labMap.rend(); ++it) {
            for (auto l : *it) {
                cout << l;
            }
            cout << endl;
        }
    }

    void run() {
        if (status != 0) {
            cout << "Connection refused or error" << endl;
            exit(1);
        }

        string state = "stop";
        string stopped_state = "run";

        while (true) {
            ReadSensors();

            measures.gpsReady = true;
            measures.gpsDirReady = true;

            if (measures.endLed) {
                cout << robName << " exiting" << endl;
                exit(0);
            }

            if (state == "stop" && measures.start) {
                state = stopped_state;
            }

            if (state != "stop" && measures.stop) {
                stopped_state = state;
                state = "stop";
            }

            if (state == "run") {
                if (measures.visitingLed) {
                    state = "wait";
                }
                if (measures.ground == 0) {
                    SetVisitingLed(true);
                }
                wander();
            } else if (state == "wait") {
                SetReturningLed(true);
                if (measures.visitingLed) {
                    SetVisitingLed(false);
                }
                if (measures.returningLed) {
                    state = "return";
                }
                DriveMotors(0.0, 0.0);
            } else if (state == "return") {
                if (measures.visitingLed) {
                    SetVisitingLed(false);
                }
                if (measures.returningLed) {
                    SetReturningLed(false);
                }
                wander();
            }
        }
    }

    vector<bool> DefineQuadrant() {
        ReadSensors();
        double compass = measures.compass;

        vector<bool> Quadrant(4, false);

        if (abs(compass) <= 45) {
            Quadrant[0] = true;
        } else if (compass > 45 && compass <= 135) {
            Quadrant[1] = true;
        } else if (abs(compass) >= 135) {
            Quadrant[2] = true;
        } else if (compass <= -45 && compass >= -135) {
            Quadrant[3] = true;
        }

        return Quadrant;
    }

    tuple<int, int, int, int, int, int> Mapper(vector<bool> flags, int y, int x) {
        const int CENTER = 0;
        const int LEFT = 1;
        const int RIGHT = 2;
        const double threshold = 1.15;

        ReadSensors();

        double center_sensor = measures.irSensor[CENTER];
        double left_sensor = measures.irSensor[LEFT];
        double right_sensor = measures.irSensor[RIGHT];

        auto update_map = [&](double sensor_value, double threshold, tuple<int, int, int, int> map_coords, int wall_value, int empty_value, int explore_value) {
            if (sensor_value >= threshold) {
                MAP[get<0>(map_coords)][get<1>(map_coords)] = wall_value;
            } else {
                MAP[get<0>(map_coords)][get<1>(map_coords)] = empty_value;
                if (MAP[get<2>(map_coords)][get<3>(map_coords)] != 80) {
                    MAP[get<2>(map_coords)][get<3>(map_coords)] = explore_value;
                }
            }
        };

        int ahead, ahead2, right, right2, left, left2;

        if (flags[0]) {
            update_map(center_sensor, threshold, make_tuple(y, x + 1, y, x + 2), 30, 20, 60);
            update_map(left_sensor, threshold, make_tuple(y - 1, x, y - 2, x), 40, 20, 60);
            update_map(right_sensor, threshold, make_tuple(y + 1, x, y + 2, x), 40, 20, 60);

            ahead = MAP[y][x + 1];
            ahead2 = MAP[y][x + 2];
            right = MAP[y + 1][x];
            right2 = MAP[y + 2][x];
            left = MAP[y - 1][x];
            left2 = MAP[y - 2][x];
        } else if (flags[1]) {
            update_map(center_sensor, threshold, make_tuple(y - 1, x, y - 2, x), 40, 20, 60);
            update_map(left_sensor, threshold, make_tuple(y, x - 1, y, x - 2), 30, 20, 60);
            update_map(right_sensor, threshold, make_tuple(y, x + 1, y, x + 2), 30, 20, 60);

            ahead = MAP[y - 1][x];
            ahead2 = MAP[y - 2][x];
            right = MAP[y][x + 1];
            right2 = MAP[y][x + 2];
            left = MAP[y][x - 1];
            left2 = MAP[y][x - 2];
        } else if (flags[2]) {
            update_map(center_sensor, threshold, make_tuple(y, x - 1, y, x - 2), 30, 20, 60);
            update_map(left_sensor, threshold, make_tuple(y + 1, x, y + 2, x), 40, 20, 60);
            update_map(right_sensor, threshold, make_tuple(y - 1, x, y - 2, x), 40, 20, 60);

            ahead = MAP[y][x - 1];
            ahead2 = MAP[y][x - 2];
            right = MAP[y - 1][x];
            right2 = MAP[y - 2][x];
            left = MAP[y + 1][x];
            left2 = MAP[y + 2][x];
        } else if (flags[3]) {
            update_map(center_sensor, threshold, make_tuple(y + 1, x, y + 2, x), 40, 20, 60);
            update_map(left_sensor, threshold, make_tuple(y, x + 1, y, x + 2), 30, 20, 60);
            update_map(right_sensor, threshold, make_tuple(y, x - 1, y, x - 2), 30, 20, 60);

            ahead = MAP[y + 1][x];
            ahead2 = MAP[y + 2][x];
            right = MAP[y][x - 1];
            right2 = MAP[y][x - 2];
            left = MAP[y][x + 1];
            left2 = MAP[y][x + 2];
        }

        return make_tuple(ahead, ahead2, right, right2, left, left2);
    }

    void Move(vector<bool> Z) {
        ReadSensors();

        double GPS_x = measures.x - GPS_x_initial;
        double GPS_y = measures.y - GPS_y_initial;

        vector<int> GPS_x_current_vet = get_current_position_vector(-26, 28, 2);
        GPS_x_current = GPS_x_current_vet[next_cell_to_explore(GPS_x_current_vet, GPS_x)];

        vector<int> GPS_y_current_vet = get_current_position_vector(-12, 14, 2);
        GPS_y_current = GPS_y_current_vet[next_cell_to_explore(GPS_y_current_vet, GPS_y)];

        double error_x = 100, error_y = 100;
        double error_x_last = 0, error_y_last = 0;

        const double lin = 0.115;
        const double kp = 0.01;
        const double kd = 0.1;
        const double threshold = 0.225;

        while ((Z[0] && error_x > threshold) || (Z[1] && error_y > threshold) || (Z[2] && error_x > threshold) || (Z[3] && error_y > threshold)) {
            ReadSensors();
            GPS_x = measures.x - GPS_x_initial;
            GPS_y = measures.y - GPS_y_initial;

            if (Z[0]) {
                error_x = (GPS_x_current + 2) - GPS_x;
                error_y = GPS_y_current - GPS_y;
                double rot = error_y * kp + (error_y - error_y_last) / 2 * kd;
                double right_rotation = lin + rot;
                double left_rotation = lin - rot;
                DriveMotors(left_rotation, right_rotation);
                error_y_last = error_y;
            }

            if (Z[1]) {
                error_x = GPS_x_current - GPS_x;
                error_y = (GPS_y_current + 2) - GPS_y;
                double rot = error_x * kp + (error_x - error_x_last) / 2 * kd;
                double right_rotation = lin - rot;
                double left_rotation = lin + rot;
                DriveMotors(left_rotation, right_rotation);
                error_x_last = error_x;
            }

            if (Z[2]) {
                error_x = GPS_x - (GPS_x_current - 2);
                error_y = GPS_y - GPS_y_current;
                double rot = error_y * kp + (error_y - error_y_last) / 2 * kd;
                double right_rotation = lin + rot;
                double left_rotation = lin - rot;
                DriveMotors(left_rotation, right_rotation);
                error_y_last = error_y;
            }

            if (Z[3]) {
                error_x = GPS_x - GPS_x_current;
                error_y = GPS_y - (GPS_y_current - 2);
                double rot = error_x * kp + (error_x - error_x_last) / 2 * kd;
                double right_rotation = lin - rot;
                double left_rotation = lin + rot;
                DriveMotors(left_rotation, right_rotation);
                error_x_last = error_x;
            }
        }
    }

    void Rotate_90_Left() {
        auto get_true_compass = [&](double compass) {
            vector<int> compass_vector = {0, 90, -180, -90, 180};
            int true_compass = compass_vector[next_cell_to_explore(compass_vector, compass)];
            return true_compass == 180 ? -180 : true_compass;
        };

        auto calculate_rotation_error = [&](double current_compass, double target_compass) {
            double rotation_error = target_compass - current_compass;
            if (rotation_error > 120) {
                rotation_error -= 360;
            }
            return rotation_error;
        };

        auto drive_with_rotation_error = [&](double rotation_error) {
            const double Kd_angulo = 0.005;
            double rotation = Kd_angulo * rotation_error;
            double right_motor_speed = rotation;
            double left_motor_speed = -rotation;
            DriveMotors(left_motor_speed, right_motor_speed);
        };

        double target_compass = get_true_compass(measures.compass) + 90;
        double rotation_error = 100;

        while (abs(rotation_error) >= 1) {
            ReadSensors();
            double current_compass = measures.compass;
            rotation_error = calculate_rotation_error(current_compass, target_compass);
            drive_with_rotation_error(rotation_error);
        }
    }

    void Rotate_90_Right() {
        auto get_true_compass = [&](double compass) {
            vector<int> compass_vector = {0, 90, -180, -90, 180};
            int true_compass = compass_vector[next_cell_to_explore(compass_vector, compass)];
            return true_compass == 180 ? -180 : true_compass;
        };

        auto calculate_rotation_error = [&](double current_compass, double target_compass) {
            double rotation_error = target_compass - current_compass;
            if (rotation_error < -120) {
                rotation_error += 360;
            }
            return rotation_error;
        };

        auto drive_with_rotation_error = [&](double rotation_error) {
            const double Kd_angulo = 0.005;
            double rotation = Kd_angulo * rotation_error;
            double right_motor_speed = rotation;
            double left_motor_speed = -rotation;
            DriveMotors(left_motor_speed, right_motor_speed);
        };

        double target_compass = get_true_compass(measures.compass) - 90;
        double rotation_error = 100;

        while (abs(rotation_error) >= 1) {
            ReadSensors();
            double current_compass = measures.compass;
            rotation_error = calculate_rotation_error(current_compass, target_compass);
            drive_with_rotation_error(rotation_error);
        }
    }

    vector<string> path_finding(vector<vector<int>> map_numpy, vector<int> not_visited_x, vector<int> not_visited_y, int current_x, int current_y) {
        try {
            vector<pair<int, int>> not_visited_positions;
            for (size_t i = 0; i < not_visited_x.size(); ++i) {
                not_visited_positions.emplace_back(not_visited_y[i], not_visited_x[i]);
            }

            vector<int> linear_movements;
            vector<vector<string>> overall_movements;

            for (auto& target : not_visited_positions) {
                vector<vector<int>> map_for_path(map_numpy.size(), vector<int>(map_numpy[0].size(), 0));
                map_for_path[current_y][current_x] = 1;

                while (map_for_path[target.first][target.second] == 0) {
                    int max_value = *max_element(map_for_path.begin(), map_for_path.end(), [](const vector<int>& a, const vector<int>& b) {
                        return *max_element(a.begin(), a.end()) < *max_element(b.begin(), b.end());
                    });

                    vector<pair<int, int>> possible_positions;
                    for (size_t j = 0; j < map_for_path.size(); ++j) {
                        for (size_t i = 0; i < map_for_path[j].size(); ++i) {
                            if (map_for_path[j][i] == max_value) {
                                possible_positions.emplace_back(j, i);
                            }
                        }
                    }

                    for (auto& pos : possible_positions) {
                        if (map_numpy[pos.first][pos.second] == 20 || map_numpy[pos.first][pos.second] == 80 || map_numpy[pos.first][pos.second] == 90) {
                            for (auto& dir : vector<pair<int, int>>{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
                                if (map_for_path[pos.first + dir.first][pos.second + dir.second] == 0 && (map_numpy[pos.first + dir.first][pos.second + dir.second] == 20 || map_numpy[pos.first + dir.first][pos.second + dir.second] == 60 || map_numpy[pos.first + dir.first][pos.second + dir.second] == 80)) {
                                    map_for_path[pos.first + dir.first][pos.second + dir.second] = max_value + 1;
                                }
                            }
                        }
                    }
                }

                vector<string> movements;
                int max_value = *max_element(map_for_path.begin(), map_for_path.end(), [](const vector<int>& a, const vector<int>& b) {
                    return *max_element(a.begin(), a.end()) < *max_element(b.begin(), b.end());
                });

                current_y = target.first;
                current_x = target.second;

                for (int i = 0; i < max_value - 1; ++i) {
                    vector<vector<int>> sides_array = {
                        {0, map_for_path[current_y - 1][current_x], 0},
                        {map_for_path[current_y][current_x - 1], map_for_path[current_y][current_x], map_for_path[current_y][current_x + 1]},
                        {0, map_for_path[current_y + 1][current_x], 0}
                    };

                    auto it = find_if(sides_array.begin(), sides_array.end(), [&](const vector<int>& row) {
                        return find(row.begin(), row.end(), max_value - 1) != row.end();
                    });

                    if (it == sides_array.end()) {
                        break;
                    }

                    int ji = distance(sides_array.begin(), it);
                    int ii = distance(it->begin(), find(it->begin(), it->end(), max_value - 1));

                    if (ji == 0 && ii == 1) {
                        movements.push_back("DOWN");
                        current_y -= 1;
                    } else if (ji == 1 && ii == 0) {
                        movements.push_back("RIGHT");
                        current_x -= 1;
                    } else if (ji == 1 && ii == 2) {
                        movements.push_back("LEFT");
                        current_x += 1;
                    } else if (ji == 2 && ii == 1) {
                        movements.push_back("UP");
                        current_y += 1;
                    }

                    max_value -= 1;
                }

                movements = vector<string>(movements.begin() + 1, movements.end());
                reverse(movements.begin(), movements.end());
                overall_movements.push_back(movements);

                int num_rotations = count_if(movements.begin() + 1, movements.begin(), movements.end(), [](const string& move) {
                    return move == "LEFT" || move == "RIGHT";
                });

                linear_movements.push_back(movements.size() + num_rotations);
            }

            auto min_it = min_element(linear_movements.begin(), linear_movements.end());
            int min_index = distance(linear_movements.begin(), min_it);

            return overall_movements[min_index];
        } catch (const exception& e) {
            cerr << "Error in path_finding: " << e.what() << endl;
            return {};
        }
    }

private:
    vector<int> get_current_position_vector(int start, int end, int step) {
        vector<int> vec;
        for (int i = start; i <= end; i += step) {
            vec.push_back(i);
        }
        return vec;
    }

    int next_cell_to_explore(const vector<int>& vec, double value) {
        auto it = lower_bound(vec.begin(), vec.end(), value);
        if (it == vec.end()) {
            return vec.size() - 1;
        }
        return distance(vec.begin(), it);
    }

    struct Measures {
        double x, y, compass;
        bool gpsReady, gpsDirReady, endLed, start, stop, visitingLed, returningLed;
        int ground;
        vector<double> irSensor;
    } measures;

    vector<vector<int>> MAP;
    vector<int> MAP_x_current, MAP_y_current;
    double GPS_x_initial, GPS_y_initial, GPS_x_start, GPS_y_start;
    string robName;
    int status;
    double lap_time;
    vector<vector<char>> labMap;
};

int main() {
    // Example usage
    vector<double> angles = {0.0, 90.0, -90.0};
    MyRob robot("MyRobot", 1, angles, "localhost");

    vector<vector<char>> labMap = {
        {'#', '#', '#', '#', '#', '#', '#'},
        {'#', ' ', ' ', ' ', ' ', ' ', '#'},
        {'#', ' ', '#', '#', '#', ' ', '#'},
        {'#', ' ', '#', ' ', '#', ' ', '#'},
        {'#', ' ', '#', ' ', '#', ' ', '#'},
        {'#', ' ', ' ', ' ', ' ', ' ', '#'},
        {'#', '#', '#', '#', '#', '#', '#'}
    };

    robot.setMap(labMap);
    robot.printMap();
    robot.run();

    return 0;
}