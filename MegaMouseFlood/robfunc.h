#ifndef MEGAROB_FLOOD_H
#define MEGAROB_FLOOD_H

#include <iostream>
#include <vector>

#include "RobSock.h"

#define RUN 1
#define STOP 2
#define WAIT 3
#define RETURN 4
#define FINISHED 5

#define CENTER_POINT 25
class MegaRob
{
private:
    /* IGNORE; NOTHING IS PRIVATE IN LIFE*/
public:
    /* data */
    // MAP DATA
    std::string map[50][50];
    int map_x_init, map_y_init;
    int map_x_current, map_y_current;

    // GPS DATA
    double GPS_x_current, GPS_y_current;
    double GPS_x_start, GPS_y_start;
    double GPS_x_init, GPS_y_init;

    MegaRob();
    ~MegaRob();
    void set_map(std::string map_value[50][50]);
    void print_map();
    int next_cell_to_explore(const std::vector<int> &list, int N);
    int get_current_position(float GPS, int start, int end, int step);
    std::vector<bool> DefineQuadrant(int compass_direction);
    std::vector<std::string> Mapper(float left, float right, float center, std::vector<bool> quadrants, int map_y, int map_x);
    std::vector<std::pair<int, int>> find_positions(const std::string map[50][50], const std::string &value);
    void save_map();

    void Move(std::vector<bool> Z, float *lPow, float *rPow);
    void rotate_left(float *lPow, float *rPow, int compass);
    void rotate_right(float *lPow, float *rPow, int compass);
    std::vector<std::pair<int,int>> path_finding(std::string map[50][50],std::vector<int> list_not_visited_x, std::vector<int> list_not_visited_y);

};

void DeterminateAction(float *lPow, float *rPow, MegaRob &rob);

#endif
