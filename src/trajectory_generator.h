#ifndef TRAJECTORYGENERATOR_H
#define TRAJECTORYGENERATOR_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);




class TrajectoryGenerator{

public:

    string state;
    int total_points;
    double target_speed = 49.5;

    /**
     * Constructor
    */
    TrajectoryGenerator();

    /**
     * Destructor
    */
    virtual ~TrajectoryGenerator();

    vector<vector<double>> keep_lane_trajectory(double car_x, double car_y, double car_yaw, \
                                                double car_speed, vector<double> previous_path_x, \
                                                vector<double> previous_path_y);
};
#endif