#ifndef TRAJECTORYGENERATOR_H
#define TRAJECTORYGENERATOR_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

double polyeval(Eigen::VectorXd coeffs, double x);

class TrajectoryGenerator{

public:

    string state;
    int total_points;
    double excu_speed;
    double target_speed = 48.0;
    int lane = 1; // start with lane 1

    double tau_p = 0.015;
    double tau_d = 0.08;
    double tau_i = 0.000002;
    double tau_dist = 0.05;

    double cte_diff = 0;
    double int_cte = 0;
    double pre_cte = 0;

    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;

    /**
     * Constructor
    */
    TrajectoryGenerator(vector<double> map_waypoints_x, \
                        vector<double> map_waypoints_y, \
                        vector<double> map_waypoints_s);

    /**
     * Destructor
    */
    virtual ~TrajectoryGenerator();

    double find_ref_v(vector<vector<double>> sensor_fusion, double car_s, double car_d);

    double pid_speed(double excu_speed, double ref_v);

    vector<vector<double>> path_planning(double car_x, double car_y, double car_s, double car_d, \
                                         double car_yaw, double car_speed, vector<double> previous_path_x, \
                                         vector<double> previous_path_y, \
                                         vector<vector<double>> sensor_fusion);

    vector<vector<double>> keep_lane_trajectory(double car_x, double car_y, double car_s, double car_d,\
                                                double car_yaw, double car_speed, double ref_v, \
                                                vector<double> previous_path_x,vector<double> previous_path_y, \
                                                vector<vector<double>> sensor_fusion);
};
#endif