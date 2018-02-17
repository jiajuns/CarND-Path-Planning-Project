#include <iostream>
#include <random>
#include <map>
#include <string>
#include <math.h>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "trajectory_generator.h"
#include "spline.h"

using namespace std;
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
    angle = min(2*pi() - angle, angle);

    if(angle > pi()/4)
    {
        closestWaypoint++;
    if (closestWaypoint == maps_x.size())
    {
        closestWaypoint = 0;
    }
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}


TrajectoryGenerator::TrajectoryGenerator(vector<double> map_waypoints_x,\
                                         vector<double> map_waypoints_y,\
                                         vector<double> map_waypoints_s){
    total_points = 50;
    excu_speed = 0;
    this -> map_waypoints_x = map_waypoints_x;
    this -> map_waypoints_y = map_waypoints_y;
    this -> map_waypoints_s = map_waypoints_s;
}

TrajectoryGenerator::~TrajectoryGenerator(){}

double TrajectoryGenerator::find_ref_v(vector<vector<double>> sensor_fusion, double car_s, double car_d)
{

    double ref_v = target_speed;
    for (int i=0; i < sensor_fusion.size(); i++){

        double d = sensor_fusion[i][6];
        if ((d < car_d + 2) && (d > car_d - 2)){

            double checked_speed = sqrt(pow(sensor_fusion[i][3], 2) + pow(sensor_fusion[i][4], 2));
            double checked_s = sensor_fusion[i][5];
            if ((checked_s > car_s) && (checked_s - car_s <= 30)){
                ref_v = checked_speed * 2.24 + (checked_s - car_s - 30) * tau_dist;
            }
        }
    }
    return ref_v;
}

double TrajectoryGenerator::pid_speed(double excu_speed, double ref_v)
{
    double cte = excu_speed - ref_v;
    cte_diff = cte - pre_cte;
    int_cte += cte;
    excu_speed = excu_speed - tau_p*cte - tau_d*cte_diff - tau_i*int_cte;
    pre_cte = cte;

    if (cte*pre_cte < 0){
        int_cte = 0;
    }
    return excu_speed;
}

vector<vector<double>> TrajectoryGenerator::keep_lane_trajectory(double car_x, double car_y, double car_s, double car_d, \
                                                                 double car_yaw, double car_speed, vector<double> previous_path_x, \
                                                                 vector<double> previous_path_y, \
                                                                 vector<vector<double>> sensor_fusion)
{
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);

    vector<double> ptsx;
    vector<double> ptsy;

    int prev_size = previous_path_x.size();

    if(prev_size < 2){
        // use last two points as the beginning of new path in order to make sure the path is tangle
        double prev_car_x = ref_x - cos(deg2rad(car_yaw));
        double prev_car_y = ref_y - sin(deg2rad(car_yaw));

        ptsx.push_back(prev_car_x);
        ptsx.push_back(ref_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(ref_y);
    } else {
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size-2];
        double ref_y_prev = previous_path_y[prev_size-2];

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    // adding way points ahead of the car
    vector<double> next_wp0 = getXY(car_s + 30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car_s + 60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car_s + 90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    // doing polynomial fit to generate optimal track to follow
    vector<double> waypoints_x;
    vector<double> waypoints_y;

    for (int i = 0; i < ptsx.size(); i++) {
        double dx = ptsx[i] - ref_x;
        double dy = ptsy[i] - ref_y;
        waypoints_x.push_back(dx * cos(-deg2rad(car_yaw)) - dy * sin(-deg2rad(car_yaw)));
        waypoints_y.push_back(dx * sin(-deg2rad(car_yaw)) + dy * cos(-deg2rad(car_yaw)));
    }

    double* ptrx = &waypoints_x[0];
    double* ptry = &waypoints_y[0];
    Eigen::Map<Eigen::VectorXd> waypoints_x_eig(ptrx, 6);
    Eigen::Map<Eigen::VectorXd> waypoints_y_eig(ptry, 6);

    for (int i=0; i < previous_path_x.size(); i++){
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    tk::spline s;
    s.set_points(waypoints_x, waypoints_y);

    // calculate how to break up points in polynomial
    double target_x = 30;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x*target_x + target_y*target_y);

    double ref_v = find_ref_v(sensor_fusion, car_s, car_d);
    excu_speed = pid_speed(excu_speed, ref_v);

    double x_add_on = 0;
    for (int i=0; i <total_points- previous_path_x.size(); i++){

        double N = (target_dist/(0.02*excu_speed/2.24));
        double x_point = x_add_on + target_x/N;
        double y_point = s(x_point);

        x_add_on = x_point;
        double x_temp = x_point;
        double y_temp = y_point;

        x_point = (x_temp*cos(deg2rad(car_yaw)) - y_temp*sin(deg2rad(car_yaw)));
        y_point = (x_temp*sin(deg2rad(car_yaw)) + y_temp*cos(deg2rad(car_yaw)));

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }

    vector<vector<double> > result(2, vector<double>(50));
    result[0] = next_x_vals;
    result[1] = next_y_vals;

    return result;
}
