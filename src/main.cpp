#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"


using namespace std;

// for convenience
using json = nlohmann::json;

// Give me the constant pi
constexpr double pi() { return M_PI; }

// For converting back and forth between radians and degrees.
double deg2rad(double x) { return x * pi() / 180; }

// For converting back and forth between radians and degrees.
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

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

// Define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
void generateTrajectory(int lane, double ref_vel, double car_x, double car_y, double car_yaw, double car_s,
        int prev_size, vector<double> previous_path_x, vector<double> previous_path_y,
        const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
        const vector<double> &map_waypoints_s,
        vector<double> &x_vals, vector<double> &y_vals, const vector<double> &par_wps)
{
    // create a list of widely spaced (x, y) waypoints, evenly spaced at 30m. Later we will interpolate
    // these points with a spline and fill with more points, such that the speed is controlled.
    vector<double> ptsx;
    vector<double> ptsy;

    // reference x, y, yaw states. either we will reference the starting point as i) where the
    // car is, or ii) at the previous path's end point
    double ref_x;
    double ref_y;
    double ref_yaw;

    if(prev_size < 2)
    {
        ref_x = car_x;
        ref_y = car_y;
        ref_yaw = deg2rad(car_yaw);

        // use two points that make the path tangent to the car
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_y);
    }
    else {
        // redefine reference state as previous path's end point
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        // use two points that make the path tangent to the previous path's end point
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    // In Frenet add evenly 30m spaced points ahead of the starting reference
    vector<double> next_wp0 = getXY(car_s+par_wps[0],2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car_s+par_wps[1],2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car_s+par_wps[2],2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

    // Complete the 5 spaced waypoints:
    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    // Transformation to car's system of reference, such that the last point of the previous path's
    // at (0, 0) with a zero angle
    for(int i=0; i < ptsx.size(); i++)
    {
        double shift_x = ptsx[i]-ref_x;
        double shift_y = ptsy[i]-ref_y;

        ptsx[i] = shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw);
        ptsy[i] = shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw);
    }

    // Create a spline
    tk::spline spl;

    // Set (x,y) points to the spline
    spl.set_points(ptsx, ptsy);

    // Start with all of the previous path points (aka whatever is left from the previous iteration plan)
    for(int i=0; i < previous_path_x.size(); i++)
    {
        x_vals.push_back(previous_path_x[i]);
        y_vals.push_back(previous_path_y[i]);
    }

    // Calculate how to break up spline points such that we travel at desired reference velocity:
    double target_x = par_wps[3];
    double target_y = spl(target_x);
    double target_d = sqrt(target_x*target_x + target_y*target_y);

    // Fill out the rest of our path planner [after the previous filling] such that we always output 50
    double x_addon = 0;
    for(int i = 1; i <= 50-previous_path_x.size(); i++)
    {
        double N = (target_d/(0.02*ref_vel/2.24));
        double x_point = x_addon+(target_x)/N;
        double y_point = spl(x_point);

        x_addon = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // rotate back to global coordinates:
        x_point = x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw);
        y_point = x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw);

        x_point += ref_x;
        y_point += ref_y;

        x_vals.push_back(x_point);
        y_vals.push_back(y_point);
    }
}

// Detect proximity of a car ahead of us
void detectCarProximity(int prev_size, int gap, double car_s, double car_v, double end_path_s,
        const vector<vector<double>> &sensor_fusion, int lane,
        bool &ahead_flag, bool &left_flag, bool &right_flag, bool &emerg_flag, double &target_vel)
{
    double car_future_s;
    // the following define parameters of the sensor fusion data, i.e. parameters of other cars
    double d;
    double s;
    double vx;
    double vy;
    double v;

    // what our car s will look like in the future
    if (prev_size > 0)
    {
        car_future_s = end_path_s;
    }
    else
    {
        car_future_s = car_s;
    }

    // Go through sensor fusion data for i cars and take action if there's a car in my lane
    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        d = sensor_fusion[i][6];
        // if another car is in my lane
        if ((d < 2 + 4 * lane + 2) && (d > 2 + 4 * lane - 2))
        {
            vx = sensor_fusion[i][3];
            vy = sensor_fusion[i][4];
            v = sqrt(vx * vx + vy * vy);
            s = sensor_fusion[i][5];

            // using previous path we can project the car so we know what it'll look like in the future
            s += (double) prev_size * 0.02 * v;

            // knowing this, is our car_s close to the other car's s? if in front of us, and gap < X [m]:
            if ((s > car_future_s) && (s - car_future_s < gap))
            {
                ahead_flag = true;
                target_vel = v;
            }
            else if (abs(s - car_future_s) < gap/2.0 && abs(v - car_v) > 15)
            {
                emerg_flag = true;
            }
        }
        // if another car is in my left lane
        // gap+2 adelante  y < 5 por atras
        else if ((d < 2 + 4 * (lane-1) + 2) && (d > 2 + 4 * (lane-1) - 2) && lane > 0)
        {
            vx = sensor_fusion[i][3];
            vy = sensor_fusion[i][4];
            v = sqrt(vx * vx + vy * vy);
            s = sensor_fusion[i][5];

            // using previous path we can project the car so we know what it'll look like in the future
            s += (double) prev_size * 0.02 * v;

            // knowing this, is our car_s close to the other car's s?:
            if (((s - car_future_s) < gap && (car_future_s -s) < 2) ||
                ((s - car_future_s) < gap+4 && (car_future_s -s) < 2 && v < 0.8*car_v) ||
                ((s - car_future_s) < gap   && (car_future_s -s) < 6 && v > 1.2*car_v))
            {
                left_flag = true;
            }
        }
        // if another car is in my right lane
        if ((d < 2 + 4 * (lane+1) + 2) && (d > 2 + 4 * (lane+1) - 2) && lane < 2)
        {
            vx = sensor_fusion[i][3];
            vy = sensor_fusion[i][4];
            v = sqrt(vx * vx + vy * vy);
            s = sensor_fusion[i][5];

            // using previous path we can project the car so we know what it'll look like in the future
            s += (double) prev_size * 0.02 * v;

            // knowing this, is our car_s close to the other car's s?:
            if (((s - car_future_s) < gap && (car_future_s -s) < 2) ||
                ((s - car_future_s) < gap+4 && (car_future_s -s) < 2 && v < 0.8*car_v) ||
                ((s - car_future_s) < gap   && (car_future_s -s) < 6 && v > 1.2*car_v))
            {
                right_flag = true;
            }
        }
    }
}

// Gives a list if possible states for each iteration of the simulator
std::vector<std::string> getPossibleStates(int lane)
{
    if(lane == 0)
    {
        return {"KL", "LCR"};
    }
    else if(lane == 1)
    {
        return {"KL", "LCR", "LCL"};
    }
    else if(lane == 2)
    {
        return {"KL", "LCL"};
    }
}

// trigger the transition function to change the current state:
void getTransition(std::vector<std::string> possible_states, vector<double> &cost_v, vector<double> &cost_a,
        std::string &next_s)
{
    // Find the extreme point, i.e. the trigger for the state transition
    std::vector<double>::iterator result;
    if (cost_a[0] > 0.00000001){
        result = std::min_element(cost_a.begin(), cost_a.end());
        next_s = possible_states[std::distance(cost_a.begin(), result)];
    }
    else {
        result = std::max_element(cost_v.begin(), cost_v.end());
        next_s = possible_states[std::distance(cost_v.begin(), result)];
    }
}

// obtain costs for trajectories associated with each state
void getCosts(bool ahead_flag, std::vector<std::string> possible_states, double ref_vel, int lane,
        double car_x, double car_y, double car_yaw, double car_s, int prev_size,
        const vector<double> &previous_path_x, const vector<double> &previous_path_y,
        const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
        const vector<double> &map_waypoints_s, vector<double> &cost_v, vector<double> &cost_a, std::string &next_s)
{
    int hip_lane;
    if (ahead_flag) {
        for (int i = 0; i < possible_states.size(); i++) {
            if (possible_states[i] == "LCL") {
                hip_lane = lane - 1;
            } else if (possible_states[i] == "LCR") {
                hip_lane = lane + 1;
            } else if (possible_states[i] == "KL") {
                hip_lane = lane;
            }

            // Define the actual points for the trajectories:
            vector<double> next_x;
            vector<double> next_y;

            // Define waypoints for the spline and how it will be broken up
            vector<double> wps = {25, 50, 75, 25};

            generateTrajectory(hip_lane, ref_vel, car_x, car_y, car_yaw, car_s, prev_size,
                               previous_path_x, previous_path_y, map_waypoints_x, map_waypoints_y,
                               map_waypoints_s, next_x, next_y, wps);

            // Compute the first cost function
            double cost = 0.0;
            for (int j = next_x.size() - 2; j < next_x.size() - 1; j++) {
                cost += (2.236936 * distance(next_x[j], next_y[j], next_x[j + 1], next_y[j + 1]) / 0.02) - ref_vel;
            }
            cost_v.push_back(cost * cost);

            // Compute the second cost function
            double cost2;
            cost2 = (2.236936 * distance(next_x[48], next_y[48], next_x[49], next_y[49]) / 0.02) -
                    (2.236936 * distance(next_x[46], next_y[46], next_x[47], next_y[47]) / 0.02);
            cost_a.push_back((cost2 - 0.0) * (cost2 - 0.0));
        }
        getTransition(possible_states, cost_v, cost_a, next_s);
    }
}

// Given the next state, i know what lane to change into
int chooseNextState(const std::string &state, int prev_lane)
{
    int lane;
    if(state == "KL")
    {
        lane = prev_lane;
    }
    else if(state == "LCL")
    {
        lane = prev_lane-1;
    }
    else if(state == "LCR")
    {
        lane = prev_lane+1;
    }
    return lane;
}

// Given the next state, i know what lane to change into
void actionNextState(const std::string &next_state, const bool &flag_ahead, const bool &flag_left,
        const bool &flag_right, const bool &flag_emerg, double &ref_vel, double &target_vel, int &lane)
{
    // accpf*22.3! gives a delta velocity in m/s2 from mph [accpf=0.224 gives a delta v of 5m/s2]
    double accpf =  0.294;

    // following refer to the KL state:
    if (ref_vel < 49.5 && flag_ahead == 0) {
        ref_vel += 1.*accpf;
    }
    else if (ref_vel < 49.5 && flag_ahead == 0 && flag_emerg) {
        ref_vel += 1.8*accpf;
    }
    else if (flag_ahead && flag_emerg) {
        ref_vel -= 1.8*accpf;
    }
    else if (next_state == "KL" && flag_ahead) {
        if(target_vel < 0.9*ref_vel){
            ref_vel -= 1.*accpf;
        } else if (target_vel >= 0.9*ref_vel){
            ref_vel += 1.*accpf;
        }
    }

    // following two refer to the LCL state:
    else if (next_state == "LCL" && flag_ahead && flag_left == 0) {
        lane = chooseNextState(next_state, lane);
        ref_vel += 0.0;
    }
    else if (next_state == "LCL" && flag_ahead && flag_left) {
        if(target_vel < ref_vel){
            ref_vel -= 1.*accpf;
        } else if (target_vel >= ref_vel){
            ref_vel += 1.*accpf;
        }
    }

    // following two refer to the LCR state:
    else if (next_state == "LCR" && flag_ahead && flag_right == 0) {
        lane = chooseNextState(next_state, lane);
        ref_vel += 0.0;
    }
    else if (next_state == "LCR" && flag_ahead && flag_right) {
        if(target_vel < ref_vel){
            ref_vel -= 1.*accpf;
        } else if (target_vel >= ref_vel){
            ref_vel += 1.*accpf;
        }
    }
}


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  // The initial lane
  int lane = 1;
  // Reference velocity
  double ref_vel = 0.0;
  // Target vehicle velocity
  double target_vel = 0.0;

  int frame = 0;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&frame, &lane, &ref_vel, &target_vel,
               &map_waypoints_x,&map_waypoints_y,&map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy]
    (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

            // Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;
            int prev_size = previous_path_x.size();

            // TODO: (done)  - Get a list of possible states
            std::vector<std::string> possible_states;
            possible_states = getPossibleStates(lane);

            // TODO: (done)  - Detect proximity of a car ahead of us given a gap in meters
            bool ahead_flag = false;        // flag that indicates proximity ahead
            bool left_flag = false;         // flag that indicates proximity in the left lane
            bool right_flag = false;        // flag that indicates proximity in the right lane
            bool emerg_flag = false;        // flag that indicates proximity in the right lane
            int gap = 28;                   // vehicle gap in meters

            detectCarProximity(prev_size, gap, car_s, car_speed, end_path_s, sensor_fusion,
                    lane, ahead_flag, left_flag, right_flag, emerg_flag, target_vel);

            // TODO: (done)  - If there's a car ahead of us, generate trajectories for each possible state
            // TODO: (done)  and compute their associated costs
            vector<double> cost_velocity;
            vector<double> cost_acc;
            std::string next_state;

            getCosts(ahead_flag, possible_states, ref_vel, lane, car_x, car_y, car_yaw, car_s, prev_size,
                    previous_path_x, previous_path_y, map_waypoints_x, map_waypoints_y, map_waypoints_s,
                    cost_velocity, cost_acc, next_state);

            // TODO: (done) Take action
            actionNextState(next_state, ahead_flag, left_flag, right_flag, emerg_flag,
                    ref_vel, target_vel, lane);
            next_state = "";

            // TODO: (done) define a path made up of x,y points that the car will visit sequentially every .02s
            // Define the actual points the planner will be using:
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            // Define waypoints for the spline and how it will be broken up
            vector<double> par_wps = {55, 90, 135, 45}; //45, 90, 135, 30

            generateTrajectory(lane, ref_vel, car_x, car_y, car_yaw, car_s, prev_size, previous_path_x,
                    previous_path_y, map_waypoints_x, map_waypoints_y, map_waypoints_s,
                    next_x_vals, next_y_vals, par_wps);

            // Continue
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;
            frame += 1;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
