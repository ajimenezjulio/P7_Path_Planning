#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include <math.h>
#include <string>
#include <vector>

using std::string;
using std::vector;

// Return flags indication the presence of cars on the front, left or right lanes and the speed of the car in front
// `auto` here will take a type of `vector<vector<double>>`, `auto` was keeped due to `sensor_fusion` var in main is of type `auto`
vector<double> predictions_from_sensor_fusion(vector<vector<double>> sensor_fusion, int prev_size, double car_s, int lane) {
  // Flags to indicate if a car is in front, left or right and close to our vehicle
  bool car_left = false;
  bool car_front = false;
  bool car_right = false;
  // Speed of the car in front of us
  double front_speed;

  // Simulator's sensor fusion reports the list of other cars in the road
  for(int i = 0; i < sensor_fusion.size(); i++) {
    // Frenet s of the current car
    double current_car_s = sensor_fusion[i][5];
    
    // Filter the cars that are too far from our car
    if(abs(current_car_s - car_s) < 100) {
      // Frenet d of the current car
      double current_car_d = sensor_fusion[i][6];
      // Get lane of the current car (each lane is 4m wide)
      int current_car_lane = (int) current_car_d / 4;
      
      // Check the speed of actual car (v = sqrt(vx^2 + vy^2))
      double current_car_vx = sensor_fusion[i][3];
      double current_car_vy = sensor_fusion[i][4];
      double current_car_speed = sqrt(pow(current_car_vx, 2) + pow(current_car_vy, 2));

      // Add distance, simulator move from point to point in 20ms
      // If using previous points we can project s value out
      current_car_s = current_car_s + prev_size * 0.02 * current_car_speed;

      // Check if the current car is in my lane, in front and too close (30m tolerance)
      if(current_car_lane == lane && current_car_s > car_s && (current_car_s - car_s) <= 30) {
        car_front = true;
        // Set the speed of the front car to use it later
        front_speed = current_car_speed; // m/s
      } 
      // Check if the current car is on the left lane (either in front or behind and close to our vehicle). Also if we are at the leftmost lane
      else if (current_car_lane == lane - 1 && (current_car_s > car_s && (current_car_s - car_s) <= 30 ||  current_car_s < car_s && (car_s - current_car_s) <= 30) || lane == 0) {
        car_left = true;
      }
      // Check if the current car is on the right lane (either in front or behind and close to our vehicle). Also if we are at the rightmost lane
      else if((current_car_lane == lane + 1) && (current_car_s > car_s && (current_car_s - car_s) <= 30 ||  current_car_s < car_s && (car_s - current_car_s) <= 30) || lane == 2) {
        car_right = true;
      }          
    }
  }

  return {car_left, car_right, car_front, front_speed};
}


// Return array of pairs of possible moves (new lane and speed targets)
vector<std::pair<int, double>> behaviour_planning(bool car_left, bool car_right, bool car_front, int lane, double ref_vel, double front_speed) {
  // Vector containing possible succesor lanes and velocities
  vector<std::pair<int, double>> successor_lanes_ref_vels;

  // Check if there is a car in front of us
  if(car_front) {
    // Push the current lane and a deacceleration rate of 5m/s2 (5m/s = 11.18mph at 50 simulator updates/s -> 11.18/50 ~= 0.224)
    // Updated to match speed of the car in front in 4 steps, before sometimes was not able to deaccelerate in time
    successor_lanes_ref_vels.push_back(std::make_pair(lane, ref_vel - 0.224));
    // If not car on the left push the left lane (taking care of the leftomst lane boundary) and accelerate at a rate of 5m/s2 without exceeding speed limit
    if(!car_left)
      successor_lanes_ref_vels.push_back(std::make_pair(std::max(lane - 1, 0), std::min(49.5, ref_vel + 0.224)));
    // If not car on the right push the right lane (taking care of the rightmost lane boundary) and accelerate at a rate of 5m/s2 without exceeding speed limit
    if(!car_right)
      successor_lanes_ref_vels.push_back(std::make_pair(std::min(lane + 1, 2), std::min(49.5, ref_vel + 0.224)));
  }
  else {
    // If no car in front accelerate at a rate of 5m/s2 up to the speed limit
    successor_lanes_ref_vels.push_back(std::make_pair(lane, std::min(49.5, ref_vel + 0.224)));
  }

  return successor_lanes_ref_vels;
}


// Return cost of a possible move taking into account lane change, acceleration and speed
double calculate_cost(int lane, double car_speed, int successor_lane, double successor_ref_vel) {
  // Cost of a lane change (0, 1/3 or 2/3)
  double cost_lane_change = std::abs(lane - successor_lane) / 3;
  // Cost of acceleration  
  double acc = (successor_ref_vel - car_speed);
  double cost_acc = 0;

  // If acceleration is too big, set it to 1 (big amount)
  if(acc < -9.8 || acc > 9.8)
    cost_acc = 1;
  else
    cost_acc = std::abs(acc) / 9.8; // Small number

  // Cost of speed
  double target_speed = 49.5;
  double cost_speed = 0;

  if(successor_ref_vel < target_speed) {
    cost_speed = (target_speed - successor_ref_vel) / target_speed;
  }
  // Big number, we don't want to exceed speed limit
  else if(successor_ref_vel > target_speed) {
    cost_speed = 5;
  }
  
  return 5 * cost_lane_change + 20 * cost_acc + 20 * cost_speed;
}


vector<vector<double>> choose_next_trajectory(int lane, double ref_vel, double car_x, double car_y, double car_yaw, double car_s, double car_d, 
                                              const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
                                              vector<double> previous_path_x, vector<double> previous_path_y ){

  // Get size of previous list of points
  int prev_size = previous_path_x.size();

  // Create a list of widely spaced (x, y) waypoints, evenly spaced at 30m,
  // later we will interpolate these waypoints with a spline and fill it in
  // with more points that control speed
  vector<double> ptsx;
  vector<double> ptsy;
  
  // Reference x, y, yaw states
  // Either we will reference the starting point as where the car is or at
  // the previous paths end point
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  // Check if previous path of points is empty or almost empty, if it does
  // use the car as starting reference
  if ( prev_size < 2 ) { 
    // Use two points that make the path tangent to the car
    // (go backwards in time (based on its angle)
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);    

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);    

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);

  }
  // Use the previous path's end point as starting reference
  else {
    // Redefine reference state as previous path last point
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];

    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    // Use two points that make the path tangent to the previous
    // path's end point
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y); 
  }

  // Find another 3 points in the future evenly spaced by 30m (30, 60 and 90 meters away from starting point)
  for(int i = 1; i < 4; i++) {
    vector<double> next_wp = getXY(car_s + 30 * i, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    ptsx.push_back(next_wp[0]);
    ptsy.push_back(next_wp[1]);
  }

  // Make a transformation to the local car's coordinates. Make sure that the 
  // last point of previous point is at origin (0,0) and its angle is at 0
  // degrees, this will ease the math later
  for(int i = 0; i  < ptsx.size(); i++) {
    // Shift car reference angle to 0 degrees
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

  // Create a spline
  tk::spline spline;

  // Set the (x, y) points to the spline
  spline.set_points(ptsx, ptsy);

  // Define the actual (x, y) points we will use for the planner
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // Start with all of the previous path points from the last time
  // this helps for a smooth transition, instead of recreating the path
  // every time, we can use previous path points
  for (int i = 0; i < prev_size; i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // Calculate how to break up spline points so that we travel at our
  // desired reference velocity
  double target_x = 30.0; // Number of points
  double target_y = spline(target_x);

  // Distance from the car to the target point (we are linearizing here
  // by using a right triangle model, it works well but a better approach
  // could be to use a better approximation)
  double target_dist = sqrt(target_x * target_x + target_y * target_y);

  double x_add_on = 0;

  // Fill up the rest of our path planner after filling it with previous
  // points, the simulator could already went through 3 points out of 50,
  // so we only need to generate 3, or could went through 40 and we will
  // use the remaining 10 and 40 generated
  for(int i = 1; i < 50 - prev_size; i++) {
    // Calculate the number of points to take in the spline based on the
    // actual velocity (N x 0.02 * vel = d). We visit a point every 0.02s,
    // that's the speed of the simulator. The `rev_vel` is in mi/h and we
    // need it in m/s, thats the reason of the 2.24 factor
    double N = target_dist / (0.02 * ref_vel / 2.24);
    double x_point = x_add_on + target_x / N;
    double y_point = spline(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // Rotate back to global coordinate system
    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  // Create vector object for returning `next_x` and `next_y` values
  vector<vector<double>> next_vals;

  next_vals.push_back(next_x_vals);
  next_vals.push_back(next_y_vals);

  return next_vals;
}

#endif  // PATH_PLANNING_H