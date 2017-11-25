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
#include "vehicle.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
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

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    //cout << "reading data file" << endl;
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

  //impacts default behavior for most states
  int SPEED_LIMIT = 49.5;

  // At each timestep, ego can set acceleration to value between -MAX_ACCEL and MAX_ACCEL
  double MAX_ACCEL = 0.224;

  // An instance of Vehicle that will hold ego's current state
  Vehicle ego;


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,SPEED_LIMIT,MAX_ACCEL,&ego](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;

    int target_lane;
    double ref_vel;

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

        	// ego's localization Data from the simulator
          	ego.x = j[1]["x"];
          	ego.y = j[1]["y"];
          	ego.s = j[1]["s"];
          	ego.d = j[1]["d"];
          	ego.yaw = j[1]["yaw"];
          	ego.speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];

          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];
          	//cout << sensor_fusion << endl;

          	json msgJson;
            vector<double> next_x_vals;
          	vector<double> next_y_vals;


          	/* ******************************/
          	/* Prediction section           */
          	/* ******************************/

            predictions pred;
            int other_vehicle_lane;

            ego.state = "KL";
            ego.lane = Vehicle::get_lane(ego.d);        // Determine current lane of ego
            target_lane = ego.lane;

            int prev_size = previous_path_x.size();

            if(prev_size > 0)
            {
                ego.s = end_path_s;
            }

            // find ref_vel to use
            for(int i = 0; i < sensor_fusion.size(); i++)
            {
                //car is in my lane
                float other_vehicle_d = sensor_fusion[i][6];
                other_vehicle_lane = Vehicle::get_lane(other_vehicle_d);

                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_ego_s = sensor_fusion[i][5];

                //if using previous points, can project s value out
                check_ego_s += ((double)prev_size * 0.02 * check_speed);

                //check if s value greater than car and s gap
                if(check_ego_s > ego.s)
                {
                    //if there is a vehicle ahead in ego's lane within safe distance, flag the need for a lane change or slow down
                    if((other_vehicle_lane == ego.lane) && ((check_ego_s - ego.s) < CURRENT_LANE_VEHICLE_AHEAD_SAFE_DISTANCE))
                    {
                        pred.current_lane_too_close = true;
                        cout << "approaching vehicle in lane, slowing down  " << check_ego_s - ego.s << endl;
                    }
                    //if there is a vehicle ahead in the lane left of ego's lane within safe distance, flag the fact that passing on left is not possible
                    else if((other_vehicle_lane == ego.lane - 1) && ((check_ego_s - ego.s) < TARGET_LANE_VEHICLE_AHEAD_SAFE_DISTANCE))
                    {
                        pred.left_lane_open = false;
                    }
                    //if there is a vehicle ahead in the lane right of ego's lane within safe distance, flag the fact that passing on right is not possible
                    else if((other_vehicle_lane == ego.lane + 1) && ((check_ego_s - ego.s) < TARGET_LANE_VEHICLE_AHEAD_SAFE_DISTANCE))
                    {
                        pred.right_lane_open = false;
                    }
                }
                // if there are vehicles behind ego in adjacent lanes, then flag the fact that passing through those lanes is not possible
                else if((check_ego_s < ego.s) && ((ego.s - check_ego_s) < TARGET_LANE_VEHICLE_BEHIND_SAFE_DISTANCE))
                {
                    if(other_vehicle_lane == ego.lane - 1)
                    {
                        pred.left_lane_open = false;
                    }
                    else if(other_vehicle_lane == ego.lane + 1)
                    {
                        pred.right_lane_open = false;
                    }
                }
            }


            /* ********************************/
          	/* Behaviour Planning section     */
          	/* ********************************/

            // Determine next best state for ego based on current state and predictions
            string next_best_state = ego.choose_next_state(pred);

            if(next_best_state == "LCL")
            {
              target_lane = ego.lane - 1;
              cout << "execute left lane change" << endl;
            }
            else if(next_best_state == "LCR")
            {
              target_lane = ego.lane + 1;
              cout << "execute right lane change" << endl;
            }


          	/* *********************************/
          	/* Trajectory Planning section     */
          	/* *********************************/

            vector<double> ptsx;
            vector<double> ptsy;

            double ref_x = ego.x;
            double ref_y = ego.y;
            double ref_yaw = deg2rad(ego.yaw);

            // Determine first two points that will be part of coordinates used to generate the spline

            // if previous path not available, calculate notional previous point using ego's current position and yaw
            if(prev_size < 2)
            {
                double prev_ego_x = ego.x - cos(ego.yaw);
                double prev_ego_y = ego.y - sin(ego.yaw);

                ptsx.push_back(prev_ego_x);
                ptsx.push_back(ego.x);

                ptsy.push_back(prev_ego_y);
                ptsy.push_back(ego.y);
            }
            // if previous path is available, use point from previous path
            else
            {
                ref_x = previous_path_x[prev_size - 1];
                ref_y = previous_path_y[prev_size - 1];

                double ref_x_prev = previous_path_x[prev_size - 2];
                double ref_y_prev = previous_path_y[prev_size - 2];
                ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                ptsx.push_back(ref_x_prev);
                ptsx.push_back(ref_x);

                ptsy.push_back(ref_y_prev);
                ptsy.push_back(ref_y);
            }

            // Generate 3 more points for the spline generation coordinates by extrapolating at 30m, 60m and 90m
            vector<double> next_wp0 = getXY(ego.s + 30, (2 + 4*target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(ego.s + 60, (2 + 4*target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(ego.s + 90, (2 + 4*target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            // Shift the points to car coordinates
            for(int i = 0; i < ptsx.size(); i++)
            {
                double shift_x = ptsx[i] - ref_x;
                double shift_y = ptsy[i] - ref_y;

                ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
                ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
            }

            // create a spline
            tk::spline s;

            // set (x,y) points to the spline
            s.set_points(ptsx, ptsy);

            //If points available from past path, add them to the planner
            for(int i =0; i < prev_size; i++)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }


            double target_x = 30.0;
            double target_y = s(target_x);

            double target_dist = sqrt((target_x * target_x) + (target_y * target_y));

            double x_add_on = 0;

            // Extract points from the spline that will be the path followed by the car
            for(int i=1; i <= 50 - previous_path_x.size(); i++)
            {
                double N = (target_dist / (0.02 * ref_vel / 2.24));
                double x_point = x_add_on + (target_x) / N;
                double y_point = s(x_point);

                x_add_on = x_point;

                double x_ref = x_point;
                double y_ref = y_point;

                //rotate back to normal
                x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
                y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

                x_point += ref_x;
                y_point += ref_y;

                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);

                if(pred.current_lane_too_close or (ref_vel > SPEED_LIMIT))
                {
                    ref_vel -= MAX_ACCEL;
                }
                else if(ref_vel < SPEED_LIMIT)
                {
                    ref_vel += MAX_ACCEL;
                }
            }

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

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
