#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

#include "stdlib.h"

const float pi = 3.1415926535897;

//compares state to map to find if there is an obstacle
bool valid_state_check(std::vector<std::vector<int>> map, std::vector<int> state){
  if(map[state[1]][state[2]] == 1){
    return false;
  } else {
    return true;
  }
}

//euclidean distance from state to goal
double heuristic_distance(int x, int y, std::vector<int> goal){
  double distance = sqrt((goal[0]-x)^2 + (goal[1]-y)^2);
  return distance;
}

//converts continuous (double) to discrete (int)
std::vector<int> return_discrete(double x, double y){
  int x_dis = round(x);
  int y_dis = round(y);

  std::vector<int> discrete_coord = [x,y];

  return discrete_coord;
}

//comparison function for the sort function below and compares
//the first item of two vectors
bool compare_vectors(const std::vector<double>& v1, const std::vector<double>& v2){
  return v1[0] < v2[0];
}

//sort function for a vector of vectors
std::vector<std::vector<double>> sort_vectors(std::vector<std::vector<double>> input){
  std::sort(input.begin(), input.end(), compare_vectors);
  return input;
}

//print path found
void print_path(std::vector<double> end, std::vector<double> start, std::vector<double> heading_changes, double drive_distance, std::vector<std::vector<double>> heading_map){
  bool FINISHED = false;
  std::vector<std::vector<double>> path = {};

  //start point
  double x1 = end[2];
  double y1 = end[3];
  double theta1 = end[4];
  double turn = end[5];

  std::vector<double> state = {x1, y1, theta1, turn};

  path.push_back(state);

  double x2 = 0;
  double y2 = 0;
  double theta2 = 0;

  while(FINISHED == false){
    x2 = x1 - drive_distance*cos(turn);
    y2 = y1 - drive_distance*sin(turn);
    theta2 = theta1 - turn;

    state[0] = x2;
    state[1] = y2;
    state[2] = theta2;

    path.push_back(state);

    std::vector<int> discrete = return_discrete(x2, y2);
    x1 = x2;
    y1 = y2;
    theta1 = theta2;
    turn = heading_map[discrete[0]][discrete[1]];

    if(turn == 0){
      FINISHED = true;
    }
  }
  std::cout << "the path found is: ";
  
}

/* ACTUAL CODE STARTS FROM HERE */
int main(){
  std::vector<std::vector<int>> map_input =
  { {0, 0, 1, 1, 1, 1, 0, 0, 0, 0},
    {0, 0, 1, 1, 1, 1, 0, 0, 0, 0},
    {0, 0, 1, 1, 1, 1, 0, 0, 0, 0},
    {0, 0, 1, 1, 1, 1, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 1, 1, 1, 1, 1, 1, 0, 0},
    {0, 0, 1, 1, 1, 1, 1, 1, 0, 0},
    {0, 0, 1, 1, 1, 1, 1, 1, 0, 0},
    {0, 0, 1, 1, 1, 1, 1, 1, 0, 0} };

  std::vector<std::vector<int>> heading_change_map = map_input;
  //define goal states
  //[x, y, theta (in rad)]
  std::vector<int> start_state = {0, 0, 0};
  std::vector<float> goal_state = {9, 9, -pi/2};
  std::vector<std::vector<int>> path = {};

  bool PATH_FOUND = false;
  bool NO_PATH = false;

  //because we do not know the wheel base calculation, the max steering angle
  //has not been placed under consideration
  double drive_distance = sqrt(2)+0.1;
  double max_steering_angle = pi/4;
  std::vector<double> heading_changes = {pi/4, 0, -pi/4};
  int expansion = 0;

  //state vector definition: [cost, expansion number, x, y, orientation, orientation from previous node]
  std::vector<int> current_state = {0, 0, start_state[0], start_state[1], start_state[2], 0};

  std::vector<std::vector<int>> open = {};
  std::vector<std::vector<int>> closed = {};
  open.push_back(current_state);

  while(open.empty()== false){
    //sort the open list to increasing order
    open = sort(open);
    //examine the lowest cost state
    current_state = open.front();
    //add the examining state to the closed set
    closed.push_back(current_state);
    //convert continuous coord to discrete
    dis_coord = return_discrete(current_state[2], current_state[3]);

    //if  the next point is within the goal state
    if(dis_coord[0] == goal_state[0] && dis_coord[1] == goal_state[1] && current_state[4] == goal_state[2]){
      std::cout << "path found: "
      PATH_FOUND = true;
      break;
    }
    for(int i = 0; i < heading_changes.size();i++ ){
      double current_x = current_state[2];
      double current_y = current_state[3];
      double current_theta = current_state[4];

      //calculate next state for this iteration of heading change
      double next_x = current_x + drive_distance*cos(heading_changes[i]);
      double next_y = current_y + drive_distance*sin(heading_changes[i]);
      double next_theta = current_theta + heading_changes[i];

      std::vector<int> discrete_next = return_discrete(next_x, next_y).push_back(next_theta);

      //if the state is valid, then calculate heuristics for the path, add to
      //the open vector
      if(valid_state_check(map_input, discrete_next) && closed.contains(discrete_next) == false){
        expansion++;

        //calculate cost of this new path using heuristics (changeable)
        double heuristic = heuristic_distance(next_x, next_y, goal_state) + drive_distance;
        std::vector<double> next_state = [heuristic, expansion, next_x, next_y, next_theta, i];

        open.push_back(next_state);
      }
    }
  }

  if(PATH_FOUND == false){
    std::cout << "no path exists";
  }
