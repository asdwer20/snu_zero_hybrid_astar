#include <ros/ros.h>
#include <iostream>
#include <vector>

#include "stdlib.h"


//assume for the sake of the argument that the grid is given as the following map
//[ [0, 0, 1, 1, 1, 1, 0, 0, 0, 0],
//  [0, 0, 1, 1, 1, 1, 0, 0, 0, 0],
//  [0, 0, 1, 1, 1, 1, 0, 0, 0, 0],
//  [0, 0, 1, 1, 1, 1, 0, 0, 0, 0],
//  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
//  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
//  [0, 0, 1, 1, 1, 1, 1, 1, 0, 0],
//  [0, 0, 1, 1, 1, 1, 1, 1, 0, 0],
//  [0, 0, 1, 1, 1, 1, 1, 1, 0, 0],
//  [0, 0, 1, 1, 1, 1, 1, 1, 0, 0] ]

//compares state to map to find if there is an obstacle
std::bool valid_state_check(std::vector<std::vector<int>> map, std::vector<int> state){
  if(map[state[1]][state[2]] == 1){
    return false;
  } else {
    return true;
  }
}

//euclidean distance from state to goal
float heuristic_distance(int x, int y, vector<int> goal){
  distance = sqrt((goal[0]-x)^2 + (goal[1]-y)^2);
  return distance;
}

//converts continuous (float) to discrete (int)
std::vector<int> return_discrete(float x, float y){
  x_dis = round(x);
  y_dis = roound(y);

  std::vector<int> discrete_coord = [x,y];

  return discrete_coord;
}

//comparison function for the sort function below and compares
//the first item of two vectors
std::bool compare_vectors(const vector<float>& v1, const vector<float>& v2){
  return v1[0] < v2[0];
}

//sort function for a vector of vectors
std::vector<std::vector<float>> sort(std::vector<std::vector<float>> input){
  std::sort(input.begin(), input.end(), compare_vectors);
  return input;
}

//print path found
void print_path(std::vector<float> end, std::vector<float> start, std::vector<float> heading_changes, float drive_distance){
  bool FINISHED = false;
  std::vector<std::vector<float>> path = [];
  std::vector<float> state = end;
  path.push_back(end);

  //start point
  float x1 = end[2];
  float y1 = end[3];
  float theta1 = end[4];
  float turn = end[5];

  while(FINISHED == false){
    
  }
}
int main(){
  std::vector<std::vector<int>> map_input =
  [ [0, 0, 1, 1, 1, 1, 0, 0, 0, 0],
    [0, 0, 1, 1, 1, 1, 0, 0, 0, 0],
    [0, 0, 1, 1, 1, 1, 0, 0, 0, 0],
    [0, 0, 1, 1, 1, 1, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 1, 1, 1, 1, 1, 0, 0],
    [0, 0, 1, 1, 1, 1, 1, 1, 0, 0],
    [0, 0, 1, 1, 1, 1, 1, 1, 0, 0],
    [0, 0, 1, 1, 1, 1, 1, 1, 0, 0] ];

  //define goal states
  //[x, y, theta (in rad)]
  std::vector<int> start_state = [0, 0, 0];
  std::vector<int> goal_state = [map_input.size() - 1, map_input.size()-1, -pi/2];
  std::vector<std::vector<int>> path = [];

  std::bool PATH_FOUND = false;
  std::bool NO_PATH = false;

  //because we do not know the wheel base calculation, the max steering angle
  //has not been placed under consideration
  float = drive_distance = sqrt(2)+0.1;
  float = max_steering_angle = pi/4;
  std::vector<float> heading_changes = [pi/4, 0, -pi/4];
  int expansion = 0;

  //state vector definition: [cost, expansion number, x, y, orientation, orientation from previous node]
  std::vector<float> current_state = [0, 0, start_state[0], start_state[1], start_state[2], 0];

  std::vector<std::vector<float>> open = [];
  std::vector<std::vector<float>> closed = [];
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
      float current_x = current_state[2];
      float current_y = current_state[3];
      float current_theta = current_state[4];

      //calculate next state for this iteration of heading change
      float next_x = current_x + drive_distance*cos(heading_changes[i]);
      float next_y = current_y + drive_distance*sin(heading_changes[i]);
      float next_theta = current_theta + heading_changes[i];

      discrete_next = return_discrete(next_x, next_y).push_back(next_theta);

      //if the state is valid, then calculate heuristics for the path, add to
      //the open vector
      if(valid_state_check(map_input, discrete_next) && closed.contains(discrete_next) == false){
        expansion++;

        //calculate cost of this new path using heuristics (changeable)
        float heuristic = heuristic_distance(next_x, next_y, goal_state) + drive_distance;
        std::vector<float> next_state = [heuristic, expansion, next_x, next_y, next_theta, i];

        open.push_back(next_state);
      }
    }
  }

  if(PATH_FOUND == false){
    std::cout << "no path exists";
  }
