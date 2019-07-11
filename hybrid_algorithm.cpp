#include <ros/ros.h>
#include <iostream>
#include "stdlib.h"
#include "vector.h"

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

std::bool valid_state_check(std::vector<std::vector<int>> map, std::vector<int> state){
  if(map[state[1]][state[2]] == 1){
    return false;
  } else {
    return true;
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv);
  ros::NodeHandle nh;
  
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
  std::vector<int> start_state = [0, 0, 0] //[x, y, z (in rad)]
  std::vector<int> goal_state = [map_input.size() - 1, map_input.size()-1, -pi/2)];
  
  std::bool PATH_FOUND = false;
  std::bool NO_PATH = false;
  
  drive_distance = sqrt(2)+0.1;
  
  while(PATH_FOUND == false and NO_PATH == false){
    std::vector<int> current_state = [0, start_state[0], start_state[1], start_state[2]];
    
    
  
