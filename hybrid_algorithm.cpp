#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

#include "stdlib.h"

const float pi = 3.1415926535897;

//compares state to map to find if there is an obstacle
bool valid_state_check(std::vector<std::vector<float>> map, std::vector<float> state) {
	if (state[0] >= map.size() || state[1] >= map.size()) {
		return false;
	} else if (state[0] < 0 || state[1] < 0) {
		return false;
	} else if (map[state[0]][state[1]] == 1) {
		return false;
	} else {
		return true;
	}
}

//euclidean distance from state to goal
float heuristic_distance(float x, float y, float goal_x, float goal_y) {
	float distance = sqrtf(pow(goal_x - x, 2) + pow(goal_y - y, 2))/sqrt(10);
	return distance;
}

//converts continuous (float) to discrete (int)
std::vector<float> return_discrete(float x, float y) {
	float x_dis = round(x);
	float y_dis = round(y);

	std::vector<float> discrete_coord = { x_dis,y_dis };

	return discrete_coord;
}

//comparison function for the sort function below and compares
//the first item of two vectors
bool compare_vectors(const std::vector<float>& v1, const std::vector<float>& v2) {
	return v1[0] > v2[0];
}

//sort function for a vector of vectors
std::vector<std::vector<float>> sort_vectors(std::vector<std::vector<float>> input) {
	std::sort(input.begin(), input.end(), compare_vectors);
	return input;
}

bool vector_contains(std::vector<std::vector<float>> input, std::vector<float> item) {
	bool FOUND = false;
	for (int i = 0; i < input.size(); i++) {
		if (input[i] == item) {
			FOUND = true;
			break;
		}
	}
	return FOUND;
}

void print_2Dvector(std::vector<std::vector<float>> input) {
	if (input.empty()) {
		std::cout <<"empty \n";
	} else for (int i = 0; i < input.size(); i++) {
		for (int j = 0; j < input[i].size(); j++) {
			std::cout << "'" << input[i][j] << "' ";
		}
		std::cout << std::endl;
	}
}

void print_1Dvector(std::vector<float> input) {
	for (int i = 0; i < input.size(); i++) {
		std::cout << " " << input[i] << "' ";
	}
	std::cout << std::endl;
}
//print path found
void print_path(std::vector<float> end, std::vector<float> start, std::vector<float> heading_changes, float drive_distance, std::vector<std::vector<float>> heading_map) {
	bool FINISHED = false;
	float turn_index = 0;
	std::vector<std::vector<float>> path = {};

	//start point
	float x1 = end[2];
	float y1 = end[3];

	std::vector<float> discrete_coor = return_discrete(x1, y1);
	float theta1 = end[4];
	turn_index = heading_map[discrete_coor[0]][discrete_coor[1]];
	float turn = heading_changes[turn_index];

	std::vector<float> state = { x1, y1, theta1 };

	path.push_back(state);

	float x2 = 0;
	float y2 = 0;
	float theta2 = 0;

	while (FINISHED == false) {
		x2 = x1 - drive_distance * cos(turn);
		y2 = y1 - drive_distance * sin(turn);
		theta2 = theta1 - turn;

		state[0] = x2;
		state[1] = y2;
		state[2] = theta2;

		path.push_back(state);

		std::vector<float> discrete = return_discrete(x2, y2);
		x1 = x2;
		y1 = y2;
		theta1 = theta2;
		turn_index = heading_map[discrete[0]][discrete[1]];
		turn = heading_changes[turn_index];

		if (x1 == start[0] && y1 == start[1]) {
			FINISHED = true;
		}
	}
	std::cout << "the path found is: \n";

	print_2Dvector(path);

}

/* ACTUAL CODE STARTS FROM HERE */
int main() {
	//std::vector<std::vector<float>> map_input =
	//{ {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	//  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	//  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	//  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	//  {0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
	//  {0, 0, 0, 0, 1, 1, 0, 0, 0, 0},
	//  {0, 0, 0, 0, 1, 1, 0, 0, 0, 0},
	//  {0, 0, 0, 0, 1, 1, 0, 0, 0, 0},
	//  {0, 0, 0, 0, 1, 1, 0, 0, 0, 0},
	//  {0, 0, 0, 0, 1, 1, 0, 0, 0, 0} };

	std::vector<std::vector<float>> map_input =
	{ {0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0},
	  {0, 0, 1, 0, 0},
	  {0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0} };

	//create and empty heading change map
	std::vector<std::vector<float>> heading_change_map = map_input;
	for (int i = 0; i < heading_change_map.size(); i++) {
		for (int j = 0; j < heading_change_map[i].size(); j++) {
			heading_change_map[i][j] = 0;
		}
	}

	//define goal states
	//[x, y, theta (in rad)]
	std::vector<float> start_state = { 0, 0, pi/2 };
	std::vector<float> goal_state = { 4, 4, -pi / 2 };

	bool PATH_FOUND = false;
	bool NO_PATH = false;

	//because we do not know the wheel base calculation, the max steering angle
	//has not been placed under consideration
	float drive_distance = sqrt(2);
	float max_steering_angle = pi / 4;
	std::vector<float> heading_changes = { pi / 4, 0, -pi / 4 };
	float expansion = 0;

	//state vector definition: [cost, expansion number, x, y, orientation, orientation from previous node]
	std::vector<float> current_state = { 0, 0, start_state[0], start_state[1], start_state[2], 0 };

	std::vector<std::vector<float>> open = {};
	std::vector<std::vector<float>> closed = {};
	open.push_back(current_state);
	float cost = current_state[0];

	//DEBUG PURPOSES
	std::cout << "current state: ";
	print_1Dvector(current_state);
	std::cout << "open vector:";
	print_2Dvector(open);
	std::cout << "closed vector: ";
	print_2Dvector(closed);

	while (open.empty() == false) {
		//sort the open list to increasing order
		open = sort_vectors(open);
		//examine the lowest cost state
		std::cout << "open vector: \n";
		print_2Dvector(open);
		//std::cout << "closed vector: ";
		//print_2Dvector(closed);
		current_state = open.back();
		open.pop_back();
		//convert continuous coord to discrete
		std::vector<float> dis_coord = return_discrete(current_state[2], current_state[3]);
		//add the examining state to the closed set
		closed.push_back(dis_coord);

		//if  the next point is within the goal state
		if (dis_coord[0] == goal_state[0] && dis_coord[1] == goal_state[1]) {
			std::cout << "path found: ";
			print_path(current_state, start_state, heading_changes, drive_distance, heading_change_map);
			PATH_FOUND = true;
			break;
		}
		for (float i = 0; i < heading_changes.size(); i++) {
			float current_x = current_state[2];
			float current_y = current_state[3];
			float current_theta = current_state[4];

			//calculate next state for this iteration of heading change
			float next_x = current_x + drive_distance * cos(heading_changes[i]);
			float next_y = current_y + drive_distance * sin(heading_changes[i]);
			float next_theta = current_theta + heading_changes[i];

			std::vector<float> discrete_next = return_discrete(next_x, next_y);
			discrete_next.push_back(next_theta);

			//if the state is valid, then calculate heuristics for the path, add to
			//the open vector
			if (valid_state_check(map_input, discrete_next) && vector_contains(closed, discrete_next) == false) {
				expansion++;

				//calculate cost of this new path using heuristics (changeable)
				float heuristic = heuristic_distance(next_x, next_y, goal_state[0], goal_state[1]) + drive_distance + cost;
				std::vector<float> next_state = {heuristic, expansion, next_x, next_y, next_theta, i };
				heading_change_map[discrete_next[0]][discrete_next[1]] = i;
				open.push_back(next_state);
			}
		}
	}
	std::cout << std::endl;
	print_2Dvector(heading_change_map);

	if (PATH_FOUND == false) {
		std::cout << "no path exists";
	}
}
