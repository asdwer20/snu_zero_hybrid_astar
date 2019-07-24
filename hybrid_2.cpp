// hybrid_2.cpp

// libraries
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

//Euclidean distance from 'state' to 'goal'
float euclidean_dist(float x, float y, float goal_x, float goal_y) {
	float distance = sqrtf(pow(goal_x - x, 2) + pow(goal_y - y, 2))/sqrt(10);
	return distance;
}

//converts continuous (float) to discrete (int)
std::vector<float> lattice_reg(float x, float y) {
	float x_dis = round(x);
	float y_dis = round(y);

	std::vector<float> discrete_coord = { x_dis, y_dis };

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
void print_path(std::vector<float> end, std::vector<float> start, std::vector<float> heading_changes, float drive_distance, std::vector<std::vector<float>> heading_map, std::vector<std::vector<float>> display) {
	bool FINISHED = false;
	float turn_index = 0;
	std::vector<std::vector<float>> path = {};

	//start point
	float x1 = end[2];
	float y1 = end[3];

	std::vector<float> discrete_coor = lattice_reg(x1, y1);
	float theta1 = end[4];
	turn_index = heading_map[discrete_coor[0]][discrete_coor[1]];
	float turn = heading_changes[turn_index];

	std::vector<float> state = { x1, y1, theta1 };

	path.push_back(state);

	float x2 = 0;
	float y2 = 0;
	float theta2 = 0;
	std::vector<float> discrete = {};

	while (FINISHED == false) {
		theta2 = theta1 - turn;
		x2 = x1 - drive_distance * cos(theta1);
		y2 = y1 - drive_distance * sin(theta1);
		

		state[0] = x2;
		state[1] = y2;
		state[2] = theta2;
	
		std::cout << std::endl;
		path.push_back(state);

		discrete = lattice_reg(x2, y2);
		print_1Dvector(discrete);
		display[discrete[0]][discrete[1]] = 8;

		x1 = x2;
		y1 = y2;
		theta1 = theta2;
		turn_index = heading_map[discrete[0]][discrete[1]];
		turn = heading_changes[turn_index];

		if (discrete[0] == start[0] && discrete[1] == start[1]) {
			FINISHED = true;
		}
	}
	std::cout << "the path found is: \n";

	print_2Dvector(path);
	print_2Dvector(display);

}


/* ACTUAL CODE STARTS FROM HERE */
int main() {
	std::vector<std::vector<float>> map_input =
	{ {0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} };

	//std::vector<std::vector<float>> map_input =
	//{ {0, 0, 0, 0, 0},
	//  {0, 0, 0, 0, 0},
	//  {0, 0, 1, 0, 0},
	//  {0, 0, 0, 0, 0},
	//  {0, 0, 0, 0, 0} };

	//create and empty heading change map
	std::vector<std::vector<float>> heading_change_map = map_input;
	for (int i = 0; i < heading_change_map.size(); i++) {
		for (int j = 0; j < heading_change_map[i].size(); j++) {
			heading_change_map[i][j] = 0;
		}
	}
	//create and empty expansion change map
	std::vector<std::vector<float>> expansion_map = map_input;
	for (int i = 0; i < expansion_map.size(); i++) {
		for (int j = 0; j < expansion_map[i].size(); j++) {
			expansion_map[i][j] = 0;
		}
	}

	//create display change map
	std::vector<std::vector<float>> display_map = map_input;

	//define goal states
	//[x, y, theta (in rad)]
	std::vector<float> start_state = { 0, 0, 0 };
	std::vector<float> goal_state = { 38, 38, -pi / 2 };

	bool PATH_FOUND = false;
	bool NO_PATH = false;

	//because we do not know the wheel base calculation, the max steering angle
	//has not been placed under consideration
	float drive_distance = sqrt(2)+0.001;
	float max_steering_angle = pi / 4;
	std::vector<float> heading_changes = { pi / 4, 0, -pi / 4 };
	float expansion = 0;

	//state vector definition: [cost, expansion number, x, y, orientation, orientation from previous node]
	std::vector<float> current_state = { 0, 0, start_state[0], start_state[1], start_state[2], 0 };

	std::vector<std::vector<float>> open = {};
	std::vector<std::vector<float>> closed = {};
	open.push_back(current_state);
	float cost = current_state[0];
	int c = 0;

	while (open.empty() == false) {
		c++;
		std::cout << open.size() << "    " << closed.size() << std::endl;
		//sort the open list to increasing order
		open = sort_vectors(open);
		//examine the lowest cost state
		current_state = open.back();
		open.pop_back();
		//convert continuous coord to discrete
		std::vector<float> dis_coord = lattice_reg(current_state[2], current_state[3]);
		//add the examining state to the closed set
		closed.push_back(dis_coord);

		//if (c % 50 == 0) {
		//	print_2Dvector(expansion_map);
		//}


		//if  the next point is within the goal state
		if (dis_coord[0] == goal_state[0] && dis_coord[1] == goal_state[1]) {
			std::cout << "path found: ";
			print_path(current_state, start_state, heading_changes, drive_distance, heading_change_map, display_map);
			PATH_FOUND = true;
			break;
		}
		for (float i = 0; i < heading_changes.size(); i++) {
			float current_x = current_state[2];
			float current_y = current_state[3];
			float current_theta = current_state[4];

			//calculate next state for this iteration of heading change
			float next_theta = current_theta + heading_changes[i];
			float next_x = current_x + drive_distance * cos(next_theta);
			float next_y = current_y + drive_distance * sin(next_theta);

			std::vector<float> discrete_next = lattice_reg(next_x, next_y);
			discrete_next.push_back(next_theta);

			//if the state is valid, then calculate heuristics for the path, add to
			//the open vector
			if (valid_state_check(map_input, discrete_next) && expansion_map[discrete_next[0]][discrete_next[1]] == 0) {
				expansion++;

				//calculate cost of this new path using heuristics (changeable)
				float heuristic = euclidean_dist(next_x, next_y, goal_state[0], goal_state[1]) + drive_distance + cost;
				std::vector<float> next_state = {heuristic, expansion, next_x, next_y, next_theta, i };
				heading_change_map[discrete_next[0]][discrete_next[1]] = i;
				expansion_map[discrete_next[0]][discrete_next[1]] = expansion;
				open.push_back(next_state);
			}
		}
	}

	if (PATH_FOUND == false) {
		std::cout << "no path exists";
	}
}
