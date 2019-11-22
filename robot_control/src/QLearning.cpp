#include "QLearning.h"



QLearning::QLearning()
{
}

void QLearning::initMap()
{
	cv::Mat image;
	image = cv::imread("BigWorldV2.png", cv::IMREAD_COLOR);									//Change path
	int width = image.cols;
	int height = image.rows;

	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			cv::Vec3b point = image.at<cv::Vec3b>(y, x);
			if (point[0] < 240)
			{
				environment[y][x] = '#';
			}
			else
			{
				environment[y][x] = ' ';
			}
		}
	}
}

void QLearning::printEnvironment()
{
	for (int y = 0; y < ROWS; y++)
	{
		for (int x = 0; x < COLUMNS; x++)
				std::cout << "" << environment[y][x];

		std::cout << std::endl;
	}
}

void QLearning::insertMarbles()
{
	environment[5][5] = '+';
	environment[5][60] = '+';
	environment[5][40] = '+';
	environment[40][10] = '+';
	environment[70][10] = '+';
	environment[30][110] = '+';
	environment[70][110] = '+';
	environment[65][20] = '+';
	environment[70][50] = '+';
	environment[50][55] = '+';
	environment[70][70] = '+';

}

// Get the next state given a current state s and an action a:
state QLearning::GetNextState(state s, action a)
{
	if (environment[s.y][s.x] != ' ')
		return TERMINAL_STATE;

	switch (a) {
	case UP:     s.y -= 1; break;
	case DOWN:   s.y += 1; break;
	case LEFT:   s.x -= 1; break;
	case RIGHT:  s.x += 1; break;
	}

	if (s.x < 0 || s.y < 0 || s.x >= COLUMNS || s.y >= ROWS)
		return TERMINAL_STATE;

	s.is_outside_environment = false;
	return s;
}

// Get the reward given a state and an action:
float QLearning::GetReward(state s, action a)
{
	state next = GetNextState(s, a);
	if (next.is_outside_environment)
	{
		return 0;
	}
	else {
		if (environment[next.y][next.x] == '+')
			return 1.0;

		if (environment[next.y][next.x] == '-')
			return -1.0;

		return 0;
	}
}

// Get the next action according to the current policy:
action QLearning::GetNextAction(state s)
{
	action a;
	action maxAction = RIGHT;

	float maxReward = -2;

	for (int i = 0; i < 4; i++)
	{
		switch (i) {
		case 0:   a = UP; break;
		case 1:   a = DOWN; break;
		case 2:   a = LEFT; break;
		case 3:   a = RIGHT; break;
		}
		if (!GetNextState(s, a).is_outside_environment) {
			float vnext = GetReward(s, a) + discount_rate * Q[GetNextState(s, a).y][GetNextState(s, a).x];

			if (vnext > maxReward)
			{
				maxAction = a;
				maxReward = vnext;
			}
		}
	}
	return maxAction;
}

QLearning::~QLearning()
{
}
