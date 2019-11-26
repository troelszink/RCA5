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
	for (int y = 0; y < ROWS; y++)				//testWorld
	{
		for (int x = 0; x < COLUMNS; x++)
			std::cout << "" << environment[y][x];

		std::cout << std::endl;
	}
}

void QLearning::printQTable()
{
	for (int y = 0; y < ROWS; y++)
	{
		for (int x = 0; x < COLUMNS; x++)
			std::cout << " " << Q[y][x].UP;

		std::cout << std::endl;
	}
}

void QLearning::insertMarbles()
{
	//environment[5][5] = '+';
	//environment[5][60] = '+';
	//environment[5][40] = '+';
	//environment[40][10] = '+';
	//environment[70][10] = '+';
	//environment[25][110] = '+';
	//environment[70][110] = '+';
	//environment[65][20] = '+';
	//environment[70][50] = '+';
	//environment[50][55] = '+';
	//environment[70][70] = '+';
	//environment[40][55] = '+';

	//environment[10][20] = 'r';
	//environment[10][55] = 'r';
	//environment[25][40] = 'r';
	//environment[50][10] = 'r';
	//environment[30][110] = 'r';
	//environment[60][110] = 'r';
	//environment[50][25] = 'r';
	//environment[75][58] = 'r';
	//environment[55][90] = 'r';
	//environment[68][85] = 'r';


	environment[3][1] = '+';				// Test World
	environment[3][5] = '+';
	environment[3][7] = '+';


}

// Get the next state given a current state s and an action a:
state QLearning::GetNextState(state s, action a)
{
	if (environment[s.y][s.x] == '#')										//Change for Bigworld
		return TERMINAL_STATE;

	switch (a) {
	case UP:     s.y -= 1; break;
	case DOWN:   s.y += 1; break;
	case LEFT:   s.x -= 1; break;
	case RIGHT:  s.x += 1; break;
	}

	if (s.x < 0 || s.y < 0 || s.x >= COLUMNS || s.y >= ROWS || environment[s.y][s.x] == '#')
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
		return -1;
	}
	else {
		if (environment[next.y][next.x] == '+')
			return 10.0;

		if (environment[next.y][next.x] == '#')
			return -1.0;

		if (environment[next.y][next.x] == 'r')
			return 5;

		return 0;
	}
}

// Get the next action according to the current policy:
action QLearning::GetNextAction(state s)
{
	action a;
	action maxAction = RIGHT;

	float random = (float)rand() / (float)RAND_MAX;

	if (random < epsilon)
	{
		int i = rand() % 4;
		//std::cout << "Random action: " << i << std::endl;

		switch (i) {
		case 0:   a = UP; break;
		case 1:   a = DOWN; break;
		case 2:   a = LEFT; break;
		case 3:   a = RIGHT; break;
		}
	}
	else
	{
		a = GetNextBestAction(s);
	}

	return a;
}

action QLearning::GetNextBestAction(state s)
{
	action a;
	float vnext[4];

	if (!GetNextState(s, UP).is_outside_environment)
		vnext[0] = Q[s.y][s.x].UP + alpha * (GetReward(s, UP) + discount_rate * (Q[GetNextState(s, UP).y][GetNextState(s, UP).x].UP) - Q[s.y][s.x].UP);
	if (!GetNextState(s, DOWN).is_outside_environment) 
		vnext[1] = Q[s.y][s.x].DOWN + alpha * (GetReward(s, DOWN) + discount_rate * (Q[GetNextState(s, DOWN).y][GetNextState(s, DOWN).x].DOWN) - Q[s.y][s.x].DOWN);
	if (!GetNextState(s, LEFT).is_outside_environment)
		vnext[2] = Q[s.y][s.x].LEFT + alpha * (GetReward(s, LEFT) + discount_rate * (Q[GetNextState(s, LEFT).y][GetNextState(s, LEFT).x].LEFT) - Q[s.y][s.x].LEFT);
	if (!GetNextState(s, RIGHT).is_outside_environment)
		vnext[3] = Q[s.y][s.x].RIGHT + alpha * (GetReward(s, RIGHT) + discount_rate * (Q[GetNextState(s, RIGHT).y][GetNextState(s, RIGHT).x].RIGHT) - Q[s.y][s.x].RIGHT);
		
	int maxReward = vnext[0];
	int maxAction = 0;

	for (int i = 0; i < 4; i++)
	{
		if (vnext[i] > maxReward)
		{
			maxAction = i;
			maxReward = vnext[i];
		}
	}

	switch (maxAction) {
	case 0:   a = UP; break;
	case 1:   a = DOWN; break;
	case 2:   a = LEFT; break;
	case 3:   a = RIGHT; break;
	}

	return a;
}


int QLearning::getStateNumber(state s)
{
	int state_number;
	if (s.y > 0)
		state_number = (s.y) * COLUMNS + s.x;
	else
		state_number = s.x;


	return state_number;
}

QLearning::~QLearning()
{
}
