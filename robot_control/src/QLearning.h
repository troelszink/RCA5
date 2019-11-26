#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

//#define COLUMNS 120				// Bigworld
//#define ROWS 80

#define COLUMNS 10					// Test Environment
#define ROWS 6

//Actions:
enum action { UP, DOWN, LEFT, RIGHT };


struct state
{
	int x;
	int y;
	bool is_outside_environment;
	float UP;
	float DOWN;
	float LEFT;
	float RIGHT;
};


class QLearning
{
public:
	QLearning();

	void initMap();
	void printEnvironment();
	void printQTable();
	void insertMarbles();
	state GetNextState(state s, action a);
	float GetReward(state s, action a);
	action GetNextAction(state s);
	action GetNextBestAction(state s);


	// Discount rate:
	float discount_rate = 0.9;

	// Learning rate:
	float alpha = 0.5;

	// A convenient definition of the terminal state
	const state TERMINAL_STATE = { -1, -1, true };

	// Current estimate of state values under the current policy:
	struct state Q[ROWS][COLUMNS];

	// Epsilon
	float epsilon = 0.1;

	// Maximum steps
	int max_steps = 15;

	char environment[ROWS][COLUMNS] = { { '#', '#', '#', '#', '#', '#', '#', '#', '#', '#' },			// alpha = 0.5  epsilon = 0.1 discount = 0.9
										{ '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#' },
										{ '#', ' ', '#', ' ', ' ', ' ', '#', ' ', ' ', '#' },
										{ '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#' },
										{ '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ' ,'#' }, 
										{ '#', '#', '#', '#', '#', '#', '#', '#', '#' , '#'} };

	//char environment[ROWS][COLUMNS] = {};

	int getStateNumber(state s);

	~QLearning();
private:
	

};

