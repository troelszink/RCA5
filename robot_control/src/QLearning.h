#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#define COLUMNS 120
#define ROWS 80

struct state
{
	int x;
	int y;
	bool is_outside_environment;
};


// Actions:
enum action { UP, DOWN, LEFT, RIGHT };

class QLearning
{
public:
	QLearning();

	void initMap();
	void printEnvironment();
	void insertMarbles();
	state GetNextState(state s, action a);
	float GetReward(state s, action a);
	action GetNextAction(state s);


	// Discount rate:
	float discount_rate = 0.9;

	// Learning rate:
	float alpha = 0.7;

	// A convenient definition of the terminal state
	const state TERMINAL_STATE = { -1, -1, true };

	// Current estimate of state values under the current policy:
	float Q[ROWS][COLUMNS];


	~QLearning();
private:
	char environment[ROWS][COLUMNS] = {};
};

