#pragma once
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <random>
#include <fstream>

#include <algorithm>
#include <iterator>
#include <numeric>

#define COLUMNS 13		// Bigworld
#define ROWS 14

//#define COLUMNS 4					// Test Environment
//#define ROWS 3

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
	int history[ROWS][COLUMNS];
};


class QLearning
{
public:
	QLearning();

	void printEnvironment();
	void printQTable(int index);
	void insertMarbles();
	state GetNextState(state s, action a);
	float GetReward(state s, action a);
	action GetNextAction(state s);
	action GetNextBestAction(state s);

	// Discount rate:
	float discount_rate = 0.9;

	// Learning rate:
	float alpha = 0.5;

	// Epsilon
	float epsilon = 0.1;

	// Maximum steps
	int max_steps = 20;

	// A convenient definition of the terminal state
	const state TERMINAL_STATE = { -1, -1, true };

	// Current estimate of state values under the current policy:
	//struct state Q[ROWS][COLUMNS];
	std::vector<struct state> Q;

	int posToIndex(int rows, int cols);


	// Stores different Q-tables
	std::vector< std::vector<struct state>> QVector;

	// Stores history for different Q-tables
	std::vector <std::vector<std::vector<int>>> QTableHistory;			// Index has to match QVector

	// Find index of Q-vector for current history
	int FindQTable(std::vector<std::vector<int>> vec);

	// Index to position
	std::vector<int> indexToPos(int index);

	// test
	float GetNextBestActionIndex(state s, int index);

	// Generate random marbles
	void generateMarbles();

	// 
	//int previousStates[ROWS][COLUMNS];
	std::vector<std::vector<int>> previousStates;

	//
	bool newMove(state s);

	// Stores value for max marbles collected during training
	int max_marbles_collected = 0;


	//Stores random generated marbles
	std::vector<std::vector<int>> marble_vector;


	char environment[ROWS][COLUMNS] = { { '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#' , '#' },			// World1
										{ '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ' , '#' },
										{ '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ' , '#' },
										{ '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ' , '#' },
										{ '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ' , '#' },
										{ '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ' , '#' },
										{ '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ' , '#' },
										{ '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ' , '#' },
										{ '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ' , '#' },
										{ '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ' , '#' },
										{ '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ' , '#' },
										{ '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ' , '#' },
										{ '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ' , '#' },
										{ '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#' , '#' } };

  //char environment[ROWS][COLUMNS] = { { '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#' , '#' },			// World 2
	//									{ '#', ' ', ' ', ' ', '#', ' ', ' ', '#', ' ', ' ', '#', ' ' , '#' },
	//									{ '#', ' ', ' ', ' ', '#', ' ', ' ', '#', ' ', ' ', '#', ' ' , '#' },
	//									{ '#', ' ', '#', '#', '#', ' ', '#', '#', ' ', ' ', '#', ' ' , '#' },
	//									{ '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ' , '#' },
	//									{ '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ' , '#' },
	//									{ '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ' , '#' },
	//									{ '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ' , '#' },
	//									{ '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ' , '#' },
	//									{ '#', '#', '#', '#', '#', '#', '#', ' ', ' ', '#', ' ', ' ' , '#' },
	//									{ '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', '#', ' ', ' ' , '#' },
	//									{ '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ' , '#' },
	//									{ '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ' , '#' },
	//									{ '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#' , '#' } };


	//char environment[ROWS][COLUMNS] = { { '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#' , '#', '#', '#', '#', '#', '#', '#', '#' },			// alpha = 0.5  epsilon = 0.1 discount = 0.9
	//									{ '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ' , '#' },
	//									{ '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ' , '#' },
	//									{ '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ' , '#' },
	//									{ '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ' , '#' },
	//									{ '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ' , '#' },
	//									{ '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ' , '#' },
	//									{ '#', '#', ' ', ' ', '#', ' ', ' ', '#', '#', '#', '#', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ' , '#' },
	//									{ '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ' , '#' },
	//									{ '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ' , '#' },
	//									{ '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ' , '#' },
	//									{ '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ' , '#' },
	//									{ '#', ' ', ' ', ' ', '#', '#', '#', '#', '#', '#', '#', ' ', ' ', ' ', ' ', '#', ' ', '#', ' ' , '#' },
	//									{ '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ' , '#' },
	//									{ '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ' , '#' },
	//									{ '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ' , '#' },
	//									{ '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ' , '#' },
	//									{ '#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ' , '#' },
	//									{ '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#' , '#', '#', '#', '#', '#', '#', '#', '#' } };



	//struct state Q_max_marbles[ROWS][COLUMNS];

	//int randomCount = 0;
	//int bestCount = 0;


	void printPreviousStates();


	void runModel();

	void createCSVFile(int steps, int marbles_collected);

	~QLearning();
private:


};

