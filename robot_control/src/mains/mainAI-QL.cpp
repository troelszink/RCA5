#include <iostream>
#include <math.h>
#include <algorithm>
#include <time.h>
#include <stdlib.h>
#include "QLearning.h"

int main()
{
	srand(time(NULL));
	QLearning ql;

	// Initialize vector	
	for (int i = 0; i < ROWS*COLUMNS; i++)
	{
		std::vector<int> stateVec = ql.indexToPos(i);
		state s = { stateVec[0],stateVec[1] };
		ql.Q.push_back(s);
	}

	//ql.initMap();
	ql.generateMarbles();
	ql.insertMarbles();
	ql.printEnvironment();

	for (int y = 0; y < ROWS; y++)						// Initialize Q-Vector to 0
	{
		for (int x = 0; x < COLUMNS; x++)
		{
			ql.Q[ql.posToIndex(y, x)].UP = 0;
			ql.Q[ql.posToIndex(y, x)].DOWN = 0;
			ql.Q[ql.posToIndex(y, x)].LEFT = 0;
			ql.Q[ql.posToIndex(y, x)].RIGHT = 0;
		}
	}


	for (int episode = 0; episode < 1000; episode++)			// Number of training episodes
	{
		std::cout << "Episode: " << episode << std::endl;
		state s = { 6, 5 };
		state next_state;
		int steps = 0;
		int marbles_collected = 0;
		ql.insertMarbles();

		//Reset history of moves
		ql.previousStates.erase(ql.previousStates.begin(), ql.previousStates.end());

		//while (!s.is_outside_environment && steps < ql.max_steps)		// Run until illegal move or maximum number of steps is exceeded
		while (!s.is_outside_environment && marbles_collected != 3)
		{
			
			if (ql.newMove(s))		// Check if new move
			{
				// Save new move
				std::vector<int> moveVec{ s.y, s.x };
				ql.previousStates.push_back(moveVec);
			}

			// Find QTable for current history
			int index = ql.FindQTable(ql.previousStates);

			if (ql.environment[s.y][s.x] == '+')
			{
				ql.environment[s.y][s.x] = ' ';
				marbles_collected++;
			}

			action a = ql.GetNextAction(s);
			float reward = ql.GetReward(s, a);
			next_state = ql.GetNextState(s, a);

			// Locate Q-Vector after next action		
			// Find Q-value for next best action

			std::vector<std::vector<int>> previousStatesCopy;
			for (int i = 0; i < ql.previousStates.size(); i++)
				previousStatesCopy.push_back(ql.previousStates[i]);
			std::vector<int> nextPosition{ next_state.y, next_state.x };
			previousStatesCopy.push_back(nextPosition);

			int nextIndex = ql.FindQTable(previousStatesCopy);

			// Best Q-value for best action of next state
			float aNext = ql.GetNextBestActionIndex(next_state, nextIndex);		

			//std::cout << "action: " << a << std::endl;
			if (a == UP)
			{
				if (!next_state.is_outside_environment)
				{
					ql.QVector[index][ql.posToIndex(s.y, s.x)].UP = ql.QVector[index][ql.posToIndex(s.y, s.x)].UP + ql.alpha * (reward + ql.discount_rate * (aNext) - ql.QVector[index][ql.posToIndex(s.y, s.x)].UP);
				}
				
			}
			if (a == DOWN)
			{
				if (!next_state.is_outside_environment)
				{
					ql.QVector[index][ql.posToIndex(s.y, s.x)].DOWN = ql.QVector[index][ql.posToIndex(s.y, s.x)].DOWN + ql.alpha * (reward + ql.discount_rate * (aNext) - ql.QVector[index][ql.posToIndex(s.y, s.x)].DOWN);
				}
				
			}
			if (a == LEFT)
			{
				if (!next_state.is_outside_environment)
				{
					ql.QVector[index][ql.posToIndex(s.y, s.x)].LEFT = ql.QVector[index][ql.posToIndex(s.y, s.x)].LEFT + ql.alpha * (reward + ql.discount_rate * (aNext) - ql.QVector[index][ql.posToIndex(s.y, s.x)].LEFT);
				}

			}
			if (a == RIGHT)
			{
				if (!next_state.is_outside_environment)
				{
					ql.QVector[index][ql.posToIndex(s.y, s.x)].RIGHT = ql.QVector[index][ql.posToIndex(s.y, s.x)].RIGHT + ql.alpha * (reward + ql.discount_rate * (aNext) - ql.QVector[index][ql.posToIndex(s.y, s.x)].RIGHT);
				}
				
			}
			
			s = next_state;
			steps++;
			// Find QTable index for current history
			index = ql.FindQTable(ql.previousStates);

		}

		if (marbles_collected > ql.max_marbles_collected)
		{
			ql.max_marbles_collected = marbles_collected;

		}
		std::cout << "Marbles collected during training: " << marbles_collected  << "   steps: " << steps << std::endl;
		
		if (episode > 10)
			ql.runModel();
	}
	
	system("pause");
	return 0;
}
