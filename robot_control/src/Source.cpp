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
		while (!s.is_outside_environment && marbles_collected != 7)
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

			if (steps > 1000)
			{
				std::cout << "Too many steps" << std::endl;
				break;
			}

			//std::cout << "Training " << "      Episode: " << episode << "       Step: " << steps << std::endl;


			// Print PreviousStates and QTable
			/*ql.printPreviousStates();
			std::cout << "Size of QVector: " << ql.QVector.size() << std::endl;
			std::cout << "Marbles collected: " << marbles_collected << std::endl;
			ql.printQTable();*/


		}

		if (marbles_collected > ql.max_marbles_collected)
		{
			ql.max_marbles_collected = marbles_collected;

		}
		//std::cout << "Marbles collected during training: " << marbles_collected  << std::endl;

			ql.runModel();

	}


	//************************************** Run trained model **********************************************'

	std::cout << "Maximum marbles collected: " << ql.max_marbles_collected << std::endl;
	std::cout << "Trained model" << std::endl;

	for (int episode = 0; episode < 1; episode++)			// Number of training episodes
	{
		state s = { 6, 5 };
		state next_state;
		int steps = 0;
		int marbles_collected = 0;
		int rCounter = 0;

		// Reset history of moves
		ql.previousStates.erase(ql.previousStates.begin(), ql.previousStates.end());

		// Add current position to history
		/*std::vector<int> curPos{ s.y, s.x };
		ql.previousStates.push_back(curPos);*/

		// Find index for current QTable
		/*int index = ql.FindQTable(ql.previousStates);
		std::copy(std::begin(ql.QVector[index]), std::end(ql.QVector[index]), std::begin(ql.Q));*/

		//ql.initMap();
		ql.insertMarbles();

		while (!s.is_outside_environment && steps < ql.max_steps)		// Run until illegal move or maximum number of steps is exceeded
		{

			if (ql.newMove(s))		// Check if new move
			{
				std::vector<int> moveVec{ s.y, s.x };
				ql.previousStates.push_back(moveVec);
				//std::copy(std::begin(ql.previousStates), std::end(ql.previousStates), std::begin(s.history));
			}
			
			int index = ql.FindQTable(ql.previousStates);
			//std::copy(std::begin(ql.QVector[index]), std::end(ql.QVector[index]), std::begin(ql.Q));


			//for (int i = 0; i < ql.previousStates.size(); i++)
			//{
			//	if (ql.environment[ql.previousStates[i][0]][ql.previousStates[i][1]] == '+')
			//	{
			//		ql.environment[s.y][s.x] = ' ';
			//		marbles_collected++;
			//	}
			//}

			if (ql.environment[s.y][s.x] == '+')
			{
				ql.environment[s.y][s.x] = ' ';
				marbles_collected++;
			}

			ql.environment[s.y][s.x] = 'X';			// Draw path

			action a = ql.GetNextBestAction(s);
			float reward = ql.GetReward(s, a);
			next_state = ql.GetNextState(s, a);

			std::cout << "Position: " << s.x << " " << s.y << "  map: " << ql.environment[s.y][s.x] << "   reward: " << ql.GetReward(s, a);

			//if (a == UP)
				std::cout << "  Action: UP" << "   Q: " << ql.QVector[index][ql.posToIndex(s.y, s.x)].UP << std::endl;
			//if (a == DOWN)
				std::cout << "  Action: DOWN" << "   Q: " << ql.QVector[index][ql.posToIndex(s.y, s.x)].DOWN << std::endl;
			//if (a == LEFT)
				std::cout << "  Action: LEFT" << "   Q: " << ql.QVector[index][ql.posToIndex(s.y, s.x)].LEFT << std::endl;
			//if (a == RIGHT)
				std::cout << "  Action: RIGHT" << "   Q: " << ql.QVector[index][ql.posToIndex(s.y, s.x)].RIGHT << std::endl;

			std::cout << std::endl << std::endl;



			ql.printPreviousStates();
			std::cout << std::endl;
			//ql.printQTable(index);


			//std::cout << test << std::endl;
			s = next_state;
			steps++;
			//ql.printQTable();
		}
		std::cout << "Marbles collected: " << marbles_collected << "    steps: " << steps <<  std::endl << std::endl;
		//std::cout << std::endl;

	}

	//ql.printQTable();
	ql.printEnvironment();



	system("pause");
	return 0;
}