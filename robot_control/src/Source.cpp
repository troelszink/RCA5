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
	//ql.initMap();
	ql.insertMarbles();
	ql.printEnvironment();

	for (int y = 0; y < ROWS; y++)						// Initialize Q-Vector to 0
	{
		for (int x = 0; x < COLUMNS; x++)
		{
			ql.Q[y][x].UP = 0;
			ql.Q[y][x].DOWN = 0;
			ql.Q[y][x].LEFT = 0;
			ql.Q[y][x].RIGHT = 0;
		}
	}

	for (int episode = 0; episode < 10000; episode++)			// Number of training episodes
	{
		//state s = { 60, 40 };			//Bigworld
		state s = { 1, 1 };
		state next_state;
		int steps = 0;
		int marbles_collected = 0;

		ql.insertMarbles();

		while (!s.is_outside_environment && steps < ql.max_steps)		// Run until illegal move or maximum number of steps is exceeded
		{
			if (ql.environment[s.y][s.x] == '+')
			{
				ql.environment[s.y][s.x] = ' ';
				marbles_collected++;
			}
			if (ql.environment[s.y][s.x] == 'r')
			{
				ql.environment[s.y][s.x] = ' ';
			}

			action a = ql.GetNextAction(s);
			float reward = ql.GetReward(s, a);
			next_state = ql.GetNextState(s, a);

			if (a == UP)
			{
				if (!next_state.is_outside_environment)
				{
					//float test = ql.Q[s.y][s.x].UP + ql.alpha * (reward + ql.discount_rate * (ql.Q[next_state.y][next_state.x].UP) - ql.Q[s.y][s.x].UP);
					ql.Q[s.y][s.x].UP = ql.Q[s.y][s.x].UP + ql.alpha * (reward + ql.discount_rate * (ql.Q[next_state.y][next_state.x].UP) - ql.Q[s.y][s.x].UP);
				}
				else
				{
					//float test = ql.Q[s.y][s.x].UP + ql.alpha * (reward + ql.discount_rate * (-1) - ql.Q[s.y][s.x].UP);
					ql.Q[s.y][s.x].UP = ql.Q[s.y][s.x].UP + ql.alpha * (reward + ql.discount_rate * (-1) - ql.Q[s.y][s.x].UP);
				}
			}
			if (a == DOWN)
			{
				if (!next_state.is_outside_environment)
				{
					//float test = ql.Q[s.y][s.x].DOWN + ql.alpha * (reward + ql.discount_rate * (ql.Q[next_state.y][next_state.x].DOWN) - ql.Q[s.y][s.x].DOWN);
					ql.Q[s.y][s.x].DOWN = ql.Q[s.y][s.x].DOWN + ql.alpha * (reward + ql.discount_rate * (ql.Q[next_state.y][next_state.x].DOWN) - ql.Q[s.y][s.x].DOWN);
				}
				else
				{
					//float test = ql.Q[s.y][s.x].DOWN + ql.alpha * (reward + ql.discount_rate * (-1) - ql.Q[s.y][s.x].DOWN);
					ql.Q[s.y][s.x].DOWN = ql.Q[s.y][s.x].DOWN + ql.alpha * (reward + ql.discount_rate * (-1) - ql.Q[s.y][s.x].DOWN);
				}
			}
			if (a == LEFT)
			{
				if (!next_state.is_outside_environment)
				{
					//float test = ql.Q[s.y][s.x].LEFT + ql.alpha * (reward + ql.discount_rate * (ql.Q[next_state.y][next_state.x].LEFT) - ql.Q[s.y][s.x].LEFT);
					ql.Q[s.y][s.x].LEFT = ql.Q[s.y][s.x].LEFT + ql.alpha * (reward + ql.discount_rate * (ql.Q[next_state.y][next_state.x].LEFT) - ql.Q[s.y][s.x].LEFT);
				}
				else
				{
					//float test = ql.Q[s.y][s.x].LEFT + ql.alpha * (reward + ql.discount_rate * (-1) - ql.Q[s.y][s.x].LEFT);
					ql.Q[s.y][s.x].LEFT = ql.Q[s.y][s.x].LEFT + ql.alpha * (reward + ql.discount_rate * (-1) - ql.Q[s.y][s.x].LEFT);
				}
			}
			if (a == RIGHT)
			{
				if (!next_state.is_outside_environment)
				{
					//float test = ql.Q[s.y][s.x].RIGHT + ql.alpha * (reward + ql.discount_rate * (ql.Q[next_state.y][next_state.x].RIGHT) - ql.Q[s.y][s.x].RIGHT);
					ql.Q[s.y][s.x].RIGHT = ql.Q[s.y][s.x].RIGHT + ql.alpha * (reward + ql.discount_rate * (ql.Q[next_state.y][next_state.x].RIGHT) - ql.Q[s.y][s.x].RIGHT);
				}
				else
				{
					//float test = ql.Q[s.y][s.x].RIGHT + ql.alpha * (reward + ql.discount_rate * (-1) - ql.Q[s.y][s.x].RIGHT);
					ql.Q[s.y][s.x].RIGHT = ql.Q[s.y][s.x].RIGHT + ql.alpha * (reward + ql.discount_rate * (-1) - ql.Q[s.y][s.x].RIGHT);
				}
			}
			/*if (!next_state.is_outside_environment)
			{
				float test = ql.Q[ql.getStateNumber(s)][a].val + ql.alpha * (reward + ql.discount_rate * (ql.Q[next_state.y][next_state.x].val) - ql.Q[ql.getStateNumber(s)][a].val);
				ql.Q[ql.getStateNumber(s)][a].val = ql.Q[ql.getStateNumber(s)][a].val + ql.alpha * (reward + ql.discount_rate * (ql.Q[ql.getStateNumber(next_state)][a].val) - ql.Q[ql.getStateNumber(s)][a].val);
			}
			else
			{
				float test = ql.Q[ql.getStateNumber(s)][a].val * (reward + ql.discount_rate * (-1) - ql.Q[ql.getStateNumber(s)][a].val);
				ql.Q[ql.getStateNumber(s)][a].val = ql.Q[ql.getStateNumber(s)][a].val + ql.alpha * (reward + ql.discount_rate * (-1) - ql.Q[ql.getStateNumber(s)][a].val);
			}*/

			//std::cout << "Position: " << s.x << " " << s.y << "  map: " << ql.environment[s.y][s.x] << "   reward: " << ql.GetReward(s, a) << "      Q: " << ql.Q[s.y][s.x] << std::endl;

			//std::cout << test << std::endl;
			s = next_state;
			steps++;
		}
		//std::cout << "Marbles collected: " << marbles_collected << "    steps: " << steps << std::endl << std::endl;
		//std::cout << std::endl;
		//ql.printQTable();
	}


	//************************************** Run trained model **********************************************'

	std::cout << "Trained model" << std::endl;

	for (int episode = 0; episode < 1; episode++)			// Number of training episodes
	{
		//state s = { 60, 40 };			//Bigworld
		state s = { 1, 1 };
		state next_state;
		int steps = 0;
		int marbles_collected = 0;
		int rCounter = 0;

		ql.insertMarbles();

		while (!s.is_outside_environment && steps < ql.max_steps)		// Run until illegal move or maximum number of steps is exceeded
		{
			if (ql.environment[s.y][s.x] == '+')
			{
				ql.environment[s.y][s.x] = ' ';
				marbles_collected++;
			}
			if (ql.environment[s.y][s.x] == 'r')
			{
				ql.environment[s.y][s.x] = ' ';
				rCounter++;
			}

			ql.environment[s.y][s.x] = 'X';			// Draw path

			action a = ql.GetNextBestAction(s);
			float reward = ql.GetReward(s, a);
			next_state = ql.GetNextState(s, a);
			
			std::cout << "Position: " << s.x << " " << s.y << "  map: " << ql.environment[s.y][s.x] << "   reward: " << ql.GetReward(s, a) << std::endl;

			//std::cout << test << std::endl;
			s = next_state;
			steps++;
		}
		std::cout << "Marbles collected: " << marbles_collected << "    steps: " << steps << "    r: " << rCounter << std::endl << std::endl;
		//std::cout << std::endl;
		//ql.printQTable();
	}

	//ql.printQTable();
	ql.printEnvironment();



	system("pause");
	return 0;
}