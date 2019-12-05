#include "QLearning.h"



QLearning::QLearning()
{
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

void QLearning::printQTable(int index)
{
	for (int y = 0; y < ROWS; y++)
	{
		for (int x = 0; x < COLUMNS; x++)
			std::cout << " " << QVector[index][posToIndex(y, x)].UP;

		std::cout << std::endl;
	}
	std::cout << std::endl;

	for (int y = 0; y < ROWS; y++)
	{
		for (int x = 0; x < COLUMNS; x++)
			std::cout << " " << QVector[index][posToIndex(y, x)].DOWN;

		std::cout << std::endl;
	}
	std::cout << std::endl;

	for (int y = 0; y < ROWS; y++)
	{
		for (int x = 0; x < COLUMNS; x++)
			std::cout << " " << QVector[index][posToIndex(y, x)].LEFT;

		std::cout << std::endl;
	}
	std::cout << std::endl;

	for (int y = 0; y < ROWS; y++)
	{
		for (int x = 0; x < COLUMNS; x++)
			std::cout << " " << QVector[index][posToIndex(y, x)].RIGHT;

		std::cout << std::endl;
	}
	std::cout << std::endl;
}

void QLearning::generateMarbles()
{

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(0, 1);

	for (int i = 0; i < 5; i++)
	{
		int y = dis(gen) * ROWS;
		int x = dis(gen) * COLUMNS;

		while (environment[y][x] == '#' || environment[y][x] == '+')
		{
			y = dis(gen) * ROWS;
			x = dis(gen) * COLUMNS;
		}

		std::vector<int> marble_vec{ x,y };
		marble_vector.push_back(marble_vec);
	}
}

void QLearning::insertMarbles()
{

	environment[2][2] = '+';				// Test World2
	environment[2][6] = '+';
	environment[6][4] = '+';
	environment[11][2] = '+';
	environment[11][7] = '+';
	environment[3][11] = '+';
	environment[12][11] = '+';

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

		return 0.0;
	}
}

// Get the next action according to the current policy:
action QLearning::GetNextAction(state s)
{
	action a;

	float random = (float)rand() / (float)RAND_MAX;

	if (random < epsilon)
	{
		int i = rand() % 4;
		//std::cout << "Random action: " << i << std::endl;
		//randomCount++;

		switch (i) {
		case 0:   a = UP; break;
		case 1:   a = DOWN; break;
		case 2:   a = LEFT; break;
		case 3:   a = RIGHT; break;
		}
	}
	else
	{
		//bestCount++;
		a = GetNextBestAction(s);
	}

	return a;
}

action QLearning::GetNextBestAction(state s)
{

	std::vector<std::vector<int>> previousStatesCopy;
	for (int i = 0; i < previousStates.size(); i++)
		previousStatesCopy.push_back(previousStates[i]);

	//std::copy(std::begin(ql.previousStates), std::end(ql.previousStates), std::begin(previousStatesCopy));

	action a;
	float vnext[4];

	int index = FindQTable(previousStates);

	if (!GetNextState(s, UP).is_outside_environment)
	{
		state next_state = GetNextState(s, UP);
		std::vector<int> nextPosition{ next_state.y, next_state.x };
		previousStatesCopy.push_back(nextPosition);
		int nextIndex = FindQTable(previousStatesCopy);
		// Best Q-value for next best action of next state
		float aNext = GetNextBestActionIndex(next_state, nextIndex);

		vnext[0] = QVector[index][posToIndex(s.y, s.x)].UP + alpha * (GetReward(s, UP) + discount_rate * (aNext) -QVector[index][posToIndex(s.y, s.x)].UP);
	}
	if (!GetNextState(s, DOWN).is_outside_environment)
	{
		state next_state = GetNextState(s, DOWN);
		std::vector<int> nextPosition{ next_state.y, next_state.x };
		previousStatesCopy.push_back(nextPosition);
		int nextIndex = FindQTable(previousStatesCopy);
		// Best Q-value for next best action of next state
		float aNext = GetNextBestActionIndex(next_state, nextIndex);

		vnext[1] = QVector[index][posToIndex(s.y, s.x)].DOWN + alpha * (GetReward(s, DOWN) + discount_rate * (aNext)-QVector[index][posToIndex(s.y, s.x)].DOWN);
	}
	if (!GetNextState(s, LEFT).is_outside_environment)
	{
		state next_state = GetNextState(s, LEFT);
		std::vector<int> nextPosition{ next_state.y, next_state.x };
		previousStatesCopy.push_back(nextPosition);
		int nextIndex = FindQTable(previousStatesCopy);
		// Best Q-value for next best action of next state
		float aNext = GetNextBestActionIndex(next_state, nextIndex);

		vnext[2] = QVector[index][posToIndex(s.y, s.x)].LEFT + alpha * (GetReward(s, LEFT) + discount_rate * (aNext)-QVector[index][posToIndex(s.y, s.x)].LEFT);
	}
	if (!GetNextState(s, RIGHT).is_outside_environment)
	{
		state next_state = GetNextState(s, RIGHT);
		std::vector<int> nextPosition{ next_state.y, next_state.x };
		previousStatesCopy.push_back(nextPosition);
		int nextIndex = FindQTable(previousStatesCopy);
		// Best Q-value for next best action of next state
		float aNext = GetNextBestActionIndex(next_state, nextIndex);

		vnext[3] = QVector[index][posToIndex(s.y, s.x)].RIGHT + alpha * (GetReward(s, RIGHT) + discount_rate * (aNext)-QVector[index][posToIndex(s.y, s.x)].RIGHT);
	}

	int randomMax = rand() % 4;

	int maxReward = vnext[randomMax];
	int maxAction = randomMax;

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

bool QLearning::newMove(state s)
{
	for (int i = 0; i < previousStates.size(); i++)
	{
		if (previousStates[i][0] == s.y && previousStates[i][1] == s.x)
		{
			return false;
		}
	}
	return true;
}

int QLearning::FindQTable(std::vector<std::vector<int>> vec)
{
	// Find QTable if it exists
	

	for (int i = 0; i < QVector.size(); i++)
	{
		bool Qexists = true;
		bool element = false;

		// Check if arrays are equal

		if (vec.size() == QTableHistory[i].size())
		{
			for (int j = 0; j < vec.size(); j++)
			{
				element = false;
				for (int p = 0; p < vec.size(); p++)
				{
					if (vec[p] == QTableHistory[i][j])
					{
						element = true;
					}
				}
				if (element == false)
					Qexists = false;
			}
		}
		else
		{
			Qexists = false;
		}

		if (Qexists == true)
		{
			return i;
		}
		
	}
	
	// If QTable does not exist create the table

	for (int y = 0; y < ROWS; y++)						// Initialize Q-Vector to 0
	{
		for (int x = 0; x < COLUMNS; x++)
		{
			Q[posToIndex(y, x)].UP = 0;
			Q[posToIndex(y, x)].DOWN = 0;
			Q[posToIndex(y, x)].LEFT = 0;
			Q[posToIndex(y, x)].RIGHT = 0;
		}
	}

	QVector.push_back(Q);
	QTableHistory.push_back(previousStates);

	return QVector.size() - 1;				// Return index of new Qtable
}


int QLearning::posToIndex(int y, int x)
{
	if (y < 1)
		return x;

	return y * COLUMNS + x;
}

std::vector<int> QLearning::indexToPos(int index)
{
	int x = index % COLUMNS;
	int y = index / COLUMNS;

	std::vector<int> vec{ y,x };
	return vec;
}

float QLearning::GetNextBestActionIndex(state s, int index)
{
	action a;
	float vnext[4];


	
	if (!GetNextState(s, UP).is_outside_environment)
	{
		vnext[0] = QVector[index][posToIndex(s.y, s.x)].UP + alpha * (GetReward(s, UP) + discount_rate * (QVector[index][posToIndex(GetNextState(s, UP).y, GetNextState(s, UP).x)].UP) - QVector[index][posToIndex(s.y, s.x)].UP);
	}
	if (!GetNextState(s, DOWN).is_outside_environment)
	{
		vnext[1] = QVector[index][posToIndex(s.y, s.x)].DOWN + alpha * (GetReward(s, DOWN) + discount_rate * (QVector[index][posToIndex(GetNextState(s, DOWN).y, GetNextState(s, DOWN).x)].DOWN) - QVector[index][posToIndex(s.y, s.x)].DOWN);
	}
	if (!GetNextState(s, LEFT).is_outside_environment)
	{	
		vnext[2] = QVector[index][posToIndex(s.y, s.x)].LEFT + alpha * (GetReward(s, LEFT) + discount_rate * (QVector[index][posToIndex(GetNextState(s, LEFT).y, GetNextState(s, LEFT).x)].LEFT) - QVector[index][posToIndex(s.y, s.x)].LEFT);
	}
	if (!GetNextState(s, RIGHT).is_outside_environment)
	{	
		vnext[3] = QVector[index][posToIndex(s.y, s.x)].RIGHT + alpha * (GetReward(s, RIGHT) + discount_rate * (QVector[index][posToIndex(GetNextState(s, RIGHT).y, GetNextState(s, RIGHT).x)].RIGHT) - QVector[index][posToIndex(s.y, s.x)].RIGHT);
	}
	int randomMax = rand() % 4;

	int maxReward = vnext[randomMax];
	int maxAction = randomMax;

	for (int i = 0; i < 4; i++)
	{
		if (vnext[i] > maxReward)
		{
			maxAction = i;
			maxReward = vnext[i];
		}
	}

	return maxReward;
}


void QLearning::printPreviousStates() 
{
	for (int i = 0; i < previousStates.size(); i++)
	{
		std::cout << " " << previousStates[i][1] << "," << previousStates[i][0];
	}
	std::cout << std::endl;
}


void QLearning::runModel()
{
	for (int episode = 0; episode < 1; episode++)			// Number of training episodes
	{
		state s = { 6, 5 };
		state next_state;
		int steps = 0;
		int marbles_collected = 0;
		int rCounter = 0;
		float QSum = 0;

		// Reset history of moves
		previousStates.erase(previousStates.begin(), previousStates.end());

		insertMarbles();

		//while (!s.is_outside_environment && steps < max_steps)		// Run until illegal move or maximum number of steps is exceeded
		while (!s.is_outside_environment && marbles_collected != 3)
		{

			if (newMove(s))		// Check if new move
			{
				std::vector<int> moveVec{ s.y, s.x };
				previousStates.push_back(moveVec);
			}

			int index = FindQTable(previousStates);

			if (environment[s.y][s.x] == '+')
			{
				environment[s.y][s.x] = ' ';
				marbles_collected++;
			}

			action a = GetNextBestAction(s);
			float reward = GetReward(s, a);
			next_state = GetNextState(s, a);

			s = next_state;
			steps++;

		}
		std::cout << "Marbles collected: " << marbles_collected << "    steps: " << steps << std::endl << std::endl;
		createCSVFile(steps, marbles_collected);

	}
	
}

void QLearning::createCSVFile(int steps, int marbles_collected)
{
	// File pointer 
	std::fstream fout;

	// Opens an existing csv file or creates a new file.
   // Remember to delete old file, if you are using the same name. Else there will be data from several tests in the same file
	fout.open("world1_ex1.csv", std::ios::out | std::ios::app);

	// Insert the data to the file (the first row)
	fout << steps << ", "
		<< marbles_collected
		<< "\n";
}

QLearning::~QLearning()
{
}
