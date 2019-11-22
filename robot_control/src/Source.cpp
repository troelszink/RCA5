#include <iostream>
#include <math.h>
#include <algorithm>
#include "QLearning.h"

int main()
{
	QLearning ql;
	ql.initMap();
	ql.insertMarbles();
	ql.printEnvironment();

	for (int episode = 0; episode < 100; episode++)
	{
		state s = { 40, 60 };
		state next_state;

		while (!s.is_outside_environment)
		{
			action a = ql.GetNextAction(s);
			float reward = ql.GetReward(s, a);
			next_state = ql.GetNextState(s, a);
			ql.Q[s.y][s.x] = ql.Q[s.y][s.x] + ql.alpha * (reward + ql.discount_rate * (ql.Q[next_state.y][next_state.x]) - ql.Q[s.y][s.x]);
			s = next_state;
		}
	}


	system("pause");
	return 0;
}