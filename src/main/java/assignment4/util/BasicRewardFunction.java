package assignment4.util;

import assignment4.BasicGridWorld;
import burlap.oomdp.core.objects.ObjectInstance;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;

public class BasicRewardFunction implements RewardFunction {

	int goalX;
	int goalY;

	int pitX;
	int pitY;
	
	public BasicRewardFunction(int goalX, int goalY, int pitX, int pitY) {
		this.goalX = goalX;
		this.goalY = goalY;
		
		this.pitX = pitX;
		this.pitY = pitY;
	}

	@Override
	public double reward(State s, GroundedAction a, State sprime) {

		// get location of agent in next state
		ObjectInstance agent = sprime.getFirstObjectOfClass(BasicGridWorld.CLASSAGENT);
		int ax = agent.getIntValForAttribute(BasicGridWorld.ATTX);
		int ay = agent.getIntValForAttribute(BasicGridWorld.ATTY);

		// are they at goal location?
		if (ax == this.goalX && ay == this.goalY) {
			return 100.;
		}
		
		if (ax == this.pitX && ay == this.pitY) {
			return -100.;
		}

		return -1;
	}

}
