package de.gdl.rl.cli;
import de.gdl.rl.Coordinator;
import de.gdl.rl.agents.RosTrainingAgent;

import java.util.List;
import de.gdl.rl.environment.GDLGameEnvironment;

public class TrainingCLI<ConcreteEnvironment extends GDLGameEnvironment> extends Coordinator<ConcreteEnvironment> {
    
    public TrainingCLI(ConcreteEnvironment env) {
        super(true, env, false, 0);
    }
    
    protected void onInit() {
        System.out.println("");
        System.out.println("Welcome to the GDL-RL-Training-CLI");
        System.out.println("    Game: " + this.env.getNameOfGame());
        System.out.println("");
    }

    protected void onStartGame() {
        System.out.println("New game");
        System.out.println(""); // TBD: print which roles are controlled by whom
        System.out.println(this.env.getStateAsReadableString());
    }

    protected void onMoveWasIllegal(String move, List<String> legalMoves) {
        System.out.println("Move was not legal");
    }
    
    protected void onPreDoMove(String move, String playerDescription) {
        System.out.println(playerDescription + ": " + move);
    }
    
    protected void onPostDoMove() {
        System.out.println(this.env.getStateAsReadableString());
    }
    
    protected void onGameIsOver(int numberOfEpisodesPlayed) {
        System.out.println(this.env.getStateAsReadableString());
        System.out.println("Game is over, Episode: "+ (numberOfEpisodesPlayed - 1));
        
        for (RosTrainingAgent agent : this.getTrainingAgents() ) {

            // print the calculated reward
            float reward = (float) this.env.getReward(agent.currentGdlRoleName, agent);
            System.out.println("Reward for "+ agent.name +": " + reward);
            
        }
        
        System.out.println("Goals: ");
        for (List<String> goal : this.env.getReachedGoals()) {
            System.out.println("(" + String.join(", ", goal) + ")");
        }
        System.out.println("");
    }
    
}
