package de.gdl.rl.cli;
import de.gdl.rl.Coordinator;

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
        System.out.println("    Roles: " +  String.join(", ", this.env.getAvailableRoles()));
        System.out.println("");
    }

    protected void onStartGame() {
        System.out.println("New game");
        System.out.println(""); // TBD: print which roles are controlled by whom
        System.out.println(this.env.getStateAsReadableString());
    }

    protected void onMoveWasIllegal(String move) {
        System.out.println("Move was not legal");
    }
    
    protected void onPreDoMove(String move, String playerDescription) {
        System.out.println(playerDescription + ": " + move);
    }
    
    protected void onPostDoMove() {
        System.out.println(this.env.getStateAsReadableString());
    }
    
    protected void onGameIsOver() {
        System.out.println("Game is over");
        System.out.println("Goals: ");
        for (List<String> goal : this.env.getReachedGoals()) {
            System.out.println("(" + String.join(", ", goal) + ")");
            // print the goal:
            if (this.doesTrainingAgentControlRole(goal.get(0))) {
                // print the calculated reward
                float reward = (float) this.env.getReward(goal.get(0));
                System.out.println("Reward: " + reward);
            } 

        }
        System.out.println("");
    }
    
}
