package de.gdl.rl.environment.games.tictactoe;

import de.gdl.rl.agents.Agent;
import de.gdl.rl.agents.RosAgent;
import de.gdl.rl.agents.RosTrainingAgent;

import de.gdl.rl.environment.GDLGameEnvironment;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import org.apache.commons.collections4.BidiMap;
import org.apache.commons.collections4.bidimap.DualHashBidiMap;
import java.util.stream.Collectors;


public class TicTacToeEnv extends GDLGameEnvironment {
    
    private BidiMap<Integer, String> actionMoveMap = new DualHashBidiMap<Integer, String>();
    private HashMap<String, Integer> numberOfPossibleActionsForRole = new HashMap<String, Integer>();

    private RosTrainingAgent trainingAgent;
    
    public TicTacToeEnv() {
        
        this.trainingAgent = new RosTrainingAgent()
            .withName("Training Agent")
            .withType("DQN")
            .withGdlRoleNames(new String[]{"x", "o"})
            .withGameOverForIllegalActions(true)
            .withCurrentGdlRoleName("x")
            .withStateTopic("/gdl/tictactoe/trainingAgent/state")
            .withActionTopic("/gdl/tictactoe/trainingAgent/action")
            .withTerminalTopic("/gdl/tictactoe/trainingAgent/terminal")
            .withRewardTopic("/gdl/tictactoe/trainingAgent/reward")
            .withResetTopic("/gdl/tictactoe/trainingAgent/reset");
            
        RosAgent agent = new RosAgent()
            .withName("Agent")
            .withType("DQN")
            .withGdlRoleNames(new String[]{"x", "o"})
            .withStateTopic("/gdl/tictactoe/agent/state")
            .withLegalActionsTopic("/gdl/tictactoe/agent/legal_actions")
            .withActionTopic("/gdl/tictactoe/agent/action");

        RosAgent selfPlayAgent = agent
                                .copy()
                                .withNumberOfRandomEpisodes(50)
                                .withEpsilon(0.9f)
                                .withEpsilonDecay(0.9f);

        TicTacToeMiniMaxAgent miniMaxAgent = new TicTacToeMiniMaxAgent();
        
        this.addToTrainingConfiguration(this.trainingAgent);
        this.addToTrainingConfiguration(selfPlayAgent);

        this.addToGamingConfiguration(agent);
        this.addToGamingConfiguration(miniMaxAgent);

        this.numberOfPossibleActionsForRole.put("x", 9);
        this.numberOfPossibleActionsForRole.put("o", 9);

        actionMoveMap.put(0, "(mark 1 1)");
        actionMoveMap.put(1, "(mark 1 2)");
        actionMoveMap.put(2, "(mark 1 3)");

        actionMoveMap.put(3, "(mark 2 1)");
        actionMoveMap.put(4, "(mark 2 2)");
        actionMoveMap.put(5, "(mark 2 3)");

        actionMoveMap.put(6, "(mark 3 1)");
        actionMoveMap.put(7, "(mark 3 2)");
        actionMoveMap.put(8, "(mark 3 3)");

    }

    protected String getPathToGdlModel() {
        return "src/main/resources/gdl/games/TicTacToe.gdl";
    }

    public int getNumberOfActionsForRole(String roleName)  {
        return this.numberOfPossibleActionsForRole.get(roleName);
    }

    public void onNextEpisode() {

        if (this.trainingAgent.currentGdlRoleName == "x") {
            this.trainingAgent.currentGdlRoleName = "o";
        } else {
            this.trainingAgent.currentGdlRoleName = "x";
        }

    }

    // get state as float representation
    public float[] getStateAsFloatRepresentation(String gdlRoleName, Agent agent) {
        
        boolean inverted = gdlRoleName.equals("o");

        float[] output = new float[27];

        for (int i = 0; i < 9; i++) {
            String fieldStatus = this.getCurrentState().get(i).get(3);
            
            switch (fieldStatus) {
                case "x":
                    output[3 * i] = 0.0f;
                    output[3 * i + 1] = inverted ? 0.0f : 1.0f;
                    output[3 * i + 2] = inverted ? 1.0f : 0.0f;
                    break;
                case "o":
                    output[3 * i] = 0.0f;
                    output[3 * i + 1] = inverted ? 1.0f : 0.0f;
                    output[3 * i + 2] = inverted ? 0.0f : 1.0f;
                    break;
                default:
                    output[3 * i] = 1.0f;
                    output[3 * i + 1] = 0.0f;
                    output[3 * i + 2] = 0.0f;
                    break;
            }
        }
        return output; 
    }

    // convert action from agent to mapped move
    public String getMoveStringFromAction(int action, String gdlRoleName, Agent agent) {
        
        return gdlRoleName + " " + actionMoveMap.inverseBidiMap().getKey(action);
    }

    // ... and the direction back!
    // used for calculating legal-move-matrix
    public int getActionFromMoveString(List<String> moveStringSplitted, String gdlRoleName, Agent agent) {
        return actionMoveMap.getKey("(" + moveStringSplitted.stream().collect(Collectors.joining(" ")) + ")");
    }

    // calculate the reward for specific role
    public float calculateRewardFromGoals(List<List<String>> goals, String gdlRoleName, Agent agent) {

        List<String> rolesInControl = this.whichRolesHaveControl();

        if ((!this.wasLastMoveIllegal() || !rolesInControl.contains(gdlRoleName)) && this.isTerminal() && goals.size() == 2) {
            if (goals.get(0).get(0) == gdlRoleName) {
                return Integer.parseInt(goals.get(0).get(1)) > 0 ? 1.0f : -1.0f;
            } else {
                return Integer.parseInt(goals.get(1).get(1)) > 0 ? 1.0f : -1.0f;
            }
        }  else if(this.wasLastMoveIllegal() && rolesInControl.contains(gdlRoleName)) { 
            return -10.0f;
        }

        return 0.0f;

    }

    // CLI-functions

    public String getStateAsReadableString() {
        
        // representation:
        //     1     2     3
        // 1 [ x ] [   ] [   ]
        // 2 [   ] [ o ] [   ]
        // 3 [   ] [   ] [ x ]

        StringBuilder readableStringBuilder = new StringBuilder();

        readableStringBuilder.append("    1     2     3 \n");

        for (int i = 0; i < 9; i++) {
            String fieldStatus = this.getCurrentState().get(i).get(3);
            
            String infix = (i % 3 == 0) ? (((int) i / 3) + 1) + " " : "";

            switch (fieldStatus) {
                case "x":
                    readableStringBuilder.append(infix + "[ x ] ");
                    break;
                case "o":
                    readableStringBuilder.append(infix + "[ o ] ");
                    break;
                default:
                    readableStringBuilder.append(infix + "[   ] ");
                    break;
            }
            if ((i + 1) % 3 == 0) {
                readableStringBuilder.append("\n");
            }

        }
        return readableStringBuilder.toString();
    }

    public static void main(String[] args) throws Exception {
        GDLGameEnvironment.initEnvironment(new TicTacToeEnv(), args);
    }

}