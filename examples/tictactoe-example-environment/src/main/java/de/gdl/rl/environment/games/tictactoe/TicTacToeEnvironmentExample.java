package de.gdl.rl.environment.games.tictactoe;

import de.gdl.rl.agents.Agent;
import de.gdl.rl.agents.RosAgent;
import de.gdl.rl.agents.RosTrainingAgent;

import de.gdl.rl.environment.GDLGameEnvironment;
import de.monticore.lang.gdl.InterpreterOptions;
import de.monticore.lang.gdl.types.GDLNumber;
import de.monticore.lang.gdl.types.GDLType;

import java.util.List;
import java.util.Map;


public class TicTacToeEnvironmentExample extends GDLGameEnvironment {

    private RosTrainingAgent trainingAgent;
    
    public TicTacToeEnvironmentExample() {
        super(new InterpreterOptions().withTypes(true).debugMode(true));

        final GDLType roleX = GDLType.createFromLine("x");
        final GDLType roleO = GDLType.createFromLine("o");

        final GDLType[] roles = new GDLType[]{roleX, roleO};

        this.trainingAgent = new RosTrainingAgent()
            .withName("Training Agent")
            .withType("DQN")
            .withGdlRoles(roles)
            .withGameOverForIllegalActions(true)
            .withCurrentGdlRole(roleX)
            .withStateTopic("/gdl/tictactoe/trainingAgent/state")
            .withActionTopic("/gdl/tictactoe/trainingAgent/action")
            .withTerminalTopic("/gdl/tictactoe/trainingAgent/terminal")
            .withRewardTopic("/gdl/tictactoe/trainingAgent/reward")
            .withResetTopic("/gdl/tictactoe/trainingAgent/reset");
            
        RosAgent agent = new RosAgent()
            .withName("Agent")
            .withType("DQN")
            .withGdlRoles(roles)
            .withStateTopic("/gdl/tictactoe/agent/state")
            .withLegalActionsTopic("/gdl/tictactoe/agent/legal_actions")
            .withActionTopic("/gdl/tictactoe/agent/action");

        RosAgent selfPlayAgent = agent
                                .copy()
                                .withNumberOfRandomEpisodes(50)
                                .withEpsilon(0.9f)
                                .withEpsilonDecay(0.9f);

        
        this.addToTrainingConfiguration(this.trainingAgent);
        this.addToTrainingConfiguration(selfPlayAgent);

        this.addToGamingConfiguration(agent);
    }

    protected String getPathToGdlModel() {
        return "src/main/resources/gdl/games/TicTacToeTyped.gdl";
    }

    @Override
    public void onNextEpisode() {
        this.trainingAgent.currentGdlRole
            = this.trainingAgent.currentGdlRole.equals(GDLType.createFromLine("x")) ?
                GDLType.createFromLine("o") : GDLType.createFromLine("x");
    }

    public float calculateRewardFromGoals(Map<GDLType, GDLType> goals, GDLType gdlRole, Agent agent) {
        List<GDLType> rolesInControl = this.whichRolesHaveControl();

        if ((!this.wasLastMoveIllegal() || !rolesInControl.contains(gdlRole)) && this.isTerminal() && goals.size() == 2) {
            return ((GDLNumber) goals.get(gdlRole)).getValue().intValue() > 0 ? 1.0f : -1.0f;
        } else if(this.wasLastMoveIllegal() && rolesInControl.contains(gdlRole)) { 
            return -10.0f;
        }

        return 0.0f;
    }

    public static void main(String[] args) throws Exception {
        GDLGameEnvironment.initEnvironment(new TicTacToeEnvironmentExample(), args);
    }

}