package de.gdl.rl.environment.games.tictactoe;

import de.gdl.rl.agents.Agent;
import de.gdl.rl.agents.RosAgent;
import de.gdl.rl.agents.RosTrainingAgent;

import de.gdl.rl.environment.GDLGameEnvironment;
import de.monticore.lang.gdl.InterpreterOptions;
import de.monticore.lang.gdl.types.GDLNumber;
import de.monticore.lang.gdl.types.GDLType;

import java.util.Map;


public class TicTacToeEnvironmentExample extends GDLGameEnvironment {

    private RosTrainingAgent trainingAgent;
    
    public TicTacToeEnvironmentExample() {
        super(new InterpreterOptions().withTypes(true));

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

        TicTacToeMiniMaxAgent minimax = new TicTacToeMiniMaxAgent();
        minimax.gdlRoles.add(roleX);
        minimax.gdlRoles.add(roleO);

        
        this.addToTrainingConfiguration(this.trainingAgent);
        this.addToTrainingConfiguration(selfPlayAgent); // alternatively: use provided minimax

        this.addToGamingConfiguration(agent);
    }

    protected String getPathToGdlModel() {
        return "src/main/resources/gdl/games/TicTacToeTyped.gdl";
    }

    private int episode;
    private int won, lost, tie, illegal, other;

    @Override
    public void onNextEpisode() {
        episode++;
        System.out.printf("Episode: %d. Last 1000: (Won: %d    Lost: %d    Tie: %d    Illegal: %d    Other: %d)\n", episode, won, lost, tie, illegal, other);
        if (episode % 1000 == 0) {
            won = lost = tie = illegal = other = 0;
        }

        this.trainingAgent.currentGdlRole
            = this.trainingAgent.currentGdlRole.equals(GDLType.createFromLine("x")) ?
                GDLType.createFromLine("o") : GDLType.createFromLine("x");
    }


    public float calculateRewardFromGoals(Map<GDLType, GDLType> goals, GDLType gdlRole, Agent agent) {
        // first 5k: legal only
        if (episode < 1050*5) {
            if (this.wasLastMoveIllegal()) {
                illegal++;
                return -10f;
            }
    
            other++;
            return 1f;
        } else {

            if (!this.wasLastMoveIllegal() && this.isTerminal()) {
                int value = ((GDLNumber) goals.get(gdlRole)).getValue().intValue();

                if (value == 0) {
                    lost++;
                    return -5;
                }
                if (value == 50) {
                    tie++;
                    return 0;
                }
                if (value == 100)  {
                    won++;
                    return 1;
                }

                return (value/50) - 1;
            } else if (this.wasLastMoveIllegal() && this.whichRolesHaveControl().contains(gdlRole)) {
                illegal++;
                return -5;
            } else {
                other++;
                return 0;
            }
        }
    }

    public static void main(String[] args) throws Exception {
        GDLGameEnvironment.initEnvironment(new TicTacToeEnvironmentExample(), args);
    }

}