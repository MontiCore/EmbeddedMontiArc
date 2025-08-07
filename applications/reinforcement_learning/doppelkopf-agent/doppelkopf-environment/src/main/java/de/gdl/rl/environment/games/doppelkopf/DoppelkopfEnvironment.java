package de.gdl.rl.environment.games.doppelkopf;

import de.gdl.rl.agents.Agent;
import de.gdl.rl.agents.RosAgent;
import de.gdl.rl.agents.RosTrainingAgent;

import de.gdl.rl.environment.GDLGameEnvironment;
import de.monticore.lang.gdl.InterpreterOptions;
import de.monticore.lang.gdl.types.GDLNumber;
import de.monticore.lang.gdl.types.GDLType;

import java.util.Map;


public class DoppelkopfEnvironment extends GDLGameEnvironment {

    private RosTrainingAgent trainingAgent;
    
    public DoppelkopfEnvironment() {
        super(new InterpreterOptions().withTypes(true));

        final GDLType player1 = GDLType.createFromLine("player1");
        final GDLType player2 = GDLType.createFromLine("player2");
        final GDLType player3 = GDLType.createFromLine("player3");
        final GDLType player4 = GDLType.createFromLine("player4");

        final GDLType[] roles = new GDLType[]{player1, player2, player3, player4};

        this.trainingAgent = new RosTrainingAgent()
            .withName("Training Agent")
            .withType("DQN")
            .withGdlRoles(roles)
            .withGameOverForIllegalActions(true)
            .withCurrentGdlRole(player1)
            .withStateTopic("/gdl/doppelkopf/trainingAgent/state")
            .withActionTopic("/gdl/doppelkopf/trainingAgent/action")
            .withTerminalTopic("/gdl/doppelkopf/trainingAgent/terminal")
            .withRewardTopic("/gdl/doppelkopf/trainingAgent/reward")
            .withResetTopic("/gdl/doppelkopf/trainingAgent/reset");
            
        RosAgent agent = new RosAgent()
            .withName("Agent")
            .withType("DQN")
            .withGdlRoles(roles)
            .withStateTopic("/gdl/doppelkopf/agent/state")
            .withLegalActionsTopic("/gdl/doppelkopf/agent/legal_actions")
            .withActionTopic("/gdl/doppelkopf/agent/action");

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
        return "src/main/resources/gdl/games/DoppelkopfReducedTyped.gdl";
    }

    private int episode;
    private int won, lost, tie, illegal, other;

    @Override
    public void onNextEpisode() {
        episode++;
        System.out.printf("Episode: %d. Last 100: (Won: %d    Lost: %d    Tie: %d    Illegal: %d    Other: %d)\n", episode, won, lost, tie, illegal, other);
        if (episode % 100 == 0) {
            won = lost = tie = illegal = other = 0;
        }

        if (this.trainingAgent.currentGdlRole.equals(GDLType.createFromLine("player1"))) {
            this.trainingAgent.currentGdlRole = GDLType.createFromLine("player2");
        } else if (this.trainingAgent.currentGdlRole.equals(GDLType.createFromLine("player2"))) {
            this.trainingAgent.currentGdlRole = GDLType.createFromLine("player3");
        } else if (this.trainingAgent.currentGdlRole.equals(GDLType.createFromLine("player3"))) {
            this.trainingAgent.currentGdlRole = GDLType.createFromLine("player4");
        } else if (this.trainingAgent.currentGdlRole.equals(GDLType.createFromLine("player4"))) {
            this.trainingAgent.currentGdlRole = GDLType.createFromLine("player1");
        }
    }


    public float calculateRewardFromGoals(Map<GDLType, GDLType> goals, GDLType gdlRole, Agent agent) {
        if (episode < 5*110) {
            if (this.wasLastMoveIllegal() && this.whichRolesHaveControl().contains(gdlRole)) {
                return -8;
            } else {
                return 0.75f;
            }
        }

        if (!this.wasLastMoveIllegal() && this.isTerminal()) {
            int value = ((GDLNumber) goals.get(gdlRole)).getValue().intValue();

            if (value == 0) {
                lost++;
                return -6;
            }
            if (value == 50) {
                tie++;
                return 2;
            }
            if (value == 100)  {
                won++;
                return 5;
            }

            return (value/50) - 1;
        } else if (this.wasLastMoveIllegal() && this.whichRolesHaveControl().contains(gdlRole)) {
            illegal++;
            return -8;
        } else {
            other++;
            return 0.75f;
        }
    }

    public static void main(String[] args) throws Exception {
        GDLGameEnvironment.initEnvironment(new DoppelkopfEnvironment(), args);
    }

}