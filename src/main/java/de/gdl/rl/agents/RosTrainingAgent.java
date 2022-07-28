package de.gdl.rl.agents;

import java.util.ArrayList;
import java.util.List;

import de.monticore.lang.gdl.types.GDLType;


public class RosTrainingAgent extends RosAgent {

    public GDLType currentGdlRole = null;

    // ros topics
    public String terminalTopic = "";
    public String rewardTopic = "";
    public String resetTopic = "";

    public RosTrainingAgent() {
    }


    public RosTrainingAgent withName(String name) {
        this.name = name;
        return this;
    }

    public RosTrainingAgent withType(String type) {
        this.type = type;
        return this;
    }

    public RosTrainingAgent withGdlRoles(GDLType[] gdlRoles) {
        for (GDLType role : gdlRoles) {
            this.gdlRoles.add(role);
        }
        return this;
    }

    public RosTrainingAgent withGameOverForIllegalActions(boolean gameOverForIllegalActions) {
        this.gameOverForIllegalActions = gameOverForIllegalActions;
        return this;
    }

    public RosTrainingAgent withCurrentGdlRole(GDLType currentGdlRole) {
        this.currentGdlRole = currentGdlRole;
        return this;
    }

    public RosTrainingAgent withStateTopic(String stateTopic) {
        this.stateTopic = stateTopic;
        return this;
    }

    public RosTrainingAgent withLegalActionsTopic(String legalActionsTopic) {
        this.legalActionsTopic = legalActionsTopic;
        return this;
    }

    public RosTrainingAgent withActionTopic(String actionTopic) {
        this.actionTopic = actionTopic;
        return this;
    }

    public RosTrainingAgent withNumberOfRandomEpisodes(int numberOfRandomEpisodes) {
        this.numberOfRandomEpisodes = numberOfRandomEpisodes;
        return this;
    }

    public RosTrainingAgent withEpsilon(float epsilon) {
        this.epsilon = epsilon;
        return this;
    }

    public RosTrainingAgent withEpsilonDecay(float epsilonDecay) {
        this.epsilonDecay = epsilonDecay;
        return this;
    }

    public RosTrainingAgent withTerminalTopic(String terminalTopic) {
        this.terminalTopic = terminalTopic;
        return this;
    }

    public RosTrainingAgent withRewardTopic(String rewardTopic) {
        this.rewardTopic = rewardTopic;
        return this;
    }

    public RosTrainingAgent withResetTopic(String resetTopic) {
        this.resetTopic = resetTopic;
        return this;
    }

    public RosTrainingAgent copy() {
        List<GDLType> gdlRolesList = new ArrayList<>(this.gdlRoles);
        GDLType[] gdlRoleArr = gdlRolesList.toArray(new GDLType[gdlRolesList.size()]);

       return new RosTrainingAgent()
        .withName(this.name)
        .withType(this.type)
        .withGdlRoles(gdlRoleArr)
        .withGameOverForIllegalActions(this.gameOverForIllegalActions)
        .withStateTopic(this.stateTopic)
        .withLegalActionsTopic(this.legalActionsTopic)
        .withActionTopic(this.actionTopic)
        .withNumberOfRandomEpisodes(this.numberOfRandomEpisodes)
        .withEpsilon(this.epsilon)
        .withEpsilonDecay(this.epsilonDecay)
        .withCurrentGdlRole(this.currentGdlRole)
        .withTerminalTopic(this.terminalTopic)
        .withRewardTopic(this.rewardTopic)
        .withResetTopic(this.resetTopic);
    }
}
