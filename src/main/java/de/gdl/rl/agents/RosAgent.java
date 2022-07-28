package de.gdl.rl.agents;
import java.util.ArrayList;
import java.util.List;

import de.monticore.lang.gdl.types.GDLType;

public class RosAgent extends Agent {


    // ros - topics
    public String stateTopic = "";
    public String legalActionsTopic = "";
    public String actionTopic = "";



    
    public RosAgent() {
    }

    public RosAgent withName(String name) {
        this.name = name;
        return this;
    }

    public RosAgent withType(String type) {
        this.type = type;
        return this;
    }

    public RosAgent withGdlRoles(GDLType[] gdlRoles) {
        for (GDLType role : gdlRoles) {
            this.gdlRoles.add(role);
        }
        return this;
    }

    public RosAgent withGameOverForIllegalActions(boolean gameOverForIllegalActions) {
        this.gameOverForIllegalActions = gameOverForIllegalActions;
        return this;
    }

    public RosAgent withStateTopic(String stateTopic) {
        this.stateTopic = stateTopic;
        return this;
    }

    public RosAgent withLegalActionsTopic(String legalActionsTopic) {
        this.legalActionsTopic = legalActionsTopic;
        return this;
    }

    public RosAgent withActionTopic(String actionTopic) {
        this.actionTopic = actionTopic;
        return this;
    }

    public RosAgent withNumberOfRandomEpisodes(int numberOfRandomEpisodes) {
        this.numberOfRandomEpisodes = numberOfRandomEpisodes;
        return this;
    }

    public RosAgent withEpsilon(float epsilon) {
        this.epsilon = epsilon;
        return this;
    }

    public RosAgent withEpsilonDecay(float epsilonDecay) {
        this.epsilonDecay = epsilonDecay;
        return this;
    }

    public RosAgent copy() {
        List<GDLType> gdlRolesList = new ArrayList<>(this.gdlRoles);
        GDLType[] gdlRoleArr = gdlRolesList.toArray(new GDLType[gdlRolesList.size()]);

       return new RosAgent()
        .withName(this.name)
        .withType(this.type)
        .withGdlRoles(gdlRoleArr)
        .withGameOverForIllegalActions(this.gameOverForIllegalActions)
        .withStateTopic(this.stateTopic)
        .withLegalActionsTopic(this.legalActionsTopic)
        .withActionTopic(this.actionTopic)
        .withNumberOfRandomEpisodes(this.numberOfRandomEpisodes)
        .withEpsilon(this.epsilon)
        .withEpsilonDecay(this.epsilonDecay);
    }
}
