package de.gdl.rl.agents;
import java.util.HashSet;
import java.util.ArrayList;
import java.util.List;

public class RosAgent {

    public String name = "";
    public String type = ""; // not yet relevant (all are DQN atm)

    public HashSet<String> gdlRoleNames = new HashSet<String>();

    public boolean gameOverForIllegalActions = false;

    // ros - topics
    public String stateTopic = "";
    public String legalActionsTopic = "";
    public String actionTopic = "";

    // for training
    public int numberOfRandomEpisodes = 0; // determines how many initial episodes should be played completely randomly
    public float epsilon = 0.0f; // percentage of steps taken completely at random
    public float epsilonDecay = 0.0f; // determines how much epsilon decreases over time


    
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

    public RosAgent withGdlRoleNames(String[] gdlRoleNames) {
        
        for (String roleName : gdlRoleNames) {
            this.gdlRoleNames.add(roleName);
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

        List<String> gdlRoleNamesList = new ArrayList<String>(this.gdlRoleNames);
        String[] gdlRoleNameArr = gdlRoleNamesList.toArray(new String[gdlRoleNamesList.size()]);

       return new RosAgent()
        .withName(this.name)
        .withType(this.type)
        .withGdlRoleNames(gdlRoleNameArr)
        .withGameOverForIllegalActions(this.gameOverForIllegalActions)
        .withStateTopic(this.stateTopic)
        .withLegalActionsTopic(this.legalActionsTopic)
        .withActionTopic(this.actionTopic)
        .withNumberOfRandomEpisodes(this.numberOfRandomEpisodes)
        .withEpsilon(this.epsilon)
        .withEpsilonDecay(this.epsilonDecay);
    }
}
