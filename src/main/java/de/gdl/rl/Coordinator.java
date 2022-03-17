package de.gdl.rl;

import de.gdl.rl.ros.RosConnector;
import de.gdl.rl.ros.RosConnectorSubscriber;

import de.gdl.rl.environment.GDLGameEnvironment;

import java.util.Arrays;

import de.gdl.rl.agents.LocalAgent;
import de.gdl.rl.agents.RosAgent;
import de.gdl.rl.agents.RosTrainingAgent;

import java.util.HashMap;
import java.util.HashSet;

import java.util.concurrent.Semaphore;
import java.util.logging.Logger;
import java.util.logging.LogManager;
import java.util.logging.Level;
import java.util.logging.Handler;

import java.util.concurrent.ThreadLocalRandom;

public class Coordinator<ConcreteEnvironment extends GDLGameEnvironment> implements RosConnectorSubscriber
{

    private boolean WITH_DEBUG_OUTPUT = false;
    private boolean WITH_STATE_OUTPUT = false;
    private boolean WITH_GAME_RESULT_OUTPUT = true;

    public ConcreteEnvironment env;
    private RosConnector connector;

    private HashMap<String, Boolean> receivedResetFrom = new HashMap<String, Boolean>();
    
    private boolean isInTraining = false;
    private int numberOfEpisodesPlayed = 0;

    protected HashSet<String> manualRoles = new HashSet<String>();

    private final Semaphore resetSemaphore = new Semaphore(1, true);
    
    private RosAgent waitForActionFromAgent = null;
    private String waitForActionForRole = "";
    private String waitForActionFromTopic = "";

    protected boolean evaluation = false;
    protected int evaluationSamples = 0;

    public Coordinator(boolean isInTraining, ConcreteEnvironment env, boolean evaluation, int evaluationSamples) {
        
        this.evaluation = evaluation;
        this.evaluationSamples = evaluationSamples;

        this.isInTraining = isInTraining;

        if (!this.isInTraining) {
            Logger rootLogger = LogManager.getLogManager().getLogger("");
            rootLogger.setLevel(Level.OFF);
            for (Handler h : rootLogger.getHandlers()) {
                h.setLevel(Level.OFF);
            }
        }

        this.env = env;
        this.connector = new RosConnector(this);

        if (isInTraining) {
            for (RosTrainingAgent trainingAgent : this.env.train_config_trainingAgents) {
                // send to the training agent
                this.connector.addFloatArrPublisher(trainingAgent.stateTopic);
                if (trainingAgent.legalActionsTopic != "") this.connector.addFloatArrPublisher(trainingAgent.legalActionsTopic);
                this.connector.addFloatPublisher(trainingAgent.rewardTopic);
                this.connector.addBoolPublisher(trainingAgent.terminalTopic);
                // receive from the training agent
                this.connector.addIntSubscriber(trainingAgent.actionTopic);
                this.connector.addBoolSubscriber(trainingAgent.resetTopic);
    
                this.receivedResetFrom.put(trainingAgent.resetTopic, false);
            }
        }

        for (RosAgent agent : isInTraining ? this.env.train_config_agents : this.env.game_config_agents) {
            // send to the agent
            this.connector.addFloatArrPublisher(agent.stateTopic);
            this.connector.addFloatArrPublisher(agent.legalActionsTopic);
            // receive from the agent
            this.connector.addIntSubscriber(agent.actionTopic);
        }

        if (!isInTraining) {
            this.onInit();
            this.onStartGame();
            this.next();
        }

        // we can now wait for all training agents to send their reset()

        try {
            this.freeze();
        }  catch (Exception e) {
            if (this.WITH_DEBUG_OUTPUT) System.out.println(e);
        }

    }

    // RosConnectorSubscriber interface

    @Override
    public void receivedData(String topic, boolean value) {

        try {
            if (this.receivedResetFrom.containsKey(topic)) {
                resetSemaphore.acquire();
                this.receivedResetFrom.put(topic ,true);
                this.receivedReset();
            } 
        } catch (Exception e) {

        }

    }
    @Override
    public void receivedData(String topic, int value) {

        if (this.waitForActionFromTopic.equals(topic)) {
            this.performAction(value, this.waitForActionForRole, this.waitForActionFromAgent);
        }

    }

    protected boolean doesTrainingAgentControlRole(String roleName) {
        if (this.manualRoles.contains(roleName)) {
            return false;
        }

        for (RosTrainingAgent trainingAgent : this.env.train_config_trainingAgents) {
            if (trainingAgent.currentGdlRoleName.equals(roleName)) {
                return true;
            }
        }
        return false;
    } 

    private void receivedReset() {
        // check if we received all resets so that we can restart / initate a game

        for (String from : this.receivedResetFrom.keySet()) {
            if (!this.receivedResetFrom.get(from)) {
                resetSemaphore.release();
                return;
            }
        }

        // we received all: we can reset the list

        for (String from : this.receivedResetFrom.keySet()) {
            this.receivedResetFrom.put(from ,false);
        }
        
        // reset the GDL-Interpreter
        this.env.reset();
        // release the semaphore
        resetSemaphore.release();

        // give cli a chance to show the new state
        this.onStartGame();

        // we can go on playing
        this.next();

    }

    protected String getCurrentPlayerOfRole(String role) {
        if (this.manualRoles.contains(role)) return "manual";
        if (this.isInTraining) {
            for (RosTrainingAgent trainingAgent : this.env.train_config_trainingAgents) {
                if (trainingAgent.currentGdlRoleName.equals(role)) {
                    return trainingAgent.name;
                }
            }
        }
        for (RosAgent agent : this.isInTraining ? this.env.train_config_agents : this.env.game_config_agents) {
            if (agent.gdlRoleNames.contains(role)) {
                return agent.name;
            }
        }

        for (LocalAgent agent : this.isInTraining ? this.env.train_config_localAgents : this.env.game_config_localAgents) {
            if (agent.gdlRoleNames.contains(role)) {
                return agent.name;
            }
        }
        return "None";
    }

    private void next() {
        

        String currentGdlRoleName = this.env.whichRoleHasControl(); // needs to be implemented
        System.out.println("Role " + currentGdlRoleName + "has control");
        if (this.manualRoles.contains(currentGdlRoleName)) {

            this.waitForActionFromTopic = "-------NULL------";

            // get manually the move by CLI or GUI
            String moveString = this.onEnterMove(currentGdlRoleName);
            this.performAction(moveString, "CLI/GUI");
            return;
        } 
            
        // check if there is a training agent who can perform the action
        if (this.isInTraining) {
            for (RosTrainingAgent trainingAgent : this.env.train_config_trainingAgents) {
                if (trainingAgent.currentGdlRoleName.equals(currentGdlRoleName)) {
                    
                    float[] state = this.env.getStateAsFloatRepresentation(currentGdlRoleName);
                    boolean isTerminal = this.env.isTerminal();
                    float reward = (float) this.env.getReward(currentGdlRoleName);
                    
                    this.waitForActionForRole = currentGdlRoleName;
                    this.waitForActionFromTopic = trainingAgent.actionTopic;
                    this.waitForActionFromAgent = trainingAgent;
    
                    this.connector.publish(trainingAgent.stateTopic, state);
                    this.connector.publish(trainingAgent.terminalTopic, isTerminal);
                    this.connector.publish(trainingAgent.rewardTopic, reward);
    
                    if (trainingAgent.legalActionsTopic != "") {
                        float[] legalActions = this.env.getLegalActionsForPlayerAsIndicatorArray(currentGdlRoleName);
                        this.connector.publish(trainingAgent.legalActionsTopic, legalActions);
                    }
    
                    return;
                }
            }
        }

        // check if there is an agent who can perform the action
        for (RosAgent agent : this.isInTraining ? this.env.train_config_agents : this.env.game_config_agents) {
            if (agent.gdlRoleNames.contains(currentGdlRoleName)) {
                if (this.numberOfEpisodesPlayed < agent.numberOfRandomEpisodes) {
                    // play randomly
                    this.performAction(this.env.getRandomLegalMove(), agent.name + " (initial random)");
                    return;
                } else if(ThreadLocalRandom.current().nextDouble() > (1.0f - agent.epsilon)) {
                    // play randomly & decrease epsilon
                    agent.epsilon = agent.epsilon * agent.epsilonDecay;
                    this.performAction(this.env.getRandomLegalMove(), agent.name + " (epsiolon random)");
                    return;
                } else if(this.connector.gotPublisherForTopic(agent.actionTopic)) {
                    float[] state = this.env.getStateAsFloatRepresentation(currentGdlRoleName);
                    float[] legalActions = this.env.getLegalActionsForPlayerAsIndicatorArray(currentGdlRoleName);

                    this.waitForActionForRole = currentGdlRoleName;
                    this.waitForActionFromTopic = agent.actionTopic;
                    this.waitForActionFromAgent = agent;

                    this.connector.publish(agent.stateTopic, state);
                    this.connector.publish(agent.legalActionsTopic, legalActions);
                    return;
                }                    
          
            }
        }

         // check if there is a local agent who can perform the action
        for (LocalAgent agent : this.isInTraining ? this.env.train_config_localAgents : this.env.game_config_localAgents) {
            if (agent.gdlRoleNames.contains(currentGdlRoleName)) {
                this.performAction(agent.getMove(this.env.currentState, this.env.getLegalMovesForRole(currentGdlRoleName), currentGdlRoleName, this.numberOfEpisodesPlayed), agent.name);
                return;
            }
        }

        // play a random move if neither an agent nor an user takes action
        this.performAction(this.env.getRandomLegalMove(), "Random");
        

        
        

    }

    private void performAction(String move, String playerDescription) {
        // to be implemented
        this.onPreDoMove(move, playerDescription);
        if(this.env.step(move, null)) {
            this.onPostDoMove();
            this.postPerformAction();
        } else {
            this.onPostDoMove();
            this.onMoveWasIllegal(move);
            this.postPerformAction();
        }   
    }

    private void performAction(int action, String gdlRoleName, RosAgent agent) {
        
        this.onPreDoMove(this.env.getMoveStringFromAction(action, gdlRoleName), agent.name);

        if(this.env.step(action, gdlRoleName, agent)) {
            this.onPostDoMove();
            this.postPerformAction();
        } else {
            this.onPostDoMove();
            this.onMoveWasIllegal(this.env.getMoveStringFromAction(action, gdlRoleName));
            if (!agent.gameOverForIllegalActions) {
                this.performAction(this.env.getRandomLegalMove(), "Random");
            } else {
                this.postPerformAction();
            }
            // alternatively we try again
            //this.next();
        }
    }

    private void postPerformAction() {
        if(!this.env.isTerminal()) {
            // we executed the action and the game is on
            this.next();
        } else {
            // we executed the action and the game is over

            this.numberOfEpisodesPlayed++;

            // tell all the training agents that it is over
            for (RosTrainingAgent trainingAgent : this.env.train_config_trainingAgents) {
                float[] state = this.env.getStateAsFloatRepresentation(trainingAgent.currentGdlRoleName);
                boolean isTerminal = true;
                float reward = (float) this.env.getReward(trainingAgent.currentGdlRoleName);
                    
                this.connector.publish(trainingAgent.stateTopic, state);
                this.connector.publish(trainingAgent.terminalTopic, isTerminal);
                this.connector.publish(trainingAgent.rewardTopic, reward);
            }
            
            this.onGameIsOver();

            if (!this.isInTraining) {
                // in this case we can also reset the gdl interpreter
                this.env.reset();
                // we can go on playing                
                // give cli a chance to show the new state etc.
                this.onStartGame();
                // go on 
                this.next();
            }
            // else:
            //      if we are in the training, we can wait for all training-agents to get ready again


        }
    }


    // methods which can be used for subclassing:

    protected void onInit() {}
    protected void onStartGame() {}
    protected void onGameIsOver() {}
    protected String onEnterMove(String role) { return ""; }
    protected void onPreDoMove(String move, String playerDescription) {}
    protected void onPostDoMove() {}
    protected void onMoveWasIllegal(String move) {}

    public void freeze() throws InterruptedException {
        Object obj = new Object();
        synchronized (obj) {
            obj.wait();
        }
     }

}