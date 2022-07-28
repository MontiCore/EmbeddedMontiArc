package de.gdl.rl;

import de.gdl.rl.ros.RosConnector;
import de.gdl.rl.ros.RosConnectorSubscriber;
import de.monticore.lang.gdl.Command;
import de.monticore.lang.gdl.types.GDLType;
import de.gdl.rl.environment.GDLGameEnvironment;

import de.gdl.rl.agents.LocalAgent;
import de.gdl.rl.agents.RosAgent;
import de.gdl.rl.agents.RosTrainingAgent;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.Semaphore;
import java.util.logging.Logger;
import java.util.logging.LogManager;
import java.util.logging.Level;
import java.util.logging.Handler;

import java.util.concurrent.ThreadLocalRandom;

public class Coordinator<ConcreteEnvironment extends GDLGameEnvironment> implements RosConnectorSubscriber {

    private boolean WITH_DEBUG_OUTPUT = false;

    public ConcreteEnvironment env;
    private RosConnector connector;

    private Map<String, Boolean> receivedResetFrom = new HashMap<>();
    
    private boolean isInTraining = false;
    private int numberOfEpisodesPlayed = 0;

    /**
     * Can be used by a subclass in onInit()
     * to define roles to be controlled manually. 
     * Then, moves for these roles are resolved via onEnterMove().
     */
    protected Set<GDLType> manualRoles = new HashSet<>();

    private final Semaphore resetSemaphore = new Semaphore(1, true);
    
    private RosAgent waitForActionFromAgent = null;
    private GDLType waitForActionForRole = null;
    private String waitForActionFromTopic = null;

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
            e.printStackTrace();
        }
    }

    @Override
    public void receivedData(String topic, int value) {
        if (topic.equals(this.waitForActionFromTopic)) {
            this.performAction(value, this.waitForActionForRole, this.waitForActionFromAgent);
        }
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

    private void next() {
        List<GDLType> rolesInControl = this.env.whichRolesHaveControl(); 
        GDLType currentGdlRole = null;
        if (rolesInControl.size() > 1) {
            currentGdlRole = this.env.whichRoleShouldBeNext(rolesInControl);     
        } else {
            currentGdlRole = rolesInControl.get(0);
        }
        
        
        System.out.println(currentGdlRole + "'s turn");
        if (this.manualRoles.contains(currentGdlRole)) {

            this.waitForActionFromTopic = "-------NULL------";

            // get manually the move by CLI or GUI
            Command move = this.onEnterMove(currentGdlRole);
            this.performAction(move, currentGdlRole, "CLI/GUI");
            return;
        } 
            
        // check if there is a training agent who can perform the action
        if (this.isInTraining) {
            for (RosTrainingAgent trainingAgent : this.env.train_config_trainingAgents) {
                if (trainingAgent.currentGdlRole.equals(currentGdlRole)) {
                    
                    float[] state = this.env.getStateAsFloatRepresentation(currentGdlRole, trainingAgent);
                    boolean isTerminal = this.env.isTerminal();
                    float reward = (float) this.env.getReward(currentGdlRole, trainingAgent);
                    
                    this.waitForActionForRole = currentGdlRole;
                    this.waitForActionFromTopic = trainingAgent.actionTopic;
                    this.waitForActionFromAgent = trainingAgent;
    
                    this.connector.publish(trainingAgent.stateTopic, state);
                    this.connector.publish(trainingAgent.terminalTopic, isTerminal);
                    this.connector.publish(trainingAgent.rewardTopic, reward);
    
                    if (trainingAgent.legalActionsTopic != "") {
                        float[] legalActions = this.env.getLegalActionsForRoleAsIndicatorArray(currentGdlRole, trainingAgent);
                        this.connector.publish(trainingAgent.legalActionsTopic, legalActions);
                    }
    
                    return;
                }
            }
        }

        // check if there is an agent who can perform the action
        for (RosAgent agent : this.isInTraining ? this.env.train_config_agents : this.env.game_config_agents) {
            if (agent.gdlRoles.contains(currentGdlRole)) {
                if (this.numberOfEpisodesPlayed < agent.numberOfRandomEpisodes) {
                    // play randomly
                    this.performAction(this.env.getRandomLegalMove(), currentGdlRole, agent.name + " (initial random)");
                    return;
                } else if(ThreadLocalRandom.current().nextDouble() > (1.0f - agent.epsilon)) {
                    // play randomly & decrease epsilon
                    agent.epsilon = agent.epsilon * agent.epsilonDecay;
                    this.performAction(this.env.getRandomLegalMove(), currentGdlRole, agent.name + " (epsilon random)");
                    return;
                } else if(this.connector.gotPublisherForTopic(agent.actionTopic)) {
                    float[] state = this.env.getStateAsFloatRepresentation(currentGdlRole, agent);
                    float[] legalActions = this.env.getLegalActionsForRoleAsIndicatorArray(currentGdlRole, agent);

                    this.waitForActionForRole = currentGdlRole;
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
            if (agent.gdlRoles.contains(currentGdlRole)) {
                this.performAction(agent.getMove(this.env.getCurrentState(), this.env.getLegalMovesForRole(currentGdlRole), currentGdlRole, this.numberOfEpisodesPlayed), currentGdlRole, agent.name);
                return;
            }
        }

        // play a random move if neither an agent nor an user takes action
        this.performAction(this.env.getRandomLegalMove(), currentGdlRole, "Random");
    }

    private void performAction(Command move, GDLType gdlRole, String playerDescription) {
        this.onPreDoMove(move, playerDescription);
        if(this.env.step(move, null)) {
            this.onPostDoMove();
            this.postPerformAction();
        } else {
            this.onPostDoMove();
            this.onMoveWasIllegal(move, this.env.getLegalMovesForRole(gdlRole));
            this.postPerformAction();
        }   
    }

    private void performAction(int action, GDLType gdlRole, RosAgent agent) {
        
        this.onPreDoMove(this.env.getMoveFromAction(action, gdlRole, agent), agent.name);

        if(this.env.step(action, gdlRole, agent)) {
            this.onPostDoMove();
            this.postPerformAction();
        } else {
            this.onPostDoMove();
            this.onMoveWasIllegal(this.env.getMoveFromAction(action, gdlRole, agent), null);
            if (!agent.gameOverForIllegalActions) {
                this.performAction(this.env.getRandomLegalMove(), gdlRole, "Random");
            } else {
                this.postPerformAction();
            }
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
            if (this.isInTraining) {
                for (RosTrainingAgent trainingAgent : this.env.train_config_trainingAgents) {
                    float[] state = this.env.getStateAsFloatRepresentation(trainingAgent.currentGdlRole, trainingAgent);
                    boolean isTerminal = true;
                    float reward = (float) this.env.getReward(trainingAgent.currentGdlRole, trainingAgent);
                        
                    this.connector.publish(trainingAgent.stateTopic, state);
                    this.connector.publish(trainingAgent.terminalTopic, isTerminal);
                    this.connector.publish(trainingAgent.rewardTopic, reward);
                }
            }
            
            this.onGameIsOver(this.numberOfEpisodesPlayed);

            if (!this.isInTraining) {
                this.env.reset();
                this.onStartGame();
                this.next();
            }
        }
    }

    /**
     * Returns a description of the player/agent 
     * currently controlling a role
     * @param role a role
     * @return a description of the agent/player in control of the role
     */
    protected String getCurrentPlayerOfRole(GDLType role) {
        if (this.manualRoles.contains(role)) return "manual";
        if (this.isInTraining) {
            for (RosTrainingAgent trainingAgent : this.env.train_config_trainingAgents) {
                if (trainingAgent.currentGdlRole.equals(role)) {
                    return trainingAgent.name;
                }
            }
        }
        for (RosAgent agent : this.isInTraining ? this.env.train_config_agents : this.env.game_config_agents) {
            if (agent.gdlRoles.contains(role)) {
                return agent.name;
            }
        }

        for (LocalAgent agent : this.isInTraining ? this.env.train_config_localAgents : this.env.game_config_localAgents) {
            if (agent.gdlRoles.contains(role)) {
                return agent.name;
            }
        }
        return "None";
    }

    /**
     * Returns all training agents of the training configuration of the environment
     * @return lost of all training agents
     */
    protected List<RosTrainingAgent> getTrainingAgents() {
        return this.env.train_config_trainingAgents;
    }

    /**
     * Checks if the specified role is controlled by a training agent 
     * and returns if applicable.
     * @param roleName a role
     * @return the training agent which controls the role or otherwise null
     */
    protected RosTrainingAgent doesTrainingAgentControlRole(GDLType role) {
        if (this.manualRoles.contains(role)) {
            return null;
        }

        for (RosTrainingAgent trainingAgent : this.env.train_config_trainingAgents) {
            if (trainingAgent.currentGdlRole.equals(role)) {
                return trainingAgent;
            }
        }
        return null;
    } 

    /**
     * Event: Called after initialization 
     */
    protected void onInit() {}
    
    /**
     * Event: Called when a new episode/game starts 
     */
    protected void onStartGame() {}
    
    /**
     * Event: Called when an episode / a game is over 
     * @param numberOfEpisodesPlayed the number of the episode which is over
     */
    protected void onGameIsOver(int numberOfEpisodesPlayed) {}

    /**
     * Event: Called when a role that is manually controlled by the user is on the move
     * @param role the role for which the move is
     * @return a move for the role as a string
     */
    protected Command onEnterMove(GDLType role) { return null; }
    
    /**
     * Event: Called before a move is executed
     * @param move the move that is executed
     * @param playerDescription a description of the player/agent making the move
     */
    protected void onPreDoMove(Command move, String playerDescription) {}
    
    /**
     * Event: Called after a move was executed
     */
    protected void onPostDoMove() {}
    
    /**
     * Event: Called if a move was illegal and therefore not executed
     * @param move the illegal move
     * @param legalMoves a list of all legal moves or null if an agent executed the illegal move
     */
    protected void onMoveWasIllegal(Command move, List<Command> legalMoves) {}

    private void freeze() throws InterruptedException {
        Object obj = new Object();
        synchronized (obj) {
            obj.wait();
        }
     }

}