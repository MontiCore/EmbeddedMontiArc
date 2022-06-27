package de.gdl.rl.environment;

import de.gdl.rl.agents.Agent;
import de.gdl.rl.agents.LocalAgent;
import de.gdl.rl.agents.RosAgent;
import de.gdl.rl.agents.RosTrainingAgent;

import de.monticore.lang.gdl.Interpreter;
import de.monticore.lang.gdl.InterpreterOptions;
import de.monticore.lang.gdl.GDLInterpreter;
import de.monticore.lang.gdl._ast.ASTGame;
import de.monticore.lang.gdl._cocos.*;
import java.util.List;
import java.util.Set;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;

import de.monticore.lang.gdl.Command;
import de.monticore.lang.gdl.ASTGameExpressionCoCo;
import java.util.concurrent.ThreadLocalRandom;
import java.util.stream.Collectors;

import org.apache.commons.collections4.list.UnmodifiableList;

import de.gdl.rl.cli.GamingCLI;
import de.gdl.rl.cli.TrainingCLI;
import org.junit.Assert;

public abstract class GDLGameEnvironment implements RlGdlGameEnvironment {
    
    public List<RosAgent> train_config_agents = new ArrayList<RosAgent>();
    public List<RosTrainingAgent> train_config_trainingAgents = new ArrayList<RosTrainingAgent>();
    public List<LocalAgent> train_config_localAgents = new ArrayList<LocalAgent>();

    public List<RosAgent> game_config_agents = new ArrayList<RosAgent>();
    public List<LocalAgent> game_config_localAgents = new ArrayList<LocalAgent>();


    private Interpreter interpreter;
    
    private Set<List<String>> currentState;
    private Set<List<String>> currentHiddenState;
    private boolean terminal = false;

    private boolean lastStepWasIllegal = false;
    private String lastExecutedMove = "";

    private Set<List<String>> legalMovesCached;
    private boolean legalMovesCouldHaveChanged = true;

    private final ASTGame ast;
    private final InterpreterOptions options;

    public GDLGameEnvironment() {
        this(null);
    }

    public GDLGameEnvironment(InterpreterOptions options) {
        ast = GDLInterpreter.parse(this.getPathToGdlModel());
        this.options = options;
        GDLCoCoChecker checker = new GDLCoCoChecker();
        
        checker.addCoCo(new ASTGameExpressionCoCo());
        checker.checkAll(ast);
        
        try {
            this.interpreter = new Interpreter(ast);
            if (options != null) {
                interpreter.init(options);
            } else {
                interpreter.init();
            }

            this.currentState = this.interpreter.getGameState();
            this.currentHiddenState = this.interpreter.getHiddenGameState();
            this.terminal = this.interpreter.isTerminal();
        } catch (Exception e) {
            this.interpreter = null;
            e.printStackTrace();
        }
    }

    /**
    * [Override me]
    * Specifies the path to the GDL model 
    * to be used by the game environment.
    * @return relative path to the GDL model
    */
    protected abstract String getPathToGdlModel();

    /**
    * Adds a ROS agent to the gaming configuration
    * @param  agent the agent to be added to the configuration
    */
    protected void addToGamingConfiguration(RosAgent agent) {
        game_config_agents.add(agent);
    }

    /**
    * Adds a local agent to the gaming configuration.
    * Local agents can be used as training baselines or to connect game engines, etc.
    * @param  agent the local agent to be added to the configuration
    */
    protected void addToGamingConfiguration(LocalAgent agent) {
        game_config_localAgents.add(agent);
    }

    /**
    * Adds a ROS agent to the training configuration
    * @param  agent the agent to be added to the configuration
    */
    protected void addToTrainingConfiguration(RosAgent agent) {
        train_config_agents.add(agent);
    }

    /**
    * Adds a ROS training agent to the training configuration
    * @param  agent the ROS training agent to be added to the configuration
    */
    protected void addToTrainingConfiguration(RosTrainingAgent agent) {
        train_config_trainingAgents.add(agent);
    }

    /**
    * Adds a local agent to the gaming configuration.
    * Local agents can be used as training baselines or to connect game engines, etc.
    * @param  agent the local agent to be added to the configuration
    */
    protected void addToTrainingConfiguration(LocalAgent agent) {
        train_config_localAgents.add(agent);
    }

    /**
    * Determines which role should perform the next move 
    * if there are multiple roles to choose from.
    * @param  availableRoles  a list of all roles that are available for selection
    * @return The role that should perform the next move.
    */
    public String whichRoleShouldBeNext(List<String> availableRoles) {
        return availableRoles.get(ThreadLocalRandom.current().nextInt(0, availableRoles.size()));
    }

    /**
    * Returns a list of roles that are allowed to perform a move.
    * @return List of roles that are allowed to make a move
    */
    public List<String> whichRolesHaveControl() {

        HashSet<String> rolesInConrol = new HashSet<String>();
        if (!this.interpreter.isTerminal()) {
            Set<List<String>> allLegalMoves = _getAllLegalMoves();
            for(List<String> move : allLegalMoves) {
                rolesInConrol.add(move.get(0));
            }
        }
        List<String> result = new ArrayList<String>();
        result.addAll(rolesInConrol);

        return result;
    }

    /**
    * Returns a list of all legal moves for a given role 
    * @param gdlRoleName the role for which the legal moves are to be determined 
    * @return List of of legal moves 
    */
    public List<String> getLegalMovesForRole(String gdlRoleName) {
        Set<List<String>> legalMovesForPlayer = this._getAllLegalMovesForRole(gdlRoleName);
        List<String> legalActions = new ArrayList<String>();

        for (List<String> move : legalMovesForPlayer) {
            legalActions.add(Command.createMoveFromList(move).toString());
        }
        return legalActions;
    }

    private Set<List<String>> _getAllLegalMoves() {
        if (!legalMovesCouldHaveChanged) {
            return this.legalMovesCached;
        }
        legalMovesCouldHaveChanged = false;
        legalMovesCached = this.interpreter.getAllLegalMoves();
        return legalMovesCached;
    }

    private  Set<List<String>> _getAllLegalMovesForRole(String gdlRoleName) {
        Set<List<String>> allLegalMoves = this._getAllLegalMoves();
        if (allLegalMoves != null) {
            allLegalMoves = allLegalMoves
            .stream()
            .filter(
                l -> l != null && l.size() > 0 && l.get(0).equals(gdlRoleName)
            )
            .collect(Collectors.toSet());
            return allLegalMoves;
        } else {
            return Set.of();
        }  
    }

    /**
    * Returns an indicator vector as float array indicating
    * which moves are legal for an agent and role
    * @param gdlRoleName the role for which the indicator vector is determined
    * @param agent the agent for which the indicator vector is determined 
    * @return the float indicator array that encodes the legal moves for a role and agent
    */
    public float[] getLegalActionsForRoleAsIndicatorArray(String gdlRoleName, Agent agent) {
        return getLegalActionsForRoleAsIndicatorArray(gdlRoleName, false, agent);
    }

    /**
    * Returns an indicator vector as float array indicating
    * which moves are legal for an agent and role. 
    * If training is performed, all moves are considered legal.
    * @param gdlRoleName the role for which the indicator vector is determined
    * @param isTraining indicates whether a training is performed
    * @param agent the agent for which the indicator vector is determined
    * @return the float indicator array that encodes the legal moves for a role and agent
    */
    public float[] getLegalActionsForRoleAsIndicatorArray(String gdlRoleName, boolean isTraining, Agent agent) {
        float[] output = new float[this.getNumberOfActionsForRole(gdlRoleName)];
        
        if (isTraining) {
            // we allow all moves during the training
            Arrays.fill(output, 1.0f);
        } else {
            int[] legal_actions = this.getLegalActionsForPlayer(gdlRoleName, agent);
            Arrays.fill(output, 0.0f);
            for (int i = 0; i < legal_actions.length; i++) {
                output[legal_actions[i]] = 1.0f;
            }
        }
        
        return output;
    }

    /**
    * Returns a list of all numbers of actions 
    * that an agent can legally perform for a role
    * @param gdlRoleName 
    * @param agent 
    * @return list of all numbers of legal actions
    */
    public int[] getLegalActionsForPlayer(String gdlRoleName, Agent agent) {
        Set<List<String>> legalMovesForPlayer = this._getAllLegalMovesForRole(gdlRoleName);

        List<Integer> legalActions = new ArrayList<Integer>();

        for (List<String> move : legalMovesForPlayer) {

            String role = move.get(0);
            List<String> moveStringSplitted = UnmodifiableList.unmodifiableList(move.subList(1, move.size()));

            legalActions.add(this.getActionFromMoveString(moveStringSplitted, role, agent));
        }

        return legalActions.stream().mapToInt(i -> i).toArray();
    }

    /**
    * Returns a random legal move
    * @return a random legal move
    */
    public String getRandomLegalMove() {

        List<String> rolesInControl = this.whichRolesHaveControl();
        if (rolesInControl.size() < 1) {
            return "";
        }

        List<String> legalMoves = getLegalMovesForRole( rolesInControl.get(ThreadLocalRandom.current().nextInt(0, rolesInControl.size())) );
        if (legalMoves.size() == 0) {
            return "";
        }
        int randomNum = ThreadLocalRandom.current().nextInt(0, legalMoves.size());
        return legalMoves.get(randomNum);
    }

    /**
    * Converts an agent's action for a role to the appropriate move and performs it on the game environment.
    * @param action the number of the action
    * @param gdlRoleName the corresponding role
    * @param agent the agent that performs the action
    * @return true if the move was legal and executed; otherwise false
    */
    public boolean step(int action, String gdlRoleName, Agent agent) {

        return this.step(this.getMoveStringFromAction(action, gdlRoleName, agent), agent);

    }

    /**
    * Performs a move on the game environment specified as a string
    * @param moveString the entire move (including the name of the role) to be performed
    * @param gdlRoleName the corresponding role
    * @param agent the agent that performs the move if the move comes from an agent, otherwise null
    * @return true if the move was legal and executed; otherwise false
    */
    public boolean step(String moveString, Agent agent) {
        Set<List<String>> nextState = interpreter.interpret(moveString);

        if (nextState != null) { 
            this.legalMovesCouldHaveChanged = true;
            this.currentState = nextState;
            this.lastExecutedMove = moveString;

            if (interpreter.isTerminal()) {
                this.terminal = true;
            } 

            return true;
            
        } else if (agent != null && agent.gameOverForIllegalActions){
            this.terminal = true;
            this.lastStepWasIllegal = true;
        } 

        return false;
    }

    /**
     * Returns a list of all currently achieved goals
     * @return a list of tuple describing all the goals achieved
     */
    public Set<List<String>> getReachedGoals() {
        return interpreter.getAllModels("goal");
    }

    /**
     * Returns a list of all currently achieved targets. The tuple are encoded as strings.
     * @return a list of strings describing tuple of all the goals achieved
     */
    public List<String> getReachedGoalsAsStrings() {
        Set<List<String>> goals = interpreter.getAllModels("goal");
        List<String> stringGoals = new ArrayList<String>();

        for (List<String> goal : goals) {
            String stringGoal = "(" + String.join(", ", goal) + ")";
            stringGoals.add(stringGoal);
        }
        return stringGoals;

    }

    /**
     * Gives the reward of an agent received for a specific role.
     * @param gdlRoleName the role for which the reward was achieved
     * @param agent the agent for which the reward is
     * @return a decimal number indicating the reward
     */
    public float getReward(String gdlRoleName, Agent agent) {
        Set<List<String>> goals = interpreter.getAllModels("goal");
        if (goals != null) {
            return this.calculateRewardFromGoals(goals, gdlRoleName, agent);
        }
        return 0;
    }  
    
    /**
     * Called before a new episode begins.
     * Can be used by a subclass.
     */
    public void onNextEpisode() {}

    /**
     * Resets the game environment to its original state. 
     * Can be used to start a new episode.
     */
    public void reset() {
        try {
            if (options != null)
                this.interpreter = new Interpreter(ast).init(options);
            else
                this.interpreter = new Interpreter(ast).init();
        } catch (Exception e) {
            e.printStackTrace();
        }
        this.lastStepWasIllegal = false;
        this.legalMovesCouldHaveChanged = true;
        this.currentState = this.interpreter.getGameState();
        this.terminal = this.interpreter.isTerminal();
     
        this.onNextEpisode();
    }
    /**
     * Returns a list of all tuple of the current state.
     * @return a list of all tuple (encoded as list of strings) of the current state
     */
    public Set<List<String>> getCurrentState() {
        return this.currentState;
    }

    /**
     * Returns a list of all tuple of the current hidden state.
     * @return a list of all tuple (encoded as list of strings) of the current hidden state
     */
    public Set<List<String>> getCurrentHiddenState() {
        return this.currentHiddenState;
    }

    /**
     * Returns a list of all tuple of the current visible state by role.
     * @return a list of all tuple (encoded as list of strings) of the current visible state by role
     */
    public Set<List<String>> getCurrentStateByRole(String role) {
        return this.interpreter.getGameStateForRole(role);
    }

    /**
     * Indicates whether the game is in a final state or has ended.
     * @return true if the game is over, false otherwise 
     */
    public boolean isTerminal() {
        return this.terminal;
    }

    /**
     * Returns the last move performed on the game environment
     * @return the last move as string
     */
    public String getLastExecutedMove() {
        return this.lastExecutedMove;
    }

    /**
     * Indicates whether the last executed move was illegal.
     * @return true if the last move was illegal, false otherwise
     */
    public boolean wasLastMoveIllegal() {
        return this.lastStepWasIllegal;
    }

    /**
     * Returns the name of the loaded GDL model
     * @return the name of the loaded GDL model
     */
    public String getNameOfGame() {
        String[] splitted = this.getPathToGdlModel().split("/");
        return splitted[splitted.length - 1];
    }

    /**
     * Returns the current state as a string 
     * that lists raw all tuple of the state
     * @return a string that lists all tuple of the current state
     */
    public String getStateAsString() {
        StringBuilder readableStringBuilder = new StringBuilder();
        interpreter.getGameState().forEach(s -> readableStringBuilder.append("\t" + s + "\n"));
        interpreter.getHiddenGameState().forEach(s -> readableStringBuilder.append("\t" + s + "\n"));
        return readableStringBuilder.toString();
    }

    /**
     * [Override me]
     * This method should be overridden by a subclass 
     * and return a readable encoding of the current state as a string.
     * @return a readable representation of the current state as a string
     */
    public String getStateAsReadableString() {
        StringBuilder readableStringBuilder = new StringBuilder();
        readableStringBuilder.append("---- Game State (" + interpreter.getGameState().size() +  ") ----\n");
        readableStringBuilder.append(this.getStateAsString());
        for (int i = 0; i < ("" + interpreter.getGameState().size()).length() + 23; i++) {
            readableStringBuilder.append("-");
        }
        readableStringBuilder.append("\n");
        return readableStringBuilder.toString();
    }
 
    /**
     * [Override me]
     * Returns the reward for an agent for a role.
     * By default, the reward is the currently achieved goal of the specified role.
     * If no goal is reached, 0 is returned. 
     * The method can be overridden by a subclass
     * to allow a custom reward function.
     * @param goals a list of all achieved goals as tuple
     * @param gdlRoleName the role for which the reward was achieved
     * @param agent the agent for which the reward is
     * @return a number that gives the reward for an agent for a role
     */
    public float calculateRewardFromGoals(Set<List<String>> goals, String gdlRoleName, Agent agent) {
        
        for (List<String> goal : goals) {
            if (goal.get(0) == gdlRoleName) {
                return Float.parseFloat(goal.get(1));
            }
        }
        return 0.0f;
    }

    /**
     * Can be used to run a CLI for training or gaming for a gaming environment. 
     * This method should be called in a concrete environment in the main method 
     * and the concrete environment should pass itself.
     * It reads CLI parameters and starts the corresponding configuration.
     * @param env a concrete game environment that should be executed
     * @param args arguments read via console
     */
    public static <ConcreteEnvironment extends GDLGameEnvironment> void initEnvironment(ConcreteEnvironment env, String[] args) {

        boolean training = false;
        boolean evaluation = false;
        int evaluationSamples = 0;
        boolean evaluationNext = false;

        List<String> commands = List.of(args);
        for (int i = 0; i < commands.size(); i++) {
            String command = commands.get(i);
            if (command.equals("--training") || command.equals("-t")) {
                training = true;
            }
            if (command.equals("--gaming") || command.equals("-g")) {
                training = false;
            }

            if (command.equals("--evaluation") || command.equals("-eval")) {
                evaluation = true;
                evaluationNext = true;
                System.out.println("Hit enter twice to start the evaluation without controlling roles manually.");
            } else if(evaluationNext) {
                evaluationNext = false;
                try {
                    evaluationSamples = Integer.parseInt(command);
                } catch (NumberFormatException e) {
                    
                }
            }
        }
        
        try {
            if (!training) {
                new GamingCLI<ConcreteEnvironment>(env, evaluation, evaluationSamples);
            } else {
                new TrainingCLI<ConcreteEnvironment>(env);
            }

        } catch (Exception e) {
        
            e.printStackTrace();
            System.out.println(e);
        }
    }

    /**
     * Converts a move encoded as a string into a list 
     * containing the role and the individual entries of the tuple describing the move
     * @param move the move as string
     * @return the move as a list of strings
     */
    public static List<String> splitMoveString(String move) {
        if (!move.matches("[a-zA-z0-9]+ \\(([a-zA-z0-9])+( [a-zA-z0-9]+)*\\)")) {
            System.out.println("Move format wrong");
            return null;
        }
        String[] split = move
            .replace("(", "")
            .replace(")", "")
            .split(" ");
       return List.of(split);
    }

    /**
     * Tests the mapping of the state to a float array.
     * The conversion is tested for all specified agents of all 
     * configurations and all possible roles.
     * For verification, the state is printed before and after conversion. 
     * The outputs can be used to check for correctness.
     * A number of random moves can be executed to test a range of state conversions.
     * @param env a concrete game environment for which the state transformation should be tested
     * @param numberOfRandomMoves a number of random moves to be executed to generate a test series
     */
    public static <ConcreteEnvironment extends GDLGameEnvironment> void stateMappingTest(ConcreteEnvironment env, int numberOfRandomMoves) {
        
        if (numberOfRandomMoves < 0) {
            return;
        }

        System.out.println("Please check the following conversions for correctness:");

        ArrayList<Agent> allAgentsForTesting = new ArrayList<Agent>();
        allAgentsForTesting.addAll(env.train_config_trainingAgents);
        allAgentsForTesting.addAll(env.train_config_agents);
        allAgentsForTesting.addAll(env.game_config_agents);

        for (Agent agent : allAgentsForTesting) {
            for (String roleName : agent.gdlRoleNames) {
                env.reset();
                for (int i = 0; i < numberOfRandomMoves + 1; i++) {
                    System.out.println("stateConversionTest - agent: " + agent.name + ", roleName: " + roleName);
                    System.out.println("Input:");
                    
                    String readableString = env.getStateAsReadableString();
                    String rawString = env.getStateAsString();

                    if (!readableString.equals(rawString)) {
                        System.out.println("getStateAsReadableString()");
                        System.out.println(readableString);
                    } 
                    System.out.println("getStateAsString()");
                    System.out.println(rawString);
                    float[] floaRepr = env.getStateAsFloatRepresentation(roleName, agent);
                    System.out.println("Output:");
                    System.out.println(Arrays.toString(floaRepr));

                    if (!env.isTerminal()) {
                        env.step(env.getRandomLegalMove(), null);
                    } else {
                        break;
                    }

                }
            }
        }
    }

    /**
     * Tests the numbering of the actions or moves.
     * The bidirectional mapping is tested for all specified agents of all 
     * configurations and all possible roles.
     * Each action coded as a number is first 
     * mapped to the corresponding move and back.
     * For each action it is checked if the 
     * original number corresponds to the mapped one.
     * All mappings are printed out so that it can be used for additional manual inspection.
     * @param env a concrete game environment for which the numbering of the action should be tested.
     */
    public static <ConcreteEnvironment extends GDLGameEnvironment> void bidirectionalActionMappingTest(ConcreteEnvironment env) {

        ArrayList<Agent> allAgentsForTesting = new ArrayList<Agent>();
        allAgentsForTesting.addAll(env.train_config_trainingAgents);
        allAgentsForTesting.addAll(env.train_config_agents);
        allAgentsForTesting.addAll(env.game_config_agents);

        for (Agent agent : allAgentsForTesting) {
            for (String roleName : agent.gdlRoleNames) {

                System.out.println("bidirectionalMappingTest - agent: " + agent.name + ", roleName: " + roleName);

                int moves = env.getNumberOfActionsForRole(roleName);
                for (int j = 0; j < moves; j++) {
                    
                    String moveString = env.getMoveStringFromAction(j, roleName, agent);
                    ArrayList<String> splitted = new ArrayList<String>();
                    splitted.addAll(GDLGameEnvironment.splitMoveString(moveString));
                    
                    String splittedRoleName = splitted.get(0);
                    splitted.remove(0);
                    int actionNumber = env.getActionFromMoveString(splitted, splittedRoleName, agent);
                    
                    Assert.assertEquals(j, actionNumber);

                    System.out.print("Action: " + j + " - ");
                    System.out.print(moveString);
                    System.out.println(" - " + actionNumber + " - OK");
                }
            }
        }
    }

    public static void main(String[] args) throws Exception {
        
        System.out.println("Please implement main method");
        System.out.println("Call GDLGameEnvironment.initEnvironment(YourEnvironment env, String[] args) within main");
    }

}
