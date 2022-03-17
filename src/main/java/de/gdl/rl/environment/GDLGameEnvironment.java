package de.gdl.rl.environment;

import de.gdl.rl.agents.LocalAgent;
import de.gdl.rl.agents.RosAgent;
import de.gdl.rl.agents.RosTrainingAgent;

import de.monticore.lang.gdl.Interpreter;
import de.monticore.lang.gdl.GDLInterpreter;
import de.monticore.lang.gdl._ast.ASTGame;
import de.monticore.lang.gdl._cocos.*;
import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import de.monticore.lang.gdl.Command;
import de.monticore.lang.gdl.ASTGameExpressionCoCo;
import java.util.concurrent.ThreadLocalRandom;

import de.gdl.rl.cli.GamingCLI;
import de.gdl.rl.cli.TrainingCLI;


public abstract class GDLGameEnvironment implements RlGdlGameEnvironment {
    
    public List<RosAgent> train_config_agents = new ArrayList<RosAgent>();
    public List<RosTrainingAgent> train_config_trainingAgents = new ArrayList<RosTrainingAgent>();
    public List<LocalAgent> train_config_localAgents = new ArrayList<LocalAgent>();

    public List<RosAgent> game_config_agents = new ArrayList<RosAgent>();
    public List<LocalAgent> game_config_localAgents = new ArrayList<LocalAgent>();


    private Interpreter interpreter;
    
    public List<List<String>> currentState;
    public boolean terminal = false;

    private boolean lastStepWasIllegal = false;
    private String lastExecutedMove = "";
    
    protected HashMap<String, Integer> numberOfPossibleActionsForRole = new HashMap<String, Integer>();


    public GDLGameEnvironment() {

        final ASTGame ast = GDLInterpreter.parse(this.getPathToGdlModel());
        GDLCoCoChecker checker = new GDLCoCoChecker();
        
        checker.addCoCo(new ASTGameExpressionCoCo());
        checker.checkAll(ast);
        
        try {
            this.interpreter = new Interpreter(ast).init();

            this.currentState = this.interpreter.getGameState();
            this.terminal = this.interpreter.isTerminal();
        } catch (Exception e) {
            this.interpreter = null;
        }
        

    }

    protected abstract String getPathToGdlModel();

    // add new agent to the environment

    protected void addToGamingConfiguration(RosAgent agent) {
        game_config_agents.add(agent);
    }

    protected void addToGamingConfiguration(LocalAgent agent) {
        game_config_localAgents.add(agent);
    }

    protected void addToTrainingConfiguration(RosAgent agent) {
        train_config_agents.add(agent);
    }

    protected void addToTrainingConfiguration(RosTrainingAgent agent) {
        train_config_trainingAgents.add(agent);
    }

    protected void addToTrainingConfiguration(LocalAgent agent) {
        train_config_localAgents.add(agent);
    }

    // determine who has control

    public String whichRoleHasControl() {
        for (List<String> tuple : this.currentState) {
            if (tuple.get(0).equals("control")) {
                return tuple.get(1);
            }
        }
        return "";
    }

    // get legal actions & moves for a role

    public List<String> getLegalMovesForRole(String gdlRoleName) {
        List<List<String>> legalMovesForPlayer = this.interpreter.getAllLegalMovesForPlayer(gdlRoleName);
        List<String> legalActions = new ArrayList<String>();

        for (List<String> move : legalMovesForPlayer) {
            legalActions.add(Command.createMoveFromList(move).toString());
        }
        return legalActions;
    }

    public float[] getLegalActionsForPlayerAsIndicatorArray(String gdlRoleName) {
        return getLegalActionsForPlayerAsIndicatorArray(gdlRoleName, false);
    }

    public float[] getLegalActionsForPlayerAsIndicatorArray(String gdlRoleName, boolean isTraining) {
        float[] output = new float[this.numberOfPossibleActionsForRole.get(gdlRoleName)];
        
        if (isTraining) {
            // we allow all moves during the training
            Arrays.fill(output, 1.0f);
        } else {
            int[] legal_actions = this.getLegalActionsForPlayer(gdlRoleName);
            Arrays.fill(output, 0.0f);
            for (int i = 0; i < legal_actions.length; i++) {
                output[legal_actions[i]] = 1.0f;
            }
        }
        
        return output;
    }

    public int[] getLegalActionsForPlayer(String gdlRoleName) {
        List<List<String>> legalMovesForPlayer = this.interpreter.getAllLegalMovesForPlayer(gdlRoleName);

        List<Integer> legalActions = new ArrayList<Integer>();

        for (List<String> move : legalMovesForPlayer) {
            legalActions.add(this.getActionFromMoveString(Command.createMoveFromList(move)));
        }

        return legalActions.stream().mapToInt(i -> i).toArray();
    }

    public String getRandomLegalMove() {
        List<String> legalMoves = getLegalMovesForRole(this.whichRoleHasControl());
        if (legalMoves.size() == 0) {
            return "";
        }
        int randomNum = ThreadLocalRandom.current().nextInt(0, legalMoves.size());
        return legalMoves.get(randomNum);
    }

    // do a move / perform an action

    public boolean step(int action, String gdlRoleName, RosAgent agent) {

        return this.step(this.getMoveStringFromAction(action, gdlRoleName), agent);

    }

    public boolean step(String moveString, RosAgent agent) {
        List<List<String>> nextState = interpreter.interpret(moveString);
        
        if (nextState != null) { 
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

    public boolean wasLastStepIllegal() {
        return this.lastStepWasIllegal;
    }

    public List<List<String>> getReachedGoals() {
        return interpreter.getAllModels("goal");
    }

    public List<String> getReachedGoalsAsStrings() {
        List<List<String>> goals = interpreter.getAllModels("goal");
        List<String> stringGoals = new ArrayList<String>();

        for (List<String> goal : goals) {
            String stringGoal = "(" + String.join(", ", goal) + ")";
            stringGoals.add(stringGoal);
        }
        return stringGoals;

    }

    public float getReward(String gdlRoleName) {
        List<List<String>> goals = interpreter.getAllModels("goal");
        if (goals != null) {
            return this.calculateRewardFromGoals(goals, gdlRoleName);
        }
        return 0;
    }  
    

    public boolean isTerminal() {
        return this.terminal;
    }

    public void reset() {
        this.interpreter.reset();
        this.lastStepWasIllegal = false;
        this.currentState = this.interpreter.getGameState();
        this.terminal = this.interpreter.isTerminal();

    }

    public String getLastExecutedMove() {
        return this.lastExecutedMove;
    }

    public String[] getAvailableRoles() {
        return new String[] {"x", "o"}; // TBD: get available roles from GDL - Interpreter
    }

    // to be subclassed by concrete environment

    public String getNameOfGame() {
        String[] splitted = this.getPathToGdlModel().split("/");
        return splitted[splitted.length - 1];
    }

    public String getStateAsReadableString() {
        StringBuilder readableStringBuilder = new StringBuilder();
        readableStringBuilder.append("---- Game State (" + interpreter.getGameState().size() +  ") ----\n");
        interpreter.getGameState().forEach(s -> readableStringBuilder.append("\t" + s + "\n"));

        for (int i = 0; i < ("" + interpreter.getGameState().size()).length() + 23; i++) {
            readableStringBuilder.append("-");
        }
        readableStringBuilder.append("\n");
        return readableStringBuilder.toString();
    }

    public float calculateRewardFromGoals(List<List<String>> goals, String gdlRoleName) {
        return 0.0f;
    }

    protected int getActionFromMoveString(Command move) {
        return -1;
    }

    public String getMoveStringFromAction(int action, String gdlRoleName) {
        return "";
    }

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
    public static void main(String[] args) throws Exception {
        
        System.out.println("Please implement main method");
        System.out.println("Call GDLGameEnvironment.initEnvironment(ConcreteEnvironment env, String[] args) within main");
    }

}
