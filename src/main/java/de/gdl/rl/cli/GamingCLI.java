package de.gdl.rl.cli;
import de.gdl.rl.Coordinator;

import java.util.HashMap;
import java.util.Scanner;
import java.util.Map.Entry;
import java.util.List;
import java.util.Map;
import java.util.regex.Pattern;
import de.gdl.rl.environment.GDLGameEnvironment;
import de.monticore.lang.gdl.Command;
import de.monticore.lang.gdl.types.GDLNumber;
import de.monticore.lang.gdl.types.GDLType;



public class GamingCLI<ConcreteEnvironment extends GDLGameEnvironment> extends Coordinator<ConcreteEnvironment> {

    boolean moveWasEnteredByUser = false;
    Scanner scanner = new Scanner(System.in).useDelimiter(Pattern.compile("(\\n)|;"));

    private int numberOfEpisodes = 0;

    protected Map<String, Map<String, Integer>> evaluationTable;


    public GamingCLI(ConcreteEnvironment env, boolean evaluation, int evaluationSamples) {
        super(false, env, evaluation, evaluationSamples);
    }

    protected void onInit() {

        if (this.scanner == null) {
            this.scanner = new Scanner(System.in).useDelimiter(Pattern.compile("(\\n)|;"));
        }

        System.out.println("");
        System.out.println("Welcome to the GDL-Gaming-CLI");
        System.out.println("    Game: " + this.env.getNameOfGame());
        System.out.println("");
        
        if (this.evaluation) {
            System.out.println("Evaluation: " +this.evaluationSamples + " episodes");
        }
        
        System.out.println("Type comma-separated all roles you would like to play:");
        System.out.println("    (e.g. role1, role2, role3, ...)");
        String[] manualRolesRaw = this.scanner.next().split(",");
        for (String pseudo_role : manualRolesRaw) {
            if (!pseudo_role.isBlank()) {
                this.manualRoles.add(GDLType.createFromLine(pseudo_role.strip()));
            }
        }
        this.scanner.nextLine();

        // TBD: check if manualRoles contains only roles which exist

        System.out.println("");
    }

    protected void onStartGame() {
        System.out.println("New game");
        System.out.println(""); // TBD: print which roles are controlled by whom
        System.out.println(this.env.getStateAsReadableString());
    }

    protected Command onEnterMove(GDLType role) {
        System.out.println("Enter a move for '" + role + "':");
        this.scanner = new Scanner(System.in).useDelimiter(Pattern.compile("(\\n)|;"));
        String move = this.scanner.next();
        this.scanner.nextLine();
        this.moveWasEnteredByUser = true;
        // TBD: check if move has special format: /legalMoves -> shows all legal moves for the role

        return Command.createFromLine(move); 
    }

    protected void onMoveWasIllegal(String move, List<String> legalMoves) {
        System.out.println("Move was not legal");
        if (legalMoves != null) {
            for(String legalMove : legalMoves) {
                System.out.println(legalMove);
            }
        }
    }
    
    protected void onPreDoMove(String move, String playerDescription) {
        if (!this.moveWasEnteredByUser) {
            System.out.println(playerDescription + ": " + move);
        } else {
            this.moveWasEnteredByUser = false;
        }
    }
    
    protected void onPostDoMove() {
        System.out.println(this.env.getStateAsReadableString());
    }
    
    protected void onGameIsOver(int numberOfEpisodesPlayed) {
        System.out.println("Game is over");
        System.out.println("Goals: ");
        this.numberOfEpisodes++;
        for (Entry<GDLType, GDLType> goal : this.env.getReachedGoals().entrySet()) {
            System.out.println(goal);
        }

        if (this.evaluation) {

            if (evaluationTable == null) {
                evaluationTable = new HashMap<String, Map<String, Integer>>();
            }

            for (Entry<GDLType, GDLType> goal : this.env.getReachedGoals().entrySet()) {
                if (goal.getValue() instanceof GDLNumber) {
                    Map<String, Integer> currentMap = null;
                    String playerName = this.getCurrentPlayerOfRole(goal.getKey());
                    if (evaluationTable.containsKey(playerName)) {
                        currentMap = evaluationTable.get(playerName);
                    } else {
                        currentMap = new HashMap<>();
                    }

                    if (currentMap.containsKey(goal.getValue().toString())) {
                        currentMap.put(goal.getValue().toString(), currentMap.get(goal.getValue().toString()) + 1);
                    } else {
                        currentMap.put(goal.getValue().toString(), 1);
                    }

                    evaluationTable.put(playerName, currentMap);
                }
            }
        }

        if (this.evaluation && this.numberOfEpisodes >= this.evaluationSamples) {
            this.evaluate();
            System.exit(0);
        }

        System.out.println("");
    }

    private void evaluate() {
        for (String role : evaluationTable.keySet()) {
            System.out.print(role + ": ");
            Map<String, Integer> results = evaluationTable.get(role);
            float total = 0.0f;
            float count = 0.0f;
            for (String res : results.keySet()) {
                System.out.print("("+ res + ", " + results.get(res) + "x) ");
                count += (float) results.get(res);
                total = Integer.parseInt(res) * results.get(res);
            }
            System.out.println(" - Average: " + total / count);
        }
    }
}
