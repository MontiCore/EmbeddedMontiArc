package de.gdl.rl.cli;
import de.gdl.rl.Coordinator;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Scanner;
import java.util.List;
import java.util.regex.Pattern;
import de.gdl.rl.environment.GDLGameEnvironment;



public class GamingCLI<ConcreteEnvironment extends GDLGameEnvironment> extends Coordinator<ConcreteEnvironment> {

    boolean moveWasEnteredByUser = false;
    Scanner scanner = new Scanner(System.in).useDelimiter(Pattern.compile("(\\n)|;"));

    private int numberOfEpisodes = 0;

    protected HashMap<String, HashMap<String, Integer>> evaluationTable;


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
        System.out.println("    Roles: " +  String.join(", ", this.env.getAvailableRoles()));
        System.out.println("");
        
        if (this.evaluation) {
            System.out.println("Evaluation: " +this.evaluationSamples + " episodes");
        }
        
        System.out.println("Type comma-separated all roles you would like to play:");
        System.out.println("    (e.g. role1, role2, role3, ...)");
        String[] manualRolesRaw = this.scanner.next().split(",");
        for (String pseudo_role : manualRolesRaw) {
            String[] ws_splitted_role = pseudo_role.split(" ");      
            for (String role : ws_splitted_role) {
                if (!role.equals("")) {
                    this.manualRoles.add(role);
                }
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

    protected String onEnterMove(String role) {
        System.out.println("Enter a move for '" + role + "':");
        this.scanner = new Scanner(System.in).useDelimiter(Pattern.compile("(\\n)|;"));
        String move = this.scanner.next();
        this.scanner.nextLine();
        this.moveWasEnteredByUser = true;
        // TBD: check if move has special format: /legalMoves -> shows all legal moves for the role

        return move; 
    }

    protected void onMoveWasIllegal(String move) {
        System.out.println("Move was not legal");
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
    
    protected void onGameIsOver() {
        System.out.println("Game is over");
        System.out.println("Goals: ");
        this.numberOfEpisodes++;
        for (String goal : this.env.getReachedGoalsAsStrings()) {
            System.out.println(goal);
        }

        if (this.evaluation) {

            if (evaluationTable == null) {
                evaluationTable = new HashMap<String, HashMap<String, Integer>>();
            }

            List<List<String>> goals = this.env.getReachedGoals();
            for (List<String> goal : goals) {
                if (goal.size() == 2) {
                    HashMap<String, Integer> currentMap = null;
                    String playerName = this.getCurrentPlayerOfRole(goal.get(0));
                    if (evaluationTable.containsKey(playerName)) {
                        currentMap = evaluationTable.get(playerName);
                    } else {
                        currentMap = new HashMap<String, Integer>();
                    }

                    if (currentMap.containsKey("" + goal.get(1))) {
                        currentMap.put("" + goal.get(1), currentMap.get("" + goal.get(1)) + 1);
                    } else {
                        currentMap.put("" + goal.get(1), 1);
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
            HashMap<String, Integer> results = evaluationTable.get(role);
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
