package de.monticore.lang.gdl.cli;

import java.util.List;
import java.util.Scanner;
import java.util.Set;

import de.monticore.lang.gdl.Interpreter;

public class GDLCLI implements Runnable {
    
    private final Interpreter interpreter;

    public GDLCLI(Interpreter interpreter) {
        this.interpreter = interpreter;
    }

    private void printGameState(String role) {
        if (role == null) {
            List<List<String>> gameState = interpreter.getGameState();
            List<List<String>> hiddenGameState = interpreter.getHiddenGameState();
            System.out.println("Current Game State (" + gameState.size() +  "):");
            gameState.forEach(s -> System.out.println("  " + s));
    
            System.out.println();

            System.out.println("Current Hidden Game State (" + hiddenGameState.size() +  "):");
            hiddenGameState.forEach(s -> System.out.println("  " + s));
        } else {
            List<List<String>> gameState = interpreter.getGameStateForRole(role);
            System.out.printf("Current Game State for role '%s' (%d):\n", role, gameState.size());
            gameState.forEach(s -> System.out.println("  " + s));
        }
    }

    private void printHelp() {
        String help = 
            "Usage:\n" +
            "  {player} ([args])" + "\t" + "Do a game move\n" + 
            "\n" +
            "Additional functions:\n" +
            "  /help" + "\t\t\t" + "Show the CLI usage\n" +
            "  /exit" + "\t\t\t" + "Exit the CLI\n" +
            "  /eval {func}" + "\t\t" + "Calculate all models for a function {func}\n" +
            "  /roles" + "\t\t\t" + "Print all defined roles.\n" +
            "  /state {role}" + "\t\t" + "Print the current game state (for a role {role})\n" +
            "  /legal {role}" + "\t\t" + "Print all currently legal moves (for a role {role})\n" +
            "";
        System.out.print(help);
    }

    private void evaluate(String expression) {
        String[] args = expression.split(" ");
        if (args.length > 2) {
            printHelp();
            return;
        }
        String function = args[1];
        List<List<String>> allModels = interpreter.getAllModels(function);
        if (allModels != null) {
            System.out.println(allModels);
        } else {
            System.out.println("Function '" + function + "' does not exist.");
        }
    }

    private void printRoles() {
        Set<String> roles = interpreter.getRoles();
        
        System.out.printf("All Roles (%d):\n", roles.size());
        for (String role : roles) {
            System.out.println("  " + role);
        }
    }

    private void printGameOver() {
        System.out.println("---- -----");
        System.out.println("Game over.");
        System.out.println("---- -----");
        List<List<String>> goals = interpreter.getAllModels("goal");
        if (goals != null) {
            for (List<String> goal : goals) {
                if (goal.size() == 2) {
                    System.out.printf("\tPlayer %s achieved %s points.\n", goal.get(0), goal.get(1));
                } else {
                    System.out.println("\t" + goal);
                }
            }
        }
    }

    private void printLegal(String role) {
        List<List<String>> legal;
        if (role == null) legal = interpreter.getAllLegalMoves();
        else legal = interpreter.getAllLegalMovesForPlayer(role);

        if (legal.isEmpty()) {
            if (role == null) System.out.println("There are no legal moves.");
            else System.out.printf("There are no legal moves for role '%s'.\n", role);
        } else {
            System.out.printf("Legal moves (%d):\n", legal.size());
            
            for (List<String> move: legal) {
                if (move.size() <= 1) {
                    System.out.println("  [!err]");
                    continue;
                }

                System.out.print("  ");
                System.out.print(move.get(0));
                System.out.print(" (");
                for (int i = 1; i < move.size(); i++) {
                    System.out.print(move.get(i));
                    if (i < move.size() - 1) {
                        System.out.print(" ");
                    }
                }
                System.out.println(")");
            }
        }
    }

    private void printWelcome() {
        String welcome = "Welcome to the GDL Interpreter!";
        System.out.println(welcome);
    }

    private void printExit() {
        System.out.println("Goodbye!");
    }

    @Override
    public void run() {
        printWelcome();
        printHelp();

        System.out.println("");
        System.out.print("> ");

        Scanner s = new Scanner(System.in);
        String line;
        while (s.hasNextLine()) {
            line = s.nextLine();
            if (line == null || line.equals("/exit")) {
                break;
            } else if (line.trim().length() == 0) {
                System.out.print("> ");
                continue;
            } else if (line.startsWith("/")) {
                if (line.startsWith("/eval ") || line.startsWith("/evaluate ")) {
                    evaluate(line);
                } else if(line.startsWith("/roles")) {
                    printRoles();
                } else if (line.startsWith("/state")) {
                    if (line.length() > 7) {
                        printGameState(line.substring(7));
                    } else {
                        printGameState(null);
                    }
                } else if (line.startsWith("/help")) {
                    printHelp();
                } else if (line.startsWith("/legal")) {
                    if (line.length() > 7) {
                        printLegal(line.substring(7));
                    } else {
                        printLegal(null);
                    }
                } else {
                    System.out.println("Unknown command: " + line);
                    printHelp();
                }
            } else {
                List<List<String>> nextState = interpreter.interpret(line);

                if (nextState == null) {
                    System.out.println("Move was illegal! Type /help for usage");
                } else {
                    System.out.println(nextState);

                    if (interpreter.isTerminal()) {
                        printGameOver();
                        break;
                    }
                }
            }
            System.out.print("> ");
        }
        printExit();
        s.close();
        System.exit(0);
    }

}
