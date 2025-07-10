package de.monticore.lang.gdl.cli;

import java.util.Arrays;
import java.util.Map;
import java.util.Scanner;
import java.util.Set;
import java.util.Map.Entry;

import de.monticore.lang.gdl.Command;
import de.monticore.lang.gdl.Interpreter;
import de.monticore.lang.gdl.types.GDLType;

public class GDLCLI implements Runnable {
    
    private final Interpreter interpreter;

    public GDLCLI(Interpreter interpreter) {
        this.interpreter = interpreter;
    }

    private void printGameState(String role) {
        if (role == null) {
            Set<GDLType> gameState = interpreter.getVisibleGameState();
            Map<GDLType, Set<GDLType>> hiddenGameState = interpreter.getHiddenGameState();
            System.out.println("Current Game State (" + gameState.size() +  "):");
            gameState.forEach(s -> System.out.println("  " + s));
    
            System.out.println();

            System.out.println("Current Hidden Game State (" + hiddenGameState.size() +  "):");
            for (GDLType roleKey : hiddenGameState.keySet()) {
                System.out.println("  " + roleKey + ":");
                hiddenGameState.get(roleKey).forEach(s -> System.out.println("    " + s));
            }
        } else {
            GDLType gdlRole = GDLType.createFromLine(role);
            Set<GDLType> gameState = interpreter.getGameStateForRole(gdlRole);
            System.out.printf("Current Game State for role '%s' (%d):\n", gdlRole.toString(), gameState.size());
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
            "  /reset" + "\t\t" + "Reset the interpreter\n" +
            // "  /eval {func}" + "\t\t" + "Calculate all models for a function {func}\n" +
            "  /roles" + "\t\t" + "Print all playable roles.\n" +
            "  /state {role}" + "\t\t" + "Print the current game state (for a role {role})\n" +
            "  /legal {role}" + "\t\t" + "Print all currently legal moves (for a role {role})\n" +
            (interpreter.isWithTypes() ?
            "  /dimensions" + "\t\t" + "Print all known space dimensions\n" +
            "  /indicator {role}" + "\t" + "Print the indicator matrix for a role {role}\n"
            : "") +
            "";
        System.out.print(help);
    }

    private void reset() {
        interpreter.reset();
        System.out.println("Interpreter was reset.");
        printGameState(null);
    }

    // private void evaluate(String expression) {
    //     String[] args = expression.split(" ");
    //     if (args.length > 2) {
    //         printHelp();
    //         return;
    //     }
    //     String function = args[1];
    //     Set<List<String>> allModels = interpreter.getAllModels(function);
    //     if (allModels != null) {
    //         System.out.println(allModels);
    //     } else {
    //         System.out.println("Function '" + function + "' does not exist.");
    //     }
    // }

    private void printRoles() {
        Set<GDLType> roles = interpreter.getRoles();
        
        System.out.printf("All Roles (%d):\n", roles.size());
        for (GDLType role : roles) {
            System.out.println("  " + role);
        }
    }

    private void printGameOver() {
        System.out.println("---- -----");
        System.out.println("Game over.");
        System.out.println("---- -----");
        Map<GDLType, GDLType> goals = interpreter.getGoals();
        for (Entry<GDLType, GDLType> goal : goals.entrySet()) {
            System.out.printf("\tPlayer %s achieved goal %s.\n", goal.getKey(), goal.getValue());
        }
    }

    private void printLegal(String role) {
        Set<Command> legal;
        GDLType gdlRole = null;
        if (role == null) legal = interpreter.getAllLegalMoves();
        else {
            gdlRole = GDLType.createFromLine(role);
            legal = interpreter.getAllLegalMovesForRole(gdlRole);
        }

        if (legal.isEmpty()) {
            if (role == null) System.out.println("There are no legal moves.");
            else System.out.printf("There are no legal moves for role '%s'.\n", gdlRole);
        } else {
            System.out.printf("Legal moves (%d):\n", legal.size());
            
            for (Command command: legal) {
                System.out.print("  ");
                System.out.println(command.toString());
            }
        }
    }

    private void printDimensions() {
        if (!interpreter.isWithTypes()) {
            System.out.println("The Model does not support types!");
        } else {
            System.out.printf("State Space Dimension:\t%d\n", interpreter.getStateSpaceDimension());
            System.out.printf("Action Space Dimension:\t%d\n", interpreter.getActionSpaceDimension());
        }
    }

    private void printIndicator(String role) {
        if (!interpreter.isWithTypes()) {
            System.out.println("The Model does not support types!");
            return;
        }

        GDLType gdlRole = GDLType.createFromLine(role);
        float[] matrix = interpreter.getStateIndicatorMatrixForRole(gdlRole);

        System.out.println("Indicator matrix:");
        System.out.println("  " + Arrays.toString(matrix));
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
                // if (line.startsWith("/eval ") || line.startsWith("/evaluate ")) {
                //     evaluate(line);
                // } else
                if(line.startsWith("/roles")) {
                    printRoles();
                } else if (line.startsWith("/state")) {
                    if (line.length() > 7) {
                        printGameState(line.substring(7));
                    } else {
                        printGameState(null);
                    }
                } else if (line.startsWith("/help")) {
                    printHelp();
                } else if (line.startsWith("/reset")) {
                    reset();
                } else if (line.startsWith("/legal")) {
                    if (line.length() > 7) {
                        printLegal(line.substring(7));
                    } else {
                        printLegal(null);
                    }
                } else if (line.startsWith("/dimensions")) {
                    printDimensions();
                } else if (line.startsWith("/indicator")) {
                    if (line.length() > "/indicator ".length()) {
                        printIndicator(line.substring("/indicator ".length()));
                    } else {
                        printHelp();
                    }
                } else {
                    System.out.println("Unknown command: " + line);
                    printHelp();
                }
            } else {
                boolean legal = interpreter.interpret(line);

                if (!legal) {
                    System.out.println("Move was illegal! Type /help for usage");
                } else {
                    printGameState(null);

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
