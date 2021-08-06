package de.monticore.lang.gdl.cli;

import java.util.List;
import java.util.Scanner;

import de.monticore.lang.gdl.Interpreter;

public class GDLCLI implements Runnable {
    
    private final Interpreter interpreter;

    public GDLCLI(Interpreter interpreter) {
        this.interpreter = interpreter;
    }

    private void printGameState() {
        System.out.println("Current Game State (" + interpreter.getGameState().size() +  "): ");
        interpreter.getGameState().forEach(s -> System.out.println("\t" + s));
    }

    private void printHelp() {
        String help = 
            "Usage:\n" +
            "  {player} ([args])" + "\t" + "Do a game move\n" + 
            "\n" +
            "Additional functions:\n" +
            "  /help" + "\t\t\t\t" + "Show the CLI usage\n" +
            "  /exit" + "\t\t\t\t" + "Exit the CLI\n" +
            "  /state" + "\t\t\t" + "Print the current game state\n" +
            "  /eval {func}" + "\t" + "Calculate all models for a function {func}\n" +
            "";
        System.out.print(help);
    }

    private void evaluate(String expression) {
        String[] args = expression.split(" ");
        if (args.length != 2) {
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
                } else if (line.startsWith("/state")) {
                    printGameState();
                } else if (line.startsWith("/help")) {
                    printHelp();
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
