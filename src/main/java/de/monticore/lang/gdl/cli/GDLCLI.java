package de.monticore.lang.gdl.cli;

import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import org.sosy_lab.java_smt.api.NumeralFormula.IntegerFormula;

import de.monticore.lang.gdl.Interpreter;
import de.monticore.lang.gdl._ast.ASTGameExpression;

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
            "  {player} (move [args])" + "\t" + "Do a game move\n" + 
            "\n" +
            "Additional functions:\n" +
            "  /help" + "\t\t\t\t" + "Show the CLI usage\n" +
            "  /exit" + "\t\t\t\t" + "Exit the CLI\n" +
            "  /state" + "\t\t\t" + "Print the current game state\n" +
            "  /eval {func} [const | ?token]" + "\t" + "Calculate all models for a Function {func} with the given Arguments\n" +
            "";
        System.out.print(help);
    }

    private void evaluate(String line) {
        String[] args = line.split(" ");
        if (args.length > 1) {
            String function = args[1];
            List<IntegerFormula> arguments = new ArrayList<>();

            for (int i = 2; i < args.length; i++) {
                String arg = args[i];
                if (arg.startsWith("?")) {
                    arguments.add(interpreter.getImgr().makeVariable(arg.substring(1)));
                } else {
                    arguments.add(interpreter.getImgr().makeNumber(interpreter.getIntegerValue(arg)));
                }
            }

            List<List<String>> allModels = interpreter.getAllModels(function, arguments);
            System.out.println(allModels);
        } else {
            printHelp();
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
        while (!(line = s.nextLine()).equals("/exit") && line != null) {
            if (line.startsWith("/")) {
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
                List<ASTGameExpression> nextState = interpreter.interpret(line);

                if (nextState == null) {
                    System.out.println("Move was illegal! Type /help for usage");
                } else {
                    System.out.println(nextState);
                }
            }
            System.out.print("> ");
        }
        printExit();
        s.close();
        System.exit(0);
    }

}
