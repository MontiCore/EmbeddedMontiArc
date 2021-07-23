package de.monticore.lang.gdl.cli;

import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import org.sosy_lab.java_smt.api.NumeralFormula.IntegerFormula;

import de.monticore.lang.gdl.Interpreter2;
import de.monticore.lang.gdl._ast.ASTGameExpression;

public class GDLCLI implements Runnable {
    
    private final Interpreter2 interpreter;

    public GDLCLI(Interpreter2 interpreter) {
        this.interpreter = interpreter;
    }

    private void printGameState() {
        System.out.println("Current Game State (" + interpreter.getGameState().size() +  "): ");
        interpreter.getGameState().forEach(s -> System.out.println("\t" + s));
    }

    private void printHelp() {

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

    @Override
    public void run() {
        Scanner s = new Scanner(System.in);
        String line;
        while (!(line = s.nextLine()).equals("/exit") && line != null) {
            if (line.startsWith("/eval ") || line.startsWith("/evaluate ")) {
                evaluate(line);
            } else {
                List<ASTGameExpression> nextState = interpreter.interpret(line);

                if (nextState == null) {
                    System.out.println("Move was illegal!");
                } else {
                    System.out.println(nextState);
                }
            }
        }
        s.close();
    }

}
