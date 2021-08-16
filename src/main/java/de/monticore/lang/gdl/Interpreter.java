package de.monticore.lang.gdl;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Scanner;
import java.util.Set;
import java.util.concurrent.Semaphore;
import java.util.stream.Collectors;

import de.monticore.lang.gdl._ast.ASTGame;
import de.monticore.lang.gdl._ast.ASTGameExpression;
import de.monticore.lang.gdl._ast.ASTGameFunction;
import de.monticore.lang.gdl._ast.ASTGameFunctionDefinition;
import de.monticore.lang.gdl._ast.ASTGameFunctionHead;
import de.monticore.lang.gdl._ast.ASTGameGoal;
import de.monticore.lang.gdl._ast.ASTGameInference;
import de.monticore.lang.gdl._ast.ASTGameInit;
import de.monticore.lang.gdl._ast.ASTGameNext;
import de.monticore.lang.gdl._ast.ASTGameTerminal;
import de.monticore.lang.gdl.visitors.PrologPrinter;

public class Interpreter {

    private Semaphore initSemaphore;
    private int initTrueCounter;

    private BufferedOutputStream out;
    private OutputStreamWriter writer;
    private BufferedInputStream in, err;
    private Process prologProcess;

    private Thread printIn, printErr;

    private final ASTGame game;

    private Set<FunctionSignature> functionSignatures;
    private Set<FunctionSignature> statesSignatures;
    private Set<FunctionSignature> nextSignatures;

    private boolean hasTerminal;

    private List<String> currentEvaluation = null;
    private Semaphore evalSemaphore;

    public Interpreter(ASTGame game) {
        this.game = game;
        functionSignatures = new HashSet<>();
        statesSignatures = new HashSet<>();
        nextSignatures = new HashSet<>();
        hasTerminal = false;
    }

    public Interpreter init() throws Exception {
        initSemaphore = new Semaphore(0);
        initTrueCounter = 0;

        prologProcess = Runtime.getRuntime().exec("swipl");
        out = new BufferedOutputStream(prologProcess.getOutputStream());
        writer = new OutputStreamWriter(out);
        in = new BufferedInputStream(prologProcess.getInputStream());
        err = new BufferedInputStream(prologProcess.getErrorStream());

        printIn = new Thread(new Runnable() {
            @Override
            public void run() {
                Scanner s = new Scanner(in);
                while(s.hasNextLine()) {
                    String line = s.nextLine();
                    if (line.length() == 0) {
                        continue;
                    }
                    readIn(line);
                }
                s.close();
            }
        });

        printErr = new Thread(new Runnable() {
            @Override
            public void run() {
                Scanner s = new Scanner(err);
                while(s.hasNextLine()) {
                    String line = s.nextLine();
                    if (line.length() == 0) {
                        continue;
                    }
                    readErr(line);
                }
                s.close();
            }
        });

        printIn.start();
        printErr.start();

        initFunctionSet();
        initNextSet();

        String stateDynamics = initStatesSet();
        
        PrologPrinter printer = new PrologPrinter();
        game.accept(printer.getTraverser());
        String prologProgram = printer.getContent();

        System.out.println(prologProgram);

        writer.write("[user].\n");
        writer.write(stateDynamics);
        writer.write(prologProgram);
        writer.write("end_of_file.\n");
        writer.write("set_prolog_flag(answer_write_options,[max_depth(0)]).\n");

        writer.flush();

        initSemaphore.acquire();

        return this;
    }

    private class FunctionSignature {
        private final String functionName;
        private final int arity;
        private FunctionSignature(String functionName, int arity) {
            this.functionName = functionName;
            this.arity = arity;
        }
        @Override
        public boolean equals(Object obj) {
            if (obj instanceof FunctionSignature) {
                FunctionSignature fs = (FunctionSignature) obj;
                return this.functionName.equals(fs.functionName) && this.arity == fs.arity;
            }
            return super.equals(obj);
        }
        @Override
        public int hashCode() {
            return (functionName + "?" + arity).hashCode();
        }
        @Override
        public String toString() {
            return functionName + "/" + arity;
        }
    }

    private String initStatesSet() {
        game.getGameExpressionList()
            .forEach(expression -> {
                if (expression.getType() instanceof ASTGameInit) {
                    ASTGameExpression innerExpression = (ASTGameExpression) expression.getArguments(0);
                    String state = ((ASTGameFunction)innerExpression.getType()).getFunction();
                    int arity = innerExpression.getArgumentsList().size();
                    FunctionSignature s = new FunctionSignature(state, arity);
                    statesSignatures.add(s);
                }
            });

        StringBuilder sb = new StringBuilder();
        for (FunctionSignature s : statesSignatures) {
            sb.append(":- dynamic state_function_" + s.functionName + "/" + s.arity + ".\n");
        }
        return sb.toString();
    }

    private void initFunctionSet() {
        game.getGameExpressionList()
            .forEach(expr -> {
                if (expr instanceof ASTGameFunctionDefinition) {
                    ASTGameFunctionHead head = ((ASTGameFunctionDefinition) expr).getHead();
                    String functionName = head.getName();
                    int arity = head.getParametersList().size();

                    FunctionSignature signature = new FunctionSignature(functionName, arity);
                    functionSignatures.add(signature);
                } else if (expr.getType() instanceof ASTGameFunction) {
                    String functionName = ((ASTGameFunction) expr.getType()).getFunction();
                    int arity = expr.getArgumentsList().size();

                    FunctionSignature signature = new FunctionSignature(functionName, arity);
                    functionSignatures.add(signature);
                } else if (expr.getType() instanceof ASTGameInference) {
                    ASTGameExpression head = (ASTGameExpression) expr.getArguments(0);
                    if (head.getType() instanceof ASTGameGoal) {
                        String functionName = "goal";
                        int arity = head.getArgumentsList().size();

                        FunctionSignature signature = new FunctionSignature(functionName, arity);
                        functionSignatures.add(signature);
                    }
                } else if (expr.getType() instanceof ASTGameTerminal) {
                    functionSignatures.add(new FunctionSignature("terminal", 0));
                    hasTerminal = true;
                }
            });
    }

    private void initNextSet() {
        game.getGameExpressionList()
            .forEach(expression -> {
                if (expression.getType() instanceof ASTGameInference) {
                    ASTGameExpression innerExpression = (ASTGameExpression) expression.getArguments(0);
                    if (innerExpression.getType() instanceof ASTGameNext) {
                        ASTGameExpression astParameters = (ASTGameExpression) innerExpression.getArguments(0);
                        String functionName = "function_next";
                        int arity = astParameters.getArgumentsList().size() + 1;
                        FunctionSignature s = new FunctionSignature(functionName, arity);
                        nextSignatures.add(s);
                    }
                }
            });
    }

    public List<List<String>> getAllModels(String function) {
        Set<FunctionSignature> signatures = functionSignatures
            .stream()
            .filter(signature -> signature.functionName.equals(function))
            .collect(Collectors.toSet());

        if (signatures.isEmpty()) {
            return null;
        }

        List<List<String>> allResults = new ArrayList<>();
        for (FunctionSignature signature : signatures) {
            List<List<String>> result = getAllModels("function_" + signature.functionName, signature.arity);
            if (result != null) {
                allResults.addAll(result);
            }
        }
        allResults = allResults
            .stream().map(
                l -> l.stream().map(s -> s.substring(6)).collect(Collectors.toList())
            ).filter(
                l -> l.stream().filter(s -> s.length() == 0).count() == 0
            ).collect(Collectors.toList());
        return allResults;
    }

    private List<List<String>> getAllModels(String functionName, int arity) {
        if (arity < 1) {
            return null;
        }
        StringBuilder sb = new StringBuilder();

        StringBuilder tupleBuilder = new StringBuilder();
        tupleBuilder.append("(");
        for (int i = 0; i < arity; i++) {
            tupleBuilder.append("X").append(i);
            if (i + 1 < arity) {
                tupleBuilder.append(", ");
            }
        }
        tupleBuilder.append(")");

        sb.append("setof(").append(tupleBuilder.toString()).append(", ");
        sb.append(functionName).append(tupleBuilder.toString()).append(", ");
        sb.append("Models).\n");

        String command = sb.toString();

        String result = execute(command);
        result = result.substring(10, result.length() - 2);

        List<List<String>> results = new LinkedList<>();

        if (arity > 1) {
            String[] tuples = result.split("\\(");
            for (int i = 1; i < tuples.length; i++) {
                String current = tuples[i].split("\\)")[0];
                String[] elements = current.split(",");
                results.add(List.of(elements));
            }
        } else {
            String[] tuples = result.split(", ");
            results.add(List.of(tuples));
        }

        return results;
    }

    private void retractAllStates() {
        StringBuilder sb = new StringBuilder();
        for (FunctionSignature state : statesSignatures) {
            sb.append("retractall(");

            sb.append("state_function_").append(state.functionName).append("(");
            for (int i = 0; i < state.arity; i++) {
                sb.append("X").append(i);
                if (i + 1 < state.arity) {
                    sb.append(", ");
                }
            }

            sb.append(")).\n");
        }
        String command = sb.toString();

        execute(command, statesSignatures.size());
    }

    private synchronized String execute(String command) {
        return execute(command, 1).get(0);
    }

    private synchronized List<String> execute(String command, int waitEvalLines) {
        currentEvaluation = new ArrayList<>(waitEvalLines);
        this.evalSemaphore = new Semaphore(0);

        try {
            writer.write(command);
            writer.flush();
            evalSemaphore.acquire(waitEvalLines);
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }

        List<String> result = currentEvaluation;
        currentEvaluation = null;

        return result;
    }

    private void readIn(String line) {
        // System.out.println("| " + line);
        if (line.equals("true.")) {
            initTrueCounter++;
            if (initTrueCounter == 2) {
                initSemaphore.release();
            }
        }

        if (currentEvaluation != null) {
            currentEvaluation.add(line);
            evalSemaphore.release(1);
        }
    }

    private void readErr(String line) {
        System.out.println("! " + line);
    }

    private boolean isLegal(Command command) {
        StringBuilder legalBuilder = new StringBuilder();

        legalBuilder.append("setof(_,");
        legalBuilder.append("function_legal(");
        legalBuilder.append(command.getPlayer()).append(", ");
        for (int i = 0; i < command.getArguments().size(); i++) {
            legalBuilder.append(command.getArguments().get(i));
            if (i + 1 < command.getArguments().size()) {
                legalBuilder.append(", ");
            }
        }
        legalBuilder.append(")");
        legalBuilder.append(",_).\n");

        String result = execute(legalBuilder.toString());
        boolean legal = result.trim().equals("true.");

        return legal;
    }

    private void buildNextStates(List<List<String>> statesList) {
        retractAllStates();
        statesSignatures = new HashSet<>();

        StringBuilder statesBuilder = new StringBuilder();

        for (List<String> state : statesList) {
            String stateName = state.get(0).substring(6);
            statesBuilder.append("state_function_").append(stateName);

            statesBuilder.append("(");

            for (int i = 1; i < state.size(); i++) {
                statesBuilder.append(state.get(i));
                if (i + 1 < state.size()) {
                    statesBuilder.append(", ");
                }
            }

            statesBuilder.append(").\n");

            FunctionSignature signature = new FunctionSignature(stateName, state.size() - 1);
            statesSignatures.add(signature);
        }
        
        StringBuilder dynamicsBuilder = new StringBuilder();
        for (FunctionSignature s : statesSignatures) {
            dynamicsBuilder.append(":- dynamic state_function_" + s.functionName + "/" + s.arity + ".\n");
        }

        StringBuilder commandBuilder = new StringBuilder();
        commandBuilder.append("[user].\n");
        commandBuilder.append(dynamicsBuilder.toString());
        commandBuilder.append(statesBuilder.toString());
        commandBuilder.append("end_of_file.\n");

        execute(commandBuilder.toString());
    }

    public List<List<String>> interpret(String line) {
        return interpret(Command.createMoveFromLine(line));
    }

    public List<List<String>> interpret(Command command) {
        if (command == null) {
            return null;
        }
        StringBuilder inputBuilder = new StringBuilder();

        inputBuilder.append("input(");
        inputBuilder.append(command.getPlayer()).append(", ");
        for (int i = 0; i < command.getArguments().size(); i++) {
            inputBuilder.append(command.getArguments().get(i));
            if (i + 1 < command.getArguments().size()) {
                inputBuilder.append(", ");
            }
        }
        inputBuilder.append(")");

        StringBuilder assertBuilder = new StringBuilder();
        assertBuilder.append("assert(").append(inputBuilder.toString()).append(").\n");

        StringBuilder retractBuilder = new StringBuilder();
        retractBuilder.append("retract(").append(inputBuilder.toString()).append(").\n");

        execute(assertBuilder.toString());

        List<List<String>> allResults = null;

        if (isLegal(command)) {
            allResults = new LinkedList<>();
            for (FunctionSignature next : nextSignatures) {
                List<List<String>> models = getAllModels(next.functionName, next.arity);
                allResults.addAll(models);
            }
        }

        execute(retractBuilder.toString());

        if (allResults != null) {
            buildNextStates(allResults);
            
            allResults = allResults
                .stream().map(
                    l -> l.stream().map(s -> s.substring(6)).collect(Collectors.toList())
                ).collect(Collectors.toList());
        }


        return allResults;
    }

    public List<List<String>> getGameState() {
        List<List<String>> allResults = new LinkedList<>();
        for (FunctionSignature state : statesSignatures) {
            List<List<String>> models = getAllModels("state_function_" + state.functionName, state.arity);

            models = models.stream().map(list -> {
                List<String> l = new ArrayList<>();
                l.add("123456" + state.functionName);
                l.addAll(list);
                return l;
            }).collect(Collectors.toList());

            allResults.addAll(models);
        }

        allResults = allResults
            .stream().map(
                l -> l.stream().map(s -> s.substring(6)).collect(Collectors.toList())
            ).collect(Collectors.toList());
        
        return allResults;
    }

    public boolean isTerminal() {
        if (!hasTerminal) {
            return false;
        }
        String command = "setof(_,function_terminal(),_).\n";
        String result = execute(command);
        boolean legal = result.trim().equals("true.");

        return legal;
    }

}
