package de.monticore.lang.gdl;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.HashMap;
import java.util.Map;
import java.util.List;
import java.util.Scanner;
import java.util.Set;
import java.util.concurrent.Semaphore;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;

import de.monticore.lang.gdl._ast.ASTGame;
import de.monticore.lang.gdl.visitors.PrologPrinter;

import sun.misc.Signal;

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
    private Set<FunctionSignature> hiddenStatesSignatures;
    private Set<FunctionSignature> nextSignatures;
    private Set<FunctionSignature> hiddenNextSignatures;
    private Set<FunctionSignature> legalSignatures;

    private boolean hasTerminal;
    private boolean hasRandom;
    private Set<String> roles;

    private List<String> currentEvaluation = null;
    private Semaphore evalSemaphore;

    private String placeholderStates;

    private Map<String, List<Runnable>> onStateHasChangedMap = new HashMap<>();

    private InterpreterOptions options = new InterpreterOptions();

    public Interpreter(ASTGame game) {
        this.game = game;
    }

    public Interpreter init(final InterpreterOptions options) throws Exception {
        this.options = options;
        return init();
    }

    public Interpreter init() throws Exception {
        initSemaphore = new Semaphore(0);
        initTrueCounter = 0;

        prologProcess = Runtime.getRuntime().exec("swipl");
        out = new BufferedOutputStream(prologProcess.getOutputStream());
        writer = new OutputStreamWriter(out) {
            @Override
            public void write(String str) throws IOException {
                if (options.isDebugMode()) System.out.println("? " + str);
                super.write(str);
            }
        };
        in = new BufferedInputStream(prologProcess.getInputStream());
        err = new BufferedInputStream(prologProcess.getErrorStream());

        Signal.handle(new Signal("INT"),  // SIGINT
            signal -> {
                prologProcess.destroy();
                printIn.interrupt();
                printErr.interrupt();
                System.exit(0);
            });

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

        PrologPrinter printer = new PrologPrinter();
        game.accept(printer.getTraverser());
        String prologProgram = printer.getContent();
        statesSignatures = printer.getStatesSignatures();
        hiddenStatesSignatures = printer.getHiddenStatesSignatures();
        functionSignatures = printer.getFunctionSignatures();
        legalSignatures = printer.getLegalSignatures();
        nextSignatures = printer.getNextSignatures();
        hiddenNextSignatures = printer.getHiddenNextSignatures();
        String util = loadUtil();
        placeholderStates = printer.getPlaceholderStates();

        hasTerminal = printer.hasTerminal();
        hasRandom = printer.hasRandom();
        roles = printer.getRoles();

        writer.write("[user].\n");
        writer.write(util);
        writer.write(placeholderStates);
        writer.write(prologProgram);

        writer.write("end_of_file.\n");
        writer.flush();

        writer.write("set_prolog_flag(answer_write_options,[max_depth(0)]).\n");
        writer.flush();

        initSemaphore.acquire();

        if (hasRandom && !options.isManualRandom()) {
            doRandom();
        }

        return this;
    }

    private String loadUtil() {
        InputStream stream = Interpreter.class.getResourceAsStream("util.pl");
        try {
            BufferedReader reader = new BufferedReader(new InputStreamReader(stream, "UTF-8"));
            return reader.lines().reduce("", (s1, s2) -> s1 + "\n" + s2) + "\n";
        } catch (Exception e) {
            System.err.println("Unable to load util.pl.");
            return "";
        }
        
    }

    public Set<List<String>> getAllModels(String function) {
        Set<FunctionSignature> signatures;

        if (function.equals("legal")) {
            return getAllLegalMoves();
        } else {
            signatures = functionSignatures
            .stream()
            .filter(signature -> signature.functionName.equals(function))
            .collect(Collectors.toSet());
        }

        if (signatures.isEmpty()) {
            return null;
        }

        Set<List<String>> allResults = new HashSet<>();
        for (FunctionSignature signature : signatures) {
            Set<List<String>> result = getAllModels("function_" + signature.functionName, signature.arity);
            if (result != null) {
                allResults.addAll(result);
            }
        }
        allResults = allResults
            .stream().map(
                l -> l.stream().map(Interpreter::convertPLValue2Interpreter).collect(Collectors.toList())
            ).filter(
                l -> l.stream().filter(s -> s.length() == 0).count() == 0
            ).collect(Collectors.toSet());
        return allResults;
    }

    private Set<List<String>> getAllModels(String functionName, int arity) {
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

        Set<List<String>> results = new HashSet<>();

        if (result.equals("false.")) {
            return results;
        }
        result = result.substring(10, result.length() - 2);

        if (arity > 1) {
            String[] tuples = result.split("\\(");
            for (int i = 1; i < tuples.length; i++) {
                String current = tuples[i].split("\\)")[0];
                String[] elements = current.split(",");
                results.add(List.of(elements));
            }
        } else {
            String[] tuples = result.split(",");
            for (String s: tuples) {
                results.add(List.of(s));
            }
        }

        return results;
    }

    private void retractAllStates() {
        String command = "retractAllStates().\n";
        execute(command);
    }

    private synchronized String execute(String command) {
        List<String> result = execute(command, 1);
        return result == null ? null : result.get(0);
    }

    private synchronized List<String> execute(String command, int waitEvalLines) {
        currentEvaluation = new ArrayList<>(waitEvalLines);
        this.evalSemaphore = new Semaphore(0);

        try {
            writer.write(command);
            writer.flush();

            for (int i = 0; i < waitEvalLines; i++) {
                evalSemaphore.acquire(1);

                if (evalError) {
                    currentEvaluation = null;
                    
                    if (!options.isDebugMode()) throw new RuntimeException("The current evaluation faced an unexpected Error. Re-run your program in debug mode to get more detail.");
                    throw new RuntimeException("The current evaluation faced an unexpected Error.");
                }
            }
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
            return null;
        }

        List<String> result = currentEvaluation;
        currentEvaluation = null;

        return result;
    }

    private void readIn(String line) {
        if (options.isDebugMode()) System.out.println("| " + line);

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

    private boolean evalError;
    private void readErr(String line) {
        if (options.isDebugMode()) System.out.println("! " + line);

        if (currentEvaluation != null) {
            if (line.startsWith("ERROR")) {
                evalError = true;
                evalSemaphore.release(1);
            }
        }
    }

    public Set<List<String>> getAllLegalMoves() {
        Set<List<String>> result = getAllModels("function_legal", 2);
        result = result
            .stream().map(
                l -> l.stream().map(Interpreter::convertPLValue2Interpreter).collect(Collectors.toList())
            ).filter(
                l -> l.stream().filter(s -> s.length() == 0).count() == 0
            ).collect(Collectors.toSet());
        return result;
    }

    public Set<List<String>> getAllLegalMovesForPlayer(String player) {
        Set<List<String>> allLegalMoves = this.getAllLegalMoves();
        if (allLegalMoves != null) {
            allLegalMoves = allLegalMoves
            .stream()
            .filter(
                l -> l != null && l.size() > 0 && l.get(0).equals(player)
            )
            .collect(Collectors.toSet());
            return allLegalMoves;
        } else {
            return new HashSet<List<String>>();
        }
        
    }

    private boolean isLegal(Command command) {
        StringBuilder legalBuilder = new StringBuilder();

        legalBuilder.append("setof(_,");
        legalBuilder.append("function_legal(");
        legalBuilder.append(command.getPlayer()).append(", (");
        for (int i = 0; i < command.getArguments().size(); i++) {
            legalBuilder.append(command.getArguments().get(i));
            if (i + 1 < command.getArguments().size()) {
                legalBuilder.append(", ");
            }
        }
        legalBuilder.append("))");
        legalBuilder.append(",_).\n");

        String result = execute(legalBuilder.toString());
        boolean legal = result.trim().equals("true.");

        return legal;
    }

    private void buildNextStates(Set<List<String>> statesList, Set<List<String>> hiddenStatesList) {
        retractAllStates();
        statesSignatures = new HashSet<>();
        hiddenStatesSignatures = new HashSet<>();

        StringBuilder statesBuilder = new StringBuilder();
        statesBuilder.append(placeholderStates);

        for (List<String> state : statesList) {
            String stateName = state.get(0).substring(6);
            statesBuilder.append("state(function_").append(stateName);

            if (state.size() > 1) {
                statesBuilder.append(", (");

                for (int i = 1; i < state.size(); i++) {
                    statesBuilder.append(state.get(i));
                    if (i + 1 < state.size()) {
                        statesBuilder.append(", ");
                    }
                }
    
                statesBuilder.append(")");
            }

            statesBuilder.append(").\n");

            FunctionSignature signature = new FunctionSignature(stateName, state.size() - 1);
            statesSignatures.add(signature);
        }

        for (List<String> state : hiddenStatesList) {
            String stateName = state.get(0).substring(6);
            statesBuilder.append("state_hidden(function_").append(stateName);

            statesBuilder.append(", ").append(state.get(1));

            if (state.size() > 2) {
                statesBuilder.append(", (");

                for (int i = 2; i < state.size(); i++) {
                    statesBuilder.append(state.get(i));
                    if (i + 1 < state.size()) {
                        statesBuilder.append(", ");
                    }
                }

                statesBuilder.append(")");
            }

            statesBuilder.append(").\n");

            FunctionSignature signature = new FunctionSignature(stateName, state.size() - 1);
            hiddenStatesSignatures.add(signature);
        }

        StringBuilder commandBuilder = new StringBuilder();
        commandBuilder.append("[user].\n");
        commandBuilder.append(statesBuilder.toString());
        commandBuilder.append("end_of_file.\n");

        execute(commandBuilder.toString());
    }

    public Set<List<String>> interpret(String line) {
        return interpret(Command.createMoveFromLine(line));
    }

    public synchronized Set<List<String>> interpret(Command command) {
        if (command == null) {
            return null;
        }
        StringBuilder inputBuilder = new StringBuilder();

        inputBuilder.append("input((");
        inputBuilder.append(command.getPlayer()).append(", ");
        for (int i = 0; i < command.getArguments().size(); i++) {
            inputBuilder.append(command.getArguments().get(i));
            if (i + 1 < command.getArguments().size()) {
                inputBuilder.append(", ");
            }
        }
        inputBuilder.append("))");

        StringBuilder assertBuilder = new StringBuilder();
        assertBuilder.append("assert(").append(inputBuilder.toString()).append(").\n");

        StringBuilder retractBuilder = new StringBuilder();
        retractBuilder.append("retract(").append(inputBuilder.toString()).append(").\n");

        execute(assertBuilder.toString());

        if (command.getArguments().get(0).equals("value_noop") && isLegal(command)) {
            execute(retractBuilder.toString());

            return getGameState();
        }

        Set<List<String>> allResults = null;
        Set<List<String>> allHiddenResults = null;


        if (isLegal(command)) {
            allResults = new HashSet<>();
            for (FunctionSignature next : nextSignatures) {
                Set<List<String>> models = getAllModels(next.functionName, next.arity);
                allResults.addAll(models);
            }

            allHiddenResults = new HashSet<>();
            for (FunctionSignature next : hiddenNextSignatures) {
                Set<List<String>> models = getAllModels(next.functionName, next.arity);
                allHiddenResults.addAll(models);
            }

            allHiddenResults = allHiddenResults.stream().map(
                list -> {
                    ArrayList<String> l = new ArrayList<>(list);
                    Collections.swap(l, 0, 1);
                    return l;
                }
            ).collect(Collectors.toSet());
        }

        execute(retractBuilder.toString());

        if (allResults != null) {
            buildNextStates(allResults, allHiddenResults);
            allResults.addAll(allHiddenResults);
            allResults = allResults
                .stream().map(
                    l -> l.stream().map(Interpreter::convertPLValue2Interpreter).collect(Collectors.toList())
                ).collect(Collectors.toSet());
        }

        if (hasRandom && !options.isManualRandom()) {
            Set<List<String>> randomResults = doRandom();
            if (randomResults != null) allResults = randomResults;
        }
        
        notifyAllObservers();

        return allResults;
    }

    private final Pattern randomMoveResultPattern = Pattern.compile("(\\(.*\\))", Pattern.MULTILINE);
    private List<String> getRandomMove() {
        if (legalSignatures.isEmpty()) {
            return null;
        }

        String command = "get_random_legal(A).\n";
        String result = execute(command);

        if (result.equals("false.")) {
            return null;
        }

        if (result.endsWith(").")) {
            // A =  (3, 3).
            // result = result.substring(6, result.length() - 2);
            final Matcher m = randomMoveResultPattern.matcher(result);
            m.find();
            result = m.group();
            result = result.substring(1, result.length()-1);

            List<String> moveResult = new ArrayList<String>(Arrays.asList(result.split(",")));
            moveResult.add(0, "value_random");
            moveResult = moveResult.stream().map(s -> s.strip()).collect(Collectors.toList());

            return moveResult;
        }

        // A = 2.
        result = result.substring(4, result.length() - 1);
        
        return List.of("value_random", result);
    }

    private Set<List<String>> doRandom() {
        Set<List<String>> finalResults = null;
        List<String> randomMove = getRandomMove();

        while (randomMove != null) {
            // random command
            // randomMove = randomMove.stream().map(s -> "value_" + s).collect(Collectors.toList());
            Command command = Command.createMoveFromList(randomMove);
            if (options.isDebugMode()) System.out.println("R " + command);

            StringBuilder inputBuilder = new StringBuilder();
    
            inputBuilder.append("input((");
            inputBuilder.append(command.getPlayer()).append(", ");
            for (int i = 0; i < command.getArguments().size(); i++) {
                inputBuilder.append(command.getArguments().get(i));
                if (i + 1 < command.getArguments().size()) {
                    inputBuilder.append(", ");
                }
            }
            inputBuilder.append("))");
    
            StringBuilder assertBuilder = new StringBuilder();
            assertBuilder.append("assert(").append(inputBuilder.toString()).append(").\n");
    
            StringBuilder retractBuilder = new StringBuilder();
            retractBuilder.append("retract(").append(inputBuilder.toString()).append(").\n");
    
            execute(assertBuilder.toString());
    
            Set<List<String>> allResults = null;
            Set<List<String>> allHiddenResults = null;
    
            if (isLegal(command)) {
                allResults = new HashSet<>();
                for (FunctionSignature next : nextSignatures) {
                    Set<List<String>> models = getAllModels(next.functionName, next.arity);
                    allResults.addAll(models);
                }
            
                allHiddenResults = new HashSet<>();
                for (FunctionSignature next : hiddenNextSignatures) {
                    Set<List<String>> models = getAllModels(next.functionName, next.arity);
                    allHiddenResults.addAll(models);
                }

                allHiddenResults = allHiddenResults.stream().map(
                    list -> {
                        ArrayList<String> l = new ArrayList<>(list);
                        Collections.swap(l, 0, 1);
                        return l;
                    }
                ).collect(Collectors.toSet());
            }
    
            execute(retractBuilder.toString());
    
            if (allResults != null) {
                buildNextStates(allResults, allHiddenResults);
            
                allResults.addAll(allHiddenResults);
                allResults = allResults
                    .stream().map(
                        l -> l.stream().map(Interpreter::convertPLValue2Interpreter).collect(Collectors.toList())
                    ).collect(Collectors.toSet());
            }
    
            finalResults = allResults;

            randomMove = getRandomMove();
        }

        return finalResults;
    }

    public Set<List<String>> getGameStateForRole(String role) {
        Set<List<String>> hiddenGameState = getHiddenGameState();

        hiddenGameState = hiddenGameState.stream().filter(list -> list.get(1).equals(role)).map(
            list -> {
                ArrayList<String> l = new ArrayList<>(list);
                l.remove(1);
                return l;
            }
        ).collect(Collectors.toSet());

        hiddenGameState = new HashSet<>(hiddenGameState);
        hiddenGameState.addAll(getGameState());
        return hiddenGameState;
    }

    public Set<List<String>> getHiddenGameState() {
        Set<List<String>> allHiddenResults = new HashSet<>();
        for (FunctionSignature state : hiddenStatesSignatures) {
            int arity = state.arity;
            String functionName = state.functionName;

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

            StringBuilder tupleBuilderMinus1 = new StringBuilder();
            tupleBuilderMinus1.append("(");
            for (int i = 1; i < arity; i++) {
                tupleBuilderMinus1.append("X").append(i);
                if (i + 1 < arity) {
                    tupleBuilderMinus1.append(", ");
                }
            }
            tupleBuilderMinus1.append(")");

            sb.append("setof(").append(tupleBuilder.toString()).append(", ");
            sb.append("state_hidden(").append("function_" + functionName).append(", ");
            sb.append("X0");

            if (arity > 1) sb.append(", ").append(tupleBuilderMinus1.toString());

            sb.append("), ");
            sb.append("Models).\n");

            String command = sb.toString();
            String result = execute(command);

            Set<List<String>> results = new HashSet<>();

            if (result.equals("false.")) {
                return results;
            }
            result = result.substring(10, result.length() - 2);

            if (arity > 1) {
                String[] tuples = result.split("\\(");
                for (int i = 1; i < tuples.length; i++) {
                    String current = tuples[i].split("\\)")[0];
                    String[] elements = current.split(",");
                    results.add(List.of(elements));
                }
            } else {
                String[] tuples = result.split(",");
                for (String s: tuples) {
                    results.add(List.of(s));
                }
            }

            results = results.stream().map(list -> {
                List<String> l = new ArrayList<>();
                l.add("value_" + state.functionName);
                l.addAll(list);
                return l;
            }).collect(Collectors.toSet());

            allHiddenResults.addAll(results);
        }

        allHiddenResults = allHiddenResults
            .stream().map(
                l -> l.stream().map(Interpreter::convertPLValue2Interpreter).collect(Collectors.toList())
            ).collect(Collectors.toSet());

        return allHiddenResults;
    }

    public Set<List<String>> getGameState() {
        Set<List<String>> allResults = new HashSet<>();
        for (FunctionSignature state : statesSignatures) {
            int arity = state.arity;
            String functionName = state.functionName;

            if (arity == 0) {
                allResults.add(List.of("value_" + functionName));
                continue;
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
            sb.append("state(").append("function_" + functionName).append(", ");
            sb.append(tupleBuilder.toString()).append("), ");
            sb.append("Models).\n");

            String command = sb.toString();
            String result = execute(command);

            Set<List<String>> results = new HashSet<>();

            if (result.equals("false.")) {
                return results;
            }
            result = result.substring(10, result.length() - 2);

            if (arity > 1) {
                String[] tuples = result.split("\\(");
                for (int i = 1; i < tuples.length; i++) {
                    String current = tuples[i].split("\\)")[0];
                    String[] elements = current.split(",");
                    results.add(List.of(elements));
                }
            } else {
                String[] tuples = result.split(",");
                for (String s: tuples) {
                    results.add(List.of(s));
                }
            }

            results = results.stream().map(list -> {
                List<String> l = new ArrayList<>();
                l.add("value_" + state.functionName);
                l.addAll(list);
                return l;
            }).collect(Collectors.toSet());

            allResults.addAll(results);
        }



        allResults = allResults
            .stream().map(
                l -> l.stream().map(Interpreter::convertPLValue2Interpreter).collect(Collectors.toList())
            ).collect(Collectors.toSet());

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

    /**
     * Retrieve all roles playable in the current gdl model.
     * @return Set of all playable roles.
     */
    public Set<String> getRoles() {
        return roles.stream().filter(r -> !r.equals("random")).collect(Collectors.toSet());
    }

    public void addStateObserver(String player, Runnable observer) {
        if (!roles.contains(player)) return;

        if (onStateHasChangedMap.get(player) == null) {
            onStateHasChangedMap.put(player, new ArrayList<>());
        }

        onStateHasChangedMap.get(player).add(observer);
    }

    private void notifyStateObservers(String player) {
        List<Runnable> observers = onStateHasChangedMap.get(player);
        if (observers == null) return;

        for (Runnable r : observers) {
            r.run();
        }
    }

    private void notifyAllObservers() {
        for (String role : roles) {
            notifyStateObservers(role);
        }
    }

    public static String convertPLValue2Interpreter(String plValue) {
        if (!plValue.startsWith("value_") && !plValue.startsWith("valnn_")) {
            throw new IllegalArgumentException("Cannot convert '" + plValue + "' into Interpreter value");
        }

        String vPart = plValue.substring(6);
        if (plValue.startsWith("valnn_")) {
            vPart = "-" + vPart;
        }

        return vPart;
    }

    public static String convertInterpreterValue2PL(String inValue) {
        if (inValue.startsWith("-")) {
            return "valnn_" + inValue.substring(1);
        }
        return "value_" + inValue;
    }

}
