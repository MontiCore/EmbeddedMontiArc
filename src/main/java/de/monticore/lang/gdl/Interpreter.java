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
import java.util.LinkedList;
import java.util.List;
import java.util.Scanner;
import java.util.Set;
import java.util.concurrent.Semaphore;
import java.util.stream.Collectors;

import de.monticore.lang.gdl._ast.ASTGame;
import de.monticore.lang.gdl.visitors.PrologPrinter;

import sun.misc.Signal;

public class Interpreter {

    private boolean debugMode = false;

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

    List<List<String>> initialStateList = null; 

    private Map<String, List<List<String>>[]> savedStates = new HashMap<String, List<List<String>>[]>();

    public Interpreter(ASTGame game) {
        this.game = game;
    }

    public Interpreter init() throws Exception {
        initSemaphore = new Semaphore(0);
        initTrueCounter = 0;

        prologProcess = Runtime.getRuntime().exec("swipl");
        out = new BufferedOutputStream(prologProcess.getOutputStream());
        writer = new OutputStreamWriter(out) {
            @Override
            public void write(String str) throws IOException {
                if (debugMode) System.out.println("? " + str);
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
        String stateDynamics = printer.getStateDynamics();
        statesSignatures = printer.getStatesSignatures();
        hiddenStatesSignatures = printer.getHiddenStatesSignatures();
        functionSignatures = printer.getFunctionSignatures();
        legalSignatures = printer.getLegalSignatures();
        nextSignatures = printer.getNextSignatures();
        hiddenNextSignatures = printer.getHiddenNextSignatures();
        String util = loadUtil();

        hasTerminal = printer.hasTerminal();
        hasRandom = printer.hasRandom();
        roles = printer.getRoles();

        writer.write("[user].\n");
        writer.write(stateDynamics);
        writer.write(prologProgram);
        writer.write(util);

        writer.write("end_of_file.\n");
        writer.flush();

        writer.write("set_prolog_flag(answer_write_options,[max_depth(0)]).\n");
        writer.flush();

        initSemaphore.acquire();

        this.saveState("initialState");

        if (hasRandom) {
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

    public List<List<String>> getAllModels(String function) {
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

        List<List<String>> results = new LinkedList<>();

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
        for (FunctionSignature state : hiddenStatesSignatures) {
            sb.append("retractall(");

            sb.append("state_hidden_function_").append(state.functionName).append("(");
            for (int i = 0; i < state.arity; i++) {
                sb.append("X").append(i);
                if (i + 1 < state.arity) {
                    sb.append(", ");
                }
            }

            sb.append(")).\n");
        }
        String command = sb.toString();

        execute(command, statesSignatures.size() + hiddenStatesSignatures.size());
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
                    
                    if (!debugMode) throw new RuntimeException("The current evaluation faced an unexpected Error. Re-run your program in debug mode to get more detail.");
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
        if (debugMode) System.out.println("| " + line);

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
        if (debugMode) System.out.println("! " + line);

        if (currentEvaluation != null) {
            if (line.startsWith("ERROR")) {
                evalError = true;
                evalSemaphore.release(1);
            }
        }
    }

    public List<List<String>> getAllLegalMoves() {
        List<List<String>> result = getAllModels("function_legal", 2);
        result = result
            .stream().map(
                l -> l.stream().map(s -> s.substring(6)).collect(Collectors.toList())
            ).filter(
                l -> l.stream().filter(s -> s.length() == 0).count() == 0
            ).collect(Collectors.toList());
        return result;
    }

    public List<List<String>> getAllLegalMovesForPlayer(String player) {
        List<List<String>> allLegalMoves = this.getAllLegalMoves();
        if (allLegalMoves != null) {
            allLegalMoves = allLegalMoves
            .stream()
            .filter(
                l -> l != null && l.size() > 0 && l.get(0).equals(player)
            )
            .collect(Collectors.toList());
            return allLegalMoves;
        } else {
            return new ArrayList<List<String>>();
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

    private void buildNextStates(List<List<String>> statesList, List<List<String>> hiddenStatesList) {
        retractAllStates();
        statesSignatures = new HashSet<>();
        hiddenStatesSignatures = new HashSet<>();

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

        for (List<String> state : hiddenStatesList) {
            String stateName = state.get(0).substring(6);
            statesBuilder.append("state_hidden_function_").append(stateName);

            statesBuilder.append("(");

            for (int i = 1; i < state.size(); i++) {
                statesBuilder.append(state.get(i));
                if (i + 1 < state.size()) {
                    statesBuilder.append(", ");
                }
            }

            statesBuilder.append(").\n");

            FunctionSignature signature = new FunctionSignature(stateName, state.size() - 1);
            hiddenStatesSignatures.add(signature);
        }
        
        StringBuilder dynamicsBuilder = new StringBuilder();
        for (FunctionSignature s : statesSignatures) {
            dynamicsBuilder.append(":- dynamic state_function_" + s.functionName + "/" + s.arity + ".\n");
        }
        for (FunctionSignature s : hiddenStatesSignatures) {
            dynamicsBuilder.append(":- dynamic state_hidden_function_" + s.functionName + "/" + s.arity + ".\n");
        }

        StringBuilder commandBuilder = new StringBuilder();
        commandBuilder.append("[user].\n");
        commandBuilder.append(dynamicsBuilder.toString());
        commandBuilder.append(statesBuilder.toString());
        commandBuilder.append("end_of_file.\n");

        execute(commandBuilder.toString());
    }

    public void saveState(String name) {
        List<List<String>> stateToSave = this.getGameState();
        List<List<String>> hiddenStateToSave = this.getGameState();
        List<List<String>> state = stateToSave.stream().map(l -> l.stream().map(s -> "value_" + s).collect(Collectors.toList())).collect(Collectors.toList());
        List<List<String>> hiddenState = hiddenStateToSave.stream().map(l -> l.stream().map(s -> "value_" + s).collect(Collectors.toList())).collect(Collectors.toList());
        
        @SuppressWarnings("unchecked")
        List<List<String>>[] save = new List[2];
        save[0] = state;
        save[1] = hiddenState;

        this.savedStates.put(name, save);
    }

    public boolean restoreState(String name) {
        List<List<String>>[] stateToRestore = this.savedStates.get(name);
        if (stateToRestore != null) {
            this.buildNextStates(stateToRestore[0], stateToRestore[1]);
            return true;
        }
        return false;
    }

    public void reset() {
        this.restoreState("initialState");
    }

    public List<List<String>> interpret(String line) {
        return interpret(Command.createMoveFromLine(line));
    }

    public List<List<String>> interpret(Command command) {
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

        List<List<String>> allResults = null;
        List<List<String>> allHiddenResults = null;

        if (isLegal(command)) {
            allResults = new LinkedList<>();
            for (FunctionSignature next : nextSignatures) {
                List<List<String>> models = getAllModels(next.functionName, next.arity);
                allResults.addAll(models);
            }

            allHiddenResults = new LinkedList<>();
            for (FunctionSignature next : hiddenNextSignatures) {
                List<List<String>> models = getAllModels(next.functionName, next.arity);
                allHiddenResults.addAll(models);
            }

            allHiddenResults = allHiddenResults.stream().map(
                list -> {
                    ArrayList<String> l = new ArrayList<>(list);
                    Collections.swap(l, 0, 1);
                    return l;
                }
            ).collect(Collectors.toList());
        }

        execute(retractBuilder.toString());

        if (allResults != null) {
            buildNextStates(allResults, allHiddenResults);
            System.out.println(allResults);
            allResults = allResults
                .stream().map(
                    l -> l.stream().map(s -> s.substring(6)).collect(Collectors.toList())
                ).collect(Collectors.toList());
        }

        if (hasRandom) {
            List<List<String>> randomResults = doRandom();
            if (randomResults != null) return randomResults;
        }

        return allResults;
    }

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
            result = result.substring(6, result.length() - 2);

            List<String> moveResult = new ArrayList<String>(Arrays.asList(result.split(", ")));
            moveResult.add(0, "value_random");
            return moveResult;
        }

        // A = 2.
        result = result.substring(4, result.length() - 1);
        
        return List.of("value_random", result);
    }

    private List<List<String>> doRandom() {
        List<List<String>> finalResults = null;
        List<String> randomMove = getRandomMove();

        while (randomMove != null) {
            // random command
            // randomMove = randomMove.stream().map(s -> "value_" + s).collect(Collectors.toList());
            Command command = Command.createMoveFromList(randomMove);
            if (debugMode) System.out.println("R " + command);

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
    
            List<List<String>> allResults = null;
            List<List<String>> allHiddenResults = null;
    
            if (isLegal(command)) {
                allResults = new LinkedList<>();
                for (FunctionSignature next : nextSignatures) {
                    List<List<String>> models = getAllModels(next.functionName, next.arity);
                    allResults.addAll(models);
                }
            
                allHiddenResults = new LinkedList<>();
                for (FunctionSignature next : hiddenNextSignatures) {
                    List<List<String>> models = getAllModels(next.functionName, next.arity);
                    allHiddenResults.addAll(models);
                }

                allHiddenResults = allHiddenResults.stream().map(
                    list -> {
                        ArrayList<String> l = new ArrayList<>(list);
                        Collections.swap(l, 0, 1);
                        return l;
                    }
                ).collect(Collectors.toList());
            }
    
            execute(retractBuilder.toString());
    
            if (allResults != null) {
                buildNextStates(allResults, allHiddenResults);
            
                allResults = allResults
                    .stream().map(
                        l -> l.stream().map(s -> s.substring(6)).collect(Collectors.toList())
                    ).collect(Collectors.toList());
            }
    
            finalResults = allResults;

            randomMove = getRandomMove();
        }

        return finalResults;
    }

    public List<List<String>> getGameStateForRole(String role) {
        List<List<String>> hiddenGameState = getHiddenGameState();

        hiddenGameState = hiddenGameState.stream().filter(list -> list.get(1).equals(role)).map(
            list -> {
                ArrayList<String> l = new ArrayList<>(list);
                l.remove(1);
                return l;
            }
        ).collect(Collectors.toList());

        hiddenGameState = new ArrayList<>(hiddenGameState);
        hiddenGameState.addAll(getGameState());
        return hiddenGameState;
    }

    public List<List<String>> getHiddenGameState() {
        List<List<String>> allHiddenResults = new LinkedList<>();
        for (FunctionSignature state : hiddenStatesSignatures) {
            List<List<String>> models = getAllModels("state_hidden_function_" + state.functionName, state.arity);

            if (models == null) {
                allHiddenResults.add(List.of("123456" + state.functionName));
                continue;
            }

            models = models.stream().map(list -> {
                List<String> l = new ArrayList<>();
                l.add("123456" + state.functionName);
                l.addAll(list);
                return l;
            }).collect(Collectors.toList());

            allHiddenResults.addAll(models);
        }

        allHiddenResults = allHiddenResults
            .stream().map(
                l -> l.stream().map(s -> s.substring(6)).collect(Collectors.toList())
            ).collect(Collectors.toList());
        
        return allHiddenResults;
    }

    public List<List<String>> getGameState() {
        List<List<String>> allResults = new LinkedList<>();
        for (FunctionSignature state : statesSignatures) {
            List<List<String>> models = getAllModels("state_function_" + state.functionName, state.arity);

            if (models == null) {
                allResults.add(List.of("123456" + state.functionName));
                continue;
            }

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

    /**
     * Retrieve all roles playable in the current gdl model.
     * @return Set of all playable roles.
     */
    public Set<String> getRoles() {
        return roles.stream().filter(r -> !r.equals("random")).collect(Collectors.toSet());
    }

    public void setDebugMode(boolean debugMode) {
        this.debugMode = debugMode;
    }

}
