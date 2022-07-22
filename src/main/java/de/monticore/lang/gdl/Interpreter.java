package de.monticore.lang.gdl;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.Reader;
import java.io.StringReader;
import java.io.Writer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Scanner;
import java.util.Set;
import java.util.concurrent.Semaphore;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;

import de.monticore.lang.gdl._ast.ASTGame;
import de.monticore.lang.gdl._parser.GDLParser;
import de.monticore.lang.gdl._symboltable.GDLScopesGenitor;
import de.monticore.lang.gdl._symboltable.IGDLGlobalScope;
import de.monticore.lang.gdl._visitor.GDLTraverser;
import de.monticore.lang.gdl.cocos.AllCoCosChecker;
import de.monticore.lang.gdl.cocos.types.TypesCoCosChecker;
import de.monticore.lang.gdl.types.GDLNumber;
import de.monticore.lang.gdl.types.GDLTuple;
import de.monticore.lang.gdl.types.GDLType;
import de.monticore.lang.gdl.visitors.ASTTypeInflator;
import de.monticore.lang.gdl.visitors.PrologPrinter;
import de.monticore.lang.gdl.visitors.TypeTemplatePrinter;
import de.se_rwth.commons.logging.Log;

public class Interpreter extends EventSource<GDLType, Set<GDLType>> implements AutoCloseable {

    private static final Pattern RESULT_ARRAY_PATTERN = Pattern.compile("\\[(.*)\\]");
    private static final Pattern SINGLE_RESULT_PATTERN = Pattern.compile("=\\s*(.*?)\\.");

    private final String prologProgram;
    private final InterpreterOptions options;

    private Process prologProcess;
    private Writer prologWriter;
    private Thread printInThread, printErrThread;

    private int initTrueCounter;
    private Semaphore initSemaphore;

    private List<String> currentEvaluation = null;
    private Semaphore evalSemaphore;
    private int evalToRelease = 0;
    private Throwable evalError;

    private Semaphore executeSemaphore = new Semaphore(1);

    private boolean withTypes = false;
    private int stateSpaceDimension = 0;
    private int actionSpaceDimension = 0;

    private Interpreter(String prolog, InterpreterOptions options) {
        this.prologProgram = prolog;
        this.options = options == null ? new InterpreterOptions() : options;
    }

    public String getPrologProgram() {
        return this.prologProgram;
    }

    private synchronized void init() {
        try {
            long time = System.currentTimeMillis();
            prologProcess = Runtime.getRuntime().exec("swipl");

            prologProcess.onExit().thenAcceptAsync(p -> {
                int exitValue = p.exitValue();
                if (currentEvaluation != null) {
                    evalError = new IllegalStateException("The prolog Process exited unexpectedly with exit code " + exitValue);
                    evalSemaphore.release(evalToRelease);
                }
            });

            // setup pipes
            OutputStream out = new BufferedOutputStream(prologProcess.getOutputStream());
            prologWriter = new BufferedWriter(new OutputStreamWriter(out)) {
                @Override
                public void write(String str) throws IOException {
                    if (options.isDebugMode()) System.out.print("? " + str);
                    super.write(str);
                }
            };

            final InputStream in = new BufferedInputStream(prologProcess.getInputStream());
            printInThread = new Thread(new Runnable() {
                @Override
                public void run() {
                    Scanner s = new Scanner(in);
                    while(s.hasNextLine()) {
                        String line = s.nextLine();
                        if (line.strip().length() == 0) {
                            continue;
                        }
                    
                        readIn(line.strip());
                    }
                    s.close();
                }
            });
    
            final InputStream err = new BufferedInputStream(prologProcess.getErrorStream());
            printErrThread = new Thread(new Runnable() {
                @Override
                public void run() {
                    Scanner s = new Scanner(err);
                    while(s.hasNextLine()) {
                        String line = s.nextLine();
                        if (line.strip().length() == 0) {
                            continue;
                        }
                        readErr(line.strip());
                    }
                    s.close();
                }
            });
    
            printInThread.start();
            printErrThread.start();


            // load program into Prolog
            initSemaphore = new Semaphore(0);
            initTrueCounter = 0;

            prologWriter.write("[user].\n");
            prologWriter.write(prologProgram);
            prologWriter.write("end_of_file.\n");
            prologWriter.flush();

            initSemaphore.acquire();

            // load options
            loadOptions();

            // init state
            execute("gdli_init.");

            if (options.isWithTypes()) loadTypes();

            if (options.isShowTimes()) {
                long ellapsed = System.currentTimeMillis() - time;
                System.out.printf("X Initialization took %.4f seconds.\n", ellapsed/1000f);
            }
        } catch (IOException | InterruptedException e) {
            Log.error("Failed to initialize interpreter.", e);
        }
    }

    @Override
    public void close() throws Exception {
        printInThread.interrupt();
        printErrThread.interrupt();
        prologWriter.flush();
        prologWriter.close();
        prologProcess.destroy();
    }

    private void readIn(String line) {
        if (options.isDebugMode()) System.out.println("| " + line);

        if (line.equals("true.")) {
            initTrueCounter++;
            if (initTrueCounter == 1) {
                initSemaphore.release();
            }
        }

        if (currentEvaluation != null) {
            currentEvaluation.add(line);
            evalSemaphore.release(1);
        }
    }

    private void readErr(String line) {
        if (options.isDebugMode()) System.out.println("! " + line);

        if (currentEvaluation != null) {
            if (line.startsWith("ERROR")) {
                evalError = new IllegalStateException("The current evaluation faced an unexpected error in the prolog process.");
                evalSemaphore.release(evalToRelease);
            }
        }
    }

    private void loadOptions() {
        if (options.isManualRandom()) {
            execute("assertz(gdli_options(manual_random)).");
        }
    }

    private void loadTypes() {
        String withTypesResult = execute("gdli_with_types.");
        if (withTypesResult.startsWith("true")) {
            this.withTypes = true;

            String stateDimensionResult = execute("gdlt_get_dimension(state, X).");
            String actionDimensionResult = execute("gdlt_get_dimension(action, X).");

            GDLNumber stateDimension = getSingleGDLType(stateDimensionResult, GDLNumber.class);
            GDLNumber actionDimension = getSingleGDLType(actionDimensionResult, GDLNumber.class);

            this.stateSpaceDimension = stateDimension.getValue().intValue();
            this.actionSpaceDimension = actionDimension.getValue().intValue();
        }
    }

    private synchronized String execute(String command) {
        List<String> result = execute(command, 1);
        return result == null ? null : result.get(0);
    }

    private synchronized List<String> execute(String command, int waitEvalLines) {
        try {
            executeSemaphore.acquire();
        } catch (InterruptedException e) {
            e.printStackTrace();
            return null;
        }

        if (!command.endsWith("\n")) command = command + "\n";
        currentEvaluation = new ArrayList<>(waitEvalLines);
        this.evalSemaphore = new Semaphore(0);

        try {
            prologWriter.write(command);
            prologWriter.flush();

            this.evalToRelease = waitEvalLines;
            for (int i = 0; i < waitEvalLines; i++) {
                evalSemaphore.acquire(1);
                this.evalToRelease--;
            }

            if (evalError != null) {
                currentEvaluation = null;
                
                if (!options.isDebugMode()) throw new RuntimeException("The current evaluation faced an unexpected Error. Re-run your program in debug mode to get more detail.", evalError);
                throw new RuntimeException("The current evaluation faced an unexpected Error.", evalError);
            }
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
            executeSemaphore.release();
            return null;
        }

        List<String> result = currentEvaluation;
        currentEvaluation = null;

        executeSemaphore.release();
        return result;
    }

    public boolean interpret(String line) {
        return interpret(Command.createFromLine(line));
    }

    public boolean interpret(Command command) {
        long time = System.currentTimeMillis();

        String move = command.toPlString();
        String queryResult = execute("gdli_do_move(" + move + ").");
        boolean success = toBooleanValue(queryResult);
        if (success && !command.isNoop()) {
            stateHasChanged();
        }

        if (options.isShowTimes()) {
            long ellapsed = System.currentTimeMillis() - time;
            System.out.printf("X Move took %.4f seconds.\n", ellapsed/1000f);
        }
        return success;
    }

    public boolean isLegal(Command command) {
        String move = command.toPlString();
        String queryResult = execute("gdli_legal(" + move + ").");
        return toBooleanValue(queryResult);
    }

    /**
     * Reset the game state to the initial state.
     * @return true, if the reset was successfull, false otherwise
     */
    public boolean reset() {
        String queryResult = execute("gdli_reset().");
        boolean success = toBooleanValue(queryResult);
        if (success) {
            stateHasChanged();
        }
        return success;
    }

    /**
     * Get all moves that are legal in the current state.
     * @return Set of all legal moves
     */
    public Set<Command> getAllLegalMoves() {
        return getAllLegalMovesForRole(null);
    }

    /**
     * Get all moves that are legal for a role in the current state.
     * @param role Role that the legal moves are filtered by.
     * @return Set of all legal moves for role
     */
    public Set<Command> getAllLegalMovesForRole(GDLType role) {
        Set<Command> resultSet = new HashSet<>();

        String queryResult;

        if (role != null) {
            String plRole = role.toPlString();
            queryResult = execute("gdli_all_legal_moves(" + plRole + ", Models).");
        } else {
            queryResult = execute("gdli_all_legal_moves(Models).");
        }

        final Consumer<GDLTuple> addToResultSet = t -> resultSet.add(Command.createFromGDLTuple(t));
        matchAllGDLTypes(queryResult, addToResultSet);

        return resultSet;
    }

    /**
     * Get the openly visible part of the current game state.
     * @return Full visible game state
     */
    public Set<GDLType> getVisibleGameState() {
        String queryResult = execute("gdli_all_state(Models).");

        Set<GDLType> resultSet = new HashSet<>();
        matchAllGDLTypes(queryResult, resultSet::add);

        return resultSet;
    }

    /**
     * Get the hidden part of the current game state, where the first index
     * of each tuple indicates the visibility of the state.
     * @return Hidden game states mapped to their respective roles
     */
    public Map<GDLType, Set<GDLType>> getHiddenGameState() {
        String queryResult = execute("gdli_all_state_hidden(Models).");

        Map<GDLType, Set<GDLType>> resultMap = new HashMap<>();
        matchAllGDLTypes(queryResult, t -> {
            GDLTuple tuple = ((GDLTuple) t);
            Set<GDLType> roleState = resultMap.get(tuple.get(0));
            if (roleState == null) {
                roleState = new HashSet<>();
                resultMap.put(tuple.get(0), roleState);
            }
            roleState.add(tuple.get(1));
        });

        return resultMap;
    }

    /**
     * Get the combined game state for a role, which consists of the visible state and
     * the part of the hidden state only visible to role.
     * @param role
     * @return Game state visible to role
     */
    public Set<GDLType> getGameStateForRole(GDLType role) {
        String plRole = role.toPlString();
        String queryResult = execute("gdli_full_state_role(" + plRole + ", Models).");

        Set<GDLType> resultSet = new HashSet<>();
        matchAllGDLTypes(queryResult, resultSet::add);

        return resultSet;
    }

    /**
     * Evaluates the terminal condition.
     * @return true, if the program is in terminal state, false otherwise
     */
    public boolean isTerminal() {
        String queryResult = execute("gdli_terminal().");
        return toBooleanValue(queryResult);
    }

    /**
     * Evaluates the goal condition.
     * @return Roles mapped to their respective goals.
     */
    public Map<GDLType, GDLType> getGoals() {
        String queryResult = execute("gdli_all_goal(Models).");

        Map<GDLType, GDLType> resultMap = new HashMap<>();
        
        final Consumer<GDLTuple> addToResultMap = t ->
                resultMap.put(t.get(0), t.get(1));
        matchAllGDLTypes(queryResult, addToResultMap);

        return resultMap;
    }

    /**
     * Get all roles defined in the program
     * @return Set of all roles
     */
    public Set<GDLType> getRoles() {
        String queryResult = execute("gdli_all_role(Models).");

        Set<GDLType> resultSet = new HashSet<>();
        matchAllGDLTypes(queryResult, resultSet::add);

        return resultSet;
    }

    public boolean isWithTypes() {
        return this.withTypes;
    }

    public int getStateSpaceDimension() {
        return this.stateSpaceDimension;
    }

    public int getActionSpaceDimension() {
        return this.actionSpaceDimension;
    }

    public int getIndexForAction(GDLType action) {
        String result = execute("gdlt_index_map(action, " + action.toPlString() + ", X).");
        GDLNumber index = getSingleGDLType(result, GDLNumber.class);

        return index.getValue().intValue();
    }

    public GDLType getActionForIndex(int index) {
        GDLNumber gdlIndex = GDLNumber.createFromLine(index + "");
        String result = execute("gdlt_index_map(action, X, " + gdlIndex.toPlString() + ").");
        GDLType action = getSingleGDLType(result, GDLType.class);
        return action;
    }

    public int getIndexForState(GDLType state) {
        String result = execute("gdlt_index_map(state, " + state.toPlString() + ", X).");
        GDLNumber index = getSingleGDLType(result, GDLNumber.class);

        return index.getValue().intValue();
    }

    public float[] getStateIndicatorMatrixForRole(GDLType role) {
        String queryResult = execute("gdlt_role_indicator_matrix(" + role.toPlString() + ", Matrix).");

        float[] result = new float[stateSpaceDimension];

        final Consumer<GDLNumber> numberConsumer = n -> result[n.getValue().intValue()] = 1;
        matchAllGDLTypes(queryResult, numberConsumer);

        return result;
    }

    private <E extends GDLType> E getSingleGDLType(String queryResult, Class<E> typeClass) {
        Matcher matcher = SINGLE_RESULT_PATTERN.matcher(queryResult);

        if (matcher.find()) {
            String valueString = matcher.group(1);

            @SuppressWarnings("unchecked")
            E result = (E) GDLType.createFromPl(valueString);

            return result;
        }

        throw new IllegalArgumentException("Cannot convert '" + queryResult + "' into Interpreter value");
    }

    private <E extends GDLType> void matchAllGDLTypes(String queryResult, Consumer<E> tupleConsumer) {
        Matcher matcher = RESULT_ARRAY_PATTERN.matcher(queryResult);
        
        if (matcher.find()) {
            String tupleString = matcher.group();
            long time = System.currentTimeMillis();

            GDLTuple tuple = GDLTuple.createFromPl(tupleString);
            
            if (options.isShowTimes()) {
                long ellapsed = System.currentTimeMillis() - time;
                System.out.printf("X Parsing took %.4f seconds.\n", ellapsed/1000f);
            }

            @SuppressWarnings("unchecked")
            final Function<GDLType, E> cast = t -> (E) t;
            
            List<E> castList = tuple.stream().map(cast).collect(Collectors.toList());
            for (E e: castList) {
                tupleConsumer.accept(e);
            }
        }
    }

    private void stateHasChanged() {
        Set<GDLType> roles = getRoles();
        for (GDLType role: roles) {
            notifyAll(role, getGameStateForRole(role));
        }
    }

    /**
     * Converts a value in Prolog representation to a value in GDL representation.
     * @param plValue A value in Prolog representation
     * @return The value in GDL representation
     */
    public static String convertPLValue2GDL(String plValue) {
        int prefixSize;
        if (plValue.startsWith("value_")) {
            prefixSize = "value_".length();
        } else if (plValue.startsWith("numpos_")) {
            prefixSize = "numpos_".length();
        } else if (plValue.startsWith("numneg_")) {
            prefixSize = "numneg_".length();
        } else {
            throw new IllegalArgumentException("Cannot convert '" + plValue + "' into Interpreter value");
        }

        String vPart = plValue.substring(prefixSize);
        if (plValue.startsWith("numneg_")) {
            vPart = "-" + vPart;
        }

        return vPart;
    }

    /**
     * Converts a value in GDL representation to a value in Prolog representation.
     * @param gdlValue A value in GDL representation
     * @return The value in Prolog representation
     */
    public static String convertGDLValue2PL(String gdlValue) {
        Optional<Integer> maybeNumber = toNumber(gdlValue);
        if (maybeNumber.isPresent()) {
            int num = maybeNumber.get();

            if (num < 0) {
                return "numneg_" + (num * -1);
            } else {
                return "numpos_" + num;
            }
        } else {
            return "value_" + gdlValue;
        }
    }

    private static Optional<Integer> toNumber(String inValue) {
        try {
            int i = Integer.parseInt(inValue);
            return Optional.of(i);
        } catch (NumberFormatException e) {
            return Optional.empty();
        }
    }

    private static boolean toBooleanValue(String plResult) {
        return plResult.strip().contains("true");
    }

    /**
     * Generate an Interpreter with a GDL program.
     * @param gdl GDL program
     * @param options Interpreter options for debugging
     * @return An Interpreter instance for the GDL program
     */
    public static Interpreter fromGDL(String gdl, InterpreterOptions options) {
        try (Reader reader = new StringReader(gdl)) {
            GDLParser parser = GDLMill.parser();
            Optional<ASTGame> optGame = parser.parse(reader);

            if (!parser.hasErrors() && optGame.isPresent()) {
                ASTGame ast = optGame.get();
                return fromAST(ast, options);
            }
            Log.error("Model could not be parsed.");
        } catch (IOException e) {
            Log.error("Failed to parse gdl model.", e);
        }
        return null;
    }

    /**
     * Generate an Interpreter with a GDL program file.
     * @param gdlFilePath File path of the GDL program
     * @param options Interpreter options for debugging
     * @return An Interpreter instance for the GDL program
     */
    public static Interpreter fromGDLFile(String gdlFilePath, InterpreterOptions options) {
        try {
            GDLParser parser = GDLMill.parser();
            Optional<ASTGame> optGame = parser.parse(gdlFilePath);

            if (!parser.hasErrors() && optGame.isPresent()) {
                ASTGame ast = optGame.get();
                return fromAST(ast, options);
            }
            Log.error("Model could not be parsed.");
        } catch (IOException e) {
            Log.error("Failed to parse " + gdlFilePath, e);
        }
        return null;
    }

    /**
     * Generate an Interpreter with a compatible Prolog program.
     * @param prolog Prolog program
     * @param options Interpreter options for debugging
     * @return An Interpreter instance for the Prolog program
     */
    public static Interpreter fromProlog(String prolog, InterpreterOptions options) {
        final Interpreter interpreter = new Interpreter(prolog, options);
        interpreter.init();
        return interpreter;
    }

    /**
     * Generate an Interpreter with a compatible Prolog program file.
     * @param prologFilePath File path of the Prolog program
     * @param options Interpreter options for debugging
     * @return An Interpreter instance for the Prolog program
     */
    public static Interpreter fromPrologPath(String prologFilePath, InterpreterOptions options) {
        try (
            InputStream stream = new FileInputStream(new File(prologFilePath));
            BufferedReader reader = new BufferedReader(new InputStreamReader(stream, "UTF-8"))
        ) {
            String prolog = reader.lines().reduce("", (s1, s2) -> s1 + "\n" + s2) + "\n";
            return fromProlog(prolog, options);
        } catch (Exception e) {
            e.printStackTrace();
            System.err.println("Unable to load util.pl.");
            return null;
        }
    }

    /**
     * Generate an Interpreter with a GDL program in AST representation.
     * @param ast AST of the GDL Program
     * @param options Interpreter options for debugging
     * @return An Interpreter instance for the GDL program
     */
    public static Interpreter fromAST(ASTGame ast, InterpreterOptions options) {
        final IGDLGlobalScope gs = GDLMill.globalScope();
        gs.clear();

        final GDLScopesGenitor genitor = GDLMill.scopesGenitor();
        final GDLTraverser traverser = GDLMill.traverser();
        traverser.setGDLHandler(genitor);
        traverser.add4GDL(genitor);
        genitor.putOnStack(gs);

        gs.addSubScope(genitor.createFromAST(ast));

        final AllCoCosChecker checker = new AllCoCosChecker();
        checker.checkAll(ast);

        boolean typesFail = false;
        if (options != null && options.isWithTypes()) {
            Log.enableFailQuick(false);
            final TypesCoCosChecker tChecker = new TypesCoCosChecker();
            tChecker.checkAll(ast);
            if (Log.getErrorCount() > 0) {
                typesFail = true;
                Log.warn("Types can not be created.");
                Log.clearFindings();
            }
            Log.enableFailQuick(true);
        }

        if (!typesFail) {
            final ASTTypeInflator inflator = new ASTTypeInflator();
            ast.accept(inflator.getTraverser());
        }

        final PrologPrinter printer = new PrologPrinter();
        ast.accept(printer.getTraverser());
        String prolog = printer.getContent();

        if (options != null && options.isWithTypes() && !typesFail) {
            final TypeTemplatePrinter typePrinter = new TypeTemplatePrinter();
            ast.accept(typePrinter.getTraverser());

            prolog += "\n" + typePrinter.getContent();
        }

        return fromProlog(prolog, options);
    }

}
