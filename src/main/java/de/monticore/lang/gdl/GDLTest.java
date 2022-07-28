package de.monticore.lang.gdl;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import de.monticore.lang.gdl.types.GDLTuple;
import de.monticore.lang.gdl.types.GDLType;

public class GDLTest {
    
    public static void main(String[] args) throws Exception {
        if (args.length < 1) {
            System.err.println("At least one test file must be provided.");
            System.exit(-1);
        }

        for (int i = 0; i < args.length; i++) {
            File testFile = new File(args[i]);

            try {
                new GDLTest(new LocalTestImpl()).testMatch(testFile);
            } catch (Throwable e) {
                e.printStackTrace();
                System.err.println("Test failed.");
                for (Interpreter interpreter : interpreterCache.values()) {
                    interpreter.close();
                }
                System.exit(-1);
            }
        }

        for (Interpreter interpreter : interpreterCache.values()) {
            interpreter.close();
        }
    }

    private TestImpl testImpl;

    public GDLTest(TestImpl testImpl) {
        this.testImpl = testImpl;
    }

    public interface TestImpl {
        void assertEquals(String message, Object expected, Object actual);
        void assertNotNull(Object object);
        void assertTrue(String message, boolean condition);
    }

    private static class LocalTestImpl implements TestImpl {
        public void assertEquals(String message, Object expected, Object actual) {
            try {
                if (expected == null) {
                    assert actual == null;
                }
                assert expected.equals(actual);
            } catch (AssertionError e) {
                throw new AssertionError(message, e);
            }
        }
    
        public void assertNotNull(Object object) {
            try {
                assert object != null;
            } catch (AssertionError e) {
                throw e;
            }
        }
    
        public void assertTrue(String message, boolean condition) {
            try {
                assert condition;
            } catch (AssertionError e) {
                throw new AssertionError(message, e);
            }
        }
    }

    public void testMatch(File testFile) throws IOException {
        List<TestInput<?>> input = new LinkedList<>();
        String fileName = testFile.getName();
        
        String file = Files.readString(testFile.toPath());

        Iterator<String> lines = file.lines().iterator();

        Mode mode = null;

        Goal goals = new Goal();
        State states = new State();
        PartialState partialStates = new PartialState();

        String name = null;
        if (lines.hasNext()) {
            name = lines.next();
        }
        testImpl.assertNotNull(name);
        System.out.printf("Testing match '%s' for GDL model '%s'\n", fileName, name);

        while (lines.hasNext()) {
            String line = lines.next();

            if (line.startsWith("#")) {
                continue;
            }

            if (line.equals("COMMAND")) {
                if (mode == Mode.STATE) {
                    input.add(states);
                    states = new State();
                }
                if (mode == Mode.PARTIAL_STATE) {
                    input.add(partialStates);
                    partialStates = new PartialState();
                }
                if (mode == Mode.GOAL) {
                    input.add(goals);
                    goals = new Goal();
                }
                mode = Mode.COMMAND_INPUT;
            } else if (line.equals("GOAL")) {
                mode = Mode.GOAL;
            } else if (line.equals("STATE")) {
                mode = Mode.STATE;
            } else if (line.equals("PARTIAL_STATE")) {
                mode = Mode.PARTIAL_STATE;
            } else if (line.length() == 0) {
                continue;
            } else if (mode == Mode.STATE) {
                states.get().add(GDLType.createFromLine(line));
            } else if (mode == Mode.PARTIAL_STATE) {
                partialStates.get().add(GDLType.createFromLine(line));
            } else if (mode == Mode.COMMAND_INPUT) {
                input.add(new CommandInput().set(Command.createFromLine(line)));
            } else if (mode == Mode.GOAL) {
                GDLTuple tuple = GDLTuple.createFromLine(line);
                goals.get().put(tuple.get(1), tuple.get(2));
            }
        }

        if (mode == Mode.STATE) {
            input.add(states);
        }

        if (mode == Mode.PARTIAL_STATE) {
            input.add(partialStates);
        }

        if (mode == Mode.GOAL) {
            input.add(goals);
        }

        traverseInput(input, name, fileName);

        System.out.println("OK");
    }

    private static final Map<String, Interpreter> interpreterCache = new HashMap<>();

    private static Interpreter getInterpreterForGDLFile(String gdlFile) {
        Interpreter result = interpreterCache.get(gdlFile);
        if (result != null) {
            result.reset();
        } else {
            result = Interpreter.fromGDLFile(gdlFile, new InterpreterOptions().manualRandom(true));
            interpreterCache.put(gdlFile, result);
        }
        return result;
    }

    private void traverseInput(List<TestInput<?>> input, String gdlFile, String fileName) {
        try {
            Interpreter interpreter = getInterpreterForGDLFile(gdlFile);
            int moveCounter = 0;
            Command lastCommand = null;
    
            for (Object object : input) {
                if (object instanceof State) {
                    Set<GDLType> state = ((State) object).get();
    
                    Set<GDLType> interpreterState = interpreter.getVisibleGameState();
                    
                    System.out.printf("  STATE %s equals %s\n", state.toString(), interpreterState.toString());
    
                    testImpl.assertEquals(fileName + ": Last move with number " + moveCounter + " failed: " + lastCommand
                            + " with states:\n" + interpreterState.toString(), state,
                            interpreterState);

                } else if (object instanceof PartialState) {
                    Set<GDLType> partialState = ((PartialState) object).get();
    
                    Set<GDLType> interpreterState = interpreter.getVisibleGameState();
    
                    testImpl.assertTrue(fileName + ": Last move with number " + moveCounter + " failed: " + lastCommand + " with states:\n"
                            + interpreterState.toString(), partialState.size() <= interpreterState.size());
                    
                    System.out.printf("  PARTIAL_STATE %s subset of %s\n", partialState.toString(), interpreterState.toString());

                    for (GDLType state : partialState) {
                        boolean found = false;
                        for (GDLType inState : interpreterState) {
                            if (state.equals(inState)) {
                                found = true;
                                break;
                            }
                        }
                        testImpl.assertTrue(fileName + ": Last move with number " + moveCounter + " failed: " + lastCommand
                            + " with states:\n" + interpreterState.toString(), found);
                    }
                } else if (object instanceof CommandInput) {
                    lastCommand = ((CommandInput) object).get();
                    System.out.println("  COMMAND " + lastCommand);
                    boolean legal = interpreter.interpret(lastCommand);
                    testImpl.assertTrue("Move was illegal: " + lastCommand, legal);
                    moveCounter++;
                } else if (object instanceof Goal) {
                    Map<GDLType, GDLType> goal = ((Goal) object).get();
                    Map<GDLType, GDLType> interpreterGoal = interpreter.getGoals();

                    System.out.printf("  GOAL %s equals %s\n", goal.toString(), interpreterGoal.toString());

                    testImpl.assertEquals(fileName + ": Last move with number " + moveCounter + " failed: " + lastCommand
                    + " with goals:\n" + interpreterGoal.toString(), goal, interpreterGoal);
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
            testImpl.assertTrue("An unexpected Error occurred.", false);
        }
    }

    private enum Mode {
        COMMAND_INPUT,
        GOAL,
        STATE,
        PARTIAL_STATE
    }

    private interface TestInput<E> {
        TestInput<E> set(E test);
        E get();
        Mode getMode();
    }
    private final class CommandInput implements TestInput<Command> {
        Command s;
        public CommandInput set(Command test) {
            s = test;
            return this;
        }
        public Command get() {
            return s;
        }
        @Override
        public Mode getMode() {
            return Mode.COMMAND_INPUT;
        }
    }
    private final class Goal implements TestInput<Map<GDLType, GDLType>> {
        Map<GDLType, GDLType> s = new HashMap<>();
        public Goal set(Map<GDLType, GDLType> test) {
            s = test;
            return this;
        }
        public Map<GDLType, GDLType> get() {
            return s;
        }
        @Override
        public Mode getMode() {
            return Mode.GOAL;
        }
    }
    private final class State implements TestInput<Set<GDLType>> {
        Set<GDLType> s = new HashSet<>();
        public State set(Set<GDLType> test) {
            s = test;
            return this;
        }
        public Set<GDLType> get() {
            return s;
        }
        @Override
        public Mode getMode() {
            return Mode.STATE;
        }
    }
    private final class PartialState implements TestInput<Set<GDLType>> {
        Set<GDLType> s = new HashSet<>();
        public PartialState set(Set<GDLType> test) {
            s = test;
            return this;
        }
        public Set<GDLType> get() {
            return s;
        }
        @Override
        public Mode getMode() {
            return Mode.PARTIAL_STATE;
        }
    }

}
