package de.monticore.lang.gdl;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import java.io.File;
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

import org.junit.Test;

public class MatchTests {

    @Test
    public void testMatches() throws Exception {
        File testDir = new File("src/test/resources/test-matches/");
        for (File testFile : testDir.listFiles()) {
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
            assertNotNull(name);
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
                    goals.get().put(tuple.getElements().get(1), tuple.getElements().get(2));
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
    
                    assertEquals(fileName + ": Last move with number " + moveCounter + " failed: " + lastCommand
                            + " with states:\n" + interpreterState.toString(), state,
                            interpreterState);

                } else if (object instanceof PartialState) {
                    Set<GDLType> partialState = ((PartialState) object).get();
    
                    Set<GDLType> interpreterState = interpreter.getVisibleGameState();
    
                    assertTrue(fileName + ": Last move with number " + moveCounter + " failed: " + lastCommand + " with states:\n"
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
                        assertTrue(fileName + ": Last move with number " + moveCounter + " failed: " + lastCommand
                            + " with states:\n" + interpreterState.toString(), found);
                    }
                } else if (object instanceof CommandInput) {
                    lastCommand = ((CommandInput) object).get();
                    System.out.println("  COMMAND " + lastCommand);
                    boolean legal = interpreter.interpret(lastCommand);
                    assertTrue("Move was illegal: " + lastCommand, legal);
                    moveCounter++;
                } else if (object instanceof Goal) {
                    Map<GDLType, GDLType> goal = ((Goal) object).get();
                    Map<GDLType, GDLType> interpreterGoal = interpreter.getGoals();

                    System.out.printf("  GOAL %s equals %s\n", goal.toString(), interpreterGoal.toString());

                    assertEquals(fileName + ": Last move with number " + moveCounter + " failed: " + lastCommand
                    + " with goals:\n" + interpreterGoal.toString(), goal, interpreterGoal);
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
            assertTrue("An unexpected Error occurred.", false);
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
