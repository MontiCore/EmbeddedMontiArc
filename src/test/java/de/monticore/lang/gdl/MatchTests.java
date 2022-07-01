package de.monticore.lang.gdl;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import java.io.File;
import java.nio.file.Files;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import com.google.common.collect.Ordering;

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

            boolean isInCommand = false;
            boolean isInState = false;
            boolean isInPartialState = false;
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

                if (line.equals("COMMAND") || line.equals("GOAL")) {
                    if (isInState) {
                        input.add(states);
                        states = new State();
                    }
                    if (isInPartialState) {
                        input.add(partialStates);
                        partialStates = new PartialState();
                    }
                    isInCommand = true;
                    isInState = false;
                    isInPartialState = false;
                } else if (line.equals("STATE")) {
                    isInState = true;
                    isInPartialState = false;
                    isInCommand = false;
                } else if (line.equals("PARTIAL_STATE")) {
                    isInState = false;
                    isInPartialState = true;
                    isInCommand = false;
                } else if (line.length() == 0) {
                    continue;
                } else if (isInState) {
                    states.get().add(line);
                } else if (isInPartialState) {
                    partialStates.get().add(line);
                } else if (isInCommand) {
                    input.add(new Command().set(line));
                }
            }

            if (isInState) {
                input.add(states);
            }

            if (isInPartialState) {
                input.add(partialStates);
            }

            traverseInput(input, name, fileName);

            System.out.println("OK");
        }
    }

    private void traverseInput(List<TestInput<?>> input, String name, String fileName) {
        try (Interpreter interpreter = new Interpreter(GDLInterpreter.parse(name)).init(new InterpreterOptions().manualRandom(true))) {
            int moveCounter = 0;
            Command lastMove = null;
    
            for (Object object : input) {
                if (object instanceof State) {
                    List<String> state = ((State) object).get();
    
                    List<String> interpreterState = interpreter.getGameState().stream().map(list -> {
                        StringBuilder sb = new StringBuilder();
                        sb.append("(");
                        for (int i = 0; i < list.size(); i++) {
                            sb.append(list.get(i));
                            if (i + 1 < list.size()) {
                                sb.append(" ");
                            }
                        }
                        sb.append(")");
                        return sb.toString();
                    }).collect(Collectors.toList());
    
                    assertEquals(fileName + ": Last move with number " + moveCounter + " failed: " + lastMove + " with states:\n"
                            + interpreterState.toString(), state.size(), interpreterState.size());
    
                    state.sort(Ordering.natural());
                    interpreterState.sort(Ordering.natural());
                    
                    System.out.printf("  STATE %s equals %s\n", state.toString(), interpreterState.toString());
    
                    for (int i = 0; i < state.size(); i++) {
                        assertEquals(fileName + ": Last move with number " + moveCounter + " failed: " + lastMove
                                + " with states:\n" + interpreterState.toString(), state.get(i),
                                interpreterState.get(i));
                    }
                } else if (object instanceof PartialState) {
                    List<String> partialState = ((PartialState) object).get();
    
                    List<String> interpreterState = interpreter.getGameState().stream().map(list -> {
                        StringBuilder sb = new StringBuilder();
                        sb.append("(");
                        for (int i = 0; i < list.size(); i++) {
                            sb.append(list.get(i));
                            if (i + 1 < list.size()) {
                                sb.append(" ");
                            }
                        }
                        sb.append(")");
                        return sb.toString();
                    }).collect(Collectors.toList());
    
                    assertTrue(fileName + ": Last move with number " + moveCounter + " failed: " + lastMove + " with states:\n"
                            + interpreterState.toString(), partialState.size() <= interpreterState.size());
    
                    partialState.sort(Ordering.natural());
                    interpreterState.sort(Ordering.natural());
                    
                    System.out.printf("  PARTIAL_STATE %s subset of %s\n", partialState.toString(), interpreterState.toString());
    
                    for (int i = 0; i < partialState.size(); i++) {
                        boolean found = false;
                        for (int j = i; j < interpreterState.size(); j++) {
                            if (partialState.get(i).equals(interpreterState.get(j))) {
                                found = true;
                                break;
                            }
                        }
                        assertTrue(fileName + ": Last move with number " + moveCounter + " failed: " + lastMove
                                + " with states:\n" + interpreterState.toString(), found);
                    }
                } else if (object instanceof Command) {
                    String asString = ((Command) object).get();
                    if (asString.startsWith("(goal")) {
                        Set<List<String>> goals = interpreter.getAllModels("goal");
                        if (goals != null) {
                            List<String> goalsFormatted = new LinkedList<>();
                            for (List<String> goal : goals) {
                                StringBuilder sb = new StringBuilder();
                                sb.append("(goal ");
    
                                for (int i = 0; i < goal.size(); i++) {
                                    sb.append(goal.get(i));
                                    if (i + 1 < goal.size()) {
                                        sb.append(" ");
                                    }
                                }
                                sb.append(")");
    
                                goalsFormatted.add(sb.toString());
                            }
    
                            System.out.printf("  GOAL %s in %s\n", asString, goalsFormatted.toString());
                            assertTrue(fileName + ": Goal Wrong!", goalsFormatted.contains(asString));
                        }
    
                    } else {
                        lastMove = (Command) object;
                        System.out.println("  COMMAND " + lastMove.get());
                        Object result = interpreter.interpret(lastMove.get());
                        assertNotNull("Move was illegal: " + lastMove.get(), result);
                        moveCounter++;
                    }
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
            assertTrue("An unexpected Error occurred.", false);
        }
    }

    private interface TestInput<E> {
        TestInput<E> set(E test);
        E get();
    }
    private final class Command implements TestInput<String> {
        String s;
        public Command set(String test) {
            s = test;
            return this;
        }
        public String get() {
            return s;
        }
    }
    private final class State implements TestInput<List<String>> {
        List<String> s = new LinkedList<>();
        public State set(List<String> test) {
            s = test;
            return this;
        }
        public List<String> get() {
            return s;
        }
    }
    private final class PartialState implements TestInput<List<String>> {
        List<String> s = new LinkedList<>();
        public PartialState set(List<String> test) {
            s = test;
            return this;
        }
        public List<String> get() {
            return s;
        }
    }

}
