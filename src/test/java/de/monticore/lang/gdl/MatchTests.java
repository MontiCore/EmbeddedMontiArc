package de.monticore.lang.gdl;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.File;
import java.nio.file.Files;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.stream.Collectors;

import com.google.common.collect.Ordering;

import org.junit.Test;

public class MatchTests {

    @Test
    @SuppressWarnings("unchecked")
    public void TestMatches() {
        File testDir = new File("src/test/resources/test-matches/");
        for (File testFile : testDir.listFiles()) {
            List<Object> input = new LinkedList<>();
            String fileName = testFile.getName();

            try {
                String file = Files.readString(testFile.toPath());

                Iterator<String> lines = file.lines().iterator();

                boolean isInCommand = false;
                boolean isInState = false;
                List<String> states = new LinkedList<>();
                String name = null;
                if (lines.hasNext()) {
                    name = lines.next();
                }
                while (lines.hasNext()) {
                    String line = lines.next();

                    if (line.equals("COMMAND") || line.equals("GOAL")) {
                        if (isInState) {
                            input.add(states);
                            states = new LinkedList<>();
                        }
                        isInCommand = true;
                        isInState = false;
                    } else if (line.equals("STATE")) {
                        isInState = true;
                        isInCommand = false;
                    } else if (line.length() == 0) {
                        continue;
                    } else if (isInState) {
                        states.add(line);
                    } else if (isInCommand) {
                        input.add(line);
                    }
                }

                if (isInState) {
                    input.add(states);
                }

                Interpreter interpreter = new Interpreter(GDLInterpreter.parse(name))
                        .init();

                int moveCounter = 0;
                String lastMove = null;

                for (Object object : input) {
                    if (object instanceof List) {
                        List<String> state = (List<String>) object;

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

                        for (int i = 0; i < state.size(); i++) {
                            assertEquals(fileName + ": Last move with number " + moveCounter + " failed: " + lastMove
                                    + " with states:\n" + interpreterState.toString(), state.get(i),
                                    interpreterState.get(i));
                        }
                    } else if (object instanceof String) {
                        String asString = (String) object;
                        if (asString.startsWith("(goal")) {
                            List<List<String>> goals = interpreter.getAllModels("goal");
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

                                assertTrue(fileName + ": Goal Wrong!", goalsFormatted.contains(asString));
                            }

                        } else {
                            lastMove = (String) object;
                            interpreter.interpret(lastMove);
                            moveCounter++;
                        }
                    }
                }
            } catch (Exception e) {
                assertTrue(fileName + ": Unexpected Error: " + e.toString(), false);
            }
        }

    }

}
