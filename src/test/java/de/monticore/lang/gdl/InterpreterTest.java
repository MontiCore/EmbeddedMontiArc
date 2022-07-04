package de.monticore.lang.gdl;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.junit.Test;

import de.monticore.lang.gdl.types.GDLNumber;
import de.monticore.lang.gdl.types.GDLTuple;
import de.monticore.lang.gdl.types.GDLType;
import de.monticore.lang.gdl.types.GDLValue;

public class InterpreterTest {

    @Test
    public void testInit() {
        InterpreterTestCase testCase = new InterpreterTestCase("Init");
        testCase.expectedState.addAll(List.of(
            new GDLTuple("test"),
            new GDLTuple("test", "1")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testNext() {
        InterpreterTestCase testCase = new InterpreterTestCase("Next");
        testCase.moves.addAll(List.of(
            Command.createFromLine("test (do)"),
            Command.createFromLine("test (do)"),
            Command.createFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            new GDLTuple("state","0"),
            new GDLTuple("state","1"),
            new GDLTuple("state","2"),
            new GDLTuple("state","3")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testRole() {
        InterpreterTestCase testCase = new InterpreterTestCase("Role");
        testCase.moves.addAll(List.of(
            Command.createFromLine("test (copy_role none)"),
            Command.createFromLine("test (copy_role player2)"),
            Command.createFromLine("test (copy_role random)")
        ));
        testCase.expectedState.addAll(List.of(
            new GDLTuple("role_name", "player1"),
            new GDLTuple("role_name", "player2"),
            new GDLTuple("role_name", "random")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testTrue() {
        InterpreterTestCase testCase = new InterpreterTestCase("True");
        testCase.moves.addAll(List.of(
            Command.createFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            new GDLTuple("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testRandom() {
        InterpreterTestCase testCase = new InterpreterTestCase("Random");
        testCase.expectedState.addAll(List.of(
            new GDLTuple("dice", "1")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testTerminal() {
        InterpreterTestCase testCase = new InterpreterTestCase("Terminal");
        testCase.expectedState.addAll(List.of(
            new GDLTuple("test")
        ));
        assertTrue("Expected test to be terminal, but test is not terminal.", testCase.doTestCase().isTerminal());
    }

    @Test
    public void testCount() {
        InterpreterTestCase testCase = new InterpreterTestCase("Count");
        testCase.moves.addAll(List.of(
            Command.createFromLine("test (do1)"),
            Command.createFromLine("test (do2)")
        ));
        testCase.expectedState.addAll(List.of(
            new GDLTuple("success", "0"),
            new GDLTuple("success", "1"),
            new GDLTuple("success", "2")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testAdd() {
        InterpreterTestCase testCase = new InterpreterTestCase("Add");
        testCase.moves.addAll(List.of(
            Command.createFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            new GDLTuple("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testSub() {
        InterpreterTestCase testCase = new InterpreterTestCase("Sub");
        testCase.moves.addAll(List.of(
            Command.createFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            new GDLTuple("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testMult() {
        InterpreterTestCase testCase = new InterpreterTestCase("Mult");
        testCase.moves.addAll(List.of(
            Command.createFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            new GDLTuple("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testDiv() {
        InterpreterTestCase testCase = new InterpreterTestCase("Div");
        testCase.moves.addAll(List.of(
            Command.createFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            new GDLTuple("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testSucc() {
        InterpreterTestCase testCase = new InterpreterTestCase("Succ");
        testCase.moves.addAll(List.of(
            Command.createFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            new GDLTuple("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testLess() {
        InterpreterTestCase testCase = new InterpreterTestCase("Less");
        testCase.moves.addAll(List.of(
            Command.createFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            new GDLTuple("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testGreater() {
        InterpreterTestCase testCase = new InterpreterTestCase("Greater");
        testCase.moves.addAll(List.of(
            Command.createFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            new GDLTuple("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testEqual() {
        InterpreterTestCase testCase = new InterpreterTestCase("Equal");
        testCase.moves.addAll(List.of(
            Command.createFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            new GDLTuple("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testNumber() {
        InterpreterTestCase testCase = new InterpreterTestCase("Number");
        testCase.moves.addAll(List.of(
            Command.createFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            new GDLTuple("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testMod() {
        InterpreterTestCase testCase = new InterpreterTestCase("Mod");
        testCase.moves.addAll(List.of(
            Command.createFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            new GDLTuple("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testDistinct() {
        InterpreterTestCase testCase = new InterpreterTestCase("Distinct");
        testCase.moves.addAll(List.of(
            Command.createFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            new GDLTuple("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testDoes() {
        InterpreterTestCase testCase = new InterpreterTestCase("Does");
        testCase.moves.addAll(List.of(
            Command.createFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            new GDLTuple("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testInference() {
        InterpreterTestCase testCase = new InterpreterTestCase("Inference");
        testCase.moves.addAll(List.of(
            Command.createFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            new GDLTuple("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testLegal() {
        InterpreterTestCase testCase = new InterpreterTestCase("Legal");
        testCase.moves.addAll(List.of(
            Command.createFromLine("test (do)"),
            Command.createFromLine("test (do_not)")
        ));
        testCase.expectedState.addAll(List.of(
            new GDLTuple("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testNot() {
        InterpreterTestCase testCase = new InterpreterTestCase("Not");
        testCase.moves.addAll(List.of(
            Command.createFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            new GDLTuple("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testSees() {
        InterpreterTestCase testCase = new InterpreterTestCase("Sees");
        testCase.moves.addAll(List.of(
            Command.createFromLine("player0 (see player1 hidden)")
        ));
        testCase.expectedHiddenState.putAll(Map.of(
            new GDLValue("player1"), Set.of(new GDLTuple("hidden", "1")))
        );
        testCase.doTestCase();
    }

    @Test
    public void testNegative() {
        InterpreterTestCase testCase = new InterpreterTestCase("Negative");
        testCase.moves.addAll(List.of(
            Command.createFromLine("test (do tmult)"),
            Command.createFromLine("test (do tadd)"),
            Command.createFromLine("test (do tsub)"),
            Command.createFromLine("test (do tdiv)"),
            Command.createFromLine("test (do tmod)"),
            Command.createFromLine("test (do tless)"),
            Command.createFromLine("test (do tgreater)"),
            Command.createFromLine("test (do tequal)"),
            Command.createFromLine("test (do tnumber)"),
            Command.createFromLine("test (do tsucc)"),
            Command.createFromLine("test (do (-1 -1 -2))")
        ));
        testCase.expectedState.addAll(List.of(
            new GDLTuple("success", "tmult"),
            new GDLTuple("success", "tadd"),
            new GDLTuple("success", "tsub"),
            new GDLTuple("success", "tdiv"),
            new GDLTuple("success", "tmod"),
            new GDLTuple("success", "tless"),
            new GDLTuple("success", "tgreater"),
            new GDLTuple("success", "tequal"),
            new GDLTuple("success", "tnumber"),
            new GDLTuple("success", "tsucc"),
            GDLType.createFromLine("(success (-1 -1 -2))")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testReset() {
        InterpreterTestCase testCase = new InterpreterTestCase("Reset");
        testCase.moves.addAll(List.of(
            Command.createFromLine("test (do)")
        ));
        testCase.expectedState.addAll(Set.of(
            new GDLTuple("state", "a")
        ));
        Interpreter interpreter = testCase.doTestCase();
        interpreter.reset();

        assertTrue("Expected 1 legal move after reset.", interpreter.getAllLegalMoves().size() == 1);
    }

    @Test
    public void testGoal() {
        InterpreterTestCase testCase = new InterpreterTestCase("Goal");
        testCase.expectedState.addAll(List.of(
            new GDLTuple("test")
        ));
        Interpreter interpreter = testCase.doTestCase();

        Map<GDLType, GDLType> goals = interpreter.getGoals();
        Map<GDLType, GDLType> expected = Map.of(
            new GDLValue("player1"), new GDLNumber(50),
            new GDLValue("player2"), new GDLNumber(50),
            new GDLValue("player3"), new GDLNumber(100)
        );

        assertEquals(String.format("Expected goals %s do not match actual goals %s", expected, goals), expected, goals);
    }

    @Test
    public void testNoop() {
        InterpreterTestCase testCase = new InterpreterTestCase("Noop");
        testCase.moves.addAll(List.of(
            Command.createFromLine("test (noop)"),
            Command.createFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            new GDLTuple("success")
        ));
        Interpreter interpreter = testCase.doTestCase();

        boolean moveLegal = interpreter.interpret(Command.createFromLine("test (noop)"));

        assertFalse("Noop move was legal when it should not have been", moveLegal);
    }


    private final class InterpreterTestCase {
        final String modelPath;
        final List<Command> moves;
        final Set<GDLType> expectedState;
        final Map<GDLType, Set<GDLType>> expectedHiddenState;

        public InterpreterTestCase(String modelName) {
            this.modelPath = "src/test/resources/gdl/interpreter/" + modelName + ".gdl";
            this.moves = new ArrayList<>();
            this.expectedState = new HashSet<>();
            this.expectedHiddenState = new HashMap<>();
        }

        public Interpreter doTestCase() {
            final Interpreter interpreter = Interpreter.fromGDLFile(modelPath, null);
    
            for (Command c: this.moves) {
                assertNotNull("Test case moves are not well defined for model " + this.modelPath, c);
                interpreter.interpret(c);
            }
    
            Set<GDLType> state = interpreter.getVisibleGameState();
            Map<GDLType, Set<GDLType>> hiddenState = interpreter.getHiddenGameState();
    
            assertEquals(String.format("State sizes do not match: %s %s", state.toString(), this.expectedState.toString()), state.size(), this.expectedState.size());
    
            for (GDLType s : state) {
                assertTrue(String.format("State %s was found, but not expected", s), expectedState.contains(s));
            }
            
            for (GDLType s : expectedState) {
                assertTrue(String.format("State %s was expected, but not found", s), state.contains(s));
            }

            for (GDLType role : hiddenState.keySet()) {
                Set<GDLType> hiddenStateValues = hiddenState.get(role);
                Set<GDLType> expectedHiddenStateValues = expectedHiddenState.get(role);
                assertNotNull(String.format("Role %s has hidden state, but not expected", role), expectedHiddenStateValues);
                if (expectedHiddenStateValues != null) {
                    for (GDLType s : hiddenStateValues) {
                        assertTrue(String.format("Hidden state %s was found for role %s, but not expected", s, role), expectedHiddenStateValues.contains(s));
                    }
                }
            }

            for (GDLType role : expectedHiddenState.keySet()) {
                Set<GDLType> expectedHiddenStateValues = expectedHiddenState.get(role);
                Set<GDLType> hiddenStateValues = hiddenState.get(role);
                assertNotNull(String.format("Role %s has expected hidden state, but not found", role), hiddenStateValues);
                if (hiddenStateValues != null) {
                    for (GDLType s : expectedHiddenStateValues) {
                        assertTrue(String.format("Hidden state %s was expected for role %s, but not found", s, role), hiddenStateValues.contains(s));
                    }
                }
            }
    
            return interpreter;
        }

    }

}
