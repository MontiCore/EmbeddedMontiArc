package de.monticore.lang.gdl;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import org.junit.Test;

public class InterpreterTest {

    @Test
    public void testInit() {
        InterpreterTestCase testCase = new InterpreterTestCase("Init");
        testCase.expectedState.addAll(List.of(
            List.of("test"),
            List.of("test", "1")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testNext() {
        InterpreterTestCase testCase = new InterpreterTestCase("Next");
        testCase.moves.addAll(List.of(
            Command.createMoveFromLine("test (do)"),
            Command.createMoveFromLine("test (do)"),
            Command.createMoveFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            List.of("state","0"),
            List.of("state","1"),
            List.of("state","2"),
            List.of("state","3")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testRole() {
        InterpreterTestCase testCase = new InterpreterTestCase("Role");
        testCase.moves.addAll(List.of(
            Command.createMoveFromLine("test (copy_role none)"),
            Command.createMoveFromLine("test (copy_role player2)"),
            Command.createMoveFromLine("test (copy_role random)")
        ));
        testCase.expectedState.addAll(List.of(
            List.of("role_name", "player1"),
            List.of("role_name", "player2"),
            List.of("role_name", "random")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testTrue() {
        InterpreterTestCase testCase = new InterpreterTestCase("True");
        testCase.moves.addAll(List.of(
            Command.createMoveFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            List.of("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testRandom() {
        InterpreterTestCase testCase = new InterpreterTestCase("Random");
        testCase.expectedState.addAll(List.of(
            List.of("dice", "1")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testTerminal() {
        InterpreterTestCase testCase = new InterpreterTestCase("Terminal");
        testCase.expectedState.addAll(List.of(
            List.of("test")
        ));
        assertTrue("Expected test to be terminal, but test is not terminal.", testCase.doTestCase().isTerminal());
    }

    @Test
    public void testCount() {
        InterpreterTestCase testCase = new InterpreterTestCase("Count");
        testCase.moves.addAll(List.of(
            Command.createMoveFromLine("test (do1)"),
            Command.createMoveFromLine("test (do2)")
        ));
        testCase.expectedState.addAll(List.of(
            List.of("success", "0"),
            List.of("success", "1"),
            List.of("success", "2")
        ));
        testCase.doTestCase();
    }

    /*@Test
    public void testAdd() {
        InterpreterTestCase testCase = new InterpreterTestCase("Add");
        testCase.moves.addAll(List.of(
            Command.createMoveFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            List.of("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testSubtract() {
        InterpreterTestCase testCase = new InterpreterTestCase("Subtract");
        testCase.moves.addAll(List.of(
            Command.createMoveFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            List.of("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testMult() {
        InterpreterTestCase testCase = new InterpreterTestCase("Mult");
        testCase.moves.addAll(List.of(
            Command.createMoveFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            List.of("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testDiv() {
        InterpreterTestCase testCase = new InterpreterTestCase("Div");
        testCase.moves.addAll(List.of(
            Command.createMoveFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            List.of("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testSuccessor() {
        InterpreterTestCase testCase = new InterpreterTestCase("Successor");
        testCase.moves.addAll(List.of(
            Command.createMoveFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            List.of("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testLess() {
        InterpreterTestCase testCase = new InterpreterTestCase("Less");
        testCase.moves.addAll(List.of(
            Command.createMoveFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            List.of("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testGreater() {
        InterpreterTestCase testCase = new InterpreterTestCase("Greater");
        testCase.moves.addAll(List.of(
            Command.createMoveFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            List.of("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testEqual() {
        InterpreterTestCase testCase = new InterpreterTestCase("Equal");
        testCase.moves.addAll(List.of(
            Command.createMoveFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            List.of("success")
        ));
        testCase.doTestCase();
    }*/

    @Test
    public void testDistinct() {
        InterpreterTestCase testCase = new InterpreterTestCase("Distinct");
        testCase.moves.addAll(List.of(
            Command.createMoveFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            List.of("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testDoes() {
        InterpreterTestCase testCase = new InterpreterTestCase("Does");
        testCase.moves.addAll(List.of(
            Command.createMoveFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            List.of("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testInference() {
        InterpreterTestCase testCase = new InterpreterTestCase("Inference");
        testCase.moves.addAll(List.of(
            Command.createMoveFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            List.of("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testLegal() {
        InterpreterTestCase testCase = new InterpreterTestCase("Legal");
        testCase.moves.addAll(List.of(
            Command.createMoveFromLine("test (do)"),
            Command.createMoveFromLine("test (do_not)")
        ));
        testCase.expectedState.addAll(List.of(
            List.of("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testNot() {
        InterpreterTestCase testCase = new InterpreterTestCase("Not");
        testCase.moves.addAll(List.of(
            Command.createMoveFromLine("test (do)")
        ));
        testCase.expectedState.addAll(List.of(
            List.of("success")
        ));
        testCase.doTestCase();
    }

    @Test
    public void testSees() {
        InterpreterTestCase testCase = new InterpreterTestCase("Sees");
        testCase.moves.addAll(List.of(
            Command.createMoveFromLine("player0 (see player1 hidden)")
        ));
        testCase.expectedHiddenState.addAll(List.of(
            List.of("hidden", "player1", "1")
        ));
        testCase.doTestCase();
    }


    private final class InterpreterTestCase {
        final String modelPath;
        final List<Command> moves;
        final List<List<String>> expectedState;
        final List<List<String>> expectedHiddenState;

        public InterpreterTestCase(String modelName) {
            this.modelPath = "src/test/resources/gdl/interpreter/" + modelName + ".gdl";
            this.moves = new ArrayList<>();
            this.expectedState = new ArrayList<>();
            this.expectedHiddenState = new ArrayList<>();
        }

        public Interpreter doTestCase() {
            final Interpreter interpreter = new Interpreter(GDLInterpreter.parse(this.modelPath));
            
            try {
                interpreter.init();
            } catch (Exception e) {
                assertTrue("Unable to initialize Interpreter.", false);
            }
    
            for (Command c: this.moves) {
                assertNotNull("Test case moves are not well defined for model " + this.modelPath, c);
                interpreter.interpret(c);
            }
    
            List<List<String>> state = interpreter.getGameState();
            List<List<String>> hiddenState = interpreter.getHiddenGameState();
    
            assertEquals(String.format("State sizes do not match: %s %s", state.toString(), this.expectedState.toString()), state.size(), this.expectedState.size());
    
            Set<String> setExpected = this.expectedState
                .stream().map(list -> list.stream().reduce("", (s1, s2) -> s1 + "," + s2))
                .collect(Collectors.toSet());

            Set<String> setHiddenExpected = this.expectedHiddenState
                .stream().map(list -> list.stream().reduce("", (s1, s2) -> s1 + "," + s2))
                .collect(Collectors.toSet());
    
            Set<String> set = state
                .stream().map(list -> list.stream().reduce("", (s1, s2) -> s1 + "," + s2))
                .collect(Collectors.toSet());
                
            Set<String> hiddenSet = hiddenState
                .stream().map(list -> list.stream().reduce("", (s1, s2) -> s1 + "," + s2))
                .collect(Collectors.toSet());
    
            for (String s : set) {
                assertTrue(String.format("State %s was found, but not expected", s), setExpected.contains(s));
            }
            
            for (String s : setExpected) {
                assertTrue(String.format("State %s was expected, but not found", s), set.contains(s));
            }

            for (String s : hiddenSet) {
                assertTrue(String.format("Hidden state %s was found, but not expected", s), setHiddenExpected.contains(s));
            }
            
            for (String s : setHiddenExpected) {
                assertTrue(String.format("Hidden state %s was expected, but not found", s), hiddenSet.contains(s));
            }
    
            return interpreter;
        }

    }

}
