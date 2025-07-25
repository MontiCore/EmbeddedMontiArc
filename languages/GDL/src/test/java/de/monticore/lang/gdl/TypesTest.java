package de.monticore.lang.gdl;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import de.monticore.lang.gdl.types.GDLType;
import de.monticore.lang.gdl.types.GDLValue;

public class TypesTest {
    
    @Test
    public void testDimensions() {
        try (final Interpreter interpreter = Interpreter.fromGDLFile("src/test/resources/gdl/TicTacToeTyped.gdl", new InterpreterOptions().withTypes(true))) {
            assertEquals(29, interpreter.getStateSpaceDimension());
            assertEquals(9, interpreter.getActionSpaceDimension());
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    
    @Test
    public void testCombinedDimensions() {
        try (final Interpreter interpreter = Interpreter.fromGDLFile("src/test/resources/gdl/TypesCombinedTest.gdl", new InterpreterOptions().withTypes(true))) {
            assertEquals(3, interpreter.getStateSpaceDimension());
            assertEquals(0, interpreter.getActionSpaceDimension());
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Test
    public void testIndicatorMatrix() {
        try (final Interpreter interpreter = Interpreter.fromGDLFile("src/test/resources/gdl/TicTacToeTyped.gdl", new InterpreterOptions().withTypes(true))) {
            float[] indicator = interpreter.getStateIndicatorMatrixForRole(GDLValue.createFromLine("x"));

            GDLType action = interpreter.getActionForIndex(8);
            Command command = Command.createFromLine("x " + action.toString());
            boolean legal = interpreter.interpret(command);
            assertTrue("Move was illegal, should have been legal: " + action.toString() + command.toString(), legal);

            float[] nextIndicator = interpreter.getStateIndicatorMatrixForRole(GDLValue.createFromLine("x"));
            assertFalse("Indicator arrays were expected to change", arraysEqual(indicator, nextIndicator));

            GDLType nextAction = interpreter.getActionForIndex(8);
            Command nextCommand = Command.createFromLine("o " + nextAction.toString());
            assertEquals("Actions shifted, were expected to be equal", action, nextAction);

            boolean nextLegal = interpreter.interpret(nextCommand);
            assertFalse("Move was legal, should have been illegal: " + command.toString(), nextLegal);

            float[] nextIndicatorO = interpreter.getStateIndicatorMatrixForRole(GDLValue.createFromLine("o"));

            assertTrue("Indicator matrices should match for both players", arraysEqual(nextIndicator, nextIndicatorO));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private boolean arraysEqual(float[] a1, float[] a2) {
        if (a1.length != a2.length) return false;
        for (int i = 0; i < a1.length; i++) {
            if (a1[i] != a2[i]) return false;
        }
        return true;
    }

}
