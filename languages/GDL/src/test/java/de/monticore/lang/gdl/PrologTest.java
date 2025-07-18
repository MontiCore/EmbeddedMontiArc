package de.monticore.lang.gdl;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class PrologTest {
    
    @Test
    public void testInit() {
        try (Interpreter interpreter = Interpreter.fromGDLFile("src/main/resources/example/TicTacToe.gdl", null)) {
            assertTrue(true);
        } catch (Exception e) {
            e.printStackTrace();
            assertTrue(false);
        }
    }

}
