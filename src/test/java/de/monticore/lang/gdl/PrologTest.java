package de.monticore.lang.gdl;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import de.monticore.lang.gdl._ast.ASTGame;

public class PrologTest {
    
    @Test
    public void testInit() {
        ASTGame game = GDLInterpreter.parse("src/main/resources/example/TicTacToe.gdl");
        try (Interpreter interpreter = new Interpreter(game)) {
            interpreter.init();
            assertTrue(true);
        } catch (Exception e) {
            e.printStackTrace();
            assertTrue(false);
        }
    }

}
