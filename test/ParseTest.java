import static org.junit.Assert.assertTrue;

import org.junit.Test;

import de.monticore.lang.gdl.Interpreter;
import de.monticore.lang.gdl.InterpreterOptions;

public class ParseTest {
    
    @Test
    public void testChess() throws Exception {
        Interpreter interpreter = Interpreter.fromGDLFile("Chess/Chess.gdl", new InterpreterOptions());
        interpreter.close();
    }
    
    @Test
    public void testChessTyped() throws Exception {
        Interpreter interpreter = Interpreter.fromGDLFile("Chess/ChessTyped.gdl", new InterpreterOptions().withTypes(true));
        assertTrue(interpreter.isWithTypes());
        assertTrue(interpreter.getStateSpaceDimension() > 0);
        assertTrue(interpreter.getActionSpaceDimension() > 0);
        interpreter.close();
    }

}
