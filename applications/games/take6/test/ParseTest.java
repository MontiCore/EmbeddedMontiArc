import static org.junit.Assert.assertTrue;

import org.junit.Test;

import de.monticore.lang.gdl.Interpreter;
import de.monticore.lang.gdl.InterpreterOptions;

public class ParseTest {
    
    @Test
    public void testParse() throws Exception {
        Interpreter interpreter = Interpreter.fromGDLFile("6nimmt/6nimmt.gdl", new InterpreterOptions());
        interpreter.close();
    }
    
    @Test
    public void testParseTyped() throws Exception {
        Interpreter interpreter = Interpreter.fromGDLFile("6nimmtTyped/6nimmtTyped.gdl", new InterpreterOptions().withTypes(true));
        assertTrue(interpreter.isWithTypes());
        assertTrue(interpreter.getStateSpaceDimension() > 0);
        assertTrue(interpreter.getActionSpaceDimension() > 0);
        interpreter.close();
    }
}

