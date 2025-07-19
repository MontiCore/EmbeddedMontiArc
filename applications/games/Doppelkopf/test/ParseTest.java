import static org.junit.Assert.assertTrue;

import org.junit.Test;

import de.monticore.lang.gdl.Interpreter;
import de.monticore.lang.gdl.InterpreterOptions;

public class ParseTest {
    
    @Test
    public void testDoppelkopf() throws Exception {
        Interpreter interpreter = Interpreter.fromGDLFile("Doppelkopf/Doppelkopf.gdl", new InterpreterOptions());
        interpreter.close();
    }
    
    @Test
    public void testDoppelkopfTyped() throws Exception {
        Interpreter interpreter = Interpreter.fromGDLFile("DoppelkopfTyped/DoppelkopfTyped.gdl", new InterpreterOptions().withTypes(true));
        assertTrue(interpreter.isWithTypes());
        assertTrue(interpreter.getStateSpaceDimension() > 0);
        assertTrue(interpreter.getActionSpaceDimension() > 0);
        interpreter.close();
    }
    
    @Test
    public void testDoppelkopfReducedTyped() throws Exception {
        Interpreter interpreter = Interpreter.fromGDLFile("DoppelkopfReducedTyped/DoppelkopfReducedTyped.gdl", new InterpreterOptions().withTypes(true));
        assertTrue(interpreter.isWithTypes());
        assertTrue(interpreter.getStateSpaceDimension() > 0);
        assertTrue(interpreter.getActionSpaceDimension() > 0);
        interpreter.close();
    }

}
