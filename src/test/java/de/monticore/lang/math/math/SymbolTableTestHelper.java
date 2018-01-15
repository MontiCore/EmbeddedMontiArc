package de.monticore.lang.math.math;

import de.monticore.lang.math._ast.ASTMathCompilationUnit;
import de.monticore.lang.math._symboltable.MathLanguage2;
import de.monticore.lang.math._symboltable.MathSymbolTableCreator;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class SymbolTableTestHelper {

    public static void createSymbolTableOnInput(String content) throws Exception {
        MathLanguage2 mathLanguage = new MathLanguage2();
        ASTMathCompilationUnit ast = mathLanguage.getParser().parse_String(content).orElse(null);
        assertNotNull(ast);

        MathSymbolTableCreator stc = mathLanguage.getSymbolTableCreator();
        ast.accept(stc);
    }

    public static void createSymbolTableOnInputAndExpectErrorCode(String content, String expectedErrorCode) throws Exception {
        boolean foundError = false;
        Log.enableFailQuick(false);
        createSymbolTableOnInput(content);
        for (Finding f: Log.getFindings()) {
            if (f.getMsg().contains(expectedErrorCode)) {
                foundError = true;
                break;
            }
        }
        assertTrue("The test case did not produce the expected error code '" + expectedErrorCode + "'", foundError);
    }
}
