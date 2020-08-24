/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math;

import de.monticore.io.paths.ModelPath;
import de.monticore.lang.math._ast.ASTMathCompilationUnit;
import de.monticore.lang.math._symboltable.MathLanguage;
import de.monticore.lang.math._symboltable.MathSymbolTableCreator;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class SymbolTableTestHelper {

    public static void createSymbolTableOnInput(String content) throws Exception {
        MathLanguage mathLanguage = new MathLanguage();
        ASTMathCompilationUnit ast = mathLanguage.getParser().parse_String(content).orElse(null);
        assertNotNull(ast);

        ResolvingConfiguration resolvingConfig = new ResolvingConfiguration();
        resolvingConfig.addDefaultFilters(mathLanguage.getResolvingFilters());

        MutableScope scope = (MutableScope) ast.getSpannedScopeOpt().orElse(null);
        MathSymbolTableCreator stc = mathLanguage.getSymbolTableCreator(resolvingConfig, scope).orElse(null);
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
