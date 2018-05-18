/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.math.math;

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
