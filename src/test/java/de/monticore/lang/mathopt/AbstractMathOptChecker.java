/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2018, Software Engineering Group at RWTH Aachen,
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
package de.monticore.lang.mathopt;

import de.monticore.ModelingLanguageFamily;
import de.monticore.cocos.helper.Assert;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.mathopt._ast.ASTMathOptCompilationUnit;
import de.monticore.lang.mathopt._cocos.MathOptCoCoChecker;
import de.monticore.lang.mathopt._parser.MathOptParser;
import de.monticore.lang.mathopt._symboltable.MathOptLanguage;
import de.monticore.lang.mathopt._symboltable.MathOptSymbolTableCreator;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;
import org.antlr.v4.runtime.RecognitionException;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Collection;
import java.util.Optional;

import static org.junit.Assert.assertEquals;

/**
 * Abstract checker class to check MontiMathOpt models
 * @author Christoph Richter
 */
public abstract class AbstractMathOptChecker {

    // override fields
    protected GlobalScope globalScope;
    protected MathOptParser parser = new MathOptParser();

    abstract protected MathOptCoCoChecker getChecker();

    protected void testModelForErrors(String model, Collection<Finding> expectedErrors) {
        MathOptCoCoChecker checker = getChecker();
        ASTMathOptCompilationUnit root = loadModel(model);
        checker.checkAll(root);
        Assert.assertEqualErrorCounts(expectedErrors, Log.getFindings());
        Assert.assertErrorMsg(expectedErrors, Log.getFindings());
    }

    protected void testModelNoErrors(String model) {
        MathOptCoCoChecker checker = getChecker();
        ASTMathOptCompilationUnit root = loadModel(model);
        checker.checkAll(root);
        assertEquals(0, Log.getFindings().stream().filter(f -> f.isError()).count());
    }

    protected ASTMathOptCompilationUnit loadModel(String modelFullQualifiedFilename) {
        Path model = Paths.get(modelFullQualifiedFilename);

        try {
            Optional<ASTMathOptCompilationUnit> root = parser.parse(model.toString());
            if (root.isPresent()) {
                // create Symboltable
                ModelingLanguageFamily fam = new ModelingLanguageFamily();
                MathOptLanguage lang = new MathOptLanguage();
                fam.addModelingLanguage(lang);
                final ModelPath mp = new ModelPath(model.toAbsolutePath());
                this.globalScope = new GlobalScope(mp, fam);

                ResolvingConfiguration ResolvingConfiguration = new ResolvingConfiguration();
                ResolvingConfiguration.addDefaultFilters(lang.getResolvingFilters());
                Optional<MathOptSymbolTableCreator> stc = lang.getSymbolTableCreator(ResolvingConfiguration, globalScope);
                if (stc.isPresent()) {
                    stc.get().createFromAST(root.get());
                }
                return root.get();
            }
        } catch (RecognitionException | IOException e) {
            e.printStackTrace();
        }
        throw new RuntimeException("Error during loading of model " + modelFullQualifiedFilename + ".");
    }
}
