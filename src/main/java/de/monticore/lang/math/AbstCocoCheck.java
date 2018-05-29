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
package de.monticore.lang.math;

/**
 * Created by Tobias PC on 12.01.2017.
 */

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.math._cocos.MathCoCoChecker;
import de.monticore.lang.math._parser.MathParser;
import de.monticore.lang.math._symboltable.MathSymbolTableCreator;
import de.monticore.lang.math._ast.ASTMathCompilationUnit;
import de.monticore.lang.math._symboltable.MathLanguage;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.ResolvingConfiguration;
import org.antlr.v4.runtime.RecognitionException;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Optional;

public abstract class AbstCocoCheck {
    private final MathLanguage mathlang = new MathLanguage();
    MathParser parser = new MathParser();
    private GlobalScope globalScope;


    public AbstCocoCheck() {
    }

    abstract protected MathCoCoChecker getChecker();

    protected ASTMathCompilationUnit loadModel(String modelFullQualifiedFilename) {
        Path model = Paths.get(modelFullQualifiedFilename);

        try {
            Optional<ASTMathCompilationUnit> root = parser.parse(model.toString());
            if (root.isPresent()) {
                // create Symboltable
                ModelingLanguageFamily fam = new ModelingLanguageFamily();
                fam.addModelingLanguage(new MathLanguage());
                final ModelPath mp = new ModelPath(model.toAbsolutePath());
                this.globalScope = new GlobalScope(mp, fam);

                ResolvingConfiguration ResolvingConfiguration = new ResolvingConfiguration();
                ResolvingConfiguration.addDefaultFilters (mathlang.getResolvingFilters());
                Optional<MathSymbolTableCreator> stc = mathlang.getSymbolTableCreator(ResolvingConfiguration, globalScope);
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
