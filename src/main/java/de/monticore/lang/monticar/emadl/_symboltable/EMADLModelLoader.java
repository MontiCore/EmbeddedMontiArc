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
package de.monticore.lang.monticar.emadl._symboltable;

import de.monticore.symboltable.*;
import de.se_rwth.commons.logging.Log;


public class EMADLModelLoader extends de.monticore.modelloader.ModelingLanguageModelLoader<de.monticore.lang.monticar.emadl._ast.ASTEMADLCompilationUnit> {

    public EMADLModelLoader(EMADLLanguage language) {
        super(language);
    }

    @Override
    protected void createSymbolTableFromAST(final de.monticore.lang.monticar.emadl._ast.ASTEMADLCompilationUnit ast, final String modelName,
                                            final MutableScope enclosingScope, final ResolvingConfiguration resolvingConfiguration) {
        final EMADLSymbolTableCreator symbolTableCreator =
                getModelingLanguage().getSymbolTableCreator(resolvingConfiguration, enclosingScope).orElse(null);

        if (symbolTableCreator != null) {
            Log.debug("Start creation of symbol table for model \"" + modelName + "\".",
                    EMADLModelLoader.class.getSimpleName());
            final Scope scope = symbolTableCreator.createFromAST(ast);

            if (!(scope instanceof ArtifactScope)) {
                Log.warn("0xA7001_184 Top scope of model " + modelName + " is expected to be an artifact scope, but"
                        + " is scope \"" + scope.getName() + "\"");
            }

            Log.debug("Created symbol table for model \"" + modelName + "\".", EMADLModelLoader.class.getSimpleName());
        }
        else {
            Log.warn("0xA7002_184 No symbol created, because '" + getModelingLanguage().getName()
                    + "' does not define a symbol table creator.");
        }
    }

    @Override
    public EMADLLanguage getModelingLanguage() {
        return (EMADLLanguage) super.getModelingLanguage();
    }

}

