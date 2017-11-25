/*
 * ******************************************************************************
 * MontiCore Language Workbench, www.monticore.de
 * Copyright (c) 2017, MontiCore, All rights reserved.
 *
 * This project is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * ******************************************************************************
 */

package de.monticore.lang.monticar.generator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.math.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Scope;

import java.io.File;
import java.io.IOException;
import java.util.List;

/**
 * The Generator interface which defines what functionality has to be supported by a Generator independent from the target language
 *
 * @author Sascha Schneiders
 */
public interface Generator {

    String getGenerationTargetPath();

    void setGenerationTargetPath(String newPath);

    String generateString(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol componentSymbol, MathStatementsSymbol mathStatementsSymbol);

    /**
     * This method should generate the source for the ExpandedComponentInstanceSymbol and
     * add MathStatementsSymbols, accordingly. Does also do this for all of its subcomponents.
     */
    List<FileContent> generateStrings(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol componentInstanceSymbol, Scope symtab);

    /**
     * This methods writes the resulting code for the ExpandedComponentInstance and its subcomponents to the corresponding files
     */
    List<File> generateFiles(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol componentSymbol, Scope symtab) throws IOException;

    boolean useAlgebraicOptimizations();

    void setUseAlgebraicOptimizations(boolean useAlgebraicOptimizations);

    boolean useThreadingOptimizations();

    void setUseThreadingOptimization(boolean useThreadingOptimizations);

    MathCommandRegister getMathCommandRegister();

    void setMathCommandRegister(MathCommandRegister mathCommandRegister);
}
