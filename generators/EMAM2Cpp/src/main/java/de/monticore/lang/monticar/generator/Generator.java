/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.monticar.generator.cmake.CMakeConfig;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;

import java.io.File;
import java.io.IOException;
import java.util.List;

/**
 * The Generator interface which defines what functionality has to be supported by a Generator independent from the target language
 *
 */
public interface Generator<T extends Symbol> {

    // Cmake
    CMakeConfig getCmakeConfig();

    boolean isGenerateCMake();

    void setGenerateCMake(boolean generateCMake);

    String getGenerationTargetPath();

    void setGenerationTargetPath(String newPath);

    /**
     * This method should generate the source for the EMAComponentInstanceSymbol and
     * add MathStatementsSymbols, accordingly. Does also do this for all of its subcomponents.
     */
    List<FileContent> generateStrings(TaggingResolver taggingResolver, T symbol);

    /**
     * This methods writes the resulting code for the ExpandedComponentInstance and its subcomponents to the corresponding files
     */
    List<File> generateFiles(TaggingResolver taggingResolver, T symbol) throws IOException;

}
