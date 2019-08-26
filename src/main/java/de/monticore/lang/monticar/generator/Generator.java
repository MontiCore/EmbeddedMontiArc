/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
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

    String generateString(TaggingResolver taggingResolver, EMAComponentInstanceSymbol componentSymbol, MathStatementsSymbol mathStatementsSymbol);

    /**
     * This method should generate the source for the EMAComponentInstanceSymbol and
     * add MathStatementsSymbols, accordingly. Does also do this for all of its subcomponents.
     */
    List<FileContent> generateStrings(TaggingResolver taggingResolver, EMAComponentInstanceSymbol componentInstanceSymbol, Scope symtab);

    /**
     * This methods writes the resulting code for the ExpandedComponentInstance and its subcomponents to the corresponding files
     */
    List<File> generateFiles(TaggingResolver taggingResolver, EMAComponentInstanceSymbol componentSymbol, Scope symtab) throws IOException;

    boolean useAlgebraicOptimizations();

    void setUseAlgebraicOptimizations(boolean useAlgebraicOptimizations);

    boolean useThreadingOptimizations();

    void setUseThreadingOptimization(boolean useThreadingOptimizations);

    MathCommandRegister getMathCommandRegister();

    void setMathCommandRegister(MathCommandRegister mathCommandRegister);
}
