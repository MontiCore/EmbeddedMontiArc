/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.lc;

import de.monticore.lang.monticar.sol.grammars.language._ast.ASTLanguageCompilationUnit;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.TemplateDeclarationSymbol;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.TemplateExclusionSymbol;

import java.util.List;

public interface LCExtractor {
    /**
     * Fetches all template declarations of a given Language AST.
     * @param node The AST of the model to be extracted.
     * @return A list of symbols for the templates in the model.
     */
    List<TemplateDeclarationSymbol> getTemplateDeclarations(ASTLanguageCompilationUnit node);

    /**
     * Fetches all template undeclarations of a given Language AST.
     * @param node The AST of the model to be extracted.
     * @return A list of symbols for the template undeclarations in the model.
     */
    List<TemplateExclusionSymbol> getTemplateUndeclarations(ASTLanguageCompilationUnit node);

    /**
     * Fetches the identifier of a given template declaration.
     * @param symbol The template declaration from which the identifier should be extracted.
     * @return The identifier of the given template declaration.
     */
    String getIdentifier(TemplateDeclarationSymbol symbol);

    /**
     * Fetches the identifier of a given template declaration.
     * @param symbol The template declaration from which the identifier should be extracted.
     * @return The identifier of the given template declaration.
     */
    String getIdentifier(TemplateExclusionSymbol symbol);

    /**
     * Fetches the path of a given template declaration.
     * @param symbol The template declaration from which the path should be extracted.
     * @return The path of the given template declaration as String.
     */
    String getPath(TemplateDeclarationSymbol symbol);

    /**
     * Fetches the label of a given template declaration.
     * @param symbol The template declaration from which the label should be extracted.
     * @return The label of the given template declaration.
     */
    String getLabel(TemplateDeclarationSymbol symbol);
}
