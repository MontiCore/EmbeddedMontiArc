/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.ld;

import de.monticore.lang.monticar.sol.grammars.language._ast.ASTLanguageCompilationUnit;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.TemplateDeclarationSymbol;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.TemplateUndeclarationSymbol;
import org.json.JSONArray;

import java.util.List;

public interface LDExtractor {
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
    List<TemplateUndeclarationSymbol> getTemplateUndeclarations(ASTLanguageCompilationUnit node);

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
    String getIdentifier(TemplateUndeclarationSymbol symbol);

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

    /**
     * Fetches the elements of a given template declaration.
     * @param symbol The template declaration from which the elements should be extracted.
     * @return The elements stored in a JSONArray of the given template declaration.
     */
    JSONArray getElements(TemplateDeclarationSymbol symbol);
}
