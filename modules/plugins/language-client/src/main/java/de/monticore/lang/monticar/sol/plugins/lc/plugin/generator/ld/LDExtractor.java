/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.ld;

import de.monticore.lang.monticar.sol.grammars.language._ast.ASTLanguageCompilationUnit;
import de.monticore.lang.monticar.sol.grammars.language._ast.ASTTemplateDeclaration;
import org.json.JSONArray;

import java.util.List;

public interface LDExtractor {
    /**
     * Fetches all template declarations of a given Language AST.
     * @param node The AST of the model to be extracted.
     * @return A list of AST nodes for the templates in the model.
     */
    List<ASTTemplateDeclaration> getTemplates(ASTLanguageCompilationUnit node);

    /**
     * Fetches the identifier of a given template declaration.
     * @param node The template declaration from which the identifier should be extracted.
     * @return The identifier of the given template declaration.
     */
    String getIdentifier(ASTTemplateDeclaration node);

    /**
     * Fetches the path of a given template declaration.
     * @param node The template declaration from which the path should be extracted.
     * @return The path of the given template declaration as String.
     */
    String getPath(ASTTemplateDeclaration node);

    /**
     * Fetches the label of a given template declaration.
     * @param node The template declaration from which the label should be extracted.
     * @return The label of the given template declaration.
     */
    String getLabel(ASTTemplateDeclaration node);

    /**
     * Fetches the elements of a given template declaration.
     * @param node The template declaration from which the elements should be extracted.
     * @return The elements stored in a JSONArray of the given template declaration.
     */
    JSONArray getElements(ASTTemplateDeclaration node);
}
