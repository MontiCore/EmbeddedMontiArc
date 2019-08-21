/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.symboltable;

import de.monticore.lang.monticar.sol.grammars.language._ast.ASTLanguageCompilationUnit;

import java.util.Optional;

/**
 * An instance of the symbol table for the active setup.
 */
public interface LanguageSymbolTable {
    /**
     * Returns the node of the root model's compilation unit.
     * @return The node of the root model's compilation unit.
     */
    Optional<ASTLanguageCompilationUnit> getRootNode();
}
