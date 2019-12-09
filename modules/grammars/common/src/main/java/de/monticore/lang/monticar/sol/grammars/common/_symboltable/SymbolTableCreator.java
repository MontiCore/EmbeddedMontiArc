/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.common._symboltable;

import de.monticore.lang.monticar.sol.grammars.common._ast.ASTImportStatement;
import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.ImportStatement;
import de.se_rwth.commons.Names;

import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import java.util.stream.Collectors;

/**
 * An interface which can be implemented by concrete SymbolTableCreators based on the Common grammar.
 */
public interface SymbolTableCreator extends de.monticore.symboltable.SymbolTableCreator {
    /**
     * Puts an ArtifactScope onto the scope stack derived from package parts and imports.
     * @param packageList The parts that form the qualified name of a package.
     * @param importList A list of imports to be translated into ImportStatements.
     */
    default void putOnStack(List<String> packageList, List<ASTImportStatement> importList) {
        String packageName = Names.getQualifiedName(packageList);
        Function<ASTImportStatement, ImportStatement> mapping = i -> {
            String qualifiedName = Names.getQualifiedName(i.getImportList());

            return new ImportStatement(qualifiedName, i.isStar());
        };
        List<ImportStatement> imports = importList.stream()
                .map(mapping)
                .collect(Collectors.toList());

        this.putOnStack(new ArtifactScope(Optional.empty(), packageName, imports));
    }
}
