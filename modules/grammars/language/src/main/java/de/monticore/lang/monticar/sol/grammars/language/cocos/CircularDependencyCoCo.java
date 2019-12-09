/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.language.cocos;

import de.monticore.lang.monticar.sol.grammars.language._ast.ASTLanguage;
import de.monticore.lang.monticar.sol.grammars.language._cocos.LanguageASTLanguageCoCo;
import de.monticore.lang.monticar.sol.grammars.language._cocos.LanguageCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageSymbol;

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

public class CircularDependencyCoCo extends CommonLanguageCoCo implements LanguageASTLanguageCoCo {
    public CircularDependencyCoCo() {
        super("LANG0003", "Detected circular dependency of language '%s'.");
    }

    @Override
    public void registerTo(LanguageCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTLanguage node) {
        node.getLanguageSymbolOpt().ifPresent(this::check);
    }

    protected void check(LanguageSymbol language) {
        this.check(language, language.getParentSymbols());
    }

    protected void check(LanguageSymbol language, List<LanguageSymbol> predecessors) {
        String fullName = language.getFullName();
        Optional<LanguageSymbol> filteredLanguage = predecessors.stream()
                .filter(predecessor -> predecessor.getFullName().equals(fullName))
                .findFirst();

        if (filteredLanguage.isPresent()) this.error(language, fullName);
        else this.nextGeneration(language, predecessors);
    }

    protected void nextGeneration(LanguageSymbol language, List<LanguageSymbol> predecessors) {
        List<LanguageSymbol> nextPredecessors = predecessors.stream()
                .flatMap(predecessor -> predecessor.getParentSymbols().stream())
                .collect(Collectors.toList());

        if (!nextPredecessors.isEmpty()) this.check(language, nextPredecessors);
    }
}
