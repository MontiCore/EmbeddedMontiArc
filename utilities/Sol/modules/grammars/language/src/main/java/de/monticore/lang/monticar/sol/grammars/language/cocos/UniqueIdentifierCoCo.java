/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.language.cocos;

import de.monticore.lang.monticar.sol.grammars.language._ast.ASTLanguage;
import de.monticore.lang.monticar.sol.grammars.language._cocos.LanguageASTLanguageCoCo;
import de.monticore.lang.monticar.sol.grammars.language._cocos.LanguageCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageSymbol;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.TemplateDeclarationSymbol;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.ResolvedSeveralEntriesException;

import java.util.*;
import java.util.stream.Collectors;

/**
 *  A context condition which checks whether an identifier for a declaration is unique.
 */
public class UniqueIdentifierCoCo extends CommonLanguageCoCo implements LanguageASTLanguageCoCo {
    public UniqueIdentifierCoCo() {
        super("LANG0001", "A template with name '%s' has already been declared. in '%s'.");
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
        try {
            this.doCheck(language);
        } catch (ResolvedSeveralEntriesException exception) {
            Deque<Symbol> symbols = new ArrayDeque<>(exception.getSymbols());
            Symbol symbol = symbols.getLast();

            this.error(language, symbol.getName(), language.getName());
        }
    }

    protected void doCheck(LanguageSymbol language) {
        String name = language.getName();
        List<String> declarations = language.getAllDeclarationSymbols().stream()
                .map(TemplateDeclarationSymbol::getName)
                .collect(Collectors.toList());
        Set<String> uniqueDeclarations = new HashSet<>(declarations);

        uniqueDeclarations.forEach(declarations::remove);
        declarations.forEach(declaration -> this.error(language, declaration, name));
    }
}
