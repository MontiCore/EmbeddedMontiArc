/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.language.cocos;

import de.monticore.lang.monticar.sol.grammars.language._ast.ASTTemplateDeclaration;
import de.monticore.lang.monticar.sol.grammars.language._cocos.LanguageASTTemplateDeclarationCoCo;
import de.monticore.lang.monticar.sol.grammars.language._cocos.LanguageCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.TemplateDeclarationSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.ResolvedSeveralEntriesException;

import java.util.*;
import java.util.stream.Collectors;

/**
 * This context condition checks whether all options in a template declaration have unique identifiers.
 */
public class UniqueOptionCoCo extends CommonLanguageCoCo implements LanguageASTTemplateDeclarationCoCo {
    public UniqueOptionCoCo() {
        super("LANG0002", "There is already an option '%s' in template '%s'.");
    }

    @Override
    public void registerTo(LanguageCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTTemplateDeclaration node) {
        node.getTemplateDeclarationSymbolOpt().ifPresent(this::check);
    }

    protected void check(TemplateDeclarationSymbol declaration) {
        try {
            this.doCheck(declaration);
        } catch (ResolvedSeveralEntriesException exception) {
            Deque<Symbol> symbols = new ArrayDeque<>(exception.getSymbols());
            Symbol symbol = symbols.getLast();

            this.error(declaration, symbol.getName(), declaration.getName());
        }
    }

    protected void doCheck(TemplateDeclarationSymbol declaration) {
        String name = declaration.getName();
        List<String> options = declaration.getOptionSymbols().stream()
                .map(OptionSymbol::getName)
                .collect(Collectors.toList());
        Set<String> uniqueOptions = new HashSet<>(options);

        uniqueOptions.forEach(options::remove);
        options.forEach(option -> this.error(declaration, option, name));
    }
}
