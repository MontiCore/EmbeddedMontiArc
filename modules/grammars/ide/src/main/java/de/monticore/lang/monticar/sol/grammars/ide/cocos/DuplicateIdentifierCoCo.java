/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide.cocos;

import de.monticore.lang.monticar.sol.grammars.ide._ast.ASTConfigurationType;
import de.monticore.lang.monticar.sol.grammars.ide._ast.ASTModuleType;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDEASTConfigurationTypeCoCo;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDEASTModuleTypeCoCo;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDECoCoChecker;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ConfigurationTypeSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ModuleTypeSymbol;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.references.SymbolReference;
import de.monticore.symboltable.resolving.ResolvedSeveralEntriesException;

import java.util.*;
import java.util.stream.Collectors;

public class DuplicateIdentifierCoCo extends CommonIDECoCo implements IDEASTConfigurationTypeCoCo, IDEASTModuleTypeCoCo {
    public DuplicateIdentifierCoCo() {
        super("IDE0001", "There is already %s '%s' in '%s'.");
    }

    @Override
    public void registerTo(IDECoCoChecker checker) {
        checker.addCoCo((IDEASTConfigurationTypeCoCo) this);
        checker.addCoCo((IDEASTModuleTypeCoCo) this);
    }

    @Override
    public void check(ASTConfigurationType node) {
        node.getConfigurationTypeSymbolOpt().ifPresent(this::check);
    }

    protected void check(ConfigurationTypeSymbol symbol) {
        this.check(symbol, symbol.getConfigurations(), "a configuration");
        this.check(symbol, symbol.getTasks(), "a task");
        this.check(symbol, symbol.getOptions(), "an option");
    }

    @Override
    public void check(ASTModuleType node) {
        node.getModuleTypeSymbolOpt().ifPresent(this::check);
    }

    protected void check(ModuleTypeSymbol symbol) {
        this.check(symbol, symbol.getModules(), "a module");
        this.check(symbol, symbol.getConfigurations(), "a configuration");
    }

    protected void check(Symbol owner, List<? extends SymbolReference<?>> references, String type) {
        try {
            this.doCheck(owner, references, type);
        } catch (ResolvedSeveralEntriesException exception) {
            Deque<Symbol> symbols = new ArrayDeque<>(exception.getSymbols());
            Symbol symbol = symbols.getLast();

            this.error(owner, type, symbol.getName(), owner.getName());
        }
    }

    protected void doCheck(Symbol owner, List<? extends SymbolReference<?>> references, String type) {
        String ownerName = owner.getName();
        List<String> names = references.stream()
                .filter(SymbolReference::existsReferencedSymbol)
                .map(SymbolReference::getReferencedSymbol)
                .map(Symbol::getName)
                .collect(Collectors.toList());
        Set<String> uniqueNames = new HashSet<>(names);

        uniqueNames.forEach(names::remove);
        names.forEach(name -> this.error(owner, type, name, ownerName));
    }
}
