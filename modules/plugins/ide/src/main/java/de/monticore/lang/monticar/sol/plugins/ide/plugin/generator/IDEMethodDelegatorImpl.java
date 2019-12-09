/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ide.plugin.generator;

import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.*;

import java.util.List;
import java.util.Optional;
import java.util.Set;

@Singleton
public class IDEMethodDelegatorImpl implements IDEMethodDelegator {
    @Override
    public List<ConfigurationSymbol> getConfigurationSymbols(SymbolWithConfigurations symbol) {
        return symbol.getConfigurationSymbols();
    }

    @Override
    public List<OptionFillSymbol> getOptionFillSymbols(SymbolWithFilledOptions symbol) {
        return symbol.getOptionFillSymbols();
    }

    @Override
    public List<OptionInheritSymbol> getOptionInheritSymbols(SymbolWithInheritedOptions symbol) {
        return symbol.getOptionInheritSymbols();
    }

    @Override
    public String getLabel(SymbolWithTypeAttribute symbol) {
        return symbol.getLabel();
    }

    @Override
    public Optional<String> getIcon(SymbolWithTypeAttribute symbol) {
        return symbol.getIcon();
    }

    @Override
    public Optional<String> getCategory(SymbolWithTypeAttribute symbol) {
        return symbol.getCategory();
    }

    @Override
    public List<ConfigurationTypeSymbol> getConfigurationTypeSymbols(SymbolWithConfigurations symbol) {
        return symbol.getConfigurationTypeSymbols();
    }

    @Override
    public List<ConfigurationTypeSymbol> getAllConfigurationTypeSymbols(SymbolWithConfigurations symbol) {
        return symbol.getAllConfigurationTypeSymbols();
    }

    @Override
    public List<ConfigurationSymbol> getZeroConfigurationSymbols(SymbolWithConfigurations symbol) {
        return symbol.getZeroConfigurationSymbols();
    }

    @Override
    public List<ConfigurationSymbol> getPositiveConfigurationSymbols(SymbolWithConfigurations symbol) {
        return symbol.getPositiveConfigurationSymbols();
    }

    @Override
    public List<ConfigurationSymbol> getNegativeConfigurationSymbols(SymbolWithConfigurations symbol) {
        return symbol.getNegativeConfigurationSymbols();
    }

    @Override
    public Set<ConfigurationTypeSymbol> getUniqueConfigurationTypeSymbols(SymbolWithConfigurations symbol) {
        return symbol.getUniqueConfigurationTypeSymbols();
    }
}
