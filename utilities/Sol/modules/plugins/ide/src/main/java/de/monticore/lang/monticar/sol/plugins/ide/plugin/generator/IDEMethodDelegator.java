/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ide.plugin.generator;

import de.monticore.lang.monticar.sol.grammars.ide._symboltable.*;

import java.util.List;
import java.util.Optional;
import java.util.Set;

/**
 * ATTENTION READER: This interface and its corresponding implementation are necessary because the Freemarker
 * version of MontiCore does not support the default methods of Java 8.
 */
public interface IDEMethodDelegator {
    List<ConfigurationSymbol> getConfigurationSymbols(SymbolWithConfigurations symbol);
    List<OptionFillSymbol> getOptionFillSymbols(SymbolWithFilledOptions symbol);
    List<OptionInheritSymbol> getOptionInheritSymbols(SymbolWithInheritedOptions symbol);
    String getLabel(SymbolWithTypeAttribute symbol);
    Optional<String> getIcon(SymbolWithTypeAttribute symbol);
    Optional<String> getCategory(SymbolWithTypeAttribute symbol);
    List<ConfigurationTypeSymbol> getConfigurationTypeSymbols(SymbolWithConfigurations symbol);
    List<ConfigurationTypeSymbol> getAllConfigurationTypeSymbols(SymbolWithConfigurations symbol);
    List<ConfigurationSymbol> getZeroConfigurationSymbols(SymbolWithConfigurations symbol);
    List<ConfigurationSymbol> getPositiveConfigurationSymbols(SymbolWithConfigurations symbol);
    List<ConfigurationSymbol> getNegativeConfigurationSymbols(SymbolWithConfigurations symbol);
    Set<ConfigurationTypeSymbol> getUniqueConfigurationTypeSymbols(SymbolWithConfigurations symbol);
}
