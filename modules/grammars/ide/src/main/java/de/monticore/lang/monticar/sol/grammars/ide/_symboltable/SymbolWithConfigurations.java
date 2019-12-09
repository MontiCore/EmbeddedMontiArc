/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide._symboltable;

import de.monticore.symboltable.Symbol;

import java.util.*;
import java.util.function.Predicate;
import java.util.stream.Collectors;

public interface SymbolWithConfigurations extends Symbol {
    void addConfiguration(ConfigurationSymbolReference configuration);
    List<ConfigurationSymbolReference> getConfigurations();
    List<ConfigurationSymbol> getConfigurationSymbols();

    default List<ConfigurationSymbol> getOrderedConfigurationSymbols(Predicate<ConfigurationSymbol> predicate) {
        //noinspection OptionalGetWithoutIsPresent
        Comparator<ConfigurationSymbol> comparator =
                Comparator.comparingInt(configuration -> configuration.getOrder().get());

        return this.getConfigurationSymbols().stream()
                .filter(configuration -> configuration.getOrder().isPresent())
                .filter(predicate)
                .sorted(comparator)
                .collect(Collectors.toList());
    }

    default List<ConfigurationSymbol> getZeroConfigurationSymbols() {
        //noinspection OptionalGetWithoutIsPresent
        Predicate<ConfigurationSymbol> predicate = configuration -> configuration.getOrder().get() == 0;

        return this.getOrderedConfigurationSymbols(predicate);
    }

    default List<ConfigurationSymbol> getPositiveConfigurationSymbols() {
        //noinspection OptionalGetWithoutIsPresent
        Predicate<ConfigurationSymbol> predicate = configuration -> configuration.getOrder().get() > 0;

        return this.getOrderedConfigurationSymbols(predicate);
    }

    default List<ConfigurationSymbol> getNegativeConfigurationSymbols() {
        //noinspection OptionalGetWithoutIsPresent
        Predicate<ConfigurationSymbol> predicate = configuration -> configuration.getOrder().get() < 0;

        return this.getOrderedConfigurationSymbols(predicate);
    }

    default List<ConfigurationTypeSymbol> getConfigurationTypeSymbols() {
        return this.getConfigurationSymbols().stream()
                .map(ConfigurationSymbol::getTypeSymbol)
                .filter(Optional::isPresent)
                .map(Optional::get)
                .collect(Collectors.toList());
    }

    default Set<ConfigurationTypeSymbol> getUniqueConfigurationTypeSymbols() {
        return this.getConfigurationSymbols().stream()
                .map(ConfigurationSymbol::getTypeSymbol)
                .filter(Optional::isPresent)
                .map(Optional::get)
                .collect(Collectors.toSet());
    }

    default List<ConfigurationTypeSymbol> getAllConfigurationTypeSymbols() {
        List<ConfigurationTypeSymbol> symbols = this.getConfigurationTypeSymbols();
        List<ConfigurationTypeSymbol> allSymbols = new ArrayList<>(symbols);

        symbols.stream()
                .flatMap(configuration -> configuration.getAllConfigurationTypeSymbols().stream())
                .forEach(allSymbols::add);

        return allSymbols;
    }
}
