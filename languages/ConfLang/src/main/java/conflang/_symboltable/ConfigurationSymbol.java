/* (c) https://github.com/MontiCore/monticore */
package conflang._symboltable;

import com.google.common.collect.Sets;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.SymbolKind;

import java.util.*;

public class ConfigurationSymbol extends ConfigurationSymbolTOP {

    private Set<ConfigurationSymbol> superConfigurations = Sets.newHashSet();

    public ConfigurationSymbol(String name) {
        super(name);
    }

    /**
     * Checks whether this configuration contains a configuration entry with the given name of any kind.
     *
     * @param name the name of the configuration entry.
     * @return true, if this configuration contains a configuration entry with the given name, false otherwise.
     */
    public boolean containsConfigurationEntry(final String name) {
        return containsConfigurationEntryOfKind(name, ConfigurationEntrySymbol.KIND) || containsConfigurationEntryOfKind(name, NestedConfigurationEntrySymbol.KIND);
    }

    public Optional<ConfigurationEntry> getConfigurationEntry(String name) {
        Optional<ConfigurationEntrySymbol> simpleConfigurationEntryOpt = getConfigurationEntryOfKindInternal(name, ConfigurationEntrySymbol.KIND);
        return simpleConfigurationEntryOpt.<Optional<ConfigurationEntry>>map(Optional::of).orElseGet(() -> getConfigurationEntryOfKindInternal(name, NestedConfigurationEntrySymbol.KIND));
    }

    public <T extends Symbol> Optional<T> getConfigurationEntryOfKind(final String name, SymbolKind kind) {
        return getConfigurationEntryOfKindInternal(name, kind);
    }

    public Set<ConfigurationSymbol> getSuperConfigurations() {
        return superConfigurations;
    }

    public void addSuperConfiguration(ConfigurationSymbol configurationSymbol) {
        superConfigurations.add(configurationSymbol);
    }

    private <T extends Symbol> Optional<T> getConfigurationEntryOfKindInternal(final String name, SymbolKind kind) {
        Scope spannedScope;
        Optional<T> configurationEntrySymbol;
        if (superConfigurations != null && !superConfigurations.isEmpty()) {
            for (ConfigurationSymbol superConfiguration : superConfigurations) {
                spannedScope = superConfiguration.getSpannedScope();
                configurationEntrySymbol = spannedScope.resolve(name, kind);
                if (configurationEntrySymbol.isPresent()) return configurationEntrySymbol;
            }
        }
        spannedScope = getSpannedScope();
        configurationEntrySymbol = spannedScope.resolve(name, kind);
        return configurationEntrySymbol;
    }

    private boolean containsConfigurationEntryOfKind(final String name, SymbolKind kind) {
        Scope spannedScope;
        Optional<ConfigurationEntrySymbol> configurationEntrySymbol;
        if (superConfigurations != null && !superConfigurations.isEmpty()) {
            for (ConfigurationSymbol superConfiguration : superConfigurations) {
                spannedScope = superConfiguration.getSpannedScope();
                configurationEntrySymbol = spannedScope.resolve(name, kind);
                if (configurationEntrySymbol.isPresent()) return true;
            }
        }
        spannedScope = getSpannedScope();
        configurationEntrySymbol = spannedScope.resolve(name, kind);
        return configurationEntrySymbol.isPresent();
    }
}