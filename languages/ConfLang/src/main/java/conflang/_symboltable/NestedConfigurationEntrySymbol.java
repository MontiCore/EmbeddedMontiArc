package conflang._symboltable;

import com.google.common.collect.Lists;
import conflang._ast.ASTConfigurationEntry;
import conflang._ast.ASTNestedConfigurationEntry;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.SymbolKind;

import java.util.List;
import java.util.Optional;

public class NestedConfigurationEntrySymbol extends NestedConfigurationEntrySymbolTOP implements ConfigurationEntry {

    private Object value;

    public NestedConfigurationEntrySymbol(String name) {
        super(name);
    }

    public List<ConfigurationEntry> getAllConfigurationEntries() {
        Optional<ASTNestedConfigurationEntry> nodeOpt = getNestedConfigurationEntryNode();
        ASTNestedConfigurationEntry node = nodeOpt.get();

        List<ASTConfigurationEntry> configurationEntries = node.getConfigurationEntryList();
        if (configurationEntries == null || configurationEntries.isEmpty()) {
            return Lists.newArrayList();
        }

        List<ConfigurationEntry> configurationSymbols = Lists.newArrayList();
        for (ASTConfigurationEntry configurationEntry : configurationEntries) {
            configurationSymbols.add((ConfigurationEntry) configurationEntry.getSymbol());
        }
        return configurationSymbols;
    }

    public Optional<ConfigurationEntry> getConfigurationEntry(String name) {
        Optional<ConfigurationEntrySymbol> simpleConfigurationEntryOpt = getConfigurationEntryOfKindInternal(name, ConfigurationEntrySymbol.KIND);
        return simpleConfigurationEntryOpt.<Optional<ConfigurationEntry>>map(Optional::of).orElseGet(() -> getConfigurationEntryOfKindInternal(name, NestedConfigurationEntrySymbol.KIND));
    }

    public <T extends Symbol> Optional<T> getConfigurationEntryOfKind(String name, SymbolKind kind) {
        Scope spannedScope = getSpannedScope();
        return spannedScope.resolve(name, kind);
    }

    public boolean hasConfigurationEntry(final String name) {
        return containsConfigurationEntryOfKindInternal(name, ConfigurationEntrySymbol.KIND)
                || containsConfigurationEntryOfKindInternal(name, NestedConfigurationEntrySymbol.KIND);
    }

    @Override
    public Object getValue() {
        return value;
    }

    @Override
    public void setValue(Object value) {
        this.value = value;
    }

    @Override
    public boolean isOfSymbolKind(SymbolKind kind) {
        return NestedConfigurationEntrySymbol.KIND.isSame(kind);
    }

    private <T extends Symbol> Optional<T> getConfigurationEntryOfKindInternal(final String name, SymbolKind kind) {
        Scope spannedScope = getSpannedScope();
        Optional<T> configurationEntrySymbol = spannedScope.resolve(name, kind);
        return configurationEntrySymbol;
    }

    private boolean containsConfigurationEntryOfKindInternal(final String name, SymbolKind kind) {
        Scope spannedScope = getSpannedScope();
        Optional<Symbol> symbolOpt = spannedScope.resolve(name, kind);
        return symbolOpt.isPresent();
    }
}