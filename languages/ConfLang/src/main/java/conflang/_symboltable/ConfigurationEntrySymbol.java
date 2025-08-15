package conflang._symboltable;

import de.monticore.symboltable.Scope;
import de.monticore.symboltable.SymbolKind;

public class ConfigurationEntrySymbol extends ConfigurationEntrySymbolTOP implements ConfigurationEntry {

    private Object value;

    public ConfigurationEntrySymbol(String name) {
        super(name);
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
        return ConfigurationEntrySymbol.KIND.isSame(kind);
    }

    public boolean isChildOfNestedConfiguration() {
        Scope scope = getEnclosingScope();
        if (scope == null) {
            return false;
        }
        return scope instanceof NestedConfigurationEntryScope;
    }
}