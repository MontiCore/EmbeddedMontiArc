package conflang._symboltable;

import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.SymbolKind;

public interface ConfigurationEntry extends Symbol {

    Object getValue();

    void setValue(Object value);

    boolean isOfSymbolKind(SymbolKind kind);
}