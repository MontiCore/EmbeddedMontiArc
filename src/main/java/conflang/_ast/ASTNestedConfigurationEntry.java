package conflang._ast;

import conflang._symboltable.NestedConfigurationEntrySymbol;
import de.monticore.mcliterals._ast.ASTSignedLiteral;

import java.util.List;
import java.util.Optional;

public class ASTNestedConfigurationEntry extends ASTNestedConfigurationEntryTOP {

    public ASTNestedConfigurationEntry() {
    }

    public ASTNestedConfigurationEntry(List<ASTConfigurationEntry> configurationEntrys, String nestedEntries, String name, ASTSignedLiteral value) {
        super(configurationEntrys, nestedEntries, name, value);
    }

    // REFACTOR: SET VIA setNestedConfigurationEntrySymbolOpt() IN SYMBOL TABLE CREATION
    @Override
    public Optional<NestedConfigurationEntrySymbol> getNestedConfigurationEntrySymbolOpt() {
        if (symbol.isPresent()) {
            NestedConfigurationEntrySymbol nestedConfigurationEntrySymbol = (NestedConfigurationEntrySymbol) symbol.get();
            return Optional.of(nestedConfigurationEntrySymbol);
        }
        return super.getNestedConfigurationEntrySymbolOpt();
    }

    @Override
    public boolean isNestedConfiguration() {
        return true;
    }
}