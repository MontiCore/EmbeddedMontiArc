package conflang._ast;

import conflang._symboltable.ConfigurationEntrySymbol;
import de.monticore.mcliterals._ast.ASTSignedLiteral;

import java.util.Optional;

public class ASTConfigurationEntry extends ASTConfigurationEntryTOP {

    public ASTConfigurationEntry() {
    }

    public ASTConfigurationEntry(String name, ASTSignedLiteral value) {
        super(name, value);
    }

    @Override
    public Optional<ConfigurationEntrySymbol> getConfigurationEntrySymbolOpt() {
        if (symbol.isPresent()) {
            ConfigurationEntrySymbol basicConfigurationEntrySymbol = (ConfigurationEntrySymbol) symbol.get();
            return Optional.of(basicConfigurationEntrySymbol);
        }
        return super.getConfigurationEntrySymbolOpt();
    }

    public boolean isNestedConfiguration() {
        return false;
    }
}