/* (c) https://github.com/MontiCore/monticore */
package conflang._symboltable;

import java.util.Optional;

public class ConfLangSymbol extends ConfLangSymbolTOP {

    public ConfLangSymbol(final String name) {
        super(name);
    }

    /**
     * @param name
     * @return
     */
    public boolean hasConfigurationEntry(String name) {
        Optional<ConfigurationEntrySymbol> entry = getEnclosingScope().resolveConfigurationEntry(name);
        return entry.isPresent();
    }

    /**
     *
     * @param name
     * @return
     */
    public Optional<ConfigurationEntrySymbol> getConfigurationEntrySymbol(String name) {
        Optional<ConfigurationEntrySymbol> entry = getEnclosingScope().resolveConfigurationEntry(name);
        return entry;
    }
}