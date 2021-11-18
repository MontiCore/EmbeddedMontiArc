package conflang._ast;

import com.google.common.collect.Lists;
import com.google.common.collect.Sets;
import conflang._symboltable.ConfigurationSymbol;
import de.monticore.mcbasictypes1._ast.ASTQualifiedName;

import java.util.List;
import java.util.Optional;
import java.util.Set;

public class ASTConfiguration extends ASTConfigurationTOP {

    private Set<ASTConfiguration> superConfigurations = Sets.newHashSet();

    public ASTConfiguration() {
    }

    public ASTConfiguration(String name, Optional<ASTQualifiedName> superConf, List<ASTConfigurationEntry> configurationEntrys) {
        super(name, superConf, configurationEntrys);
    }

    @Override
    public Optional<ConfigurationSymbol> getConfigurationSymbolOpt() {
        if (symbol.isPresent()) {
            ConfigurationSymbol configurationSymbol = (ConfigurationSymbol) symbol.get();
            return Optional.of(configurationSymbol);
        }
        return super.getConfigurationSymbolOpt();
    }

    public List<ASTConfigurationEntry> getAllConfigurationEntries() {
        if (superConfigurations.isEmpty()) {
            return getConfigurationEntryList();
        }

        List<ASTConfigurationEntry> allConfigurationEntries = Lists.newArrayList();
        for (ASTConfiguration superConfiguration : superConfigurations) {
            allConfigurationEntries.addAll(superConfiguration.getAllConfigurationEntries());
        }
        allConfigurationEntries.addAll(getConfigurationEntryList());
        return allConfigurationEntries;
    }

    public void addSuperConfiguration(ASTConfiguration superConfiguration) {
        superConfigurations.add(superConfiguration);
    }
}