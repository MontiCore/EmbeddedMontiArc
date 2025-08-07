<#-- (c) https://github.com/MontiCore/monticore -->
<#if (ast.packageName)??>
package ${ast.packageName};
</#if>
package generated;

import conflang._symboltable.ConfigurationEntry;
import conflang._symboltable.ConfigurationSymbol;
import conflang._symboltable.ConfigurationEntrySymbol;

import java.util.Optional;

import static generated.Constants.*;

public class ${ast.name} {

    private ConfigurationSymbol configuration;

    public ${ast.name}(ConfigurationSymbol configuration) {
        this.configuration = configuration;
    }

<#list ast.basicSchemaProperties as property>
    <#assign type = property.type.nativeType.simpleName>
    <#assign parameterKey = property.name?upper_case>
    <#if type == "Object">
    public Boolean get${property.name?replace("_", " ")?capitalize?replace(" ", "")}() {
        Optional<ConfigurationEntry> parameterOpt = getConfigurationParameter(${parameterKey});
        if (!parameterOpt.isPresent()) return null;
        return true;
    }
    <#else>
    public ${type} get${property.name?replace("_", " ")?capitalize?replace(" ", "")}() {
        Optional<ConfigurationEntrySymbol> parameterOpt = getBasicConfigurationParameter(${parameterKey});
        if (!parameterOpt.isPresent()) {
            return null;
        }
        ConfigurationEntrySymbol parameter = parameterOpt.get();
        return (${type}) parameter.getValue();
    }
    </#if>

</#list>
    protected Optional<ConfigurationEntry> getConfigurationParameter(String parameterKey) {
        return configuration.getConfigurationEntry(parameterKey);
    }

    protected Optional<ConfigurationEntrySymbol> getBasicConfigurationParameter(final String parameterKey) {
        return configuration.getConfigurationEntryOfKind(parameterKey, ConfigurationEntrySymbol.KIND);
    }
}