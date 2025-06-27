${tc.signature ("ASTNestedConfigurationEntry", "methodNamePrefix")}
<#assign typeMappingUtil= tc.instantiate("de.monticore.lang.monticar.cnnarch.generator.util.ConfLangMappingUtil")/>
<#assign methodNamePrafix = methodNamePrefix/>
<#assign configurationEntry = ASTNestedConfigurationEntry/>
<#assign configurationEntries = configurationEntry.getConfigurationEntryList()/>

    def get_${configurationEntry.getName()}_value(self):
        return ${typeMappingUtil.mapConfigurationValueToPythonValue(configurationEntry.getValue())}
    <#list configurationEntries as entry>
    ${tc.includeArgs("SimpleConfiguration",entry,[entry,"_"+configurationEntry.getName()])}
    </#list>
