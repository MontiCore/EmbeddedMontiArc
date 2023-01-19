${tc.signature ("ASTConfigurationEntry","methodNamePrefix")}
<#assign configurationEntry = ASTConfigurationEntry/>
<#assign methodNamePrafix = methodNamePrefix/>
<#assign typeMappingUtil= tc.instantiate("de.monticore.lang.monticar.cnnarch.generator.util.ConfLangMappingUtil")/>

    def get${methodNamePrafix}_${configurationEntry.getName()}(self):
        return ${typeMappingUtil.mapConfigurationValueToPythonValue(configurationEntry.getValue())}