${tc.signature ("schemaMembers")}
<#assign typeUtil = tc.instantiate("de.monticore.lang.monticar.cnnarch.generator.util.SchemaTypeUtil")>
<#list schemaMembers as schemaMember>
    <#assign methodName = schemaMember.getName()?cap_first><#if typeUtil.isPrimitiveSchemaMember(schemaMember)>
    def get${methodName}(self):
        self.training_configuration.get${methodName}()
    </#if>
</#list>