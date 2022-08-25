${tc.signature ("astComplexPropertyDefinition")}
<#assign schemaTypeUtil = tc.instantiate("de.monticore.lang.monticar.cnnarch.generator.util.SchemaTypeUtil")>
<#assign className = astComplexPropertyDefinition.getName()?capitalize>

    class ${className}:
        def __init__(self,outer):
         self.outer = outer
        def get_${className}_Value(self):
            return self.outer.training_configuration.get_${className}_Value()
#common hyperparameters
<#list astComplexPropertyDefinition.getSchemaMemberList() as schemaMember>
<#assign methodName = astComplexPropertyDefinition.getName()+schemaMember.getName()>
    <#if schemaTypeUtil.isPrimitiveSchemaMember(schemaMember)>
        def get${methodName}(self):
            return self.outer.training_configuration.get${methodName}()
    <#elseif schemaTypeUtil.isEnumSchemaMember(schemaMember)>
        def get${methodName}(self):
            return self.training_configuration.${methodName}()
    </#if>
</#list>
#value-specific hyperparameter
<#list astComplexPropertyDefinition.getComplexPropertyValueDefinitionList()  as astComplexPropertyValueDefinition>
    <#list astComplexPropertyValueDefinition.getSchemaMemberList() as astSchemaMember>
        <#assign methodName = astComplexPropertyValueDefinition.getName()+astSchemaMember.getName()>
        <#if schemaTypeUtil.isPrimitiveSchemaMember(astSchemaMember)>
        def get${methodName}(self):
            return self.outer.training_configuration.get${methodName}()
    </#if>
    </#list>
</#list>
