${tc.signature ("astComplexPropertyDefinition")}
<#assign typeUtil = tc.instantiate("de.monticore.lang.monticar.cnnarch.generator.util.SchemaTypeUtil")>
<#assign className = astComplexPropertyDefinition.getName()?capitalize>

    class ${className}:
        def __init__(self,outer):
         self.outer = outer
        def get${className}Value(self):
            return self.outer.training_configuration.getValue()
<#list astComplexPropertyDefinition.getComplexPropertyValueDefinitionList()  as astComplexPropertyValueDefinition>
    <#list astComplexPropertyValueDefinition.getSchemaMemberList() as astSchemaMember>
        <#assign methodName = astComplexPropertyValueDefinition.getName()+astSchemaMember.getName()>
        <#if typeUtil.isPrimitiveSchemaMember(astSchemaMember)>
        def get${methodName}(self):
            return self.outer.training_configuration.get${methodName}()
    </#if>
    </#list>
</#list>