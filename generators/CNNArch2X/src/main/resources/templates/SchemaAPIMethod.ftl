${tc.signature ("schemaMembers","astEnumeratedDeclarationList")}
<#assign typeUtil = tc.instantiate("de.monticore.lang.monticar.cnnarch.generator.util.SchemaTypeUtil")>
<#assign enumeratedDeclarations= astEnumeratedDeclarationList>

<#list schemaMembers as schemaMember>
    <#assign methodName = schemaMember.getName()><#if typeUtil.isPrimitiveSchemaMember(schemaMember)>
    def get_${methodName}(self):
        return self.training_configuration.get_${methodName}()
    <#elseif typeUtil.isObjectSchemaMember(schemaMember)>
    <#assign getterMethodName = schemaMember.getType().getGenericType()>
    def get_${getterMethodName}(self):
        return self.${getterMethodName}(self)
    <#elseif typeUtil.isComplexPropertyDefinition(schemaMember)>
    ${tc.includeArgs("templates.SchemaAPIObjectType",[schemaMember])}
    </#if>
</#list>

<#list enumeratedDeclarations as enumeratedDeclaration>
<#assign methodName = "get_"+enumeratedDeclaration.getName()>
    def ${methodName}(self):
        return self.training_configuration.${methodName}()
</#list>